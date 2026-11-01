/* SPDX-License-Identifier: MIT */
/* AUTO_HYBRID_V1 foreground adapter. One arbiter owns automatic/manual FCW
 * proposals; the proven eight-register probe owns paced reads. SysTick keeps
 * independent heater deadlines. Neither control path depends on USB/DTR. */
#include "hybrid_probe.h"
#include "gnss_probe.h"
#include "main.h"
#include "thermal_probe.h"
#include "timing_probe.h"
#include "settings_flash.h"
#include "simpleclock/fcw_trial.h"
#include "simpleclock/hybrid_input.h"
#include "simpleclock/thermal_outer.h"
#include "simpleclock/text.h"
#include "simpleclock/version.h"
#include <math.h>
#include <string.h>

static sc_hybrid_t controller;
static sc_thermal_outer_state_t outer;
static bool initial_target_adopted, centering_permitted;
static sc_fcw_trial_t manual;
static thermal_probe_snapshot_t thermal;
static uint32_t epoch_sequence, last_outer_ms, last_frame, last_report_ms;
static uint32_t duty_frames_rejected;
static uint32_t rf_ms, rf_tow_ms, rf_week;
static bool rf_valid, rf_clear, have_epoch, outer_sampled, manual_owned;
static uint32_t write_ms[4], readback_ms[3], write_mask, writes, drops, command_denied;
static uint32_t compute_us, max_compute_us, seen_transition, transition_drops;
static int64_t commanded_word;
static uint16_t observed_clip;
static bool transition_pending;
static bool config_report_pending = true;
static uint32_t bulk_pending, last_bulk_attempt_ms;
static uint32_t transition_state, transition_reason;
/* Single IRQ->foreground request slot. A HOLD latch has priority over an
 * unexecuted resume; disconnection discards commands but never clears HOLD. */
/* Longest accepted command is 33 bytes (both five-digit time constants). */
static char command_mailbox[40];
static volatile uint32_t command_length, command_ms;
static volatile uint32_t command_mailbox_denied;
static uint32_t command_busy_sent, command_reply_ms;
static char command_reply[sizeof(command_mailbox)];
static volatile uint32_t command_reply_length;
static bool command_reply_accepted;
static volatile bool hold_requested;
/* Accepted reboot survives USB disconnect. Only explicit requests reach here;
 * neither reset nor persistence is part of normal recovery. */
static uint8_t reboot_target, reboot_stage;
static uint32_t reboot_ms;
static bool reboot_resume, reboot_waiting_actuator;
static bool system_defaults_pending;
/* Fresh receiver-valid solution, independent of clock mode, HOLD or lock. */
static uint32_t last_usable_ms;
static bool usable_seen;
/* Latched one-slot events: a newer event replaces an unsent one and counts a
 * drop. Telemetry never blocks control or heater deadlines. */
static uint32_t failure_sent, reanchor_sent, event_drops, warning_bits, warning_previous,
    warning_ms;
static bool warning_pending;
/* Passive receiver TDOP (4001 rev0) for host export only; never a control gate. */
static uint32_t dop_ms, dop_week, dop_tow_ms;
static uint16_t dop_tdop;
static uint8_t dop_nr_sv;
static bool dop_seen, dop_valid;
static void consume_epoch(uint32_t now_ms);

static bool centering_permission(void)
{
    /* Thermal centering depends only on its own safe, running PID and on no
     * conflicting manual FCW trial. Clock state and GNSS are not inputs. */
    const bool permitted = initial_target_adopted && thermal.safe && thermal.warmed &&
                           thermal.running && !manual_owned;
    centering_permitted = permitted;
    if (!permitted)
    {
        /* An outage shorter than a heater frame must still break integration. */
        sc_thermal_outer_freeze(&outer);
    }
    return permitted;
}

void hybrid_probe_init(void)
{
    command_reply_length = 0U;
    reboot_target = reboot_stage = 0U;
    reboot_resume = reboot_waiting_actuator = false;
    system_defaults_pending = false;
    sc_hybrid_init(&controller);
    sc_settings_t saved;
    uint32_t generation;
    if (settings_flash_load(&saved, &generation) == SC_SETTINGS_VALID)
    {
        (void)sc_hybrid_profile_parameters(&controller, SC_HYBRID_PROFILE_PPP,
                                           (double)saved.ppp_frequency_s,
                                           (double)saved.ppp_phase_s);
        (void)sc_hybrid_profile_parameters(&controller, SC_HYBRID_PROFILE_SBAS,
                                           (double)saved.sbas_frequency_s,
                                           (double)saved.sbas_phase_s);
        (void)sc_hybrid_profile_parameters(&controller, SC_HYBRID_PROFILE_STANDALONE,
                                           (double)saved.standalone_frequency_s,
                                           (double)saved.standalone_phase_s);
        (void)sc_hybrid_profile_parameters(&controller, SC_HYBRID_PROFILE_STATIC,
                                           (double)saved.static_frequency_s,
                                           (double)saved.static_phase_s);
        (void)sc_hybrid_bias_clip(&controller, saved.bias_clip_ps);
    }
    sc_thermal_outer_config_t config;
    sc_thermal_outer_default_config(&config);
    if (!sc_thermal_outer_init(&outer, &config))
    {
        /* An outer configuration fault stops the heater (LED3 fault) but must
         * not disable otherwise healthy clock discipline. */
        thermal_probe_fault();
    }
}

static sc_settings_t current_settings(void)
{
    return (sc_settings_t){gnss_probe_selection(),
                           (uint32_t)controller.frequency_time_s,
                           (uint32_t)controller.phase_time_s,
                           (uint32_t)controller.sbas_frequency_time_s,
                           (uint32_t)controller.sbas_phase_time_s,
                           (uint32_t)controller.standalone_frequency_time_s,
                           (uint32_t)controller.standalone_phase_time_s,
                           gnss_probe_user_profile() ? 1U : 0U,
                           gnss_probe_pps_policy(),
                           controller.bias_clip_ps,
                           controller.static_frequency_time_s,
                           controller.static_phase_time_s};
}

const sc_hybrid_t *hybrid_probe_state(void)
{
    return &controller;
}

bool hybrid_probe_receiver_change_ready(void)
{
    return !controller.pending && !manual_owned && !sc_fcw_trial_active(&manual) &&
           !reboot_target && !command_length;
}

bool hybrid_probe_receiver_persist(bool managed)
{
    if (controller.pending || manual_owned || sc_fcw_trial_active(&manual))
    {
        return false;
    }
    sc_settings_t active = current_settings();
    active.receiver_user_profile = managed ? 1U : 0U;
    sc_settings_t saved;
    uint32_t generation;
    if (settings_flash_load(&saved, &generation) == SC_SETTINGS_VALID &&
        sc_settings_equal(&active, &saved))
    {
        return true;
    }
    const bool resume = controller.enabled;
    sc_hybrid_hold(&controller);
    const bool ok = settings_flash_save(&active);
    if (ok && resume && !hold_requested)
    {
        sc_hybrid_resume(&controller);
    }
    config_report_pending = true;
    return ok;
}

void hybrid_probe_defaults_finish(bool receiver_ready)
{
    const bool apply = system_defaults_pending && receiver_ready;
    system_defaults_pending = false;
    if (apply)
    {
        sc_settings_t defaults;
        sc_settings_defaults(&defaults);
        (void)sc_hybrid_profile_parameters(&controller, SC_HYBRID_PROFILE_PPP,
                                           defaults.ppp_frequency_s, defaults.ppp_phase_s);
        (void)sc_hybrid_profile_parameters(&controller, SC_HYBRID_PROFILE_SBAS,
                                           defaults.sbas_frequency_s, defaults.sbas_phase_s);
        (void)sc_hybrid_profile_parameters(&controller, SC_HYBRID_PROFILE_STANDALONE,
                                           defaults.standalone_frequency_s,
                                           defaults.standalone_phase_s);
        (void)sc_hybrid_profile_parameters(&controller, SC_HYBRID_PROFILE_STATIC,
                                           defaults.static_frequency_s, defaults.static_phase_s);
        (void)sc_hybrid_bias_clip(&controller, defaults.bias_clip_ps);
        (void)gnss_probe_pps_policy_set(defaults.pps_policy);
    }
}

uint32_t hybrid_probe_writes(void)
{
    return writes;
}

bool hybrid_probe_locked(void)
{
    /* Thermal state is a separate LED3/USB warning, not a clock lock condition. */
    return controller.locked && controller.known && controller.gnss_good && !hold_requested &&
           gnss_probe_auto_ready();
}

uint32_t hybrid_probe_thermal_warning(void)
{
    return warning_bits;
}

bool hybrid_probe_frequency_settled(void)
{
    return controller.frequency.settled;
}

bool hybrid_probe_faulted(void)
{
    return controller.state == SC_HYBRID_ACTUATOR_UNKNOWN || controller.acquisition_latched ||
           outer.config_fault || outer.safety_fault;
}

void hybrid_probe_manual_request(void)
{
    hold_requested = true;
}

void hybrid_probe_disconnect(void)
{
    command_length = 0;
    command_reply_length = 0;
}

bool hybrid_probe_command(const char *line)
{
    if (strncmp(line, "hybrid ", 7) != 0 && strncmp(line, "outer ", 6) != 0 &&
        strncmp(line, "pps ", 4) != 0 && strncmp(line, "system reboot ", 14) != 0 &&
        strcmp(line, "system defaults") != 0 && strcmp(line, "clock trial") != 0 &&
        strcmp(line, "clock abort") != 0)
    {
        return false;
    }
    if (strcmp(line, "hybrid hold") == 0 || strcmp(line, "clock abort") == 0)
    {
        hold_requested = true;
    }
    const size_t length = strlen(line);
    if (command_length || length >= sizeof(command_mailbox))
    {
        command_mailbox_denied++;
        return true; /* A full mailbox cannot postpone the independent HOLD latch. */
    }
    memcpy(command_mailbox, line, length + 1U);
    command_ms = HAL_GetTick();
    __DMB();
    command_length = (uint32_t)length;
    return true;
}

void hybrid_probe_observe(const sc_sbf_observation_t *o, uint32_t now_ms)
{
    if (o->id == SC_SBF_DOP)
    {
        dop_seen = true;
        dop_ms = now_ms;
        dop_week = o->week;
        dop_tow_ms = o->tow_ms;
        dop_valid = (o->valid & (SC_SBF_VALID_EPOCH | SC_SBF_VALID_TDOP)) ==
                    (SC_SBF_VALID_EPOCH | SC_SBF_VALID_TDOP);
        dop_nr_sv = o->data.dop.nr_sv;
        dop_tdop = o->data.dop.tdop;
    }
    if (o->id == SC_SBF_RF_STATUS)
    {
        rf_ms = now_ms;
        rf_week = o->week;
        rf_tow_ms = o->tow_ms;
        rf_valid = (o->valid & (SC_SBF_VALID_EPOCH | SC_SBF_VALID_RF_STATUS)) ==
                   (SC_SBF_VALID_EPOCH | SC_SBF_VALID_RF_STATUS);
        rf_clear = rf_valid && o->data.rf_status.flags == 0U;
    }
}

static bool rf_good(const sc_epoch_t *epoch, uint32_t now_ms)
{
    if (!epoch || !rf_valid || !rf_clear || (uint32_t)(now_ms - rf_ms) > 2500U)
    {
        return false;
    }
    const int64_t delta =
        ((int64_t)rf_week - epoch->week) * SC_SBF_WEEK_MS + (int64_t)rf_tow_ms - epoch->tow_ms;
    /* Latest RF status may precede or follow the assembled PVT by one epoch. */
    return delta >= -1000 && delta <= 1000;
}

static void append_u32(sc_text_t *text, const char *key, uint32_t value)
{
    sc_text_string(text, key);
    sc_text_u32(text, value);
}

static void append_scaled(sc_text_t *text, const char *key, double value, double scale)
{
    sc_text_string(text, key);
    const double scaled = value * scale;
    if (!isfinite(scaled) || fabs(scaled) >= 9e18)
    {
        sc_text_string(text, "NA");
    }
    else
    {
        sc_text_i64(text, (int64_t)(scaled + (scaled < 0.0 ? -0.5 : 0.5)));
    }
}

static bool send(sc_text_t *text)
{
    sc_text_string(text, "\r\n");
    return text->ok &&
           simpleclock_usb_cdc_try_send_queued((const uint8_t *)text->data, (uint16_t)text->length);
}

/* Each record is submitted separately. USB has one bounded transmit buffer,
 * not a multi-record queue: a BUSY response retains this part for a later poll.
 * At the next second, old bulk parts are dropped explicitly; transitions have
 * their own reserved slot and priority. Control never waits for either. */
static bool report(uint32_t now_ms, uint32_t part)
{
    char line[SIMPLECLOCK_TMP117_USB_TX_MAX];
    sc_text_t t;
    sc_text_init(&t, line, sizeof(line));
    if (part == 0U)
    {
        sc_text_string(&t, "type=clock_auto v=1 firmware=AUTO_HYBRID_V1 config=");
        sc_text_string(&t, SC_HYBRID_CONFIG_SHA256);
        sc_text_string(&t, " loop_mode=");
        sc_text_string(&t, sc_hybrid_loop_name((sc_hybrid_loop_mode_t)controller.loop_mode));
        sc_text_string(&t, controller.phase_enabled ? " phase_mode=RELATIVE_FIXED"
                           : controller.loop_mode == SC_HYBRID_LOOP_OFF ? " phase_mode=OFF"
                           : controller.loop_mode == SC_HYBRID_LOOP_PHASE
                               ? " phase_mode=PHASE_WAIT"
                               : " phase_mode=FREQUENCY_ONLY");
        sc_text_string(&t, " absolute_aligned=0 calibrated=0 state=");
        sc_text_string(&t, sc_hybrid_state_name(controller.state));
        append_u32(&t, " uptime_ms=", now_ms);
        append_u32(&t, " enabled=", controller.enabled);
        append_u32(&t, " reason=", controller.reason);
        append_u32(&t, " generation=", controller.generation);
        append_u32(&t, " phase_valid=", controller.phase_valid);
        append_u32(&t, " relative_lock=", controller.locked);
        append_u32(&t, " frequency_settled=", controller.frequency.settled);
        append_u32(&t,
                   " standalone=", controller.source_mode == 1U || controller.source_mode == 3U);
        append_u32(&t, " source_mode=", controller.source_mode);
        append_u32(&t, " time_system=", controller.time_system);
        append_u32(&t, " phase_enabled=", controller.phase_enabled);
        append_u32(&t, " phase_samples=", controller.phase_good_samples);
        append_u32(&t, " frequency_ramping=", controller.ramping);
        append_u32(&t, " gnss_good=", controller.gnss_good);
        append_u32(&t, " known=", controller.known);
        append_u32(&t, " qualify_ms=", controller.qualifying ? now_ms - controller.qualify_ms : 0U);
        append_u32(&t, " attempts=", controller.attempts);
        append_u32(&t, " verify_samples=", controller.verify_samples);
        append_u32(&t, " source_age_ms=", now_ms - controller.last_received_ms);
        sc_text_string(&t, " epoch_ms=");
        sc_text_u64(&t, controller.previous_epoch_ms);
        sc_text_string(&t, " reference_epoch_ms=");
        sc_text_u64(&t, controller.phase_reference_epoch_ms);
        append_scaled(&t, " bias_ps=", controller.previous_bias_ns, 1000.0);
        append_scaled(&t, " reference_ps=", controller.phase_reference_ns, 1000.0);
        append_scaled(&t, " error_ps=", controller.phase_error_ns, 1000.0);
        append_scaled(&t, " filtered_error_ps=", controller.filtered_error_ns, 1000.0);
        append_scaled(&t, " d_nppb=", controller.previous_drift_ppb, 1e9);
        append_scaled(&t, " v_nppb=", controller.desired_drift_ppb, 1e9);
        append_scaled(&t, " I_nppb=", controller.phase_integral_ppb, 1e9);
        return send(&t);
    }

    if (part == 1U)
    {
        sc_text_string(&t, "type=clock_actuator v=1");
        append_u32(&t, " uptime_ms=", now_ms);
        append_u32(&t, " manual=", manual_owned);
        append_u32(&t, " writes=", writes);
        append_u32(&t, " known=", controller.known);
        append_u32(&t, " pending=", controller.pending);
        append_scaled(&t, " requested_nppb=", controller.requested_ppb, 1e9);
        append_scaled(&t, " accumulator_nppb=", controller.accumulator_ppb, 1e9);
        append_scaled(&t, " applied_nppb=", controller.applied_ppb, 1e9);
        append_scaled(&t, " remainder_nppb=", controller.accumulator_ppb - controller.applied_ppb,
                      1e9);
        sc_text_string(&t, " commanded_fcw=");
        sc_text_i64(&t, commanded_word);
        sc_text_string(&t, " readback_fcw=");
        sc_text_i64(&t, controller.applied_word);
        append_u32(&t, " write_mask=", write_mask);
        append_u32(&t, " write_start_ms=", write_ms[0]);
        append_u32(&t, " write_0c_ms=", write_ms[1]);
        append_u32(&t, " write_0d_ms=", write_ms[2]);
        append_u32(&t, " write_0e_ms=", write_ms[3]);
        append_u32(&t, " read_0c_ms=", readback_ms[0]);
        append_u32(&t, " read_0d_ms=", readback_ms[1]);
        append_u32(&t, " read_0e_ms=", readback_ms[2]);
        append_u32(&t, " limiting=", controller.limited);
        append_u32(&t, " slew_limited=", controller.slew_limited);
        append_u32(&t, " phase_limited=", controller.phase_limited);
        append_u32(&t, " compute_us=", compute_us);
        append_u32(&t, " max_compute_us=", max_compute_us);
        append_u32(&t, " tx_drops=", drops);
        append_u32(&t, " transition_drops=", transition_drops);
        append_u32(&t, " command_denied=", command_denied + command_mailbox_denied);
        return send(&t);
    }

    if (part == 2U)
    {
        sc_text_string(&t, "type=thermal_outer v=1 gain_status=PROVISIONAL_NOT_MEASURED");
        append_u32(&t, " uptime_ms=", now_ms);
        append_u32(&t, " enabled=", outer.enabled);
        append_u32(&t, " pause_reason=", (uint32_t)outer.pause_reason);
        append_u32(&t, " centered=", outer.duty_centered);
        append_u32(&t, " target_limited=", outer.power_target_limited);
        append_u32(&t, " centering_permitted=", centering_permitted);
        append_u32(&t, " duty_permille=", thermal.completed_duty_permille);
        append_u32(&t, " mean30_permille=", outer.mean_duty_permille);
        append_u32(&t, " filtered_duty_permille=", outer.filtered_duty_permille);
        append_u32(&t, " window_valid=", outer.window_valid);
        append_u32(&t, " duty_data_valid=", outer.duty_data_valid);
        append_u32(&t, " duty_frames_rejected=", duty_frames_rejected);
        append_u32(&t, " target_duty_permille=", outer.config.target_duty_permille);
        sc_text_string(&t, " target_c=");
        sc_text_q16(&t, outer.target_q16);
        sc_text_string(&t, " raw_c=");
        sc_text_q16(&t, (int32_t)thermal.raw * 512);
        sc_text_string(&t, " filtered_c=");
        sc_text_q16(&t, thermal.filtered_q16);
        sc_text_string(&t, " error_c=");
        sc_text_q16(&t, (int32_t)thermal.raw * 512 - thermal.target_q16);
        sc_text_string(&t, " inner_p_micro=");
        sc_text_i32(&t, thermal.proportional_micro);
        sc_text_string(&t, " inner_i_micro=");
        sc_text_i32(&t, thermal.integral_micro);
        sc_text_string(&t, " inner_d_micro=");
        sc_text_i32(&t, thermal.derivative_micro);
        sc_text_string(&t, " target_rate_c_s=");
        sc_text_q16(&t, outer.target_rate_q16_per_s);
        /* The legacy three-decimal field truncates the sub-1 mK/s cap.
         * Preserve it; publish the exact Q16 rate for quantitative analysis. */
        sc_text_string(&t, " target_rate_q16_per_s=");
        sc_text_i32(&t, outer.target_rate_q16_per_s);
        sc_text_string(&t, " gain_K_per_duty=");
        sc_text_q16(&t, outer.config.gain_q16_per_unit_duty);
        sc_text_string(&t, " min_c=");
        sc_text_q16(&t, outer.config.target_min_q16);
        sc_text_string(&t, " max_c=");
        sc_text_q16(&t, outer.config.target_max_q16);
        sc_text_string(&t, " trip_c=");
        sc_text_q16(&t, outer.config.overtemperature_trip_q16);
        append_scaled(&t, " fll_time_s=",
                      controller.loop_mode == SC_HYBRID_LOOP_PHASE
                          ? 0.0
                          : sc_hybrid_frequency_time(&controller),
                      1.0);
        append_scaled(&t, " phase_time_s=",
                      controller.phase_enabled ? sc_hybrid_phase_time(&controller) : 0.0, 1.0);
        return send(&t);
    }

    if (part == 3U)
    {
        sc_text_string(&t, "type=hybrid_quality v=1");
        append_u32(&t, " uptime_ms=", now_ms);
        append_u32(&t, " reason=", controller.reason);
        append_u32(&t,
                   " rf_fresh_clear=", rf_valid && rf_clear && (uint32_t)(now_ms - rf_ms) <= 2500U);
        append_u32(&t, " statistics_samples=", controller.history.count);
        const unsigned required_points = sc_hybrid_tracking(&controller)
                                             ? SC_HYBRID_TRACKING_POINTS
                                             : SC_HYBRID_QUALIFICATION_POINTS;
        append_u32(&t, " statistics_ready=", controller.history.count >= required_points);
        append_u32(&t, " statistics_required_samples=", required_points);
        append_u32(&t, " frequency_samples=", controller.frequency.count);
        append_u32(&t, " frequency_window_valid=", controller.frequency.window_valid);
        append_u32(&t, " frequency_good_ms=",
                   controller.frequency.timing ? now_ms - controller.frequency.since_ms : 0U);
        append_scaled(&t, " frequency_mean_drift_nppb=", controller.frequency.mean_drift_ppb, 1e9);
        append_scaled(&t, " frequency_mean_error_nppb=", controller.frequency.mean_error_ppb, 1e9);
        append_scaled(&t, " frequency_sd_nppb=", controller.frequency.drift_sd_ppb, 1e9);
        append_u32(&t, " frequency_age_ms=", now_ms - controller.frequency.last_ms);
        append_u32(&t, " receiver_config_ready=", gnss_probe_auto_ready());
        append_u32(&t, " receiver_config_fault=", gnss_probe_auto_fault());
        append_scaled(&t, " drift_sd_nppb=", controller.metrics.drift_sd_ppb, 1e9);
        append_scaled(&t, " bias_slope_nppb=", controller.metrics.bias_slope_ppb, 1e9);
        append_scaled(&t, " residual_slope_nppb=", controller.metrics.residual_slope_ppb, 1e9);
        append_scaled(&t, " residual_rms_ps=", controller.metrics.residual_detrended_rms_ns,
                      1000.0);
        return send(&t);
    }
    if (part == 4U)
    {
        sc_text_init(&t, line, sizeof(line));
        static const char *const policies[] = {"CONTINUOUS", "QUALIFIED", "OFF", "MANUAL"};
        gnss_probe_pps_snapshot_t pps = {0};
        const bool pps_known = gnss_probe_pps_snapshot(&pps);
        const uint32_t policy = gnss_probe_pps_policy();
        /* v4 adds the Static-PVT pair; version= is the firmware release. */
        sc_text_string(&t, "type=hybrid_config v=4 version=" SC_FIRMWARE_VERSION
                           " phase_mode=RELATIVE_FIXED pps_policy=");
        sc_text_string(&t, policy <= SC_SETTINGS_PPS_MANUAL ? policies[policy] : "UNKNOWN");
        sc_text_string(&t, " pps_config_hex=");
        for (const char *p = pps.response; *p; p++)
        {
            sc_text_hex(&t, (uint8_t)*p, 2U);
        }
        append_u32(&t, " pps_observed=", pps_known);
        /* Verification/pending/fault/time quality are in the gnss_pps record. */
        append_u32(&t, " bias_clip_ps=", controller.bias_clip_ps);
        append_u32(&t, " additional_wait_s=", 0U);
        append_u32(&t, " host_required=", 0U);
        append_u32(&t, " preheat_s=", SC_HYBRID_THERMAL_BOOT_HEATER_OFF_WARMUP_S);
        append_u32(&t, " warmup_is_convergence_proof=", 0U);
        sc_text_string(&t, " loop_mode=");
        sc_text_string(&t, sc_hybrid_loop_name((sc_hybrid_loop_mode_t)controller.loop_mode));
        append_scaled(&t, " ppp_fll_time_s=", controller.frequency_time_s, 1.0);
        append_scaled(&t, " ppp_phase_time_s=", controller.phase_time_s, 1.0);
        append_scaled(&t, " sbas_fll_time_s=", controller.sbas_frequency_time_s, 1.0);
        append_scaled(&t, " sbas_phase_time_s=", controller.sbas_phase_time_s, 1.0);
        append_scaled(&t, " standalone_fll_time_s=", controller.standalone_frequency_time_s, 1.0);
        append_scaled(&t, " standalone_phase_time_s=", controller.standalone_phase_time_s, 1.0);
        append_scaled(&t, " static_fll_time_s=", controller.static_frequency_time_s, 1.0);
        append_scaled(&t, " static_phase_time_s=", controller.static_phase_time_s, 1.0);
        const sc_settings_t active = current_settings();
        sc_settings_t saved;
        uint32_t generation;
        const sc_settings_status_t status = settings_flash_load(&saved, &generation);
        append_u32(&t, " gnss_selection=", active.gnss_selection);
        append_u32(&t, " receiver_user_profile=", active.receiver_user_profile);
        append_u32(&t, " settings_saved=", status == SC_SETTINGS_VALID);
        append_u32(&t, " settings_dirty=",
                   status != SC_SETTINGS_VALID || !sc_settings_equal(&active, &saved));
        append_u32(&t, " settings_generation=", generation);
        append_u32(&t, " settings_fault=",
                   status == SC_SETTINGS_INVALID || status == SC_SETTINGS_IO_ERROR);
        if (send(&t))
        {
            config_report_pending = !gnss_probe_auto_ready();
            return true;
        }
        return false;
    }
    if (part == 5U)
    {
        /* Clip limit is in hybrid_config; the latched trigger is in
         * clock_failure. v2 adds bridged short receiver gaps. */
        sc_text_string(&t, "type=clock_guard v=2");
        append_u32(&t, " uptime_ms=", now_ms);
        append_u32(&t, " clipped=", controller.input_clipped);
        append_u32(&t, " clipped_samples=", controller.clipped_samples);
        append_scaled(&t, " innovation_ps=", controller.phase_innovation_ns, 1000.0);
        append_u32(&t, " integral_limited=", controller.integral_limited);
        append_u32(&t, " lock_failures=", controller.lock_failures);
        append_u32(&t, " thermal_warning=", warning_bits);
        append_u32(&t, " reanchors=", controller.reanchor_count);
        append_scaled(&t, " reanchor_total_ps=", controller.reanchor_total_ns, 1000.0);
        append_u32(&t, " bridged_gaps=", controller.bridged_gaps);
        append_u32(&t, " bridged_epochs=", controller.bridged_epochs);
        append_u32(&t,
                   " gnss_usable=", usable_seen && (uint32_t)(now_ms - last_usable_ms) <= 2500U);
        append_u32(&t, " event_drops=", event_drops);
        return send(&t);
    }
    if (part == 6U)
    {
        /* Passive receiver TDOP for host export; never a discipline gate. Raw
         * hundredths: 0 means unavailable, 65535 is a valid 655.35. age_ms is
         * UINT32_MAX until a DOP block has been seen. */
        sc_text_string(&t, "type=gnss_dop v=1");
        append_u32(&t, " uptime_ms=", now_ms);
        append_u32(&t, " valid=", dop_valid);
        append_u32(&t, " age_ms=", dop_seen ? now_ms - dop_ms : UINT32_MAX);
        append_u32(&t, " week=", dop_week);
        append_u32(&t, " tow_ms=", dop_tow_ms);
        append_u32(&t, " nr_sv=", dop_nr_sv);
        append_u32(&t, " tdop_x100=", dop_tdop);
        return send(&t);
    }
    return false;
}

static bool integer(const char **cursor, uint32_t *value)
{
    const char *p = *cursor;
    uint32_t n = 0;
    if (*p < '0' || *p > '9')
    {
        return false;
    }
    do
    {
        const uint32_t digit = (uint32_t)(*p++ - '0');
        if (n > 100000U)
        {
            return false;
        }
        n = n * 10U + digit;
    } while (*p >= '0' && *p <= '9');
    *cursor = p;
    *value = n;
    return true;
}

static void command_reply_service(uint32_t now_ms)
{
    const uint32_t length = command_reply_length;
    if (!length)
    {
        return;
    }
    char reply[192];
    sc_text_t t;
    sc_text_init(&t, reply, sizeof(reply));
    sc_text_string(&t, "type=hybrid_command v=1");
    append_u32(&t, " uptime_ms=", command_reply_ms);
    append_u32(&t, " accepted=", command_reply_accepted);
    sc_text_string(&t, " command_hex=");
    for (uint32_t i = 0; i < length; i++)
    {
        sc_text_hex(&t, (uint8_t)command_reply[i], 2U);
    }
    if (send(&t))
    {
        command_reply_length = 0U;
    }
    else if ((uint32_t)(now_ms - command_reply_ms) >= 3000U)
    {
        drops++;
        command_reply_length = 0U; /* No host can retain the mailbox indefinitely. */
    }
}

/* Settings edits need no FCW in flight. While steering, one is pending for most
 * of each second, so these wait in the mailbox for the call that follows the
 * paced readback (hybrid_probe_cycle); expiry still rejects them. */
static bool needs_settled_actuator(const char *line)
{
    return strncmp(line, "hybrid params ", 14U) == 0 || strncmp(line, "hybrid clip ", 12U) == 0 ||
           strcmp(line, "hybrid save") == 0;
}

static void commands(uint32_t now_ms)
{
    char line[sizeof(command_mailbox)];
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    const bool hold = hold_requested;
    hold_requested = false;
    uint32_t length = command_reply_length ? 0U : command_length;
    const uint32_t age = now_ms - command_ms;
    if (length)
    {
        memcpy(line, command_mailbox, length + 1U);
        if (controller.pending && age <= 1500U && needs_settled_actuator(line))
        {
            length = 0U; /* Leave it queued; nothing below consumes it. */
        }
        else
        {
            command_length = 0;
        }
    }
    __set_PRIMASK(mask);
    if (hold)
    {
        reboot_resume = false;
        sc_hybrid_hold(&controller);
        if (manual_owned)
        {
            sc_fcw_trial_abort(&manual, now_ms);
        }
    }
    if (!length)
    {
        if (command_reply_length)
        {
            command_reply_service(now_ms);
        }
        else if (command_busy_sent != command_mailbox_denied)
        {
            char reply[128];
            sc_text_t t;
            const uint32_t denied = command_mailbox_denied;
            sc_text_init(&t, reply, sizeof(reply));
            sc_text_string(&t, "type=hybrid_command v=1 accepted=0 status=busy command_hex=");
            append_u32(&t, " count=", denied);
            if (send(&t))
            {
                command_busy_sent = denied;
            }
        }
        return;
    }
    bool accepted = age <= 1500U;
    if (strcmp(line, "hybrid hold") == 0 || strcmp(line, "clock abort") == 0)
    {
        accepted = true;
    }
    else if (!accepted)
    { /* Expired input cannot start or resume anything. */
    }
    else if (strcmp(line, "hybrid status") == 0 || strcmp(line, "hybrid config") == 0 ||
             strcmp(line, "pps status") == 0)
    {
        last_report_ms = now_ms - 1000U;
        config_report_pending = true;
    }
    else if (reboot_target)
    {
        accepted = false; /* Status/HOLD above remain usable during a reboot. */
    }
    else if (strncmp(line, "system reboot ", 14U) == 0)
    {
        const char *target = line + 14;
        const uint8_t selected = strcmp(target, "mcu") == 0    ? 1U
                                 : strcmp(target, "gnss") == 0 ? 2U
                                 : strcmp(target, "all") == 0  ? 3U
                                                               : 0U;
        accepted = selected != 0U && !hold && !manual_owned && !sc_fcw_trial_active(&manual) &&
                   gnss_probe_commands_idle();
        if (accepted)
        {
            reboot_resume = controller.enabled;
            reboot_waiting_actuator = controller.pending;
            /* HOLD stops new proposals but retains an in-flight FCW word
             * until its independent paced readback has completed. */
            sc_hybrid_hold(&controller);
            reboot_target = selected;
            reboot_stage = 0U;
            reboot_ms = now_ms;
        }
    }
    else if (!strcmp(line, "system defaults"))
    {
        accepted = !controller.pending && !manual_owned && !sc_fcw_trial_active(&manual) &&
                   gnss_probe_commands_idle();
        if (accepted)
        {
            sc_hybrid_hold(&controller);
            accepted = gnss_probe_defaults_request();
            system_defaults_pending = accepted;
        }
    }
    else if (strcmp(line, "hybrid resume") == 0)
    {
        /* Clock resume needs only clock-side resources; thermal faults are
         * reported on LED3 and handled by the outer/heater commands. */
        accepted = !sc_fcw_trial_active(&manual) && manual.phase != SC_FCW_FAULT && !hold;
        if (accepted && !gnss_probe_auto_ready() && gnss_probe_auto_fault())
        {
            accepted = gnss_probe_auto_recheck();
        }
        if (accepted)
        {
            manual_owned = false;
            sc_hybrid_resume(&controller);
        }
    }
    else if (strcmp(line, "outer pause") == 0)
    {
        sc_thermal_outer_pause(&outer);
    }
    else if (strcmp(line, "outer resume") == 0)
    {
        (void)sc_thermal_outer_clear_safety_fault(&outer);
        sc_thermal_outer_resume(&outer);
    }
    else if (strncmp(line, "hybrid clip ", 12U) == 0)
    {
        /* Bounded runtime option; persisted only by explicit hybrid save. */
        const char *p = line + 12;
        uint32_t value = 0U;
        accepted = !manual_owned && !hold && integer(&p, &value) && *p == '\0' &&
                   sc_hybrid_bias_clip(&controller, value);
        config_report_pending = true;
    }
    else if (strncmp(line, "pps policy ", 11U) == 0)
    {
        const char *p = line + 11;
        const uint32_t policy = strcmp(p, "continuous") == 0  ? SC_SETTINGS_PPS_CONTINUOUS
                                : strcmp(p, "qualified") == 0 ? SC_SETTINGS_PPS_QUALIFIED
                                : strcmp(p, "off") == 0       ? SC_SETTINGS_PPS_OFF
                                : strcmp(p, "manual") == 0    ? SC_SETTINGS_PPS_MANUAL
                                                              : UINT32_MAX;
        accepted = policy != UINT32_MAX && gnss_probe_pps_policy_set(policy);
        config_report_pending = true;
    }
    else if (strcmp(line, "clock trial") == 0)
    {
        sc_hybrid_hold(&controller);
        accepted = !controller.pending && controller.known &&
                   controller.applied_ppb + 1.0 <= controller.upper_ppb &&
                   sc_fcw_trial_start(&manual, controller.applied_word, observed_clip, now_ms,
                                      controller.gnss_good && thermal.safe);
        manual_owned = accepted;
    }
    else if (strncmp(line, "hybrid params ", 14U) == 0)
    {
        const char *p = line + 14;
        sc_hybrid_profile_t profile = SC_HYBRID_PROFILE_PPP;
        if (strncmp(p, "ppp ", 4U) == 0)
        {
            p += 4;
        }
        else if (strncmp(p, "sbas ", 5U) == 0)
        {
            profile = SC_HYBRID_PROFILE_SBAS;
            p += 5;
        }
        else if (strncmp(p, "standalone ", 11U) == 0)
        {
            profile = SC_HYBRID_PROFILE_STANDALONE;
            p += 11;
        }
        else if (strncmp(p, "static ", 7U) == 0)
        {
            profile = SC_HYBRID_PROFILE_STATIC;
            p += 7;
        }
        uint32_t frequency = 0, phase = 0;
        accepted =
            !manual_owned && !hold && integer(&p, &frequency) && *p++ == ' ' &&
            integer(&p, &phase) && *p == '\0' &&
            sc_hybrid_profile_parameters(&controller, profile, (double)frequency, (double)phase);
        config_report_pending = true;
    }
    else if (strcmp(line, "hybrid save") == 0)
    {
        accepted = !manual_owned && !hold && !controller.pending && !sc_fcw_trial_active(&manual) &&
                   (gnss_probe_selection_ready() ||
                    (gnss_probe_commands_idle() && gnss_probe_user_profile()));
        if (accepted)
        {
            /* The active values are already live, so control keeps running: no
             * FCW is in flight (checked above), receiver bytes keep arriving by
             * DMA during the <=40 ms erase stall, and a late epoch is a bridged
             * gap. Holding/resuming here cost a requalification and re-anchor. */
            const sc_settings_t settings = current_settings();
            accepted = settings_flash_save(&settings);
            now_ms = HAL_GetTick();
            config_report_pending = true;
        }
    }
    else if (strncmp(line, "outer gain ", 11U) == 0)
    {
        const char *p = line + 11;
        uint32_t gain = 0;
        /* The explicit outer pause is the thermal owner's own interlock. */
        accepted =
            outer.explicit_paused && integer(&p, &gain) && *p == '\0' && gain >= 1U && gain <= 20U;
        if (accepted)
        {
            outer.config.gain_q16_per_unit_duty = (int32_t)(gain * 65536U);
            outer.target_remainder_num = 0;
        }
    }
    else
    {
        accepted = false;
    }
    if (!accepted)
    {
        command_denied++;
    }
    memcpy(command_reply, line, length);
    command_reply_ms = now_ms;
    command_reply_accepted = accepted;
    __DMB();
    command_reply_length = length;
    command_reply_service(now_ms); /* Retain through BUSY; never execute twice. */
}

static void service_reboot(uint32_t now_ms)
{
    if (!reboot_target)
    {
        return;
    }
    if (reboot_waiting_actuator && !controller.pending && controller.known)
    {
        reboot_waiting_actuator = false;
    }
    const bool failed = reboot_stage == 4U ||
                        (reboot_stage == 0U && (uint32_t)(now_ms - reboot_ms) >= 3000U) ||
                        (reboot_stage == 1U && ((reboot_waiting_actuator && !controller.known) ||
                                                (uint32_t)(now_ms - reboot_ms) > 5000U)) ||
                        (reboot_stage >= 2U &&
                         (gnss_probe_reboot_failed() || (uint32_t)(now_ms - reboot_ms) > 30000U));
    if (failed && reboot_stage != 4U)
    {
        reboot_stage = 4U;
        reboot_ms = now_ms;
    }
    const bool ready =
        reboot_stage == 2U && gnss_probe_reboot_transmitted() && gnss_probe_selection_ready();
    if (reboot_stage == 0U || failed || ready)
    {
        char line[160];
        sc_text_t t;
        sc_text_init(&t, line, sizeof(line));
        sc_text_string(&t, "type=system_reboot v=1 target=");
        sc_text_string(&t, reboot_target == 1U ? "mcu" : reboot_target == 2U ? "gnss" : "all");
        sc_text_string(&t, failed  ? " stage=failed"
                           : ready ? " stage=requalified"
                                   : " stage=accepted");
        append_u32(&t, " uptime_ms=", now_ms);
        append_u32(&t, " saved=", 0U);
        if (!send(&t))
        {
            if (failed && (uint32_t)(now_ms - reboot_ms) >= 3000U)
            {
                drops++;
                reboot_target = 0U; /* A vanished host cannot retain a failed reset request. */
            }
            return; /* Retain acknowledgement on USB BUSY. */
        }
        if (failed)
        {
            reboot_target = 0U; /* Remain held; do not retry an uncertain reset. */
            return;
        }
        if (ready)
        {
            if (reboot_target == 3U)
            {
                reboot_stage = 3U;
                reboot_ms = now_ms;
            }
            else
            {
                reboot_target = 0U;
                if (reboot_resume && !hold_requested)
                {
                    sc_hybrid_resume(&controller);
                }
            }
            return;
        }
        reboot_stage = 1U;
        reboot_ms = now_ms;
    }
    if ((reboot_stage == 1U || reboot_stage == 3U) && !reboot_waiting_actuator &&
        !controller.pending && (uint32_t)(now_ms - reboot_ms) >= 500U &&
        simpleclock_usb_cdc_tx_idle())
    {
        if (reboot_target == 1U || reboot_stage == 3U)
        {
            reboot_target = 0U;
            simpleclock_system_reset();
        }
        else if (gnss_probe_reboot_request())
        {
            reboot_stage = 2U;
            reboot_ms = now_ms;
        }
        else if ((uint32_t)(now_ms - reboot_ms) > 5000U)
        {
            reboot_stage = 2U;
            reboot_ms = now_ms - 30001U; /* Report a bounded busy timeout. */
        }
    }
}

/* Derived only from the heater's own snapshot, independent of GNSS samples.
 * REGULATION is exactly LED3's verdict; duty headroom is judged only on a
 * valid 30 s window. */
static void thermal_warning_service(uint32_t now_ms)
{
    const uint32_t bits =
        (!thermal.safe || !thermal.sensor_valid ? HYBRID_PROBE_THERMAL_SENSOR : 0U) |
        (!thermal.warmed ? HYBRID_PROBE_THERMAL_STARTUP : 0U) |
        (thermal.sensor_valid && thermal.warmed && !thermal.regulation_stable
             ? HYBRID_PROBE_THERMAL_REGULATION
             : 0U) |
        (outer.window_valid && outer.duty_data_valid &&
                 (outer.mean_duty_permille < 20U || outer.mean_duty_permille > 980U)
             ? HYBRID_PROBE_THERMAL_DUTY
             : 0U);
    if (bits != warning_bits)
    {
        event_drops += warning_pending ? 1U : 0U;
        warning_previous = warning_bits;
        warning_bits = bits;
        warning_ms = now_ms;
        warning_pending = true;
    }
}

/* Latched controller snapshot of the raw sample that dropped lock or started a
 * clipped stretch. It persists after recovery; skipped sequences are counted. */
static bool report_failure(uint32_t now_ms)
{
    const sc_hybrid_failure_t *f = &controller.last_failure;
    char line[SIMPLECLOCK_TMP117_USB_TX_MAX];
    sc_text_t t;
    sc_text_init(&t, line, sizeof(line));
    sc_text_string(&t, "type=clock_failure v=1");
    append_u32(&t, " uptime_ms=", now_ms);
    append_u32(&t, " seq=", f->sequence);
    append_u32(&t, " captured_ms=", f->now_ms);
    sc_text_string(&t, " epoch_ms=");
    sc_text_u64(&t, f->epoch_ms);
    append_u32(&t, " flags=", f->flags);
    append_scaled(&t, " bias_ps=", f->bias_ns, 1000.0);
    /* previous drift = drift - trim_delta - drift_innovation. */
    append_scaled(&t, " drift_nppb=", f->drift_ppb, 1e9);
    append_scaled(&t, " trim_delta_nppb=", f->trim_delta_ppb, 1e9);
    append_scaled(&t, " drift_innovation_nppb=", f->innovation_ppb, 1e9);
    append_scaled(&t, " phase_error_ps=", f->phase_error_ns, 1000.0);
    append_scaled(&t, " bias_slope_nppb=", f->bias_slope_ppb, 1e9);
    append_u32(&t, " source_mode=", f->source_mode);
    append_u32(&t, " time_system=", f->time_system);
    append_u32(&t, " phase_enabled=", f->phase_enabled);
    append_u32(&t, " event_drops=", event_drops);
    return send(&t);
}

/* Restart clock break: a fresh local phase reference was taken instead of
 * steering the holdover phase error back. step = new minus old reference. */
static bool report_reanchor(uint32_t now_ms)
{
    char line[192];
    sc_text_t t;
    sc_text_init(&t, line, sizeof(line));
    sc_text_string(&t, "type=phase_reanchor v=1");
    append_u32(&t, " uptime_ms=", now_ms);
    append_u32(&t, " seq=", controller.reanchor_count);
    sc_text_string(&t, " epoch_ms=");
    sc_text_u64(&t, controller.phase_reference_epoch_ms);
    append_scaled(&t, " step_ps=", controller.reanchor_step_ns, 1000.0);
    append_scaled(&t, " total_ps=", controller.reanchor_total_ns, 1000.0);
    append_u32(&t, " generation=", controller.generation);
    append_u32(&t, " source_mode=", controller.source_mode);
    append_u32(&t, " time_system=", controller.time_system);
    append_u32(&t, " event_drops=", event_drops);
    return send(&t);
}

static bool report_thermal_warning(uint32_t now_ms)
{
    char line[192];
    sc_text_t t;
    sc_text_init(&t, line, sizeof(line));
    sc_text_string(&t, "type=thermal_warning v=1");
    append_u32(&t, " uptime_ms=", now_ms);
    append_u32(&t, " changed_ms=", warning_ms);
    /* entered = bits & ~previous; cleared = previous & ~bits. */
    append_u32(&t, " bits=", warning_bits);
    append_u32(&t, " previous=", warning_previous);
    sc_text_string(&t, " error_c=");
    sc_text_q16(&t, (int32_t)thermal.raw * 512 - thermal.target_q16);
    append_u32(&t, " mean30_permille=", outer.mean_duty_permille);
    append_u32(&t, " event_drops=", event_drops);
    return send(&t);
}

void hybrid_probe_service(uint32_t now_ms)
{
    thermal_probe_snapshot(now_ms, &thermal);
    commands(now_ms);
    service_reboot(now_ms);
    sc_hybrid_service(&controller, now_ms);
    consume_epoch(now_ms);
    gnss_probe_pps_quality(usable_seen && (uint32_t)(now_ms - last_usable_ms) <= 2500U);
    (void)centering_permission();
    if (manual_owned && !thermal.safe)
    {
        sc_fcw_trial_abort(&manual, now_ms);
    }
    thermal_warning_service(now_ms);
    /* SysTick may publish a completed frame after the caller's timestamp,
     * including while consume_epoch refreshes the snapshot. Read time AFTER
     * those snapshots: unsigned age against an older time would wrap and
     * incorrectly discard a valid frame, restarting the 30 s duty window. */
    now_ms = HAL_GetTick();
    /* Follow completed one-second heater frames rather than accumulating
     * foreground scheduling jitter into an independent sampling cadence.
     * A missed frame is missing duty evidence, never a repeated last duty. */
    const uint32_t frame_delta = thermal.frame_sequence - last_frame;
    if ((frame_delta != 0U || (uint32_t)(now_ms - last_outer_ms) > 2500U) &&
        (thermal.sensor_valid || outer_sampled))
    {
        last_outer_ms = now_ms;
        const uint32_t start = TIM2->CNT;
        sc_thermal_outer_input_t input = {0};
        input.raw_temperature_q16 = (int32_t)thermal.raw * 512;
        input.filtered_temperature_q16 = thermal.filtered_q16;
        input.applied_duty_permille = thermal.completed_duty_permille;
        input.applied_duty_valid = thermal.completed_frame_valid && frame_delta == 1U &&
                                   (uint32_t)(now_ms - thermal.frame_ms) < 2500U;
        if (!input.applied_duty_valid)
        {
            duty_frames_rejected++;
        }
        last_frame = thermal.frame_sequence;
        input.sensor_valid = thermal.sensor_valid && thermal.faults == 0U;
        if (thermal.running && (!initial_target_adopted ||
                                (outer.explicit_paused && outer.target_q16 != thermal.target_q16)))
        {
            /* Drop boost/previous-target duty history once at handoff. Manual
             * target changes remain authoritative while the outer loop is
             * explicitly paused, independently of clock HOLD. */
            initial_target_adopted = sc_thermal_outer_adopt_target(&outer, thermal.target_q16);
        }
        input.centering_permitted = centering_permission();
        int32_t target;
        (void)sc_thermal_outer_update(&outer, now_ms, &input, &target);
        outer_sampled = true;
        if (outer.safety_fault || outer.config_fault)
        {
            thermal_probe_fault();
        }
        else if (thermal.running && target != thermal.target_q16)
        {
            (void)thermal_probe_set_target(target);
        }
        compute_us = TIM2->CNT - start;
        if (compute_us > max_compute_us)
        {
            max_compute_us = compute_us;
        }
    }
    if (seen_transition != controller.transitions)
    {
        if (transition_pending)
        {
            transition_drops++;
        }
        seen_transition = controller.transitions;
        transition_state = (uint32_t)controller.state;
        transition_reason = controller.reason;
        transition_pending = true;
    }
    if (transition_pending)
    {
        char line[160];
        sc_text_t t;
        sc_text_init(&t, line, sizeof(line));
        sc_text_string(&t, "type=hybrid_transition v=1");
        append_u32(&t, " uptime_ms=", now_ms);
        append_u32(&t, " seq=", seen_transition);
        append_u32(&t, " state=", transition_state);
        append_u32(&t, " reason=", transition_reason);
        if (send(&t))
        {
            transition_pending = false;
        }
    }
    /* Events share the transition priority over bulk records; each retries on
     * USB BUSY without delaying control. */
    const uint32_t failure_sequence = controller.last_failure.sequence;
    if (!transition_pending && failure_sequence != failure_sent && report_failure(now_ms))
    {
        event_drops += failure_sequence - failure_sent - 1U;
        failure_sent = failure_sequence;
    }
    const uint32_t reanchors = controller.reanchor_count;
    if (!transition_pending && reanchors != reanchor_sent && report_reanchor(now_ms))
    {
        event_drops += reanchors - reanchor_sent - 1U;
        reanchor_sent = reanchors;
    }
    if (!transition_pending && warning_pending && report_thermal_warning(now_ms))
    {
        warning_pending = false;
    }
    if ((uint32_t)(now_ms - last_report_ms) >= 1000U)
    {
        last_report_ms = now_ms;
        for (unsigned i = 0; i < 7U; i++)
        {
            drops += (bulk_pending >> i) & 1U;
        }
        /* Bit 4 (hybrid_config) only while a configuration report is due. */
        bulk_pending = config_report_pending ? 0x7FU : 0x6FU;
    }
    if (bulk_pending && !transition_pending && (uint32_t)(now_ms - last_bulk_attempt_ms) >= 10U)
    {
        last_bulk_attempt_ms = now_ms;
        for (unsigned i = 0; i < 7U; i++)
        {
            if (bulk_pending & (1U << i))
            {
                if (report(now_ms, i))
                {
                    bulk_pending &= ~(1U << i);
                }
                break;
            }
        }
    }
}

/* Consume at receiver cadence, independently of the once/s register probe.
 * During a pending write, retain the latest epoch until readback completes;
 * invalid source status revokes readiness immediately. No backlog integration. */
static void consume_epoch(uint32_t now_ms)
{
    const sc_epoch_t *epoch = timing_probe_latest();
    if (!epoch && controller.gnss_good)
    {
        /* The assembler clears its latest epoch on transport/CRC loss. That
         * explicit invalidation is immediate, not a fresh-data timeout. */
        sc_hybrid_input_t invalid = {0};
        invalid.now_ms = now_ms;
        sc_hybrid_sample(&controller, &invalid);
    }
    if (epoch)
    {
        const bool fresh = !have_epoch || epoch->sequence != epoch_sequence;
        /* Avoid repeating IEEE conversion/window work on the foreground poll.
         * Cached status can only revoke readiness; it never renews its age. */
        if (!fresh &&
            (!controller.gnss_good || (gnss_probe_auto_ready() && rf_good(epoch, now_ms) &&
                                       (uint32_t)(now_ms - controller.last_received_ms) <= 2500U)))
        {
            return;
        }
        sc_hybrid_input_t input = {0};
        (void)sc_hybrid_input_epoch(&input, epoch, now_ms, gnss_probe_auto_ready(),
                                    rf_good(epoch, now_ms), gnss_probe_static_pvt());
        /* PPS quality follows fresh receiver validity even while an FCW write
         * is pending or the clock is held; cached epochs never renew it. */
        if (fresh && (input.gnss_valid || input.standalone_valid || input.sbas_valid))
        {
            usable_seen = true;
            last_usable_ms = input.received_ms; /* Arrival time: a repeat cannot renew it. */
        }
        if ((!fresh || controller.pending) &&
            (input.gnss_valid || input.standalone_valid || input.sbas_valid))
        {
            return;
        }
        if (!controller.pending)
        {
            have_epoch = true;
            epoch_sequence = epoch->sequence;
        }
        /* A cached discontinuity must not repeatedly invalidate a newly made
         * reference. Its first observation owns the coordinate transition. */
        input.coordinate_changed = input.coordinate_changed && fresh;
        input.frequency_discontinuity = input.frequency_discontinuity && fresh;
        const uint32_t start = TIM2->CNT;
        sc_hybrid_sample(&controller, &input);
        compute_us = TIM2->CNT - start;
        if (compute_us > max_compute_us)
        {
            max_compute_us = compute_us;
        }
    }
}

void hybrid_probe_cycle(const sit5801_probe_t *probe, const uint32_t read_ms[3], uint32_t now_ms)
{
    const bool valid = sit5801_probe_word_order(probe) == 1U && probe->wire[2] == 0U &&
                       (probe->wire[5] & 0x01ffU) == 0U;
    const int64_t current = sit5801_fcw_decode(probe->wire + 3);
    if (valid)
    {
        observed_clip = probe->wire[1];
    }
    memcpy(readback_ms, read_ms, sizeof(readback_ms));
    sc_hybrid_observed(&controller, valid, current, probe->wire[1], now_ms);
    if (manual_owned)
    {
        sc_fcw_trial_observed(&manual, now_ms, valid, current);
    }
    thermal_probe_snapshot(now_ms, &thermal);
    commands(now_ms); /* Honor a HOLD received during the paced probe. */
    sc_hybrid_service(&controller, now_ms);
    consume_epoch(now_ms);
    int64_t next;
    const bool proposed =
        manual_owned
            ? sc_fcw_trial_next(&manual, now_ms, controller.gnss_good && thermal.safe, &next)
            : sc_hybrid_take_command(&controller, now_ms, &next);
    if (!proposed)
    {
        return;
    }
    uint16_t words[3];
    if (!sit5801_fcw_encode(next, words))
    {
        return;
    }
    commanded_word = next;
    write_ms[0] = HAL_GetTick();
    write_mask = 0U;
    write_ms[1] = write_ms[2] = write_ms[3] = 0U;
    /* Same M6-tested ordered independent transactions: MSB-first 0C,0D,0E;
     * 0E commits. Complete paced readback precedes the next proposal. */
    for (unsigned i = 0; i < 3U; i++)
    {
        uint8_t bytes[2] = {(uint8_t)(words[i] >> 8), (uint8_t)words[i]};
        const HAL_StatusTypeDef result =
            (I2C2->ISR & I2C_ISR_BUSY)
                ? HAL_ERROR
                : HAL_I2C_Mem_Write(&hi2c2, SIT5801_ADDRESS_7BIT << 1U, (uint16_t)(0x0cU + i),
                                    I2C_MEMADD_SIZE_8BIT, bytes, 2U, 5U);
        write_ms[i + 1U] = HAL_GetTick();
        if (result != HAL_OK)
        {
            break;
        }
        write_mask |= 1U << i;
    }
    writes++;
    if (manual_owned)
    {
        sc_fcw_trial_written(&manual, HAL_GetTick(), write_mask == 7U);
        if (write_mask != 7U)
        {
            sc_hybrid_observed(&controller, false, current, 0U, now_ms);
        }
    }
    else
    {
        sc_hybrid_written(&controller, write_mask == 7U, HAL_GetTick());
    }
}
