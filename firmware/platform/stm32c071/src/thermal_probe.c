/* SPDX-License-Identifier: MIT */
/* Autonomous thermal startup and PID, plus explicit operator commands. Acquisition
 * and control run in foreground; independent interrupt deadlines own shutdown. */
#include "thermal_probe.h"
#include "main.h"
#include "gnss_probe.h"
#include "simpleclock/heater.h"
#include "simpleclock/preheat.h"
#include "simpleclock/thermal_control.h"
#include "simpleclock/status_leds.h"
#include "simpleclock/text.h"
#include "simpleclock/tmp117.h"
#include <string.h>
#ifdef SIMPLECLOCK_ACTUATOR_TEST
#include "actuator_probe.h"
#endif
#ifdef SIMPLECLOCK_AUTO_HYBRID
#include "hybrid_probe.h"
#include "simpleclock/hybrid_defaults.h"
#include "simpleclock/thermal_boot.h"
#endif

/* User-selected manual delay. The independent OCXO qualification estimator
 * retains the plan's 300 s minimum; manual permission is never qualification. */
#define MANUAL_WARMUP_MS 120000U
#define PI_WARMUP_MS 300000U
#define AUTO_RISE_Q16 (2L * SC_THERMAL_Q16)
#ifdef SIMPLECLOCK_AUTO_HYBRID
#define TRIP_TEMPERATURE_RAW (70 * 128) /* AUTO_HYBRID independent raw trip, TMP117 counts. */
#define AUTO_HYBRID_TARGET_MIN_Q16                                                                 \
    ((int32_t)(SC_HYBRID_THERMAL_OUTER_SETPOINT_MIN_C * SC_THERMAL_Q16))
#define AUTO_HYBRID_TARGET_MAX_Q16                                                                 \
    ((int32_t)(SC_HYBRID_THERMAL_OUTER_SETPOINT_MAX_C * SC_THERMAL_Q16))
#define PROFILE_TARGET_MAX_Q16 AUTO_HYBRID_TARGET_MAX_Q16
#else
#define TRIP_TEMPERATURE_RAW (50 * 128) /* Legacy raw trip, TMP117 counts. */
#define PROFILE_TARGET_MAX_Q16 (45L * SC_THERMAL_Q16)
#endif
#define MANUAL_MAX_PERMILLE 1000U
#define OCXO_MAX_AGE_MS 1500U
#define COMMAND_MAX_AGE_MS 1000U

static sc_temperature_t temperature;
#ifdef SIMPLECLOCK_AUTO_HYBRID
/* The autonomous profile uses slope capture, never legacy power-plateau
 * qualification. Do not retain its unused 120-point (960-byte) history. */
static struct
{
    uint32_t start_ms;
} preheat;
#else
static sc_preheat_t preheat;
#endif
static sc_heater_t heater;
static sc_thermal_control_t control;
static sc_thermal_indicator_t indicator;

/* One automatic attempt per boot. IRQ OFF/faults and foreground startup share
 * the actuator lock. Clearing a fault never re-arms this state machine. */
typedef enum
{
    AUTO_WAITING,
    AUTO_ACTIVE,
    AUTO_CANCELED,
    AUTO_FAULT,
    AUTO_TARGET_RANGE,
    AUTO_BOOST
} auto_state_t;

static auto_state_t auto_state;
static int32_t auto_baseline_q16;
static int32_t auto_requested_target_q16;
static bool auto_target_clamped;
static uint32_t auto_start_ms;
static bool manual_warmup_complete, pi_warmup_complete;
static uint32_t last_pi_report_ms, pi_tx_drops;
static volatile bool initialized;
/* USB IRQ produces a single line; main consumes it under a short IRQ mask.
 * A full mailbox rejects/counts later lines; OFF is immediate and cancels it. */
static char command_line[48];
static volatile uint32_t command_length, command_drops, command_cancels;
static volatile bool command_pending;
static uint32_t command_sequence, last_report_ms, tx_drops;
static char reply[192];
static uint16_t reply_length;
#ifdef SIMPLECLOCK_AUTO_HYBRID
static char rx_line[196];
#else
static char rx_line[48];
#endif
static size_t rx_used;
static bool rx_bad;
static uint32_t ocxo_ms;
static bool ocxo_valid;
static volatile uint32_t command_received_ms;
/* Diagnostic tick counts, not calibrated time or measured delivered power.
 * IDR is sampled before this tick's write, after the preceding tick settled. */
static uint32_t drive_ticks, pin_high_ticks;
#ifdef SIMPLECLOCK_AUTO_HYBRID
static sc_thermal_boot_t boot;
static uint32_t last_boot_report_ms;
static uint32_t duty_frame_ms, duty_ticks, duty_on_ticks, duty_sequence;
static uint16_t completed_duty_permille;
static bool completed_frame_valid;
static volatile bool storage_inhibit;
#endif

static uint32_t heater_lock(void)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return mask;
}

static void heater_unlock(uint32_t mask)
{
    __DMB();
    __set_PRIMASK(mask);
}

void thermal_probe_tick(uint32_t now_ms)
{
    if (!initialized)
    {
        return;
    }
    /* USB OFF can preempt SysTick. Serialize the decision and pin write so a
     * preempted tick cannot undo a newer OFF request. No I/O waits under the mask. */
    const uint32_t mask = heater_lock();
    if (GPIOB->IDR & GPIO_PIN_3)
    {
        pin_high_ticks++;
    }
    bool on = sc_heater_tick(&heater, now_ms);
#ifdef SIMPLECLOCK_AUTO_HYBRID
    on = on && !storage_inhibit;
    if (duty_frame_ms != heater.frame_ms)
    {
        completed_frame_valid = duty_ticks >= 990U && duty_ticks <= 1010U;
        completed_duty_permille =
            duty_ticks ? (uint16_t)((duty_on_ticks * 1000U + duty_ticks / 2U) / duty_ticks) : 0U;
        duty_ticks = duty_on_ticks = 0U;
        duty_frame_ms = heater.frame_ms;
        duty_sequence++;
    }
    duty_ticks++;
    duty_on_ticks += on ? 1U : 0U;
#endif
    if (heater.faults &&
        (auto_state == AUTO_WAITING || auto_state == AUTO_ACTIVE || auto_state == AUTO_BOOST))
    {
        auto_state = AUTO_FAULT;
    }
    if (on)
    {
        drive_ticks++;
    }
    GPIOB->BSRR = on ? GPIO_PIN_3 : ((uint32_t)GPIO_PIN_3 << 16U);
    heater_unlock(mask);
}

void thermal_probe_fault(void)
{
    const uint32_t mask = heater_lock();
    if (initialized)
    {
        sc_heater_fault(&heater, SC_HEATER_FAULT_CONTROL);
        auto_state = AUTO_FAULT;
    }
    GPIOB->BRR = GPIO_PIN_3;
    heater_unlock(mask);
}

#ifdef SIMPLECLOCK_AUTO_HYBRID
void thermal_probe_storage_inhibit(bool inhibit)
{
    const uint32_t mask = heater_lock();
    storage_inhibit = inhibit;
    completed_frame_valid = false;
    duty_ticks = duty_on_ticks = 0U;
    GPIOB->BRR = GPIO_PIN_3;
    heater_unlock(mask);
}
#endif

void thermal_probe_init(uint32_t now_ms)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    preheat.start_ms = now_ms;
#else
    sc_preheat_init(&preheat, now_ms);
#endif
    sc_thermal_control_init(&control);
    memset(&indicator, 0, sizeof(indicator));
    /* AUTO_HYBRID selects slope-based full-power startup and PID. The proven
     * PWM, sensor and ISR deadlines remain shared. Legacy diagnostics keep
     * their original preheat/PI policy. USB never grants boot permission. */
    const sc_heater_profile_t profile = {true, MANUAL_MAX_PERMILLE, TRIP_TEMPERATURE_RAW};
    const uint32_t mask = heater_lock();
    auto_state = AUTO_WAITING;
    auto_baseline_q16 = 0;
    auto_requested_target_q16 = 0;
    auto_target_clamped = false;
    manual_warmup_complete = pi_warmup_complete = false;
    sc_heater_init(&heater, profile, now_ms);
#ifdef SIMPLECLOCK_AUTO_HYBRID
    if (!sc_thermal_control_pid_defaults(&control))
    {
        sc_heater_fault(&heater, SC_HEATER_FAULT_CONTROL);
        auto_state = AUTO_FAULT;
    }
    duty_frame_ms = now_ms;
    sc_thermal_boot_start(&boot, now_ms);
#endif
    if (HAL_SYSCFG_GetPinBinding(HAL_BIND_TSSOP20_PIN20) != HAL_BIND_TSSOP20_PIN20_PB3)
    {
        sc_heater_fault(&heater, SC_HEATER_FAULT_CONTROL);
    }
    initialized = true;
    GPIOB->BRR = GPIO_PIN_3;
    heater_unlock(mask);
}

void thermal_probe_command(const char *line, size_t length)
{
    /* Heater commands own the heater only. Clock HOLD is an independent,
     * explicit operator choice; a heater OFF/fault must not stop discipline. */
    if (length == 11U && memcmp(line, "thermal off", 11) == 0)
    {
        const uint32_t mask = heater_lock();
        auto_state = AUTO_CANCELED; /* Immediate, even if the reply mailbox is full. */
        if (initialized)
        {
            sc_heater_off(&heater);
        }
        GPIOB->BRR = GPIO_PIN_3;
        command_pending = false;
        command_cancels++; /* Also invalidates a command already copied by main. */
        memcpy(command_line, line, length);
        command_length = (uint32_t)length;
        command_received_ms = HAL_GetTick();
        command_pending = true;
        heater_unlock(mask);
        return;
    }
    if (length == 0U)
    {
        return;
    }
    if (command_pending || length >= sizeof(command_line))
    {
        command_drops++;
        return;
    }
    memcpy(command_line, line, length);
    command_length = (uint32_t)length;
    command_received_ms = HAL_GetTick();
    __DMB();
    command_pending = true;
}

void thermal_probe_disconnect(void)
{
#ifdef SIMPLECLOCK_ACTUATOR_TEST
    actuator_probe_disconnect();
#endif
#ifdef SIMPLECLOCK_AUTO_HYBRID
    hybrid_probe_disconnect();
#endif
    /* Closing USB is not a heat command. Cancel unexecuted requests so an old
     * partial/session command cannot begin heating after a later reconnect. */
    command_pending = false;
    command_cancels++;
    rx_used = 0;
    rx_bad = false;
}

void thermal_probe_receive(const uint8_t *bytes, uint32_t length)
{
    for (uint32_t i = 0; i < length; i++)
    {
        const uint8_t byte = bytes[i];
        if (byte == '\r' || byte == '\n')
        {
            if (rx_bad)
            {
                thermal_probe_command("invalid", 7);
            }
            else if (rx_used)
            {
                rx_line[rx_used] = '\0';
                const uint32_t profile = gnss_probe_command_profile(rx_line);
#ifdef SIMPLECLOCK_AUTO_HYBRID
                if (hybrid_probe_command(rx_line) || gnss_probe_control(rx_line))
                {
                    /* Runtime control is consumed in foreground, never USB IRQ math. */
                }
                else
#endif
#ifdef SIMPLECLOCK_ACTUATOR_TEST
                    if (actuator_probe_command(rx_line))
                {
                    /* The actuator mailbox is serviced only in foreground. */
                }
                else
#endif
                    if (profile != GNSS_PROFILE_NONE)
                {
                    gnss_probe_request(profile);
                }
                else
                {
                    thermal_probe_command(rx_line, rx_used);
                }
            }
            rx_used = 0;
            rx_bad = false;
        }
        else if (byte < 32U || byte > 126U || rx_used + 1U >= sizeof(rx_line))
        {
            rx_bad = true;
        }
        else if (!rx_bad)
        {
            rx_line[rx_used++] = (char)byte;
        }
    }
}

void thermal_probe_ocxo(const sit5801_probe_t *probe, uint32_t now_ms)
{
    const unsigned order = sit5801_probe_word_order(probe);
    const bool valid = order != 0U && sit5801_probe_word(probe, 2, order) == 0U;
    ocxo_valid = valid;
    ocxo_ms = now_ms;
#ifndef SIMPLECLOCK_AUTO_HYBRID
    const uint32_t power = sit5801_power_code(sit5801_probe_word(probe, 6, order),
                                              sit5801_probe_word(probe, 7, order));
    /* Current driver reads power halves separately. Report the plateau, but do
     * not claim qualified preheat from unverified coherence/meaningful range. */
    sc_preheat_push(&preheat, now_ms, power, valid, sc_temperature_fresh(&temperature, now_ms),
                    false);
#endif
}

static void append_u32_field(sc_text_t *text, const char *key, uint32_t value)
{
    sc_text_string(text, key);
    sc_text_u32(text, value);
}

static const char *result_name(sc_heater_result_t result)
{
    switch (result)
    {
    case SC_HEATER_ACCEPTED:
        return "OK";
    case SC_HEATER_LOCKED:
        return "DISARMED";
    case SC_HEATER_NOT_READY:
        return "NOT_READY";
    case SC_HEATER_FAULTED:
        return "FAULT_LATCHED";
    case SC_HEATER_RANGE:
        return "RANGE";
    case SC_HEATER_BUSY:
        return "BUSY";
    case SC_HEATER_EXPIRED:
        return "EXPIRED";
    default:
        return "BAD_COMMAND";
    }
}

static bool pi_permitted(uint32_t now_ms)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    /* New boot policy heats as soon as sensor and oscillator readback are
     * usable. Completion of boost is a separate clock-readiness condition. */
    return ocxo_valid && (uint32_t)(now_ms - ocxo_ms) < OCXO_MAX_AGE_MS;
#else
    return pi_warmup_complete && ocxo_valid && (uint32_t)(now_ms - ocxo_ms) < OCXO_MAX_AGE_MS;
#endif
}

static const char *auto_state_name(auto_state_t state)
{
    switch (state)
    {
    case AUTO_WAITING:
        return "WAITING";
    case AUTO_ACTIVE:
        return "ACTIVE";
    case AUTO_BOOST:
        return "BOOST";
    case AUTO_CANCELED:
        return "CANCELED";
    case AUTO_TARGET_RANGE:
        return "TARGET_RANGE";
    default:
        return "FAULT";
    }
}

#ifndef SIMPLECLOCK_AUTO_HYBRID
static void automatic_start(void)
{
    /* Serialize the final eligibility check and both O(1) starts with IRQ OFF.
     * No transport, bus access, filtering or regression runs under this mask. */
    const uint32_t mask = heater_lock();
    const uint32_t now_ms = HAL_GetTick();
    if (auto_state == AUTO_WAITING && !command_pending && !heater.faults && !heater.manual &&
        !heater.controlled && pi_permitted(now_ms) && control.filter_valid &&
        (uint32_t)(now_ms - control.update_ms) < SC_TEMPERATURE_STALE_MS &&
        sc_temperature_fresh(&temperature, now_ms))
    {
        /* Capture once, before any external heating. The baseline may still
         * drift; it is a warm-up endpoint, not a claim of thermal equilibrium. */
        auto_baseline_q16 = control.filtered_q16;
#if defined(SIMPLECLOCK_ACTUATOR_TEST)
        /* M6 actuator-validation image keeps its recorded 39.399 C target. */
        const int32_t target_q16 = (int32_t)(INT64_C(39399) * SC_THERMAL_Q16 / 1000);
        auto_requested_target_q16 = target_q16;
        auto_target_clamped = false;
#else
        const int32_t target_q16 = auto_baseline_q16 + AUTO_RISE_Q16;
        auto_requested_target_q16 = target_q16;
        auto_target_clamped = false;
#endif
        if (target_q16 > PROFILE_TARGET_MAX_Q16)
        {
            auto_state = AUTO_TARGET_RANGE; /* Never clamp or retry on cooling. */
        }
        else if (sc_heater_control_start_continuous(&heater, now_ms) == SC_HEATER_ACCEPTED)
        {
            if (sc_thermal_control_start(&control, target_q16, 0, now_ms))
            {
                auto_start_ms = now_ms;
                auto_state = AUTO_ACTIVE;
            }
            else
            {
                sc_heater_fault(&heater, SC_HEATER_FAULT_CONTROL);
                auto_state = AUTO_FAULT;
            }
        }
    }
    heater_unlock(mask);
}
#else
static void automatic_start(void)
{
    const uint32_t mask = heater_lock();
    const uint32_t now_ms = HAL_GetTick();
    if (auto_state == AUTO_WAITING && !command_pending && !heater.faults && !heater.manual &&
        !heater.controlled && pi_permitted(now_ms) && control.filter_valid &&
        sc_temperature_fresh(&temperature, now_ms))
    {
        auto_baseline_q16 = (int32_t)temperature.raw * 512;
        if (sc_heater_control_start_continuous(&heater, now_ms) == SC_HEATER_ACCEPTED)
        {
            /* Power is latched at the NEXT PWM boundary, so the reference's
             * reference delay starts there, not when the request is queued. */
            sc_thermal_boot_start(&boot, heater.frame_ms + SC_HEATER_FRAME_MS);
            auto_start_ms = now_ms;
            auto_state = AUTO_BOOST;
            if (auto_baseline_q16 < AUTO_HYBRID_TARGET_MAX_Q16)
            {
                (void)sc_heater_control_update(&heater, SC_HYBRID_THERMAL_BOOT_BOOST_DUTY_PERMILLE,
                                               now_ms);
            }
        }
    }
    heater_unlock(mask);
}

static void automatic_boost(bool updated)
{
    if (auto_state != AUTO_BOOST || !updated)
    {
        return;
    }
    const int32_t raw_q16 = (int32_t)temperature.raw * 512;
    /* The fit is foreground work, outside IRQ exclusion. Recheck ownership
     * afterwards: an OFF received during the fit must win over any result. */
    const bool ready = sc_thermal_boot_sample(&boot, temperature.sample_ms, raw_q16);
    const uint32_t mask = heater_lock();
    const uint32_t now_ms = HAL_GetTick();
    if (auto_state == AUTO_BOOST && heater.controlled && !heater.faults)
    {
        if (boot.phase == SC_THERMAL_BOOT_BAD_DATA)
        {
            sc_heater_fault(&heater, SC_HEATER_FAULT_CONTROL_UPDATE);
            auto_state = AUTO_FAULT;
        }
        else if (ready)
        {
            auto_requested_target_q16 = raw_q16;
            auto_target_clamped = boot.target_q16 != raw_q16;
            /* End the boost with zero heat, rather than inheriting a 100%
             * integral at the new setpoint. The normal PID then rebuilds
             * holding duty. The ceiling path also forces the pin OFF now. */
            if (boot.phase == SC_THERMAL_BOOT_CEILING)
            {
                sc_heater_off(&heater);
                (void)sc_heater_control_start_continuous(&heater, now_ms);
                GPIOB->BRR = GPIO_PIN_3;
            }
            if (sc_thermal_control_start(&control, boot.target_q16,
                                         SC_HYBRID_THERMAL_BOOT_HANDOFF_DUTY_PERMILLE, now_ms) &&
                sc_heater_control_update(&heater, control.duty_permille, now_ms) ==
                    SC_HEATER_ACCEPTED)
            {
                auto_state = AUTO_ACTIVE;
                pi_warmup_complete = true; /* Startup policy complete, not thermal proof. */
            }
            else
            {
                sc_heater_fault(&heater, SC_HEATER_FAULT_CONTROL);
                auto_state = AUTO_FAULT;
            }
        }
        else if (sc_heater_control_update(&heater, SC_HYBRID_THERMAL_BOOT_BOOST_DUTY_PERMILLE,
                                          now_ms) != SC_HEATER_ACCEPTED)
        {
            auto_state = AUTO_FAULT;
        }
    }
    heater_unlock(mask);
}
#endif

void thermal_probe_led_status(uint32_t now_ms, bool *available, bool *stable, bool *fault)
{
    /* The actuator can stop in SysTick/USB IRQ; take only its small snapshot
     * under the mask. Controller and indicator data remain foreground-owned. */
    const uint32_t mask = heater_lock();
    *fault = heater.faults != 0U;
    const bool active = heater.controlled;
    heater_unlock(mask);
    *available = initialized && !*fault && pi_permitted(now_ms) &&
                 sc_temperature_fresh(&temperature, now_ms);
    sc_thermal_indicator_update(&indicator, &control, *available && active, now_ms);
    *stable = indicator.stable;
}

#ifdef SIMPLECLOCK_AUTO_HYBRID
void thermal_probe_snapshot(uint32_t now_ms, thermal_probe_snapshot_t *out)
{
    const uint32_t mask = heater_lock();
    out->faults = heater.faults;
    out->running = heater.controlled && control.running;
    out->completed_duty_permille = completed_duty_permille;
    out->completed_frame_valid = completed_frame_valid;
    out->frame_sequence = duty_sequence;
    out->frame_ms = duty_frame_ms;
    heater_unlock(mask);
    out->sample_ms = temperature.sample_ms;
    out->raw = temperature.raw;
    out->filtered_q16 = control.filtered_q16;
    out->target_q16 = control.target_q16;
    out->integral_micro = control.integral_micro;
    out->derivative_micro = control.derivative_micro;
    out->proportional_micro = (int32_t)((int64_t)control.gains.kp_micro *
                                        (control.ramp_q16 - control.filtered_q16) / SC_THERMAL_Q16);
    out->warmed = pi_warmup_complete;
    out->regulation_stable = indicator.stable; /* Updated by the LED3 path. */
    out->sensor_valid = initialized && sc_temperature_fresh(&temperature, now_ms);
    out->safe = out->sensor_valid && !out->faults && out->running && control.filter_valid &&
                pi_permitted(now_ms);
}

bool thermal_probe_set_target(int32_t target_q16)
{
    return sc_thermal_control_target(&control, target_q16);
}
#endif

#ifdef SIMPLECLOCK_ACTUATOR_TEST
bool thermal_probe_trial_ready(uint32_t now_ms, bool require_stable)
{
    bool available, stable, fault;
    thermal_probe_led_status(now_ms, &available, &stable, &fault);
    return available && !fault && control.running && (!require_stable || stable) &&
           control.target_q16 == (int32_t)(INT64_C(39399) * SC_THERMAL_Q16 / 1000) &&
           control.gains.kp_micro == SC_THERMAL_DEFAULT_KP_MICRO &&
           control.gains.ki_micro == SC_THERMAL_DEFAULT_KI_MICRO;
}
#endif

static void process_pending_command(uint32_t now_ms)
{
    if (reply_length || !command_pending)
    {
        return;
    }
    char line[sizeof(command_line)];
    const uint32_t mask = heater_lock();
    if (!command_pending)
    {
        heater_unlock(mask);
        return;
    }
    const uint32_t length = command_length, cancels = command_cancels;
    const uint32_t received_ms = command_received_ms;
    memcpy(line, command_line, length);
    command_pending = false;
    heater_unlock(mask);
    sc_thermal_command_t command;
    sc_heater_result_t result = SC_HEATER_BAD_COMMAND;
    if (sc_thermal_parse(line, length, &command))
    {
        const uint32_t command_mask = heater_lock();
        now_ms = HAL_GetTick();
        if (cancels != command_cancels)
        {
            result = SC_HEATER_BUSY;
        }
        else
        {
            switch (command.action)
            {
            case SC_THERMAL_OFF:
                auto_state = AUTO_CANCELED;
                sc_heater_off(&heater);
                sc_thermal_control_stop(&control);
                result = SC_HEATER_ACCEPTED;
                break;
            case SC_THERMAL_STATUS:
                result = SC_HEATER_ACCEPTED;
                break;
            case SC_THERMAL_CLEAR:
                result = sc_heater_clear(&heater, now_ms);
                break;
            case SC_THERMAL_MANUAL:
                result = (uint32_t)(now_ms - received_ms) > COMMAND_MAX_AGE_MS
                             ? SC_HEATER_EXPIRED
                             : sc_heater_manual(&heater, command.duty_permille, command.duration_s,
                                                now_ms);
                break;
            case SC_THERMAL_PI:
                if ((uint32_t)(now_ms - received_ms) > COMMAND_MAX_AGE_MS)
                {
                    result = SC_HEATER_EXPIRED;
                }
                else if (!pi_permitted(now_ms) || !control.filter_valid ||
                         !sc_temperature_fresh(&temperature, now_ms))
                {
                    result = SC_HEATER_NOT_READY;
                }
                else
                {
                    const int32_t target_q16 =
                        (int32_t)((int64_t)command.target_milli_c * SC_THERMAL_Q16 / 1000);
                    if (target_q16 > PROFILE_TARGET_MAX_Q16)
                    {
                        result = SC_HEATER_RANGE;
                        break;
                    }
                    /* Only O(1) state setup here. Filtering/regression and PI
                     * arithmetic run outside the IRQ mask in service(). */
                    result = command.duration_s
                                 ? sc_heater_control_start(&heater, command.duration_s, now_ms)
                                 : sc_heater_control_start_continuous(&heater, now_ms);
                    if (result == SC_HEATER_ACCEPTED &&
                        !sc_thermal_control_start(&control, target_q16, heater.active_permille,
                                                  now_ms))
                    {
                        sc_heater_off(&heater);
                        result = SC_HEATER_RANGE;
                    }
#ifdef SIMPLECLOCK_AUTO_HYBRID
                    if (result == SC_HEATER_ACCEPTED)
                    {
                        pi_warmup_complete = true; /* Explicit operator target replaces boost. */
                    }
#endif
                }
                break;
            case SC_THERMAL_CONTINUE:
                result = (uint32_t)(now_ms - received_ms) > COMMAND_MAX_AGE_MS
                             ? SC_HEATER_EXPIRED
                             : sc_heater_control_continue(&heater, now_ms);
                break;
            }
            /* An accepted trial owns the session, including its eventual OFF
             * state. Automatic startup must not follow a manual trial expiry. */
            if (result == SC_HEATER_ACCEPTED &&
                (command.action == SC_THERMAL_MANUAL || command.action == SC_THERMAL_PI))
            {
                auto_state = AUTO_CANCELED;
            }
        }
        if (!heater.output_on)
        {
            GPIOB->BRR = GPIO_PIN_3;
        }
        heater_unlock(command_mask);
    }
    sc_text_t text;
    sc_text_init(&text, reply, sizeof(reply));
    sc_text_string(&text, "type=thermal_command v=1");
    append_u32_field(&text, " uptime_ms=", now_ms);
    append_u32_field(&text, " seq=", command_sequence++);
    sc_text_string(&text, " result=");
    sc_text_string(&text, result_name(result));
    sc_text_string(&text, " command_hex=");
    for (uint32_t i = 0; i < length; i++)
    {
        sc_text_hex(&text, (uint8_t)line[i], 2);
    }
    sc_text_string(&text, "\r\n");
    if (text.ok)
    {
        reply_length = (uint16_t)text.length;
    }
    else
    {
        tx_drops++;
    }
}

static void report_thermal(uint32_t now_ms)
{
    sc_heater_t snapshot;
    uint32_t dropped, canceled;
    const uint32_t mask = heater_lock();
    snapshot = heater;
    const auto_state_t startup = auto_state;
    dropped = command_drops;
    canceled = command_cancels;
    const uint32_t commanded_ticks = drive_ticks, observed_high_ticks = pin_high_ticks;
    const uint32_t pin_mode = (GPIOB->MODER >> 6U) & 3U;
    const uint32_t pin_latch = (GPIOB->ODR >> 3U) & 1U;
    const uint32_t pin_input = (GPIOB->IDR >> 3U) & 1U;
    const uint32_t pin_binding = HAL_SYSCFG_GetPinBinding(HAL_BIND_TSSOP20_PIN20);
    heater_unlock(mask);
    char line[SIMPLECLOCK_TMP117_USB_TX_MAX];
    sc_text_t text;
    sc_text_init(&text, line, sizeof(line));
    sc_text_string(&text, "type=thermal v=1 mode=");
    sc_text_string(&text,
                   snapshot.faults
                       ? "FAULT"
                       : (snapshot.controlled ? "PI" : (snapshot.manual ? "MANUAL" : "OFF")));
    append_u32_field(&text, " uptime_ms=", now_ms);
    append_u32_field(&text, " sample=", temperature.sequence);
    append_u32_field(&text, " sample_ms=", temperature.sample_ms);
    append_u32_field(&text, " sample_age_ms=",
                     temperature.sequence ? now_ms - temperature.sample_ms : UINT32_MAX);
    append_u32_field(&text, " fresh=", sc_temperature_fresh(&temperature, now_ms) ? 1U : 0U);
    append_u32_field(&text, " sensor_status=", (uint32_t)temperature.status);
    sc_text_string(&text, " temp_c=");
    if (temperature.valid)
    {
        sc_text_temperature(&text, temperature.raw);
    }
    else
    {
        sc_text_string(&text, "NA");
    }
    append_u32_field(&text, " sensor_errors=", temperature.errors);
    append_u32_field(&text, " config=", temperature.configuration);
    append_u32_field(&text, " device_id=", temperature.device_id);
    append_u32_field(&text, " faults=", snapshot.faults);
    append_u32_field(&text, " duty_permille=", snapshot.requested_permille);
    append_u32_field(&text, " active_permille=", snapshot.active_permille);
    append_u32_field(&text, " continuous=", snapshot.continuous);
    sc_text_string(&text, " auto_state=");
    sc_text_string(&text, auto_state_name(startup));
    append_u32_field(&text, " stops=", snapshot.stops);
    append_u32_field(&text, " preheat_elapsed_ms=", now_ms - preheat.start_ms);
#ifdef SIMPLECLOCK_AUTO_HYBRID
    sc_text_string(&text, " preheat_samples=0 power_p05=NA power_median=NA power_p95=NA"
                          " power_plateau=0 preheat_unconfirmed=1 power_statistics_available=0");
#else
    append_u32_field(&text, " preheat_samples=", preheat.count);
    append_u32_field(&text, " power_p05=", preheat.p05);
    append_u32_field(&text, " power_median=", preheat.median);
    append_u32_field(&text, " power_p95=", preheat.p95);
    append_u32_field(&text, " power_plateau=", preheat.plateau ? 1U : 0U);
    append_u32_field(&text, " preheat_unconfirmed=", preheat.unconfirmed ? 1U : 0U);
#endif
    append_u32_field(&text, " command_drops=", dropped);
    append_u32_field(&text, " command_cancels=", canceled);
    append_u32_field(&text, " tx_drops=", tx_drops);
    append_u32_field(&text, " pin_on=", snapshot.output_on ? 1U : 0U);
    append_u32_field(&text, " drive_ticks=", commanded_ticks);
    append_u32_field(&text, " pin_high_ticks=", observed_high_ticks);
    append_u32_field(&text, " pb3_mode=", pin_mode);
    append_u32_field(&text, " pb3_odr=", pin_latch);
    append_u32_field(&text, " pb3_idr=", pin_input);
    append_u32_field(&text, " pinmux2=", (pin_binding >> 4U) & 3U);
    append_u32_field(&text, " manual_permitted=", snapshot.preheat_permitted ? 1U : 0U);
    sc_text_string(&text,
                   " preheat_ready=0 power_evidence_verified=0 commissioned=0 manual_authorized=1"
#ifdef SIMPLECLOCK_AUTO_HYBRID
                   " preheat_policy=FULL_POWER_SLOPE_CAPTURE trip_c=");
#else
                   " preheat_policy=OPERATOR_MANUAL_AFTER_120S trip_c=");
#endif
    sc_text_q16(&text, (int32_t)((int64_t)TRIP_TEMPERATURE_RAW * SC_THERMAL_Q16 / 128));
    sc_text_string(&text, " max_duty_permille=1000 frame_ms=1000 lease_ms=250\r\n");
    if (!text.ok || !simpleclock_usb_cdc_try_send((const uint8_t *)line, (uint16_t)text.length))
    {
        tx_drops++;
    }
}

static void report_pi_control(uint32_t now_ms)
{
    /* Commands/faults may stop PI earlier in this same service call. Refresh
     * the display before publishing rather than report last loop's LED state. */
    bool available, stable, fault;
    thermal_probe_led_status(now_ms, &available, &stable, &fault);
    char line[SIMPLECLOCK_TMP117_USB_TX_MAX];
    sc_text_t text;
    sc_text_init(&text, line, sizeof(line));
    sc_text_string(&text, "type=thermal_pi v=1");
    sc_text_string(&text, " auto_baseline_c=");
    sc_text_q16(&text, auto_baseline_q16);
#ifdef SIMPLECLOCK_AUTO_HYBRID
    sc_text_string(&text, " auto_requested_c=");
    sc_text_q16(&text, auto_requested_target_q16);
    append_u32_field(&text, " auto_target_clamped=", auto_target_clamped ? 1U : 0U);
    sc_text_string(&text, " auto_target_min_c=");
    sc_text_q16(&text, AUTO_HYBRID_TARGET_MIN_Q16);
    sc_text_string(&text, " auto_target_max_c=");
    sc_text_q16(&text, AUTO_HYBRID_TARGET_MAX_Q16);
#endif
    append_u32_field(&text, " auto_start_ms=", auto_start_ms);
    append_u32_field(&text, " uptime_ms=", now_ms);
    append_u32_field(&text, " running=", control.running);
    append_u32_field(&text, " filter_valid=", control.filter_valid);
    sc_text_string(&text, " filtered_c=");
    sc_text_q16(&text, control.filtered_q16);
    sc_text_string(&text, " ramp_c=");
    sc_text_q16(&text, control.ramp_q16);
    sc_text_string(&text, " target_c=");
    sc_text_q16(&text, control.target_q16);
    sc_text_string(&text, " integral_micro=");
    sc_text_i32(&text, control.integral_micro);
    sc_text_string(&text, " proportional_micro=");
    sc_text_i32(&text, (int32_t)((int64_t)control.gains.kp_micro *
                                 (control.ramp_q16 - control.filtered_q16) / SC_THERMAL_Q16));
    append_u32_field(&text, " requested_permille=", control.duty_permille);
    append_u32_field(&text, " mean_permille=", control.mean_permille);
    append_u32_field(&text, " saturated=", control.saturated);
    append_u32_field(&text, " window_saturated=", control.window_saturated);
    append_u32_field(&text, " slope_valid=", control.slope_valid);
    sc_text_string(&text, " slope_c_per_min=");
    sc_text_q16(&text, control.slope_q16_per_min);
    append_u32_field(&text, " stable=", control.stable);
    append_u32_field(&text, " led_stable=", stable);
    append_u32_field(
        &text, " led_dwell_ms=", indicator.timing ? indicator.sample_ms - indicator.since_ms : 0U);
    append_u32_field(
        &text, " stable_elapsed_ms=", control.entry_timing ? now_ms - control.stable_since_ms : 0U);
    append_u32_field(&text, " updates=", control.updates);
    append_u32_field(&text, " pi_permitted=", pi_permitted(now_ms));
    append_u32_field(&text, " tx_drops=", pi_tx_drops);
    append_u32_field(&text, " kp_micro=", (uint32_t)control.gains.kp_micro);
    append_u32_field(&text, " ki_micro=", (uint32_t)control.gains.ki_micro);
#ifdef SIMPLECLOCK_AUTO_HYBRID
    sc_text_string(&text, " controller=PID");
    append_u32_field(&text, " kd_micro=", (uint32_t)control.derivative.kd_micro);
    append_u32_field(&text, " d_filter_ms=", control.derivative.filter_ms);
    sc_text_string(&text, " derivative_micro=");
    sc_text_i32(&text, control.derivative_micro);
    sc_text_string(&text, " d_rate_q16_per_s=");
    sc_text_i32(&text, control.derivative_rate_q16_per_s);
#endif
    append_u32_field(&text, " max_permille=", control.gains.max_permille);
    sc_text_string(&text, " led_band_c=0.250 led_slope_c_per_min=0.100"
#ifdef SIMPLECLOCK_AUTO_HYBRID
                          " auto_start_policy=FULL_POWER_SLOPE_CAPTURE qualified_clock=0\r\n");
#elif defined(SIMPLECLOCK_ACTUATOR_TEST)
                          " preheat_policy=FIXED_39.399_AFTER_300S qualified_clock=0\r\n");
#else
                          " preheat_policy=OPERATOR_PI_AFTER_300S auto_rise_c=2.000 "
                          "qualified_clock=0\r\n");
#endif
    if (!text.ok || !simpleclock_usb_cdc_try_send((const uint8_t *)line, (uint16_t)text.length))
    {
        pi_tx_drops++;
    }
}

#ifdef SIMPLECLOCK_AUTO_HYBRID
static void report_boot(uint32_t now_ms)
{
    static const char *const names[] = {"DELAY",   "REFERENCE", "DECAY",   "CAPTURED",
                                        "NO_RISE", "CEILING",   "BAD_DATA"};
    char line[384];
    sc_text_t text;
    sc_text_init(&text, line, sizeof(line));
    sc_text_string(&text, "type=thermal_boot v=1 phase=");
    sc_text_string(&text, names[boot.phase]);
    append_u32_field(&text, " uptime_ms=", now_ms);
    append_u32_field(&text, " first_power_ms=", boot.start_ms);
    append_u32_field(&text, " reference_valid=", boot.reference_valid);
    append_u32_field(&text, " samples=", boot.count);
    append_u32_field(&text, " below_count=", boot.below_count);
    append_u32_field(&text, " below_ms=", boot.below_elapsed_ms);
    append_u32_field(&text, " confirm_ms=", SC_HYBRID_THERMAL_BOOT_CAPTURE_CONFIRM_S * 1000U);
    sc_text_string(&text, " reference_q16_per_s=");
    sc_text_i32(&text, boot.reference_q16_per_s);
    sc_text_string(&text, " slope_q16_per_s=");
    sc_text_i32(&text, boot.slope_q16_per_s);
    sc_text_string(&text, " criterion=FIXED_SLOPE threshold_q16_per_s=");
    sc_text_i32(&text, SC_THERMAL_BOOT_CAPTURE_SLOPE_Q16_PER_S);
    sc_text_string(&text, " selected_target_c=");
    sc_text_q16(&text, boot.target_q16);
    sc_text_string(&text, "\r\n");
    if (!text.ok || !simpleclock_usb_cdc_try_send((const uint8_t *)line, (uint16_t)text.length))
    {
        tx_drops++;
    }
}
#endif

void thermal_probe_service(uint32_t now_ms)
{
    if (!initialized)
    {
        return;
    }
    sc_temperature_transfer_t transfer;
    if (sc_temperature_begin(&temperature, now_ms, &transfer))
    {
        uint8_t bytes[2] = {(uint8_t)(transfer.value >> 8), (uint8_t)transfer.value};
        /* One transaction per service, 5 ms HAL wait budget. Never three sensor
         * transfers back-to-back; GNSS is drained before the next bus operation. */
        /* HAL otherwise waits 25 ms for BUSY independently of its Timeout. */
        HAL_StatusTypeDef result = HAL_ERROR;
        if (!(I2C2->ISR & I2C_ISR_BUSY))
        {
            result = transfer.write
                         ? HAL_I2C_Mem_Write(&hi2c2, SIMPLECLOCK_TMP117_I2C_ADDRESS << 1U,
                                             transfer.reg, I2C_MEMADD_SIZE_8BIT, bytes, 2U, 5U)
                         : HAL_I2C_Mem_Read(&hi2c2, SIMPLECLOCK_TMP117_I2C_ADDRESS << 1U,
                                            transfer.reg, I2C_MEMADD_SIZE_8BIT, bytes, 2U, 5U);
        }
        if (I2C2->ISR & I2C_ISR_BERR)
        {
            I2C2->ICR = I2C_ICR_BERRCF;
        }
        sc_temperature_complete(&temperature, HAL_GetTick(), result == HAL_OK,
                                simpleclock_tmp117_decode_u16_be(bytes[0], bytes[1]));
    }
#ifndef SIMPLECLOCK_AUTO_HYBRID
    now_ms = HAL_GetTick();
    sc_preheat_expire(&preheat, now_ms, sc_temperature_fresh(&temperature, now_ms));
#endif
    const uint32_t mask = heater_lock();
    now_ms = HAL_GetTick();
    if (HAL_SYSCFG_GetPinBinding(HAL_BIND_TSSOP20_PIN20) != HAL_BIND_TSSOP20_PIN20_PB3)
    {
        sc_heater_fault(&heater, SC_HEATER_FAULT_CONTROL);
    }
    /* Latch completed delays so a continuously running unit does not lose
     * permission when the 32-bit millisecond clock wraps after 49.7 days. */
#ifdef SIMPLECLOCK_AUTO_HYBRID
    manual_warmup_complete = true;
#else
    manual_warmup_complete |= (uint32_t)(now_ms - preheat.start_ms) >= MANUAL_WARMUP_MS;
    pi_warmup_complete |= (uint32_t)(now_ms - preheat.start_ms) >= PI_WARMUP_MS;
#endif
    const bool manual_permitted =
        manual_warmup_complete && ocxo_valid && (uint32_t)(now_ms - ocxo_ms) < OCXO_MAX_AGE_MS;
    sc_heater_refresh(&heater, &temperature,
                      heater.controlled ? pi_permitted(now_ms) : manual_permitted, now_ms);
    const bool controlling = heater.controlled;
    if (heater.faults &&
        (auto_state == AUTO_WAITING || auto_state == AUTO_ACTIVE || auto_state == AUTO_BOOST))
    {
        auto_state = AUTO_FAULT;
    }
    if (!heater.output_on)
    {
        GPIOB->BRR = GPIO_PIN_3;
    }
    heater_unlock(mask);
    if (!controlling && control.running)
    {
        sc_thermal_control_stop(&control);
    }
    /* Foreground-only math. An interrupt OFF/fault/expiry can happen during
     * this calculation; the actuator rejects the result once controlled=false. */
    const bool updated = sc_thermal_control_update(&control, &temperature, now_ms);
#ifdef SIMPLECLOCK_AUTO_HYBRID
    automatic_boost(updated);
#endif
    if (updated && control.running)
    {
        const uint32_t output_mask = heater_lock();
        if (sc_heater_control_update(&heater, control.duty_permille, HAL_GetTick()) !=
            SC_HEATER_ACCEPTED)
        {
            sc_thermal_control_stop(&control);
        }
        if (!heater.output_on)
        {
            GPIOB->BRR = GPIO_PIN_3;
        }
        heater_unlock(output_mask);
    }
    process_pending_command(now_ms);
    automatic_start();
    if (reply_length && simpleclock_usb_cdc_try_send_queued((const uint8_t *)reply, reply_length))
    {
        reply_length = 0;
    }
    /* Keep thermal reports away from the existing resource/epoch windows. */
    if (!reply_length && (uint32_t)(now_ms - last_report_ms) >= 1000U && now_ms % 1000U >= 850U)
    {
        last_report_ms = now_ms;
        report_thermal(now_ms);
    }
    if (!reply_length && (uint32_t)(now_ms - last_pi_report_ms) >= 1000U && now_ms % 1000U >= 950U)
    {
        last_pi_report_ms = now_ms;
        report_pi_control(now_ms);
    }
#ifdef SIMPLECLOCK_AUTO_HYBRID
    if (!reply_length && (uint32_t)(now_ms - last_boot_report_ms) >= 5000U &&
        now_ms % 1000U >= 980U)
    {
        last_boot_report_ms = now_ms;
        report_boot(now_ms);
    }
#endif
}
