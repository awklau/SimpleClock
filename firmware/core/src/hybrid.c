/* SPDX-License-Identifier: MIT */
#include "simpleclock/hybrid.h"
#include "simpleclock/sbf_fields.h"
#include "simpleclock/sit5801.h"
#include <math.h>
#include <string.h>

#define CFG(name) SC_HYBRID_CLOCK_##name
#define QUAL(name) SC_HYBRID_QUALIFICATION_##name
#define DATA_TIMEOUT_MS ((uint32_t)(QUAL(RECEIVER_UPDATE_TIMEOUT_S) * 1000.0))
#define QUALIFY_DWELL_MS ((uint32_t)(SC_HYBRID_BOOT_QUALIFICATION_DWELL_S * 1000.0))
#define LOCK_DWELL_MS ((uint32_t)(CFG(LOCK_GOOD_DWELL_S) * 1000.0))
#define PHASE_DWELL_MS ((uint32_t)(CFG(PHASE_QUALIFICATION_S) * 1000.0))
#define GAP_BRIDGE_MS ((uint32_t)(CFG(SHORT_GAP_BRIDGE_S) * 1000.0))
#define SETTLE_MS ((uint32_t)(CFG(PPP_RETURN_SETTLE_S) * 1000.0))
/* Arrival slack beyond the receiver interval, as for one contiguous epoch. */
#define ARRIVAL_SLACK_MS (DATA_TIMEOUT_MS - 1000U)

static double clamp(double x, double low, double high)
{
    return x < low ? low : (x > high ? high : x);
}

static void state(sc_hybrid_t *s, sc_hybrid_state_t next)
{
    if (s->state != next)
    {
        s->state = next;
        s->transitions++;
    }
}

static void clear_window(sc_hybrid_t *s)
{
    sc_hybrid_stats_reset(&s->history);
    s->previous_valid = s->qualifying = s->locking = s->locked = s->unlocking = false;
    s->verify_samples = 0;
    s->residual_ns = 0;
    s->phase_good_samples = 0U;
}

/* Cold path (lock loss/clip start) with two callers: keep one out-of-line copy
 * so -O3 does not duplicate the snapshot into the 1 Hz paths. */
#if defined(__GNUC__)
__attribute__((noinline))
#endif
static void
failure(sc_hybrid_t *s, const sc_hybrid_input_t *in, uint32_t flags, double previous_drift,
        double trim_delta)
{
    const uint32_t sequence = s->last_failure.sequence + 1U;
    s->last_failure =
        (sc_hybrid_failure_t){.epoch_ms = in->epoch_ms,
                              .bias_ns = in->bias_ns,
                              .drift_ppb = in->drift_ppb,
                              .trim_delta_ppb = trim_delta,
                              .innovation_ppb = in->drift_ppb - previous_drift - trim_delta,
                              .phase_error_ns = s->filtered_error_ns,
                              .bias_slope_ppb = s->metrics.bias_slope_ppb,
                              .sequence = sequence,
                              .now_ms = in->now_ms,
                              .flags = flags,
                              .source_mode = s->source_mode,
                              .time_system = s->time_system,
                              .phase_enabled = s->phase_enabled};
}

static void freeze(sc_hybrid_t *s, uint32_t reason)
{
    s->reason = reason;
    s->request_ready = false;
    /* Preserve the delivered phase setpoint and fractional trim. A stopped
     * controller must not unwind either state merely because data vanished. */
    s->phase_enabled = s->ramping = false;
    if (reason & SC_HYBRID_Q_LIMIT)
    {
        s->accumulator_ppb = s->requested_ppb = s->applied_ppb;
    }
    clear_window(s);
    if (!s->known)
    {
        state(s, SC_HYBRID_ACTUATOR_UNKNOWN);
    }
    else if (!s->enabled)
    {
        state(s, SC_HYBRID_MANUAL_HOLD);
    }
    else if (!s->acquisition_latched)
    {
        state(s, SC_HYBRID_HOLDOVER);
    }
}

bool sc_hybrid_tracking(const sc_hybrid_t *s)
{
    return s->state == SC_HYBRID_TRACK || s->state == SC_HYBRID_PHASE_RECOVERING ||
           s->state == SC_HYBRID_FREQUENCY;
}

static sc_hybrid_profile_t active_profile(const sc_hybrid_t *s)
{
    return s->source_mode == SC_SBF_PVT_MODE_STANDALONE ? SC_HYBRID_PROFILE_STANDALONE
           : s->source_mode == SC_SBF_PVT_MODE_FIXED    ? SC_HYBRID_PROFILE_STATIC
           : s->source_mode == SC_SBF_PVT_MODE_SBAS     ? SC_HYBRID_PROFILE_SBAS
                                                        : SC_HYBRID_PROFILE_PPP;
}

/* Below the primary solution, or settling after a return to PPP, the bias is
 * not trusted (mode jumps of 4-13 ns, PPP re-convergence of tens of ns) while
 * the drift is: frequency loop only, one time constant for every source. */
static bool drift_only(const sc_hybrid_t *s)
{
    return s->fallback || s->settling;
}

static unsigned solution_rank(uint8_t mode)
{
    return mode == SC_SBF_PVT_MODE_PPP ? 3U : mode == SC_SBF_PVT_MODE_SBAS ? 2U : mode ? 1U : 0U;
}

static double pair_frequency_time(const sc_hybrid_t *s)
{
    const sc_hybrid_profile_t profile = active_profile(s);
    return profile == SC_HYBRID_PROFILE_STANDALONE ? s->standalone_frequency_time_s
           : profile == SC_HYBRID_PROFILE_STATIC   ? s->static_frequency_time_s
           : profile == SC_HYBRID_PROFILE_SBAS     ? s->sbas_frequency_time_s
                                                   : s->frequency_time_s;
}

static double pair_phase_time(const sc_hybrid_t *s)
{
    const sc_hybrid_profile_t profile = active_profile(s);
    return profile == SC_HYBRID_PROFILE_STANDALONE ? s->standalone_phase_time_s
           : profile == SC_HYBRID_PROFILE_STATIC   ? s->static_phase_time_s
           : profile == SC_HYBRID_PROFILE_SBAS     ? s->sbas_phase_time_s
                                                   : s->phase_time_s;
}

/* A 0/0 pair means "never steer on this solution type", in a fallback and
 * while settling too: the trim is held for as long as the receiver reports it. */
double sc_hybrid_frequency_time(const sc_hybrid_t *s)
{
    const double frequency_s = pair_frequency_time(s);
    return drift_only(s) && (frequency_s > 0.0 || pair_phase_time(s) > 0.0)
               ? CFG(DRIFT_ONLY_FREQUENCY_TIME_CONSTANT_S)
               : frequency_s;
}

double sc_hybrid_phase_time(const sc_hybrid_t *s)
{
    return drift_only(s) ? 0.0 : pair_phase_time(s);
}

static void preload_phase_pi(sc_hybrid_t *s)
{
    /* PI output = I - (2/Tphi)*e. Preserve that output when changing gains or
     * the local anchor: I = previous_output + (2/Tphi)*e. ns/s equals ppb.
     * Output is total trim in PHASE and the FLL's drift target in BOTH. */
    const double output =
        s->loop_mode == SC_HYBRID_LOOP_PHASE ? s->accumulator_ppb : s->desired_drift_ppb;
    s->phase_integral_ppb = output + 2.0 / sc_hybrid_phase_time(s) * s->filtered_error_ns;
}

void sc_hybrid_init(sc_hybrid_t *s)
{
    memset(s, 0, sizeof(*s));
    s->enabled = SC_HYBRID_BOOT_AUTOSTART_ENABLED != 0;
    s->bias_clip_ps = SC_HYBRID_BIAS_CLIP_DEFAULT_PS;
    s->frequency_time_s = (uint32_t)CFG(FREQUENCY_TIME_CONSTANT_S);
    s->phase_time_s = (uint32_t)CFG(PHASE_NATURAL_TIME_S);
    s->sbas_frequency_time_s = (uint32_t)CFG(FALLBACK_FREQUENCY_TIME_CONSTANT_S);
    s->sbas_phase_time_s = (uint32_t)CFG(FALLBACK_PHASE_NATURAL_TIME_S);
    s->standalone_frequency_time_s = s->sbas_frequency_time_s;
    s->standalone_phase_time_s = s->sbas_phase_time_s;
    s->static_frequency_time_s = (uint32_t)CFG(STATIC_FREQUENCY_TIME_CONSTANT_S);
    s->static_phase_time_s = (uint32_t)CFG(STATIC_PHASE_NATURAL_TIME_S);
    s->loop_mode = SC_HYBRID_LOOP_BOTH;
}

bool sc_hybrid_bias_clip(sc_hybrid_t *s, uint32_t picoseconds)
{
    if (!picoseconds || picoseconds > SC_HYBRID_BIAS_CLIP_MAX_PS || s->pending)
    {
        return false;
    }
    s->bias_clip_ps = picoseconds;
    s->request_ready = false;
    return true;
}

void sc_hybrid_hold(sc_hybrid_t *s)
{
    s->enabled = false;
    freeze(s, SC_HYBRID_Q_COMMAND);
}

void sc_hybrid_resume(sc_hybrid_t *s)
{
    s->enabled = true;
    s->acquisition_latched = false; /* Explicit acknowledgement, never USB-open. */
    s->attempts = 0;
    freeze(s, 0);
}

const char *sc_hybrid_loop_name(sc_hybrid_loop_mode_t mode)
{
    static const char *const names[] = {"BOTH", "FREQUENCY", "PHASE", "OFF"};
    return (unsigned)mode < sizeof(names) / sizeof(names[0]) ? names[mode] : "INVALID";
}

sc_hybrid_loop_mode_t sc_hybrid_pair_mode(double frequency_s, double phase_s)
{
    return frequency_s > 0.0 ? (phase_s > 0.0 ? SC_HYBRID_LOOP_BOTH : SC_HYBRID_LOOP_FREQUENCY)
                             : (phase_s > 0.0 ? SC_HYBRID_LOOP_PHASE : SC_HYBRID_LOOP_OFF);
}

bool sc_hybrid_parameters_valid(double frequency_s, double phase_s)
{
    return isfinite(frequency_s) && isfinite(phase_s) &&
           (frequency_s == 0.0 || frequency_s >= SC_HYBRID_RUNTIME_FREQUENCY_MIN_S) &&
           frequency_s <= SC_HYBRID_RUNTIME_TIME_MAX_S &&
           (phase_s == 0.0 || phase_s >= SC_HYBRID_RUNTIME_PHASE_MIN_S) &&
           phase_s <= SC_HYBRID_RUNTIME_TIME_MAX_S &&
           frequency_s == (double)(uint32_t)frequency_s && phase_s == (double)(uint32_t)phase_s;
}

static void select_loop(sc_hybrid_t *s)
{
    const sc_hybrid_loop_mode_t mode =
        sc_hybrid_pair_mode(sc_hybrid_frequency_time(s), sc_hybrid_phase_time(s));
    if (s->loop_mode == (uint8_t)mode)
    {
        return;
    }
    const bool was_tracking = sc_hybrid_tracking(s) || (s->state == SC_HYBRID_DISABLED && s->warm);
    const bool phase_active = s->phase_enabled;
    if (s->warm && (mode == SC_HYBRID_LOOP_BOTH || mode == SC_HYBRID_LOOP_FREQUENCY) &&
        (s->loop_mode == SC_HYBRID_LOOP_PHASE || s->loop_mode == SC_HYBRID_LOOP_OFF))
    {
        /* Direct phase control may be outside the old FLL tracking span.
         * Enter its +/-1 ppb envelope around verified delivered trim, as in
         * ordinary frequency entry, without changing that trim or history. */
        s->acquired_ppb = s->applied_ppb;
    }
    if (mode == SC_HYBRID_LOOP_BOTH && s->loop_mode == SC_HYBRID_LOOP_PHASE)
    {
        /* PHASE stores total trim in I; BOTH stores a drift target. Carry
         * delivered trim separately and start the FLL at its observed drift. */
        s->desired_drift_ppb = s->previous_valid ? s->previous_drift_ppb : 0.0;
    }
    s->loop_mode = (uint8_t)mode;
    s->request_ready = s->locking = s->locked = false;
    if (mode == SC_HYBRID_LOOP_OFF || mode == SC_HYBRID_LOOP_FREQUENCY)
    {
        s->phase_enabled = false;
        s->phase_integral_ppb = s->desired_drift_ppb = 0.0;
        if (mode == SC_HYBRID_LOOP_OFF)
        {
            s->ramping = false;
        }
    }
    else if (phase_active)
    {
        preload_phase_pi(s);
    }
    if (s->enabled && s->known && was_tracking)
    {
        state(s, mode == SC_HYBRID_LOOP_OFF                         ? SC_HYBRID_DISABLED
                 : mode == SC_HYBRID_LOOP_PHASE || s->phase_enabled ? SC_HYBRID_PHASE_RECOVERING
                                                                    : SC_HYBRID_FREQUENCY);
    }
    else if (mode == SC_HYBRID_LOOP_OFF && s->enabled && s->known)
    {
        state(s, SC_HYBRID_DISABLED); /* Cancels coarse acquisition as well. */
    }
}

bool sc_hybrid_profile_parameters(sc_hybrid_t *s, sc_hybrid_profile_t profile, double frequency_s,
                                  double phase_s)
{
    if ((unsigned)profile > SC_HYBRID_PROFILE_STATIC || s->pending ||
        !sc_hybrid_parameters_valid(frequency_s, phase_s))
    {
        return false;
    }
    const double old_phase_s = sc_hybrid_phase_time(s);
    if (profile == SC_HYBRID_PROFILE_PPP)
    {
        s->frequency_time_s = (uint32_t)frequency_s;
        s->phase_time_s = (uint32_t)phase_s;
    }
    else if (profile == SC_HYBRID_PROFILE_SBAS)
    {
        s->sbas_frequency_time_s = (uint32_t)frequency_s;
        s->sbas_phase_time_s = (uint32_t)phase_s;
    }
    else if (profile == SC_HYBRID_PROFILE_STANDALONE)
    {
        s->standalone_frequency_time_s = (uint32_t)frequency_s;
        s->standalone_phase_time_s = (uint32_t)phase_s;
    }
    else
    {
        s->static_frequency_time_s = (uint32_t)frequency_s;
        s->static_phase_time_s = (uint32_t)phase_s;
    }
    if (active_profile(s) == profile)
    {
        select_loop(s);
        if (s->phase_enabled && old_phase_s != phase_s)
        {
            preload_phase_pi(s);
        }
        s->request_ready = false; /* Never deliver a proposal computed with old gains. */
    }
    return true;
}

static void unknown(sc_hybrid_t *s)
{
    s->known = s->pending = false;
    s->recovery_count = 0;
    freeze(s, SC_HYBRID_Q_ACTUATOR);
}

void sc_hybrid_observed(sc_hybrid_t *s, bool valid, int64_t word, uint16_t clip, uint32_t now_ms)
{
    const double pull = sit5801_fcw_to_ppb(word), bound = sit5801_clip_ppb(clip);
    if (!valid || word < SIT5801_FCW_MIN || word > SIT5801_FCW_MAX || fabs(pull) > bound)
    {
        unknown(s);
        return;
    }
    if (s->boot_known && (pull < s->lower_ppb || pull > s->upper_ppb))
    {
        /* A consistent but out-of-envelope partial word is not permission to
         * widen the boot span or slew from an unsafe reconstructed command. */
        unknown(s);
        s->reason |= SC_HYBRID_Q_LIMIT;
        return;
    }
    if (s->known && ((s->pending && word != s->requested_word) ||
                     (!s->pending && s->enabled && word != s->applied_word)))
    {
        unknown(s);
        return;
    }
    if (!s->known && s->boot_known)
    {
        s->recovery_count =
            s->recovery_count && s->recovery_word == word ? s->recovery_count + 1U : 1U;
        s->recovery_word = word;
        if (s->recovery_count < 2U)
        {
            return;
        }
    }
    const bool recovered = !s->known;
    if (s->known && !s->enabled && !s->pending && word != s->applied_word)
    {
        s->accumulator_ppb = s->requested_ppb = pull;
        s->phase_valid = s->phase_enabled = false;
    }
    s->known = true;
    s->pending = false;
    s->observed_ms = now_ms;
    s->applied_word = word;
    s->applied_ppb = pull;
    if (!s->boot_known)
    {
        s->boot_known = true;
        s->boot_ppb = pull;
        s->lower_ppb = fmax(-bound, pull - CFG(AUTOMATIC_TOTAL_SPAN_ABOUT_BOOT_READBACK_PPB));
        s->upper_ppb = fmin(bound, pull + CFG(AUTOMATIC_TOTAL_SPAN_ABOUT_BOOT_READBACK_PPB));
        s->applied_ms = now_ms;
    }
    if (recovered)
    {
        s->accumulator_ppb = s->requested_ppb = pull;
        freeze(s, 0); /* Consistent probe recovery is a documented requalification event. */
    }
}

void sc_hybrid_written(sc_hybrid_t *s, bool success, uint32_t now_ms)
{
    if (!s->pending || !success)
    {
        unknown(s);
        return;
    }
    s->applied_ms = s->pending_ms = now_ms; /* Last LSW completion, not proposal time. */
    s->writes++;
}

void sc_hybrid_service(sc_hybrid_t *s, uint32_t now_ms)
{
    sc_frequency_monitor_service(&s->frequency, now_ms);
    const bool actuator_lost =
        s->known && ((uint32_t)(now_ms - s->observed_ms) > DATA_TIMEOUT_MS ||
                     (s->pending && (uint32_t)(now_ms - s->pending_ms) > DATA_TIMEOUT_MS));
    /* Tracking keeps authority through a short receiver gap (bridged_gap). */
    const uint32_t gnss_limit_ms =
        sc_hybrid_tracking(s) ? GAP_BRIDGE_MS + ARRIVAL_SLACK_MS : DATA_TIMEOUT_MS;
    const bool gnss_lost =
        s->previous_valid && (uint32_t)(now_ms - s->last_received_ms) > gnss_limit_ms;
    if (s->locked && (actuator_lost || gnss_lost))
    {
        /* One snapshot of the last accepted sample, taken before the freeze
         * clears phase authority, flagged with every timeout ending the lock. */
        const sc_hybrid_input_t last = {.now_ms = now_ms,
                                        .epoch_ms = s->previous_epoch_ms,
                                        .bias_ns = s->previous_bias_ns,
                                        .drift_ppb = s->previous_drift_ppb};
        failure(s, &last,
                (actuator_lost ? SC_HYBRID_LOCK_ACTUATOR : 0U) |
                    (gnss_lost ? SC_HYBRID_LOCK_GNSS : 0U),
                s->previous_drift_ppb, 0.0);
    }
    if (actuator_lost)
    {
        unknown(s);
    }
    if (gnss_lost && s->previous_valid) /* An actuator loss already froze the window. */
    {
        s->gnss_good = false;
        freeze(s, SC_HYBRID_Q_STALE);
    }
}

static bool frequency_window_good(sc_hybrid_t *s, unsigned points)
{
    return sc_hybrid_stats_get(&s->history, points, &s->metrics) &&
           s->metrics.drift_sd_ppb <= CFG(QUALIFIED_DRIFT_STDDEV_LIMIT_PPB);
}

static void acquire(sc_hybrid_t *s, double mean)
{
    if (s->attempts >= CFG(ACQUISITION_MAX_ATTEMPTS))
    {
        s->acquisition_latched = true;
        s->reason = SC_HYBRID_Q_LIMIT;
        state(s, SC_HYBRID_ACQUISITION_FAULT);
        return;
    }
    s->attempts++;
    s->acquire_target_ppb =
        clamp(s->applied_ppb - mean / CFG(ACTUATOR_GAIN_PPB_PER_PPB), s->lower_ppb, s->upper_ppb);
    s->accumulator_ppb = s->applied_ppb;
    s->request_ready = false;
    clear_window(s);
    state(s, SC_HYBRID_ACQUIRE);
}

static void enter_frequency(sc_hybrid_t *s, const sc_hybrid_input_t *in, double mean)
{
    if (!s->warm || s->state == SC_HYBRID_VERIFY)
    {
        /* Coarse acquisition (cold or on a restart) writes total trim directly.
         * Its verified endpoint initializes the fine accumulator; only warm
         * re-entry without acquisition carries a fractional remainder. A stale
         * accumulator would pull the fine loop back toward the pre-ramp trim. */
        s->accumulator_ppb = s->requested_ppb = s->applied_ppb;
    }
    s->acquired_ppb = s->applied_ppb;
    s->desired_drift_ppb = s->loop_mode == SC_HYBRID_LOOP_FREQUENCY ? 0.0 : mean;
    s->warm = true;
    s->ramping = true;
    /* Own timestamp: phase may qualify (a late warm entry) while this ramp
     * still runs, and a lock attempt must not restart FLL authority. */
    s->ramp_ms = in->now_ms;
    s->phase_enabled = false;
    s->qualifying = s->locking = s->locked = false;
    state(s, SC_HYBRID_FREQUENCY);
}

static bool phase_span_ready(const sc_hybrid_t *s, const sc_hybrid_input_t *in)
{
    return s->phase_good_samples >= SC_HYBRID_TRACKING_POINTS &&
           (uint32_t)(in->received_ms - s->phase_good_ms) >= PHASE_DWELL_MS;
}

static void activate_phase(sc_hybrid_t *s, const sc_hybrid_input_t *in)
{
    if (s->phase_valid && s->anchor_time_system == s->time_system)
    {
        /* Every restart of phase control takes a fresh local reference rather
         * than steering the holdover phase error back (that pull reached about
         * 1.1 ppb for 25 min in the model). The step is a logged clock break;
         * the running total keeps the offset history reconstructable. */
        s->reanchor_step_ns = in->bias_ns - s->phase_reference_ns;
        s->reanchor_total_ns += s->reanchor_step_ns;
        s->reanchor_count++;
    }
    s->phase_reference_ns = in->bias_ns;
    s->phase_reference_epoch_ms = in->epoch_ms;
    s->anchor_time_system = s->time_system;
    s->generation++;
    s->phase_valid = true;
    s->phase_error_ns = s->filtered_error_ns = 0.0;
    if (s->loop_mode == SC_HYBRID_LOOP_PHASE)
    {
        s->warm = true;
        s->ramping = false;
    }
    preload_phase_pi(s);
    s->phase_enabled = true;
    s->limited = s->slew_limited = s->phase_limited = false;
    /* Contiguous same-domain samples remain valid for trailing statistics.
     * A new constant phase reference does not change their fitted slope;
     * retain the existing offset origin and bias-minus-integrated-drift residual. */
    s->qualifying = s->locking = s->locked = false;
    s->verify_samples = 0;
    state(s, SC_HYBRID_PHASE_RECOVERING);
}

static bool phase_qualified(sc_hybrid_t *s, const sc_hybrid_input_t *in)
{
    return phase_span_ready(s, in) && frequency_window_good(s, SC_HYBRID_TRACKING_POINTS) &&
           fabs(s->metrics.residual_slope_ppb) <=
               CFG(QUALIFIED_BIAS_DRIFT_DISAGREEMENT_LIMIT_PPB) &&
           s->metrics.residual_detrended_rms_ns <=
               CFG(QUALIFIED_BIAS_MINUS_INTEGRATED_DRIFT_DETRENDED_RMS_LIMIT_NS);
}

static bool enter_phase(sc_hybrid_t *s, const sc_hybrid_input_t *in)
{
    if (s->loop_mode == SC_HYBRID_LOOP_FREQUENCY || !phase_qualified(s, in))
    {
        return false;
    }
    activate_phase(s, in);
    return true; /* No integral or actuator update at this activation boundary. */
}

static void handover_phase(sc_hybrid_t *s, const sc_hybrid_input_t *in, double innovation_ns,
                           bool keep_anchor)
{
    /* Align only the statistics origin across the receiver's reference offset;
     * retain contiguous drift history and the prior residual.
     * The control anchor is recaptured separately. This abandons the old local
     * phase target, so it cannot establish absolute UTC phase continuity.
     * Into, within or out of a fallback, phase is not steered and the anchor
     * stays that of the primary solution: the eventual re-anchor then logs
     * the step across the whole fallback in one time scale. */
    s->phase_base_ns += innovation_ns;
    s->phase_valid = s->phase_valid && keep_anchor;
    s->phase_good_samples = 0U;
    s->locking = s->locked = false;
    if (s->phase_enabled)
    {
        /* e=0, I=last output: no phase correction step. An already inhibited
         * phase loop must finish its recovery instead of gaining permission. */
        activate_phase(s, in);
    }
}

/* One nominal second per accepted epoch. Authority scales the FLL increment
 * during recovery; missing measurements never enlarge the integration step.
 * ns/s is numerically ppb: -(2/T)*phase is the PI's drift correction, and
 * dt/Tf times drift error increments the actuator's total pull in ppb. */
static double track_request(sc_hybrid_t *s, double integral, double drift, double authority,
                            double *desired, double *unbounded, double *raw_v)
{
    *raw_v = s->phase_enabled ? integral - 2.0 / sc_hybrid_phase_time(s) * s->filtered_error_ns
                              : s->desired_drift_ppb;
    *desired = clamp(*raw_v, -CFG(TRACKING_REPORTED_DRIFT_SETPOINT_LIMIT_PPB),
                     CFG(TRACKING_REPORTED_DRIFT_SETPOINT_LIMIT_PPB));
    *unbounded = s->accumulator_ppb + authority / sc_hybrid_frequency_time(s) * (*desired - drift);
    const double low =
        fmax(s->lower_ppb, s->acquired_ppb - CFG(TRACKING_SPAN_ABOUT_ACQUIRED_TRIM_PPB));
    const double high =
        fmin(s->upper_ppb, s->acquired_ppb + CFG(TRACKING_SPAN_ABOUT_ACQUIRED_TRIM_PPB));
    return clamp(*unbounded, low, high);
}

/* Require elapsed time on both clocks: buffered epochs cannot manufacture a
 * dwell, and the unsigned MCU subtraction remains valid across tick wrap. */
static bool dwell_complete(const sc_hybrid_input_t *in, uint32_t since_ms, uint64_t since_epoch_ms,
                           uint32_t duration_ms)
{
    return in->epoch_ms - since_epoch_ms >= duration_ms &&
           (uint32_t)(in->now_ms - since_ms) >= duration_ms;
}

static bool tracking_windows(sc_hybrid_t *s)
{
    return sc_hybrid_stats_get(&s->history, SC_HYBRID_TRACKING_POINTS, &s->metrics) &&
           sc_hybrid_stats_bias_slope(&s->history, SC_HYBRID_LOCK_SLOPE_POINTS,
                                      &s->metrics.bias_slope_ppb);
}

static void phase_status(sc_hybrid_t *s, const sc_hybrid_input_t *in, bool windows)
{
    /* Handover preserves active phase steering, but a lock claim needs a full
     * fresh same-domain span before starting the existing good dwell. */
    s->lock_failures =
        (!windows || !phase_span_ready(s, in) ? SC_HYBRID_LOCK_SPAN : 0U) |
        (fabs(s->filtered_error_ns) > CFG(LOCK_FILTERED_PHASE_ERROR_LIMIT_NS) ? SC_HYBRID_LOCK_PHASE
                                                                              : 0U) |
        (windows && fabs(s->metrics.bias_slope_ppb) > CFG(LOCK_BIAS_SLOPE_LIMIT_PPB)
             ? SC_HYBRID_LOCK_SLOPE
             : 0U) |
        /* Hard limits only. The per-sample slew cap is not a lock failure: at
         * short Tphi it trims measurement noise on nearly every sample, and a
         * cap that really starves the loop fails the phase and slope checks. */
        (s->phase_limited ? SC_HYBRID_LOCK_LIMIT : 0U);
    if (s->lock_failures)
    {
        s->locking = false;
        if (s->locked)
        {
            if (!s->unlocking)
            {
                s->unlocking = true;
                s->unlock_ms = in->now_ms;
                s->unlock_epoch_ms = in->epoch_ms;
            }
            else if (dwell_complete(in, s->unlock_ms, s->unlock_epoch_ms,
                                    (uint32_t)(CFG(LOCK_BAD_DWELL_S) * 1000.0)))
            {
                s->locked = false;
            }
        }
    }
    else if (s->locked)
    {
        s->unlocking = false;
    }
    else if (!s->locking)
    {
        s->unlocking = false;
        s->locking = true;
        s->lock_ms = in->now_ms;
        s->lock_epoch_ms = in->epoch_ms;
    }
    else
    {
        s->locked = dwell_complete(in, s->lock_ms, s->lock_epoch_ms, LOCK_DWELL_MS);
    }
    state(s, s->locked ? SC_HYBRID_TRACK : SC_HYBRID_PHASE_RECOVERING);
}

static void track(sc_hybrid_t *s, const sc_hybrid_input_t *in)
{
    s->integral_limited = false;
    if (s->settling && !s->fallback && (uint32_t)(in->now_ms - s->settle_ms) >= SETTLE_MS &&
        phase_qualified(s, in))
    {
        /* Settled PPP: restore its loop pair; phase restarts below from a
         * fresh, logged anchor (no pull toward the pre-fallback reference). */
        s->settling = false;
        select_loop(s);
    }
    if (!s->phase_enabled && enter_phase(s, in))
    {
        return;
    }
    if (s->loop_mode == SC_HYBRID_LOOP_PHASE && !s->phase_enabled)
    {
        /* No frequency loop or coarse drift acquisition is allowed in this
         * mode. Invalid/unqualified phase holds the exact delivered trim. */
        state(s, SC_HYBRID_PHASE_RECOVERING);
        return;
    }
    double delta_i = 0.0;
    if (s->phase_enabled)
    {
        /* No measurement filter (a fixed 10 s one caused a 15-25 s ADEV/TDEV
         * bump); the per-sample clip already bounds outliers. filtered_error
         * keeps its name for lock checks and telemetry. */
        s->phase_error_ns = s->filtered_error_ns = in->bias_ns - s->phase_reference_ns;
        const double phase_time = sc_hybrid_phase_time(s);
        /* Ki=1/T^2 times phase (ns) and the fixed 1 s sample interval gives
         * ppb. Positive bias error requests negative drift to reduce it. */
        delta_i = -s->filtered_error_ns / (phase_time * phase_time);
    }
    double authority = 1.0;
    if (s->ramping)
    {
        authority = fmin(1.0, (double)(uint32_t)(in->now_ms - s->ramp_ms) /
                                  (1000.0 * CFG(WARM_FREQUENCY_AUTHORITY_RAMP_S)));
        s->ramping = authority < 1.0;
    }
    const double rate = CFG(TRACKING_SLEW_LIMIT_PPB_PER_S) *
                        fmin(1.0, (double)(uint32_t)(in->now_ms - s->applied_ms) / 1000.0);
    if (s->loop_mode == SC_HYBRID_LOOP_PHASE)
    {
        /* Direct PI, u = I - (2/Tphi)*e, Ki = 1/Tphi^2. Unlike BOTH, its
         * output is total oscillator trim, never a drift target fed to an FLL.
         * Bounded to a span about the trim verified at the last acquisition
         * (the boot-relative envelope is wide enough for lifetime ageing).
         * Drift remains available for integrity/diagnostics, not actuation. */
        double trial_i = s->phase_integral_ppb + delta_i;
        const double raw = trial_i - 2.0 / sc_hybrid_phase_time(s) * s->filtered_error_ns;
        const double span = CFG(PHASE_TRACKING_SPAN_ABOUT_ACQUIRED_TRIM_PPB);
        const double bounded = clamp(raw, fmax(s->lower_ppb, s->acquired_ppb - span),
                                     fmin(s->upper_ppb, s->acquired_ppb + span));
        const double limited = clamp(bounded, s->applied_ppb - rate, s->applied_ppb + rate);
        if ((raw - limited) * delta_i > 0.0)
        {
            /* Skip only an I step that pushes further into an active limit, but
             * still send the limited command: recomputing the output from the
             * held I would repeat the preloaded output and stall a retained
             * error. Resetting I from the proportional term (projection) feeds
             * measurement noise into I and limit-cycles under the slew cap. */
            trial_i = s->phase_integral_ppb;
            s->integral_limited = true;
        }
        s->phase_integral_ppb = trial_i;
        s->desired_drift_ppb = 0.0; /* There is no drift setpoint in direct PI. */
        s->phase_limited = raw != bounded;
        s->slew_limited = bounded != limited;
        s->limited = s->phase_limited || s->slew_limited;
        s->accumulator_ppb = s->requested_ppb = limited;
        if (raw != bounded)
        {
            freeze(s, SC_HYBRID_Q_LIMIT);
            return;
        }
        s->request_ready = true;
        phase_status(s, in, tracking_windows(s));
        return;
    }
    double trial_i = s->phase_integral_ppb + delta_i, v, unbounded, raw_v;
    const double bounded =
        track_request(s, trial_i, in->drift_ppb, authority, &v, &unbounded, &raw_v);
    const double limited = clamp(bounded, s->applied_ppb - rate, s->applied_ppb + rate);
    /* Same rule as PHASE: hold I against a limit it pushes into, keep the
     * limited request. Back-calculating I from the measured drift would inject
     * drift measurement noise into I whenever the slew cap is active. */
    if (s->phase_enabled && ((raw_v - v) * delta_i > 0.0 || (unbounded - limited) * delta_i > 0.0))
    {
        trial_i = s->phase_integral_ppb;
        s->integral_limited = true;
    }
    s->phase_integral_ppb = trial_i;
    s->desired_drift_ppb = v;
    s->phase_limited = raw_v != v;
    s->slew_limited = bounded != limited;
    s->limited = s->phase_limited || s->slew_limited || bounded != unbounded;
    s->accumulator_ppb = s->requested_ppb = limited;
    if (bounded != unbounded)
    {
        freeze(s, SC_HYBRID_Q_LIMIT); /* Reacquire around a new trim; keep x_ref. */
        return;
    }
    s->request_ready = true;
    if (!s->phase_enabled)
    {
        s->locked = s->locking = false;
        state(s, SC_HYBRID_FREQUENCY);
        return;
    }
    phase_status(s, in, tracking_windows(s));
}

/* Caller must have a previous accepted sample. Receiver epochs must be exactly
 * 1 s apart and the MCU arrival interval must remain within the freshness limit. */
static bool contiguous_epoch(const sc_hybrid_t *s, const sc_hybrid_input_t *in)
{
    return in->epoch_ms == s->previous_epoch_ms + 1000U &&
           (uint32_t)(in->received_ms - s->last_received_ms) <= DATA_TIMEOUT_MS;
}

/* While tracking, a short run of missing epochs (one overwritten while an FCW
 * write was pending, a single invalid fix, an assembler drop) keeps authority
 * and the phase reference. The receiver runs from this OCXO, so its bias cannot
 * step across the gap without a flagged reset/jump, which recovers separately.
 * The arrival interval must match the receiver interval, as for one epoch. */
static bool bridged_gap(const sc_hybrid_t *s, const sc_hybrid_input_t *in)
{
    const uint64_t gap_ms = in->epoch_ms - s->previous_epoch_ms;
    return sc_hybrid_tracking(s) && gap_ms > 1000U && gap_ms <= GAP_BRIDGE_MS &&
           gap_ms % 1000U == 0U &&
           (uint32_t)(in->received_ms - s->last_received_ms) <= (uint32_t)gap_ms + ARRIVAL_SLACK_MS;
}

static void control_sample(sc_hybrid_t *s, const sc_hybrid_input_t *in)
{
    const bool fresh = (uint32_t)(in->now_ms - in->received_ms) <= DATA_TIMEOUT_MS;
    const bool ppp = in->gnss_valid && fresh && isfinite(in->bias_ns) && isfinite(in->drift_ppb);
    const bool fallback = (in->standalone_valid || in->sbas_valid) && fresh &&
                          isfinite(in->bias_ns) && isfinite(in->drift_ppb);
    s->request_ready = false;
    /* Preserve the accepted coordinate across freezes. Old usable observations
     * cannot change domain or manufacture qualification; only a confirmed
     * discontinuity can restart receiver time. Invalid status must still pass
     * through the gates below, including a cached invalid status update. */
    if ((ppp || fallback) && s->previous_epoch_ms && in->epoch_ms <= s->previous_epoch_ms &&
        !in->frequency_discontinuity)
    {
        return;
    }
    const uint8_t mode = ppp              ? SC_SBF_PVT_MODE_PPP
                         : in->sbas_valid ? SC_SBF_PVT_MODE_SBAS
                         : in->static_fix ? SC_SBF_PVT_MODE_FIXED
                                          : SC_SBF_PVT_MODE_STANDALONE;
    const bool domain_change =
        (ppp || fallback) && (mode != s->source_mode || in->time_system != s->time_system);
    /* Only a contiguous, already running controller can carry authority across
     * a solution/time-system change. Fault recovery and cold acquisition keep
     * their existing gates; an in-flight FCW must never inherit a new domain. */
    const bool handover = domain_change && s->source_mode != 0U && !in->coordinate_changed &&
                          !in->frequency_discontinuity && s->warm && s->previous_valid &&
                          (sc_hybrid_tracking(s) || s->state == SC_HYBRID_DISABLED) && s->enabled &&
                          s->known && !s->pending && !s->acquisition_latched &&
                          contiguous_epoch(s, in);
    const bool recover_reference =
        !handover && ((domain_change && s->source_mode != 0U) || in->coordinate_changed ||
                      in->frequency_discontinuity);
    const bool was_drift_only = drift_only(s), was_fallback = s->fallback;
    const bool primary_changed = (ppp || fallback) && in->primary_mode != s->primary_mode;
    if (primary_changed)
    {
        /* A new selection: an anchor taken in the old primary is not comparable. */
        s->primary_mode = in->primary_mode;
        s->phase_valid = s->settling = false;
    }
    if (ppp || fallback)
    {
        s->fallback = solution_rank(mode) < solution_rank(s->primary_mode);
        if (domain_change && mode == SC_SBF_PVT_MODE_PPP && s->source_mode != 0U &&
            s->source_mode != SC_SBF_PVT_MODE_PPP && s->primary_mode == SC_SBF_PVT_MODE_PPP)
        {
            /* Returning PPP starts tens of ns off and re-converges for 10-25 min. */
            s->settling = true;
            s->settle_ms = in->now_ms;
        }
    }
    const bool keep_anchor = was_fallback || s->fallback;
    if (domain_change || primary_changed || drift_only(s) != was_drift_only)
    {
        if (domain_change)
        {
            s->source_mode = mode;
            s->time_system = in->time_system;
        }
        select_loop(s);
    }
    if (recover_reference)
    {
        s->phase_valid = false;
        freeze(s, SC_HYBRID_Q_EPOCH);
    }
    s->gnss_good = ppp || fallback;
    /* Invalid inputs revoke readiness even while a transaction awaits readback.
     * The readback must still finish; it cannot preserve a stale lock label. */
    if (!s->gnss_good)
    {
        if (sc_hybrid_tracking(s) && s->previous_valid)
        {
            /* Hold the delivered trim (no command without gnss_good) but keep
             * authority and the reference: a usable epoch within the bridge
             * continues; sc_hybrid_service() freezes once the gap outlasts it. */
            s->reason = SC_HYBRID_Q_GNSS;
            s->locking = false;
            return;
        }
        freeze(s, SC_HYBRID_Q_GNSS);
        return;
    }
    if (!s->enabled || s->acquisition_latched || !s->known || s->pending)
    {
        return;
    }
    /* Thermal regulation is an independent service with no input here: its
     * startup, excursions and heater shutdowns never affect GNSS/FCW control. */
    if (recover_reference)
    {
        s->previous_epoch_ms = in->epoch_ms;
        return; /* No command on the event, regardless of bias step or gain change. */
    }
    if (s->previous_valid && !contiguous_epoch(s, in))
    {
        if (!bridged_gap(s, in))
        {
            freeze(s, SC_HYBRID_Q_EPOCH);
            return;
        }
        s->bridged_gaps++;
        s->bridged_epochs += (uint32_t)((in->epoch_ms - s->previous_epoch_ms) / 1000U) - 1U;
    }
    sc_hybrid_input_t clipped_input;
    if (s->previous_valid)
    {
        const double drift_innovation =
            in->drift_ppb - s->previous_drift_ppb - (s->applied_ppb - s->previous_applied_ppb);
        if (fabs(drift_innovation) > CFG(FREQUENCY_INNOVATION_LIMIT_PPB))
        {
            s->consecutive_rejects++;
            s->reason = SC_HYBRID_Q_INNOVATION;
            s->request_ready = s->locking = s->locked = s->gnss_good = false;
            if (handover)
            {
                s->phase_valid = false;
            }
            if (handover || !sc_hybrid_tracking(s) || s->consecutive_rejects >= 2U)
            {
                freeze(s, SC_HYBRID_Q_INNOVATION);
            }
            /* Keep last accepted point: a repeated outlier cannot establish
             * its own moving acceptance baseline. While tracking, the next
             * usable epoch is a bridged one-epoch gap; a second consecutive
             * rejection freezes above. */
            return;
        }
        /* Trapezoidal prediction over the receiver interval (1 s, or a bridged
         * gap); the PI still takes one nominal step per accepted sample. */
        const double interval_s = (double)(in->epoch_ms - s->previous_epoch_ms) / 1000.0;
        const double integrated = 0.5 * (in->drift_ppb + s->previous_drift_ppb) * interval_s;
        const double innovation = in->bias_ns - s->previous_bias_ns - integrated;
        s->phase_innovation_ns = innovation;
        if (handover)
        {
            /* Drift innovation was still checked. The FLL may make its normal
             * bounded update on this sample, including 1 Hz mode alternation. */
            handover_phase(s, in, innovation, keep_anchor);
        }
        else
        {
            /* One bounded rule for every sample: no rejection, holdover or
             * step state. Normal noise passes unchanged; an outlier moves the
             * estimate by at most the limit, a persistent shift passes at
             * limit ns/s, and much heavier noise becomes a low-pass whose
             * time constant grows with the noise. */
            const double limit_ns = (double)s->bias_clip_ps / 1000.0;
            const double used = clamp(innovation, -limit_ns, limit_ns);
            s->input_clipped = used != innovation;
            s->clipped_samples += s->input_clipped ? 1U : 0U;
            s->residual_ns += used;
            clipped_input = *in;
            clipped_input.bias_ns = s->previous_bias_ns + integrated + used;
            in = &clipped_input;
        }
    }
    else
    {
        s->phase_base_ns = in->bias_ns;
        s->residual_ns = 0.0;
    }
    s->consecutive_rejects = 0;
    s->reason = 0U;
    s->previous_valid = true;
    s->previous_epoch_ms = in->epoch_ms;
    s->last_received_ms = in->received_ms;
    s->previous_bias_ns = in->bias_ns;
    s->previous_drift_ppb = in->drift_ppb;
    s->previous_applied_ppb = s->applied_ppb;
    (void)sc_hybrid_stats_push(&s->history, in->drift_ppb, in->bias_ns - s->phase_base_ns,
                               s->residual_ns);
    if (!s->phase_good_samples)
    {
        s->phase_good_ms = in->received_ms;
    }
    if (s->phase_good_samples < SC_HYBRID_TRACKING_POINTS)
    {
        s->phase_good_samples++;
    }
    if (s->loop_mode == SC_HYBRID_LOOP_OFF)
    {
        s->phase_enabled = s->ramping = false;
        state(s, SC_HYBRID_DISABLED);
        return; /* Observations continue; no integrator or acquisition writes. */
    }
    if (sc_hybrid_tracking(s))
    {
        track(s, in);
        return;
    }
    /* Every start, PHASE included, goes through frequency qualification and,
     * when needed, coarse acquisition: a direct phase PI cannot pull in a
     * multi-ppb offset under the tracking slew cap. Verified entry hands PHASE
     * to track(), which holds trim until the phase span qualifies. */
    if (s->state == SC_HYBRID_ACQUIRE)
    {
        return; /* Ramp proposals are paced by actuator completion, not averaged again. */
    }
    if (s->state == SC_HYBRID_VERIFY)
    {
        s->verify_samples++;
        double mean;
        if (s->verify_samples >= CFG(ACQUISITION_VERIFICATION_SAMPLES) &&
            sc_hybrid_stats_drift_mean(&s->history, CFG(ACQUISITION_VERIFICATION_SAMPLES), &mean))
        {
            if (fabs(mean) <= CFG(ACQUISITION_RESIDUAL_MEAN_LIMIT_PPB))
            {
                enter_frequency(s, in, mean);
            }
            else
            {
                acquire(s, mean);
            }
        }
        return;
    }
    state(s, SC_HYBRID_QUALIFY);
    if (!s->qualifying)
    {
        s->qualifying = true;
        s->qualify_epoch_ms = in->epoch_ms;
        s->qualify_ms = in->received_ms;
        s->attempts = 0;
    }
    if (s->warm)
    {
        const unsigned points = CFG(WARM_FREQUENCY_QUALIFICATION_SAMPLES);
        const uint32_t span_ms = (points - 1U) * 1000U;
        double mean;
        if (in->epoch_ms - s->qualify_epoch_ms >= span_ms &&
            (uint32_t)(in->received_ms - s->qualify_ms) >= span_ms &&
            frequency_window_good(s, points))
        {
            /* Every restart (resume, outage or fault recovery) in every loop
             * mode re-acquires coarsely when the frequency moved beyond the
             * acquisition limit, rather than leaving a large step to the fine
             * loop (PHASE has no FLL at all). */
            if (sc_hybrid_stats_drift_mean(&s->history, points, &mean) &&
                fabs(mean) > CFG(ACQUISITION_RESIDUAL_MEAN_LIMIT_PPB))
            {
                acquire(s, mean);
            }
            else
            {
                enter_frequency(s, in, s->desired_drift_ppb);
            }
        }
        return;
    }
    /* Cold acquisition retains the >60 s qualification, coarse slew and
     * 50 fresh fixed-word verification samples. Phase qualifies independently. */
    if (in->epoch_ms - s->qualify_epoch_ms > QUALIFY_DWELL_MS &&
        (uint32_t)(in->received_ms - s->qualify_ms) > QUALIFY_DWELL_MS)
    {
        double mean;
        if (frequency_window_good(s, SC_HYBRID_QUALIFICATION_POINTS) &&
            sc_hybrid_stats_drift_mean(&s->history, CFG(ACQUISITION_AVERAGE_SAMPLES), &mean))
        {
            acquire(s, mean);
        }
        else
        {
            s->reason = SC_HYBRID_Q_WINDOW;
        }
    }
}

void sc_hybrid_sample(sc_hybrid_t *s, const sc_hybrid_input_t *in)
{
    const bool was_locked = s->locked;
    const double previous_drift = s->previous_drift_ppb;
    const double trim_delta = s->applied_ppb - s->previous_applied_ppb;
    const uint8_t source = s->source_mode, time_system = s->time_system;
    const bool was_clipped = s->input_clipped;
    s->lock_failures = 0U;
    s->input_clipped = false;
    control_sample(s, in);
    /* Snapshot the raw sample that starts a clipped stretch, not every sample
     * of it, so the latched record keeps the triggering measurement. */
    const bool rejection = (s->input_clipped && !was_clipped) ||
                           ((s->reason & SC_HYBRID_Q_INNOVATION) && !s->gnss_good);
    if ((was_locked && !s->locked) || rejection)
    {
        uint32_t flags = s->lock_failures;
        if (!s->gnss_good)
        {
            flags |= SC_HYBRID_LOCK_GNSS;
        }
        if (!s->known)
        {
            flags |= SC_HYBRID_LOCK_ACTUATOR;
        }
        if ((s->reason & SC_HYBRID_Q_INNOVATION) || s->input_clipped)
        {
            flags |= SC_HYBRID_LOCK_INNOVATION;
        }
        if (source != s->source_mode || time_system != s->time_system ||
            (s->reason & SC_HYBRID_Q_EPOCH))
        {
            flags |= SC_HYBRID_LOCK_EPOCH;
        }
        if (s->reason & SC_HYBRID_Q_LIMIT)
        {
            flags |= SC_HYBRID_LOCK_LIMIT;
        }
        if (!flags)
        {
            flags = SC_HYBRID_LOCK_SPAN;
        }
        if (!rejection || s->last_failure.epoch_ms != in->epoch_ms ||
            s->last_failure.drift_ppb != in->drift_ppb)
        {
            failure(s, in, flags, previous_drift, trim_delta);
        }
    }
    /* Measurement quality and control permission are separate. This observer
     * runs during thermal departures, acquisition and manual hold as well as
     * PPP/SBAS/standalone tracking. It cannot authorize an actuator command. */
    if (in->frequency_discontinuity)
    {
        memset(&s->frequency, 0, sizeof(s->frequency));
    }
    sc_frequency_monitor_service(&s->frequency, in->now_ms);
    if (in->frequency_valid || in->gnss_valid || in->standalone_valid || in->sbas_valid)
    {
        const double requested = sc_hybrid_tracking(s) ? s->desired_drift_ppb : 0.0;
        sc_frequency_monitor_sample(&s->frequency, in->epoch_ms, in->received_ms, in->now_ms,
                                    in->drift_ppb, requested);
    }
}

bool sc_hybrid_frequency_lit(const sc_hybrid_t *s, uint32_t now_ms)
{
    return s->frequency.settled && s->frequency.count &&
           (uint32_t)(now_ms - s->frequency.last_ms) < SC_FREQUENCY_HOLDOVER_MS;
}

bool sc_hybrid_take_command(sc_hybrid_t *s, uint32_t now_ms, int64_t *word)
{
    if (!s->enabled || s->loop_mode == SC_HYBRID_LOOP_OFF || !s->known || s->pending ||
        (!s->previous_valid && s->state != SC_HYBRID_ACQUIRE) || !s->gnss_good ||
        (uint32_t)(now_ms - s->last_received_ms) > DATA_TIMEOUT_MS ||
        (uint32_t)(now_ms - s->observed_ms) > DATA_TIMEOUT_MS)
    {
        return false;
    }
    if (s->state == SC_HYBRID_ACQUIRE)
    {
        const double rate = CFG(ACQUISITION_SLEW_LIMIT_PPB_PER_S) *
                            fmin(1.0, (double)(uint32_t)(now_ms - s->applied_ms) / 1000.0);
        s->requested_ppb =
            clamp(s->acquire_target_ppb, s->applied_ppb - rate, s->applied_ppb + rate);
        s->request_ready = true;
    }
    if (!s->request_ready)
    {
        return false;
    }
    s->request_ready = false;
    if (!sit5801_ppb_to_fcw(s->requested_ppb, word))
    {
        unknown(s);
        return false;
    }
    /* Hard overall bounds round inward; slew may differ by at most one FCW
     * quantum from nearest rounding, never an additional controller step. */
    if (sit5801_fcw_to_ppb(*word) < s->lower_ppb)
    {
        (*word)++;
    }
    if (sit5801_fcw_to_ppb(*word) > s->upper_ppb)
    {
        (*word)--;
    }
    if (*word == s->applied_word)
    {
        int64_t target_word = 0;
        bool reached = sit5801_ppb_to_fcw(s->acquire_target_ppb, &target_word);
        /* Apply identical inward rounding to the endpoint and each proposal;
         * otherwise a span-bound endpoint can remain one quantum unreachable. */
        if (sit5801_fcw_to_ppb(target_word) < s->lower_ppb)
        {
            target_word++;
        }
        if (sit5801_fcw_to_ppb(target_word) > s->upper_ppb)
        {
            target_word--;
        }
        reached = reached && target_word == s->applied_word;
        if (s->state == SC_HYBRID_ACQUIRE && reached)
        {
            clear_window(s); /* Verification uses 50 wholly NEW fixed-word epochs. */
            state(s, SC_HYBRID_VERIFY);
        }
        return false; /* Do not discard the sub-quantum tracking accumulator. */
    }
    s->requested_word = *word;
    s->pending = true;
    s->pending_ms = now_ms;
    return true;
}

const char *sc_hybrid_state_name(sc_hybrid_state_t value)
{
    static const char *const names[] = {"BOOT_CHECK",        "OCXO_PREHEAT",      "QUALIFY",
                                        "ACQUIRE_FREQUENCY", "VERIFY_FREQUENCY",  "TRACK",
                                        "PHASE_RECOVERING",  "HOLDOVER",          "MANUAL_HOLD",
                                        "ACTUATOR_UNKNOWN",  "ACQUISITION_FAULT", "CONFIG_FAULT",
                                        "THERMAL_RECOVERY",  "FREQUENCY_ONLY",    "DISCIPLINE_OFF"};
    return (unsigned)value < sizeof(names) / sizeof(names[0]) ? names[value] : "INVALID";
}
