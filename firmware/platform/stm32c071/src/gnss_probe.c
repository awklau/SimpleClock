/* SPDX-License-Identifier: MIT */
/* Bounded diagnostic receiver profiles and mixed ASCII/SBF reception.
 * DMA (normal firmware) or the USART ISR queues bytes; main parses and advances commands. */
#include "gnss_probe.h"
#include "main.h"
#include "simpleclock/sbf.h"
#include "simpleclock/sbf_fields.h"
#include "simpleclock/gnss_command.h"
#include "simpleclock/gnss_control.h"
#include "simpleclock/text.h"
#include "timing_probe.h"
#ifdef SIMPLECLOCK_AUTO_HYBRID
#include "hybrid_probe.h"
#include "settings_flash.h"
#define AUTO_RESTORE_PROFILE UINT32_C(0x10000)
#endif
#include <string.h>
#ifdef SIMPLECLOCK_ACTUATOR_TEST
#include "actuator_probe.h"
#endif

#define RX_SIZE 2048U
#ifdef SIMPLECLOCK_AUTO_HYBRID
/* Reassign 1 KiB of ASCII burst storage to the checked control mailbox.
 * Streaming USB remains bounded; every lost byte is counted and a control
 * transaction with losses is explicitly reported incomplete. */
#define REPLY_SIZE 1024U
#define CONTROL_PROFILE UINT32_C(0x20000)
#else
#define REPLY_SIZE 2048U
#endif
#define COMMAND_COUNT(list) (sizeof(list) / sizeof((list)[0]))
/* Normal firmware uses circular DMA with an IRQ wrap counter; foreground
 * snapshots its head and owns the tail. Diagnostic images use byte IRQ RX.
 * The ASCII and SBF
 * output queues are foreground-only. Their consumers advance only after USB
 * accepted a copy, so backpressure cannot acknowledge unsent data. */
static uint8_t rx[RX_SIZE], reply[REPLY_SIZE];
static volatile uint32_t rx_head, rx_tail, rx_overflow, rx_errors, rx_bytes, rx_peak;
#ifdef SIMPLECLOCK_AUTO_HYBRID
static volatile uint32_t rx_dma_cycles;
#endif
static uint32_t reply_head, reply_tail, reply_drops, reply_sequence, reply_sent_ms, reply_try_ms;
static sc_sbf_t sbf;
static uint32_t last_service_ms, max_service_gap_ms;
static const char *const identify_commands[] = {
    "lstInternalFile, Identification\r\n", "getReceiverCapabilities\r\n",
    "lstInternalFile, Permissions\r\n", "lstConfigFile, Current\r\n"};
static const char *const survey_commands[] = {"getPPSParameters\r\n",  "getClockSyncThreshold\r\n",
                                              "getTimeSyncSource\r\n", "getDataInOut, COM1\r\n",
                                              "getSBFOutput\r\n",      "lstConfigFile, Boot\r\n"};
#define OBSERVATION_BLOCKS "PVTGeodetic+ReceiverTime+ReceiverStatus+MeasEpoch+EndOfPVT"
#define AUTO_OBSERVATION_BLOCKS OBSERVATION_BLOCKS "+PosCovCartesian+VelCovCartesian+RFStatus+DOP"
static const char *const once_commands[] = {"exeSBFOnce, COM1, " OBSERVATION_BLOCKS "\r\n"};
/* `gnss trial`: add formal receiver uncertainties without touching PPS, clock
 * steering, tracking, PVT mode, Stream2, or nonvolatile Boot configuration. */
static const char *const trial_stream_commands[] = {
    "getPPSParameters\r\n",
    "getClockSyncThreshold\r\n",
    "setDataInOut, COM1, ,+SBF\r\n",
    "getDataInOut, COM1\r\n",
    "setSBFOutput, Stream1, COM1, " OBSERVATION_BLOCKS "+PosCovCartesian+VelCovCartesian, sec1\r\n",
    "getSBFOutput, Stream1\r\n",
    "getPPSParameters\r\n",
    "getClockSyncThreshold\r\n"};
static const char *const stream_commands[] = {"setPPSParameters, off\r\n",
                                              "getPPSParameters\r\n",
                                              "setClockSyncThreshold, usec500, on\r\n",
                                              "getClockSyncThreshold\r\n",
                                              "setDataInOut, COM1, ,+SBF\r\n",
                                              "getDataInOut, COM1\r\n",
                                              "setSBFOutput, Stream1, COM1, " OBSERVATION_BLOCKS
                                              ", sec1\r\n",
                                              "getSBFOutput, Stream1\r\n"};
static const char *const stop_commands[] = {"setSBFOutput, Stream1, COM1, none\r\n",
                                            "getSBFOutput, Stream1\r\n"};
static const char *const health_commands[] = {"lstInternalFile, Error\r\n"};
static const char *const e6_survey_commands[] = {"getSignalTracking\r\n", "getSignalUsage\r\n",
                                                 "getPVTMode\r\n", "getReceiverCapabilities\r\n",
                                                 "getSBFOutput, Stream2\r\n"};
static const char *const e6_stream_commands[] = {
    "setSBFOutput, Stream2, COM1, GALRawCNAV, OnChange\r\n", "getSBFOutput, Stream2\r\n"};
static const char *const e6_stop_commands[] = {"setSBFOutput, Stream2, COM1, none\r\n",
                                               "getSBFOutput, Stream2\r\n"};
static const char *const pps_commands[] = {"setPPSParameters, sec1, Low2High, 0, RxClock, 0, 5\r\n",
                                           "getPPSParameters\r\n"};
static const char *const pps_off_commands[] = {"setPPSParameters, off\r\n", "getPPSParameters\r\n"};
/* G5 1.1.0 guide section 1.10/p.25 and setPVTMode pp.111-112.
 * Accept only the board's observed Rover/StandAlone/auto baseline. Add PPP
 * without changing tracking, navigation usage, reference position or Boot.
 * Every preflight/readback is checked, not merely an acknowledged command. */
static const char *const ppp_survey_commands[] = {
    "getReceiverCapabilities\r\n", "getSignalTracking\r\n", "getSignalUsage\r\n", "getPVTMode\r\n"};
static const char *const ppp_enable_commands[] = {
    "getReceiverCapabilities\r\n", "getSignalTracking\r\n", "getSignalUsage\r\n", "getPVTMode\r\n",
    "setPVTMode, , +PPP\r\n",      "getPVTMode\r\n"};
static const char *const ppp_restore_commands[] = {
    "getPVTMode\r\n", "setPVTMode, Rover, StandAlone, auto\r\n", "getPVTMode\r\n"};
/* Explicit operator bench selections, independent of legacy PPP-test ownership.
 * G5 1.1.0 pp.111/117: SBAS eligibility is selected by PVTMode. Preserve all
 * tracking, correction-provider, clock, PPS and stream settings. Normal AUTO
 * firmware persists the verified selection; diagnostic images remain volatile. */
#ifdef SIMPLECLOCK_AUTO_HYBRID
#define BENCH_SAVE , "exeCopyConfigFile, Current, Boot\r\n"
#else
#define BENCH_SAVE
#endif
static const char *const no_has_commands[] = {
    "getReceiverCapabilities\r\n", "getPVTMode\r\n", "getSBASCorrections\r\n",
    "setPVTMode, Rover, StandAlone+SBAS, auto\r\n", "getPVTMode\r\n" BENCH_SAVE};
static const char *const has_commands[] = {
    "getReceiverCapabilities\r\n", "getPVTMode\r\n", "getSBASCorrections\r\n",
    "setPVTMode, Rover, StandAlone+PPP, auto\r\n", "getPVTMode\r\n" BENCH_SAVE};
static const char *const standalone_commands[] = {
    "getPVTMode\r\n", "setPVTMode, Rover, StandAlone, auto\r\n", "getPVTMode\r\n" BENCH_SAVE};
#undef BENCH_SAVE
#define BENCH_SET_PVT_INDEX 3U

/* G5 1.1.0 pp.20/62/105: verify both engines before saving the entire Current
 * configuration to Boot. Managed startup and periodic observations never save;
 * initial project-profile setup and explicit edits have separate save paths. */
#ifdef SIMPLECLOCK_AUTO_HYBRID
static char command_scratch[192]; /* Also the mutually exclusive AUTO reply line. */
#else
static char command_scratch[40];
#endif
static const char *const elevation_queries[] = {"getElevationMask, PVT\r\n",
                                                "getElevationMask, Tracking\r\n"};
#define SAVE_BOOT_COMMAND "exeCopyConfigFile, Current, Boot\r\n"
static const char *const elevation_commands[] = {
    "getElevationMask, PVT\r\n", "getElevationMask, Tracking\r\n", command_scratch,
    "getElevationMask, PVT\r\n", "getElevationMask, Tracking\r\n", SAVE_BOOT_COMMAND};
static const char *const elevation_reset_commands[] = {"getElevationMask, PVT\r\n",
                                                       "getElevationMask, Tracking\r\n",
                                                       "setElevationMask, PVT, -90\r\n",
                                                       "setElevationMask, Tracking, -90\r\n",
                                                       "getElevationMask, PVT\r\n",
                                                       "getElevationMask, Tracking\r\n",
                                                       SAVE_BOOT_COMMAND};
static const char *const save_boot_commands[] = {SAVE_BOOT_COMMAND};
static const char *const reboot_commands[] = {"exeResetReceiver, Soft, none\r\n"};
static bool reboot_sent, reboot_failed;
static volatile bool reboot_recovering;
static int8_t elevation_pvt, elevation_tracking;
static uint8_t elevation_flags;
#define MASK_PVT_KNOWN 1U
#define MASK_TRACKING_KNOWN 2U
#define MASK_SAVED 4U
#define MASK_FAULT 8U

static bool elevation_mutation(unsigned profile)
{
    return gnss_probe_elevation_set(profile) || profile == GNSS_PROFILE_ELEVATION_RESET;
}

static bool elevation_profile(unsigned profile)
{
    return elevation_mutation(profile) || profile == GNSS_PROFILE_ELEVATION_SURVEY;
}

static sc_cmd_expect_t elevation_exact(bool tracking)
{
    return (sc_cmd_expect_t)((tracking ? SC_CMD_EXPECT_TRACKING_MASK_BASE
                                       : SC_CMD_EXPECT_PVT_MASK_BASE) +
                             90 + (tracking ? elevation_tracking : elevation_pvt));
}

static bool bench_profile(unsigned profile)
{
    return profile == GNSS_PROFILE_NO_HAS || profile == GNSS_PROFILE_HAS ||
           profile == GNSS_PROFILE_STANDALONE;
}
#ifdef SIMPLECLOCK_AUTO_HYBRID
/* Autonomous boot is query-first. Conditional setters are skipped when their
 * readback already satisfies the contract. */
#define AUTO_COMMANDS_BEGIN                                                                        \
    "getPPSParameters\r\n", "getClockSyncThreshold\r\n", "getTimeSyncSource\r\n",                  \
        "getTimingSystem\r\n", "getDataInOut, COM1\r\n", "getSBFOutput, Stream1\r\n",              \
        "getReceiverCapabilities\r\n", "getSignalTracking\r\n", "getSignalUsage\r\n",              \
        "getPVTMode\r\n", "setClockSyncThreshold, usec500, on\r\n", "getClockSyncThreshold\r\n",   \
        "setTimingSystem, auto\r\n", "getTimingSystem\r\n", "setDataInOut, COM1, ,+SBF\r\n",       \
        "getDataInOut, COM1\r\n",                                                                  \
        "setSBFOutput, Stream1, COM1, " AUTO_OBSERVATION_BLOCKS ", sec1\r\n",                      \
        "getSBFOutput, Stream1\r\n",
#define AUTO_DYNAMICS                                                                              \
    "getReceiverDynamics\r\n", "setReceiverDynamics, Low, Static\r\n", "getReceiverDynamics\r\n"
static const char *const auto_hybrid_commands[][24] = {
    {AUTO_COMMANDS_BEGIN "setPVTMode, Rover, StandAlone+SBAS+PPP, auto\r\n", "getPVTMode\r\n",
     AUTO_DYNAMICS, SAVE_BOOT_COMMAND},
    {AUTO_COMMANDS_BEGIN "setPVTMode, Rover, StandAlone+SBAS, auto\r\n", "getPVTMode\r\n",
     AUTO_DYNAMICS, SAVE_BOOT_COMMAND},
    {AUTO_COMMANDS_BEGIN "setPVTMode, Rover, StandAlone+PPP, auto\r\n", "getPVTMode\r\n",
     AUTO_DYNAMICS, SAVE_BOOT_COMMAND},
    {AUTO_COMMANDS_BEGIN "setPVTMode, Rover, StandAlone, auto\r\n", "getPVTMode\r\n", AUTO_DYNAMICS,
     SAVE_BOOT_COMMAND}};
static const char *const defaults_commands[] = {"getCOMSettings, COM1\r\n",
                                                "getReceiverCapabilities\r\n",
                                                "exeCopyConfigFile, RxDefault, Current\r\n",
                                                "setPPSParameters, off\r\n",
                                                "getPPSParameters\r\n",
                                                "setSignalTracking, +GALE6BC\r\n",
                                                "setSignalUsage, ,+GALE6BC\r\n",
                                                "setPVTMode, Rover, StandAlone+SBAS+PPP, auto\r\n",
                                                "getPVTMode\r\n",
                                                "getElevationMask, PVT\r\n",
                                                "getElevationMask, Tracking\r\n"};
#undef AUTO_DYNAMICS
#undef AUTO_COMMANDS_BEGIN
#define AUTO_GET_PPS 0U
#define AUTO_GET_CLOCK 1U
#define AUTO_GET_TIME_SOURCE 2U
#define AUTO_GET_TIMING 3U
#define AUTO_GET_DATA 4U
#define AUTO_GET_STREAM 5U
#define AUTO_GET_CAP 6U
#define AUTO_GET_TRACKING 7U
#define AUTO_GET_USAGE 8U
#define AUTO_GET_PVT 9U
#define AUTO_SET_CLOCK 10U
#define AUTO_VERIFY_CLOCK 11U
#define AUTO_SET_TIMING 12U
#define AUTO_VERIFY_TIMING 13U
#define AUTO_SET_DATA 14U
#define AUTO_VERIFY_DATA 15U
#define AUTO_SET_STREAM 16U
#define AUTO_VERIFY_STREAM 17U
#define AUTO_SET_PVT 18U
#define AUTO_VERIFY_PVT 19U
#define AUTO_GET_DYNAMICS 20U
#define AUTO_SET_DYNAMICS 21U
#define AUTO_VERIFY_DYNAMICS 22U
#define AUTO_SAVE_BOOT 23U
#define AUTO_MAX_RETRIES 3U
#endif
/* These positions identify the first state-changing byte of each checked
 * profile. Keep them aligned with the command arrays when adding preflight. */
#define PPP_ENABLE_SET_PVT_INDEX 4U
#define PPP_RESTORE_SET_PVT_INDEX 1U
/* Set BEFORE transmitting a mutation: timeout does not prove it was rejected.
 * Retain across host disconnect/profile cancellation, clear only on verified
 * restore. A reset loses this ownership proof and must not guess old settings. */
static bool ppp_restore_pending, ppp_configured;
static uint32_t ppp_denied;
static bool mutation_hold;

#ifdef SIMPLECLOCK_AUTO_HYBRID
static bool auto_ready, auto_fault;

/* Armed only at MCU initialization or an explicitly requested receiver reset.
 * Reading Error acknowledges SOFTWARE warnings, so never poll or retry it. */
enum
{
    STARTUP_ACK_ARMED,
    STARTUP_ACK_INFLIGHT,
    STARTUP_ACK_WAIT_STATUS,
    STARTUP_ACK_DONE
};

static uint8_t startup_ack;
static uint32_t startup_ack_completed_ms, status_rx_error;
static bool status_epoch_valid;
static bool receiver_user_profile, defaults_pending;
/* 0=missing, 1=one valid transport readback, 2=invalid/duplicate. */
static uint8_t defaults_transport;
static uint8_t receiver_selection;
static bool auto_pps, auto_clock_free, auto_time_source, auto_timing_auto;
static bool auto_data_sbf, auto_stream, auto_capability, auto_tracking, auto_usage;
static bool auto_pvt_verified, auto_ppp, auto_clock_steering, auto_static_pvt;
static bool auto_seen_pps, auto_seen_clock, auto_seen_time_source, auto_seen_timing;
static bool auto_seen_data, auto_seen_stream, auto_seen_capability, auto_seen_tracking;
static bool auto_seen_usage, auto_seen_pvt;
static uint8_t auto_retry_count;
static uint32_t auto_retry_due_ms;
static bool auto_retry_scheduled, auto_manual_override;
static char auto_pps_response[96];
/* G5 v1.1.0 pp.143-144: RxClock pulses follow receiver measurement epochs and
 * continue indefinitely. Only interval/time scale/holdover belong to this
 * policy; blank fields preserve polarity, calibration delay and pulse width.
 * No receiver reset, ClockSteering edit, USB wait or routine Boot write. */
static const char *const pps_policy_on = "setPPSParameters,sec1,,,RxClock,0\r\n";
static const char *const pps_policy_off = "setPPSParameters,off,,,RxClock,0\r\n";
static const char *const pps_policy_disabled = "setPPSParameters,off\r\n";
static const char *pps_policy_commands[] = {"getPPSParameters\r\n", NULL, "getPPSParameters\r\n"};
static uint8_t pps_policy, pps_policy_rows;
static bool pps_qualified, pps_ever_qualified, pps_verified, pps_enabled, pps_fault;
static bool pps_policy_matches;
static uint32_t pps_policy_preserved, pps_policy_row_hash, pps_policy_rx_start;
static uint32_t pps_retry_due_ms;
static uint8_t pps_retries;
static bool pps_report_due;
static uint32_t pps_report_ms, pps_report_drops;
static void pps_report_service(uint32_t now_ms);
/* Stream1 contains DOP when this firmware provisioned it. Receivers set up
 * earlier get passive TDOP from a one-time operator command instead:
 * gnss write setSBFOutput,Stream3,COM1,DOP,sec5 (whitelisted, saves Boot). */
static bool dop_stream_ready;
static unsigned auto_ascii_used;
static bool auto_ascii_overflow;
static bool auto_reply_seen, auto_reply_ambiguous;
static bool auto_seen_dynamics, auto_dynamics_static;
static char control_mailbox[196], control_command[SC_GNSS_CONTROL_COMMAND_SIZE];
static char control_query[SC_GNSS_CONTROL_QUERY_SIZE];
static const char *control_commands[4];
static volatile bool control_pending;
static volatile uint32_t control_denied;
static uint32_t control_seq, control_drop_start, control_rx_start, control_preserved;
static uint32_t control_line_hash, control_started_ms;
static uint32_t control_event_ms, control_event_drops;
static unsigned control_kind, control_lines;
static bool control_matches, control_line_matches, control_overflow, control_event;
static bool control_changed;
static bool control_ready_start, control_restore_ready;
static const char *control_result;
static sc_gnss_control_lists_t control_lists;
static uint32_t control_denied_sent;
#endif
static sc_gnss_command_t transaction;

#ifdef SIMPLECLOCK_AUTO_HYBRID
static bool auto_csv_field(const char *line, unsigned wanted, const char *value);
static bool auto_csv_plus_item(const char *line, unsigned wanted, const char *value);
static void auto_copy_text(char *dst, size_t size, const char *src);

static const char *pps_policy_setter(void)
{
    return pps_policy == SC_SETTINGS_PPS_OFF                           ? pps_policy_disabled
           : pps_policy == SC_SETTINGS_PPS_MANUAL                      ? NULL
           : pps_policy == SC_SETTINGS_PPS_CONTINUOUS || pps_qualified ? pps_policy_on
                                                                       : pps_policy_off;
}

static bool pps_complete_row(const char *line)
{
    unsigned commas = 0U;
    for (const char *p = line; *p; ++p)
    {
        commas += *p == ',';
    }
    return commas == 6U &&
           (auto_csv_field(line, 2U, "Low2High") || auto_csv_field(line, 2U, "High2Low"));
}

static void pps_cache(const char *line)
{
    if (line != auto_pps_response)
    {
        auto_copy_text(auto_pps_response, sizeof(auto_pps_response), line);
    }
    auto_pps = pps_complete_row(line);
    pps_enabled = auto_pps && auto_csv_field(line, 1U, "sec1");
    const char *setter = pps_policy_setter();
    bool matched = false;
    uint32_t preserved;
    pps_verified =
        auto_pps && (setter == NULL ||
                     (sc_gnss_control_readback(setter, line, &matched, &preserved) && matched));
    if (auto_pps)
    {
        pps_fault = false;
    }
}

#endif

#ifdef SIMPLECLOCK_AUTO_HYBRID
static void auto_fail(uint32_t now_ms)
{
    auto_ready = false;
    auto_fault = true;
    if (defaults_pending)
    {
        hybrid_probe_defaults_finish(false);
        defaults_pending = false;
        auto_manual_override = true;
        control_seq++;
        control_kind = SC_GNSS_CONTROL_INVALID;
        control_event = true;
        control_event_ms = now_ms;
        control_result = "defaults_failed";
        control_command[0] = control_query[0] = '\0';
        control_drop_start = reply_drops;
        control_rx_start = rx_overflow + rx_errors;
    }
    /* The hybrid core owns holdover/phase handling. The GNSS adapter only
     * gates readiness and never latches the manual HOLD request on failure. */
    mutation_hold = false;
    /* Managed profiles only issue queries. A receiver that boots slowly must
     * recover without a settings commit, even after a prolonged outage.
     * A successfully read but incompatible setting remains inhibited until
     * the operator changes it; do not poll or repair that configuration. */
    const bool retryable = receiver_user_profile ? transaction.state == SC_CMD_TIMEOUT
                                                 : auto_retry_count < AUTO_MAX_RETRIES;
    if (!auto_manual_override && retryable)
    {
        auto_retry_due_ms = now_ms + 30000U;
        auto_retry_scheduled = true;
    }
}
#endif

static sc_cmd_expect_t command_expectation(unsigned profile, unsigned index)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    if ((profile == GNSS_PROFILE_STANDALONE && index == 3U) ||
        ((profile == GNSS_PROFILE_NO_HAS || profile == GNSS_PROFILE_HAS) && index == 5U))
    {
        return SC_CMD_EXPECT_SAVE_BOOT;
    }
    if (profile == GNSS_PROFILE_DEFAULTS)
    {
        if (index == 1U)
        {
            return SC_CMD_EXPECT_HAS_CAPABILITY;
        }
        if (index == 5U)
        {
            return SC_CMD_EXPECT_E6_TRACKING;
        }
        if (index == 6U)
        {
            return SC_CMD_EXPECT_E6_NAVIGATION;
        }
        if (index == 7U || index == 8U)
        {
            return SC_CMD_EXPECT_MULTIMODE;
        }
        if (index == 9U)
        {
            return SC_CMD_EXPECT_ELEVATION_PVT;
        }
        if (index == 10U)
        {
            return SC_CMD_EXPECT_ELEVATION_TRACKING;
        }
    }
    if ((profile == CONTROL_PROFILE && index == 3U) ||
        (profile == GNSS_PROFILE_AUTO_HYBRID && index == AUTO_SAVE_BOOT))
    {
        return SC_CMD_EXPECT_SAVE_BOOT;
    }
#endif
    if (profile == GNSS_PROFILE_SAVE_BOOT)
    {
        return SC_CMD_EXPECT_SAVE_BOOT;
    }
    if (elevation_profile(profile))
    {
        if (index < 2U)
        {
            return index == 0U ? SC_CMD_EXPECT_ELEVATION_PVT : SC_CMD_EXPECT_ELEVATION_TRACKING;
        }
        const bool reset = profile == GNSS_PROFILE_ELEVATION_RESET;
        if (index == (reset ? 6U : 5U))
        {
            return SC_CMD_EXPECT_SAVE_BOOT;
        }
        if (!reset && index == 2U)
        {
            return elevation_exact(gnss_probe_elevation_tracking(profile));
        }
        return elevation_exact(index == (reset ? 3U : 4U) || (reset && index == 5U));
    }
    if (profile == GNSS_PROFILE_STANDALONE)
    {
        return index == 0U ? SC_CMD_EXPECT_BENCH_PVT_MODE : SC_CMD_EXPECT_STANDALONE;
    }
    if (bench_profile(profile))
    {
        if (index == 0U)
        {
            return profile == GNSS_PROFILE_NO_HAS ? SC_CMD_EXPECT_SBAS_CAPABILITY
                   : profile == GNSS_PROFILE_HAS  ? SC_CMD_EXPECT_HAS_CAPABILITY
                                                  : SC_CMD_EXPECT_NONE;
        }
        if (index == 1U)
        {
            return SC_CMD_EXPECT_BENCH_PVT_MODE;
        }
        if (index >= BENCH_SET_PVT_INDEX)
        {
            return profile == GNSS_PROFILE_NO_HAS ? SC_CMD_EXPECT_STANDALONE_SBAS
                   : profile == GNSS_PROFILE_HAS  ? SC_CMD_EXPECT_STANDALONE_PPP
                                                  : SC_CMD_EXPECT_STANDALONE;
        }
        return SC_CMD_EXPECT_NONE;
    }
    static const sc_cmd_expect_t enable_checks[] = {
        SC_CMD_EXPECT_HAS_CAPABILITY, SC_CMD_EXPECT_E6_TRACKING,    SC_CMD_EXPECT_E6_NAVIGATION,
        SC_CMD_EXPECT_STANDALONE,     SC_CMD_EXPECT_STANDALONE_PPP, SC_CMD_EXPECT_STANDALONE_PPP};
    if (profile == GNSS_PROFILE_PPP_ENABLE && index < COMMAND_COUNT(enable_checks))
    {
        return enable_checks[index];
    }
    if (profile == GNSS_PROFILE_PPP_RESTORE)
    {
        return index == 0U ? SC_CMD_EXPECT_PPP_TEST_MODE : SC_CMD_EXPECT_STANDALONE;
    }
#ifdef SIMPLECLOCK_AUTO_HYBRID
    if (profile == GNSS_PROFILE_AUTO_HYBRID)
    {
        switch (index)
        {
        case AUTO_GET_CAP:
            return SC_CMD_EXPECT_HAS_CAPABILITY;
        case AUTO_GET_TRACKING:
            return SC_CMD_EXPECT_E6_TRACKING;
        case AUTO_GET_USAGE:
            return SC_CMD_EXPECT_E6_NAVIGATION;
        case AUTO_GET_PVT:
            return receiver_user_profile ? SC_CMD_EXPECT_NONE : SC_CMD_EXPECT_BENCH_PVT_MODE;
        case AUTO_SET_PVT:
        case AUTO_VERIFY_PVT:
            if (receiver_user_profile)
            {
                return SC_CMD_EXPECT_NONE;
            }
            return receiver_selection == SC_SETTINGS_GNSS_SBAS  ? SC_CMD_EXPECT_STANDALONE_SBAS
                   : receiver_selection == SC_SETTINGS_GNSS_HAS ? SC_CMD_EXPECT_STANDALONE_PPP
                   : receiver_selection == SC_SETTINGS_GNSS_STANDALONE ? SC_CMD_EXPECT_STANDALONE
                                                                       : SC_CMD_EXPECT_MULTIMODE;
        default:
            break;
        }
    }
#endif
    return SC_CMD_EXPECT_NONE;
}

#ifdef SIMPLECLOCK_AUTO_HYBRID
static const char *auto_reply_prefix(unsigned index)
{
    switch (index)
    {
    case AUTO_GET_PPS:
        return "PPSParameters,";
    case AUTO_GET_CLOCK:
    case AUTO_VERIFY_CLOCK:
        return "ClockSyncThreshold,";
    case AUTO_GET_TIME_SOURCE:
        return "TimeSyncSource,";
    case AUTO_GET_TIMING:
    case AUTO_VERIFY_TIMING:
        return "TimingSystem,";
    case AUTO_GET_DATA:
    case AUTO_VERIFY_DATA:
        return "DataInOut, COM1,";
    case AUTO_GET_STREAM:
    case AUTO_VERIFY_STREAM:
        return "SBFOutput, Stream1,";
    case AUTO_GET_PVT:
    case AUTO_SET_PVT:
    case AUTO_VERIFY_PVT:
        return "PVTMode,";
    case AUTO_GET_DYNAMICS:
    case AUTO_SET_DYNAMICS:
    case AUTO_VERIFY_DYNAMICS:
        return "ReceiverDynamics,";
    default:
        return NULL; /* Capability/signal lists use the streaming token parser. */
    }
}

static bool auto_command_readback_ok(unsigned index)
{
    if (auto_reply_prefix(index) != NULL && (!auto_reply_seen || auto_reply_ambiguous))
    {
        return false;
    }
    switch (index)
    {
    case AUTO_GET_PPS:
        return auto_seen_pps;
    case AUTO_GET_CLOCK:
        return auto_seen_clock;
    case AUTO_VERIFY_CLOCK:
        return auto_clock_free;
    case AUTO_GET_TIME_SOURCE:
        return auto_seen_time_source;
    case AUTO_GET_TIMING:
        return auto_seen_timing;
    case AUTO_VERIFY_TIMING:
        return auto_timing_auto;
    case AUTO_GET_DATA:
        return auto_seen_data;
    case AUTO_VERIFY_DATA:
        return auto_data_sbf;
    case AUTO_GET_STREAM:
        return auto_seen_stream;
    case AUTO_VERIFY_STREAM:
        return auto_stream;
    case AUTO_GET_CAP:
        auto_seen_capability = transaction.readback.seen;
        auto_capability = transaction.readback.matched;
        return auto_seen_capability;
    case AUTO_GET_TRACKING:
        auto_seen_tracking = transaction.readback.seen;
        auto_tracking = transaction.readback.matched;
        return auto_seen_tracking;
    case AUTO_GET_USAGE:
        auto_seen_usage = transaction.readback.seen;
        auto_usage = transaction.readback.matched;
        return auto_seen_usage;
    case AUTO_GET_PVT:
        if (receiver_user_profile)
        {
            auto_pvt_verified = auto_ppp;
            return auto_seen_pvt;
        }
        auto_seen_pvt = transaction.readback.seen;
        auto_pvt_verified = transaction.readback.matched;
        return auto_seen_pvt;
    case AUTO_VERIFY_PVT:
        return auto_ppp;
    case AUTO_GET_DYNAMICS:
        return auto_seen_dynamics;
    case AUTO_VERIFY_DYNAMICS:
        return receiver_user_profile ? auto_seen_dynamics : auto_dynamics_static;
    default:
        return true;
    }
}

static bool auto_skip_command(unsigned index)
{
    if (receiver_user_profile &&
        (index == AUTO_SET_CLOCK || index == AUTO_SET_TIMING || index == AUTO_SET_DATA ||
         index == AUTO_SET_STREAM || index == AUTO_SET_PVT || index == AUTO_SET_DYNAMICS))
    {
        return true;
    }
    /* Conditional changes are decided only from the preceding query. */
    return (index == AUTO_SET_CLOCK && auto_clock_free) ||
           (index == AUTO_SET_TIMING && auto_timing_auto) ||
           (index == AUTO_SET_DATA && auto_data_sbf) || (index == AUTO_SET_STREAM && auto_stream) ||
           (index == AUTO_SET_PVT && auto_ppp) ||
           (index == AUTO_SET_DYNAMICS && auto_dynamics_static);
}

static bool auto_mutating_command(unsigned index)
{
    return index == AUTO_SET_CLOCK || index == AUTO_SET_TIMING || index == AUTO_SET_DATA ||
           index == AUTO_SET_STREAM || index == AUTO_SET_PVT || index == AUTO_SET_DYNAMICS;
}
#endif

static const char *const *commands = identify_commands;
static unsigned command_count = COMMAND_COUNT(identify_commands),
                command_profile = GNSS_PROFILE_IDENTIFY;
static unsigned command_index, command_tx_index;
static uint32_t command_completed_ms, command_timeouts;
static bool command_active;
/* USB callback mailbox: main takes/clears this under PRIMASK. No command I/O in ISR. */
static volatile uint32_t requested_profile = GNSS_PROFILE_IDENTIFY;
static unsigned reply_acks, reply_rejects;

typedef struct
{
    uint8_t prefix[SC_SBF_PREFIX];
    uint16_t length, kept, valid;
    uint8_t result;
    uint32_t sequence, captured_ms;
} sbf_record_t;

static sbf_record_t records[8];
static uint32_t record_head, record_tail, record_sequence, record_drops;
static uint32_t decode_ok, decode_unknown, decode_revision, decode_bad;
static uint32_t status_ms, status_rx_state;
static uint32_t status_uptime_s;
static bool status_seen;
static uint32_t pps_denied, input_faults;

#ifdef SIMPLECLOCK_AUTO_HYBRID
static bool auto_csv_field(const char *line, unsigned wanted, const char *value)
{
    const char *p = line;
    for (unsigned field = 0U;; ++field)
    {
        const char *start;
        while (*p == ' ' || *p == '\t')
        {
            ++p;
        }
        start = p;
        while (*p != '\0' && *p != ',')
        {
            ++p;
        }
        const char *end = p;
        while (end > start && (end[-1] == ' ' || end[-1] == '\t'))
        {
            --end;
        }
        if (field == wanted)
        {
            const size_t length = (size_t)(end - start);
            return length == strlen(value) && memcmp(start, value, length) == 0;
        }
        if (*p != ',')
        {
            return false;
        }
        ++p;
    }
}

static bool auto_csv_plus_item(const char *line, unsigned wanted, const char *value)
{
    const char *p = line;
    for (unsigned field = 0U;; ++field)
    {
        while (*p == ' ' || *p == '\t')
        {
            ++p;
        }
        const char *start = p;
        while (*p != '\0' && *p != ',')
        {
            ++p;
        }
        if (field == wanted)
        {
            const char *end = p;
            while (start < end)
            {
                while (start < end && (*start == ' ' || *start == '\t' || *start == '+'))
                {
                    ++start;
                }
                const char *item = start;
                while (start < end && *start != '+' && *start != ' ' && *start != '\t')
                {
                    ++start;
                }
                if ((size_t)(start - item) == strlen(value) &&
                    memcmp(item, value, (size_t)(start - item)) == 0)
                {
                    return true;
                }
            }
            return false;
        }
        if (*p != ',')
        {
            return false;
        }
        ++p;
    }
}

static bool auto_numeric_threshold(const char *line)
{
    static const char *const allowed[] = {"usec500", "msec1", "msec2", "msec3", "msec4", "msec5"};
    for (unsigned i = 0U; i < sizeof(allowed) / sizeof(allowed[0]); ++i)
    {
        if (auto_csv_field(line, 1U, allowed[i]))
        {
            return true;
        }
    }
    return false;
}

static void auto_copy_text(char *dst, size_t size, const char *src)
{
    size_t count = strlen(src);
    if (count >= size)
    {
        count = size - 1U;
    }
    memcpy(dst, src, count);
    dst[count] = '\0';
}

static void auto_observe_reply_line(void)
{
    command_scratch[auto_ascii_used] = '\0';
    const char *line = command_scratch;
    while (*line == ' ' || *line == '\t')
    {
        ++line;
    }
    const char *prefix = auto_reply_prefix(command_index);
    if (!transaction.echo || prefix == NULL || strncmp(line, prefix, strlen(prefix)) != 0)
    {
        return;
    }
    auto_reply_ambiguous |= auto_reply_seen;
    auto_reply_seen = true;
    if (strncmp(line, "PPSParameters,", 14) == 0)
    {
        auto_seen_pps = true;
        pps_cache(line);
    }
    else if (strncmp(line, "ClockSyncThreshold,", 19) == 0)
    {
        auto_seen_clock = true;
        /* The guide lists exactly ClockSteering/usec500/msec1..msec5. Unknown
         * values are neither evidence of free running nor permission to set. */
        auto_clock_steering = auto_csv_field(line, 1U, "ClockSteering");
        auto_clock_free = auto_numeric_threshold(line) &&
                          (auto_csv_field(line, 2U, "on") || auto_csv_field(line, 2U, "off"));
    }
    else if (strncmp(line, "TimeSyncSource,", 15) == 0)
    {
        /* This is deliberately observational: the current reference route is
         * never overwritten by AUTO_HYBRID. */
        auto_seen_time_source = true;
        auto_time_source = true;
    }
    else if (strncmp(line, "TimingSystem,", 13) == 0)
    {
        auto_seen_timing = true;
        auto_timing_auto = auto_csv_field(line, 1U, "auto");
    }
    else if (strncmp(line, "DataInOut, COM1,", 16) == 0)
    {
        auto_seen_data = true;
        auto_data_sbf = auto_csv_plus_item(line, 3U, "SBF");
    }
    else if (strncmp(line, "SBFOutput, Stream1,", 19) == 0)
    {
        auto_seen_stream = true;
        auto_stream =
            auto_csv_field(line, 2U, "COM1") && auto_csv_plus_item(line, 3U, "PVTGeodetic") &&
            auto_csv_plus_item(line, 3U, "ReceiverTime") &&
            auto_csv_plus_item(line, 3U, "ReceiverStatus") &&
            auto_csv_plus_item(line, 3U, "MeasEpoch") && auto_csv_plus_item(line, 3U, "EndOfPVT") &&
            auto_csv_plus_item(line, 3U, "PosCovCartesian") &&
            auto_csv_plus_item(line, 3U, "VelCovCartesian") &&
            auto_csv_plus_item(line, 3U, "RFStatus") && auto_csv_field(line, 4U, "sec1");
        dop_stream_ready = auto_stream && auto_csv_plus_item(line, 3U, "DOP");
    }
    else if (strncmp(line, "PVTMode,", 8) == 0)
    {
        auto_seen_pvt = true;
        uint32_t selection;
        bool fixed;
        auto_ppp = sc_gnss_control_pvt_policy(line, &selection, &fixed) &&
                   selection == receiver_selection && (receiver_user_profile || !fixed);
        auto_static_pvt = auto_ppp && fixed;
    }
    else if (strncmp(line, "ReceiverDynamics,", 17) == 0)
    {
        auto_seen_dynamics = true;
        auto_dynamics_static =
            auto_csv_field(line, 1U, "Low") && auto_csv_field(line, 2U, "Static");
    }
    auto_ascii_used = 0U;
}
#endif

void gnss_probe_init(void)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    control_pending = control_event = false;
    control_seq = control_denied = control_kind = SC_GNSS_CONTROL_INVALID;
    startup_ack = STARTUP_ACK_ARMED;
    status_epoch_valid = false;
    pps_qualified = pps_ever_qualified = pps_verified = pps_enabled = pps_fault = false;
    pps_policy_rows = pps_retries = 0U;
    dop_stream_ready = false;
#endif
    reboot_sent = reboot_recovering = reboot_failed = false;
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART2_FORCE_RESET();
    __HAL_RCC_USART2_RELEASE_RESET();
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);
    /* PCLK=48 MHz, oversampling16, 8N1; nearest BRR=417 -> 115108 baud.
     * All diagnostic/PPP profiles keep this fixed transport configuration. */
    USART2->BRR = 417U;
#ifdef SIMPLECLOCK_AUTO_HYBRID
    /* RM0490: DMAMUX request 52 is USART2_RX. Circular DMA keeps receiving
     * during single-bank flash erase/program, when flash-resident IRQs stall.
     * Reuse the ring rather than allocating a second receive buffer. */
    __HAL_RCC_DMA1_CLK_ENABLE();
    DMA1_Channel1->CCR = 0U;
    DMA1->IFCR = DMA_IFCR_CGIF1;
    DMAMUX1_Channel0->CCR = 52U;
    DMA1_Channel1->CPAR = (uintptr_t)&USART2->RDR;
    DMA1_Channel1->CMAR = (uintptr_t)rx;
    DMA1_Channel1->CNDTR = RX_SIZE;
    rx_dma_cycles = rx_head = rx_tail = 0U;
    DMA1_Channel1->CCR =
        DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_TCIE | DMA_CCR_TEIE | DMA_CCR_PL_1 | DMA_CCR_EN;
    HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
    USART2->CR3 = USART_CR3_EIE | USART_CR3_DMAR;
    USART2->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE;
#else
    USART2->CR3 = USART_CR3_EIE;
    USART2->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE_RXFNEIE;
#endif
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    last_service_ms = HAL_GetTick();
#ifdef SIMPLECLOCK_AUTO_HYBRID
    sc_settings_t saved;
    uint32_t generation;
    const bool loaded = settings_flash_load(&saved, &generation) == SC_SETTINGS_VALID;
    receiver_selection = loaded ? (uint8_t)saved.gnss_selection : SC_SETTINGS_GNSS_AUTO;
    receiver_user_profile = loaded && saved.receiver_user_profile != 0U;
    pps_policy = loaded ? (uint8_t)saved.pps_policy : SC_SETTINGS_PPS_CONTINUOUS;
    defaults_pending = false;
    requested_profile = AUTO_RESTORE_PROFILE;
#endif
}

void USART2_IRQHandler(void)
{
    /* Single hardware byte per IRQ; no parsing, I2C, formatting or USB here. */
    const uint32_t status = USART2->ISR;
    if ((status & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE)) != 0U)
    {
        USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
        rx_errors++;
    }
#ifndef SIMPLECLOCK_AUTO_HYBRID
    if ((status & USART_ISR_RXNE_RXFNE) != 0U)
    {
        const uint8_t byte = (uint8_t)USART2->RDR;
        rx_bytes++;
        const uint32_t used = rx_head - rx_tail;
        if (used < RX_SIZE)
        {
            rx[rx_head & (RX_SIZE - 1U)] = byte;
            __DMB();
            rx_head++;
            if (used + 1U > rx_peak)
            {
                rx_peak = used + 1U;
            }
        }
        else
        {
            rx_overflow++;
        }
    }
#endif
}

#ifdef SIMPLECLOCK_AUTO_HYBRID
void DMA1_Channel1_IRQHandler(void)
{
    const uint32_t status = DMA1->ISR;
    if (status & DMA_ISR_TCIF1)
    {
        rx_dma_cycles += RX_SIZE;
    }
    if (status & DMA_ISR_TEIF1)
    {
        rx_errors++; /* A real DMA fault remains visible; do not disguise it. */
    }
    DMA1->IFCR = DMA_IFCR_CGIF1;
}

static void receive_dma_sync(uint32_t gap_ms)
{
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    uint32_t cycles = rx_dma_cycles;
    uint32_t remaining = DMA1_Channel1->CNDTR;
    if (DMA1->ISR & DMA_ISR_TCIF1)
    {
        /* A wrap may be pending while flash has prevented the IRQ running.
         * Re-read the position after observing that wrap. Do not clear its IRQ. */
        cycles += RX_SIZE;
        remaining = DMA1_Channel1->CNDTR;
        if (remaining == 0U)
        {
            remaining = RX_SIZE; /* Terminal count before circular reload. */
        }
    }
    const uint32_t produced = cycles + RX_SIZE - remaining;
    if (gap_ms >= 177U)
    {
        rx_errors++; /* Shared with UART/DMA IRQs: update while masked. */
    }
    __DMB();
    __set_PRIMASK(mask);
    rx_bytes += produced - rx_head;
    rx_head = produced;
    /* A complete ring takes 177.8 ms at 115200 8N1. Beyond that bound a
     * latched TC flag cannot prove how many wraps occurred: discard and flag
     * the ambiguous input. Normal flash erases are comfortably shorter. */
    if (gap_ms >= 177U)
    {
        rx_tail = rx_head;
    }
    const uint32_t used = rx_head - rx_tail;
    if (used > rx_peak)
    {
        rx_peak = used;
    }
    /* Leave one service batch of room for DMA to advance during parsing. */
    if (used > RX_SIZE - 128U)
    {
        const uint32_t lost = used - (RX_SIZE - 128U);
        rx_overflow += lost;
        rx_tail += lost;
    }
}
#endif

void gnss_probe_identify(void)
{
    if (!reboot_recovering
#ifdef SIMPLECLOCK_AUTO_HYBRID
        && !control_kind && !control_pending && !control_event && !defaults_pending
#endif
    )
    {
        requested_profile = GNSS_PROFILE_IDENTIFY;
    }
}

void gnss_probe_request(uint32_t profile)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    if (profile == GNSS_PROFILE_PPS || profile == GNSS_PROFILE_PPS_OFF)
    {
        if (!gnss_probe_pps_policy_set(profile == GNSS_PROFILE_PPS ? SC_SETTINGS_PPS_CONTINUOUS
                                                                   : SC_SETTINGS_PPS_OFF))
        {
            pps_denied++;
        }
        return;
    }
    if (profile == GNSS_PROFILE_DEFAULTS)
    {
        if (gnss_probe_commands_idle())
        {
            requested_profile = GNSS_PROFILE_DEFAULTS;
        }
        else
        {
            control_denied++;
        }
        return;
    }
#endif
    if (reboot_recovering
#ifdef SIMPLECLOCK_AUTO_HYBRID
        || control_kind || control_pending || control_event || defaults_pending
#endif
    )
    {
        return;
    }
    if (profile >= GNSS_PROFILE_IDENTIFY &&
        (profile <= GNSS_PROFILE_TRIAL_STREAM || bench_profile(profile) ||
         elevation_profile(profile) || profile == GNSS_PROFILE_SAVE_BOOT ||
#ifdef SIMPLECLOCK_AUTO_HYBRID
         profile == GNSS_PROFILE_AUTO_HYBRID
#else
         false
#endif
         ))
    {
#ifdef SIMPLECLOCK_AUTO_HYBRID
        if (bench_profile(profile))
        {
            /* Sticky until this selection's final readback succeeds. A later
             * status query must not revive readiness after a failed setter. */
            auto_pvt_verified = false;
        }
        if (profile != GNSS_PROFILE_AUTO_HYBRID && (command_profile == GNSS_PROFILE_AUTO_HYBRID ||
                                                    requested_profile == GNSS_PROFILE_AUTO_HYBRID ||
                                                    requested_profile == AUTO_RESTORE_PROFILE))
        {
            auto_manual_override = true;
            auto_retry_scheduled = false;
        }
#endif
        requested_profile = profile;
    }
}

static void receive_ascii_byte(void *context, uint8_t byte)
{
    (void)context;
    if (reply_head - reply_tail < REPLY_SIZE)
    {
        reply[reply_head++ & (REPLY_SIZE - 1U)] = byte;
    }
    else
    {
        reply_drops++;
    }
    /* A delayed reply to an earlier attempt must not complete this command
     * while its bytes are still being sent. The receiver can reply on CR,
     * so either line terminator counts; all profiles include CR/LF. Keep
     * logging other ASCII, but do not let it alter transaction/readback state. */
    if (!command_active || transaction.state != SC_CMD_WAITING || command_tx_index == 0U ||
        (commands[command_index][command_tx_index - 1U] != '\r' &&
         commands[command_index][command_tx_index - 1U] != '\n'))
    {
        return;
    }
#ifdef SIMPLECLOCK_AUTO_HYBRID
    if (command_profile == GNSS_PROFILE_PPS_POLICY && transaction.echo &&
        (command_index == 0U || command_index == 2U) && (byte == '\r' || byte == '\n'))
    {
        const char *line = transaction.line;
        while (*line == ' ' || *line == '\t')
        {
            line++;
        }
        if (!strncmp(line, "PPSParameters,", 14))
        {
            pps_policy_rows = pps_policy_rows < 2U ? pps_policy_rows + 1U : 2U;
            pps_policy_matches =
                !transaction.overflow && pps_complete_row(line) &&
                sc_gnss_control_readback(pps_policy_commands[1], line, &control_line_matches,
                                         &pps_policy_row_hash) &&
                control_line_matches;
            if (!transaction.overflow && pps_complete_row(line))
            {
                pps_cache(line);
            }
        }
    }
    if (command_profile == CONTROL_PROFILE && control_kind == SC_GNSS_CONTROL_WRITE &&
        control_lists.family && (command_index == 0U || command_index == 2U) && transaction.echo &&
        sc_gnss_control_lists_byte(&control_lists, byte, &control_line_matches))
    {
        control_lines++;
        control_matches = control_line_matches;
        control_line_hash = 0U;
    }
    if (command_profile == CONTROL_PROFILE && transaction.echo && (byte == '\r' || byte == '\n'))
    {
        if (!control_lists.family)
        {
            control_overflow = control_overflow || transaction.overflow;
        }
        if (!control_lists.family && !transaction.overflow &&
            control_kind == SC_GNSS_CONTROL_WRITE && (command_index == 0U || command_index == 2U) &&
            sc_gnss_control_readback(control_command, transaction.line, &control_line_matches,
                                     &control_line_hash))
        {
            control_lines++;
            control_matches = control_line_matches;
            /* A satisfied pre-query skips the setter/post-query. Cache that
             * verified Current too, before persisting selection or PPS state. */
            if (control_matches && !strncmp(control_command, "setPVTMode", 10))
            {
                uint32_t selection;
                bool fixed;
                if (sc_gnss_control_pvt_policy(transaction.line, &selection, &fixed))
                {
                    receiver_selection = (uint8_t)selection;
                }
            }
            if (control_matches && !strncmp(control_command, "setPPSParameters", 16))
            {
                pps_cache(transaction.line);
            }
        }
    }
#endif
#ifdef SIMPLECLOCK_AUTO_HYBRID
    if (command_profile == CONTROL_PROFILE && control_kind == SC_GNSS_CONTROL_READ &&
        transaction.echo)
    {
        /* Use this service pass's timestamp for both refresh and expiry. Reading
         * HAL time here can be one tick newer than now_ms and look wrapped. */
        transaction.started_ms = last_service_ms; /* 3 s idle, 60 s total. */
    }
    if (command_profile == GNSS_PROFILE_DEFAULTS && transaction.echo &&
        (byte == '\r' || byte == '\n') && !transaction.overflow)
    {
        const char *line = transaction.line;
        while (*line == ' ' || *line == '\t')
        {
            line++;
        }
        if (command_index == 0U && !strncmp(line, "COMSettings,", 12))
        {
            defaults_transport =
                !defaults_transport && auto_csv_field(line, 1U, "COM1") &&
                        auto_csv_field(line, 2U, "baud115200") &&
                        auto_csv_field(line, 3U, "bits8") && auto_csv_field(line, 4U, "No") &&
                        auto_csv_field(line, 5U, "bit1") && auto_csv_field(line, 6U, "none")
                    ? 1U
                    : 2U;
        }
        if (command_index == 4U && !strncmp(line, "PPSParameters,", 14))
        {
            defaults_transport = !defaults_transport && auto_csv_field(line, 1U, "off") ? 1U : 2U;
        }
    }
#endif
    sc_gnss_command_byte(&transaction, byte);
#ifdef SIMPLECLOCK_AUTO_HYBRID
    if (command_profile == GNSS_PROFILE_AUTO_HYBRID && command_active)
    {
        if (byte == '\r' || byte == '\n')
        {
            if (auto_ascii_overflow && auto_reply_prefix(command_index) != NULL)
            {
                auto_reply_ambiguous = true;
            }
            else if (auto_ascii_used != 0U)
            {
                auto_observe_reply_line();
            }
            auto_ascii_used = 0U;
            auto_ascii_overflow = false;
        }
        else if (!auto_ascii_overflow && auto_ascii_used + 1U < sizeof(command_scratch) &&
                 byte >= 0x20U && byte <= 0x7eU)
        {
            command_scratch[auto_ascii_used++] = (char)byte;
        }
        else if (!auto_ascii_overflow && auto_ascii_used + 1U >= sizeof(command_scratch))
        {
            auto_ascii_overflow = true;
        }
    }
#endif
}

static void receive_sbf_frame(void *context, const uint8_t *prefix, uint16_t length,
                              bool structure_valid)
{
    (void)context;
    const uint16_t kept = length < SC_SBF_PREFIX ? length : SC_SBF_PREFIX;
    sc_sbf_observation_t observation = {0};
    const sc_sbf_decode_result_t result = structure_valid
                                              ? sc_sbf_decode(prefix, kept, length, &observation)
                                              : SC_SBF_DECODE_STRUCTURE;
    if (result == SC_SBF_DECODE_OK)
    {
        decode_ok++;
        timing_probe_observe(&observation, HAL_GetTick());
#ifdef SIMPLECLOCK_AUTO_HYBRID
        hybrid_probe_observe(&observation, HAL_GetTick());
#endif
#ifdef SIMPLECLOCK_ACTUATOR_TEST
        actuator_probe_observe(&observation, HAL_GetTick());
#endif
        if (observation.id == SC_SBF_STATUS)
        {
#ifdef SIMPLECLOCK_AUTO_HYBRID
            status_rx_error = observation.data.status.rx_error;
            status_epoch_valid = (observation.valid & SC_SBF_VALID_EPOCH) != 0U;
            if (command_profile == GNSS_PROFILE_AUTO_HYBRID && status_seen &&
                observation.data.status.uptime_s < status_uptime_s)
            {
                auto_fail(HAL_GetTick());
                command_active = false;
                command_index = command_count;
                transaction.state = SC_CMD_REJECTED;
            }
#endif
            status_seen = true;
            status_ms = HAL_GetTick();
            status_rx_state = observation.data.status.rx_state;
            status_uptime_s = observation.data.status.uptime_s;
        }
    }
    else if (result == SC_SBF_DECODE_UNKNOWN)
    {
        decode_unknown++;
    }
    else if (result == SC_SBF_DECODE_REVISION)
    {
        decode_revision++;
    }
    else if (result == SC_SBF_DECODE_LENGTH)
    {
        decode_bad++;
    }
    const uint32_t sequence = record_sequence++;
    if (record_head - record_tail == 8U)
    {
        record_drops++;
        return;
    }
    sbf_record_t *record = &records[record_head & 7U];
    memcpy(record->prefix, prefix, kept);
    record->kept = kept;
    record->length = length;
    record->sequence = sequence;
    record->captured_ms = HAL_GetTick();
    record->result = (uint8_t)result;
    record->valid = observation.valid;
    record_head++;
}

static void service_reply_queues(uint32_t now_ms)
{
    /* Leave normal telemetry windows, but drain a half-full ASCII ring even
     * there: 110 ms at 115200 8N1 brings 1267 bytes into the 1024-byte ring.
     * The ordinary window alone can therefore lose a diagnostic reply even
     * with a healthy USB host. Pressure also takes priority over SBF export;
     * receiver parsing/control never waits for either output queue. */
    const bool reply_pressure = reply_head - reply_tail >= REPLY_SIZE / 2U;
    const uint32_t phase = now_ms % 1000U;
    if ((reply_head == reply_tail && record_head == record_tail) ||
        (!reply_pressure && (phase < 30U || (phase > 300U && phase < 400U) ||
                             (phase > 650U && phase < 760U) || phase > 940U)))
    {
        return;
    }
    /* Coalesce bursts, flush the final short tail within 20 ms. At most one
     * USB attempt per tick even when UART interrupts wake the loop faster. */
    if (now_ms == reply_try_ms)
    {
        return;
    }
    if (record_head == record_tail && reply_head - reply_tail < 64U &&
        (uint32_t)(now_ms - reply_sent_ms) < 20U)
    {
        return;
    }
    reply_try_ms = now_ms;
    if (record_head != record_tail && !reply_pressure)
    {
        const sbf_record_t *record = &records[record_tail & 7U];
        char raw_line[400];
        sc_text_t text;
        sc_text_init(&text, raw_line, sizeof(raw_line));
        sc_text_string(&text, "type=sbf_prefix v=1 seq=");
        sc_text_u32(&text, record->sequence);
        sc_text_string(&text, " uptime_ms=");
        sc_text_u32(&text, record->captured_ms);
        sc_text_string(&text, " wire_length=");
        sc_text_u32(&text, record->length);
        sc_text_string(&text, " kept=");
        sc_text_u32(&text, record->kept);
        sc_text_string(&text, " result=");
        sc_text_u32(&text, record->result);
        sc_text_string(&text, " valid=");
        sc_text_u32(&text, record->valid);
        sc_text_string(&text, " hex=");
        for (unsigned i = 0; i < record->kept; i++)
        {
            sc_text_hex(&text, record->prefix[i], 2);
        }
        sc_text_string(&text, "\r\n");
        if (text.ok &&
            simpleclock_usb_cdc_try_send_queued((const uint8_t *)raw_line, (uint16_t)text.length))
        {
            record_tail++;
        }
        return;
    }
    char line[220];
    sc_text_t text;
    sc_text_init(&text, line, sizeof(line));
    sc_text_string(&text, "type=gnss_ascii v=1 seq=");
    sc_text_u32(&text, reply_sequence);
    sc_text_string(&text, " offset=");
    sc_text_u32(&text, reply_tail);
    sc_text_string(&text, " drops=");
    sc_text_u32(&text, reply_drops);
    sc_text_string(&text, " hex=");
    uint32_t count = reply_head - reply_tail;
    if (count > 64U)
    {
        count = 64U;
    }
    for (uint32_t i = 0; i < count; i++)
    {
        sc_text_hex(&text, reply[(reply_tail + i) & (REPLY_SIZE - 1U)], 2);
    }
    sc_text_string(&text, "\r\n");
    if (text.ok &&
        simpleclock_usb_cdc_try_send_queued((const uint8_t *)line, (uint16_t)text.length))
    {
        reply_tail += count;
        reply_sequence++;
        reply_sent_ms = now_ms;
    }
}

#ifdef SIMPLECLOCK_AUTO_HYBRID
bool gnss_probe_control(const char *line)
{
    if (strncmp(line, "gnss read ", 10) && strncmp(line, "gnss write ", 11) &&
        strncmp(line, "gnss exec ", 10))
    {
        return false;
    }
    const size_t length = strlen(line);
    if (length >= sizeof(control_mailbox) || control_pending || control_kind || control_event ||
        !gnss_probe_commands_idle())
    {
        control_denied++;
        return true;
    }
    memcpy(control_mailbox, line, length + 1U);
    __DMB();
    control_pending = true;
    return true;
}

static bool control_report(uint32_t now_ms, const char *stage, const char *status)
{
    char line[640];
    sc_text_t t;
    sc_text_init(&t, line, sizeof(line));
    sc_text_string(&t, "type=gnss_control v=1 seq=");
    sc_text_u32(&t, control_seq);
    sc_text_string(&t, " uptime_ms=");
    sc_text_u32(&t, now_ms);
    sc_text_string(&t, " stage=");
    sc_text_string(&t, stage);
    sc_text_string(&t, " status=");
    sc_text_string(&t, status);
    sc_text_string(&t, " kind=");
    sc_text_u32(&t, control_kind);
    sc_text_string(&t, " index=");
    sc_text_u32(&t, command_index);
    sc_text_string(&t, " changed=");
    sc_text_u32(&t, control_changed);
    sc_text_string(&t, " saved=");
    sc_text_u32(&t, !strcmp(status, "saved") || !strcmp(status, "defaults_saved") ||
                        (!strcmp(status, "acknowledged") &&
                         !strncmp(control_command, "exeCopyConfigFile", 17)));
    sc_text_string(&t, " pending_reset=");
    sc_text_u32(&t, control_changed && !strncmp(control_command, "setClockSyncThreshold", 21));
    sc_text_string(&t, " ascii_end=");
    sc_text_u32(&t, reply_head);
    sc_text_string(&t, " drops=");
    sc_text_u32(&t, reply_drops - control_drop_start);
    sc_text_string(&t, " rx_loss=");
    sc_text_u32(&t, rx_overflow + rx_errors - control_rx_start);
    sc_text_string(&t, " truncated=");
    sc_text_u32(&t, control_kind == SC_GNSS_CONTROL_WRITE && control_overflow);
    sc_text_string(&t, " denied=");
    sc_text_u32(&t, control_denied);
    sc_text_string(&t, " lost_events=");
    sc_text_u32(&t, control_event_drops);
    sc_text_string(&t, " command_hex=");
    const char *command = command_profile == CONTROL_PROFILE && command_index < command_count
                              ? commands[command_index]
                              : control_command;
    for (unsigned i = 0; command[i] && command[i] != '\r'; ++i)
    {
        sc_text_hex(&t, (uint8_t)command[i], 2);
    }
    sc_text_string(&t, "\r\n");
    return t.ok && simpleclock_usb_cdc_try_send_queued((const uint8_t *)line, (uint16_t)t.length);
}

static void control_begin(uint32_t now_ms)
{
    control_seq++;
    control_started_ms = now_ms;
    control_changed = control_overflow = false;
    control_drop_start = reply_drops;
    control_rx_start = rx_overflow + rx_errors;
    control_command[0] = control_query[0] = '\0';
    control_kind =
        sc_gnss_control_prepare(control_mailbox, control_command, sizeof(control_command),
                                control_query, sizeof(control_query));
    (void)sc_gnss_control_lists_init(&control_lists,
                                     control_kind == SC_GNSS_CONTROL_WRITE ? control_command : "");
    if (control_lists.bad)
    {
        control_kind = SC_GNSS_CONTROL_INVALID;
    }
    const bool ready = control_kind == SC_GNSS_CONTROL_READ ||
                       (hybrid_probe_receiver_change_ready() &&
                        (control_kind != SC_GNSS_CONTROL_WRITE || !(elevation_flags & MASK_FAULT)));
    if (!control_kind || !ready)
    {
        control_result = ready ? "invalid" : "busy";
        control_event = true;
        control_event_ms = now_ms;
        control_kind = SC_GNSS_CONTROL_INVALID;
    }
    else
    {
        command_profile = CONTROL_PROFILE;
        command_index = 0U;
        command_count = control_kind == SC_GNSS_CONTROL_WRITE ? 4U : 1U;
        control_commands[0] =
            control_kind == SC_GNSS_CONTROL_WRITE ? control_query : control_command;
        control_commands[1] = control_command;
        control_commands[2] = control_query;
        control_commands[3] = SAVE_BOOT_COMMAND;
        commands = control_commands;
        command_completed_ms = now_ms;
        control_ready_start = auto_ready;
        control_restore_ready = strncmp(control_command, "setPVTMode", 10) &&
                                strncmp(control_command, "setClockSyncThreshold", 21) &&
                                strncmp(control_command, "setTimingSystem", 15) &&
                                strncmp(control_command, "setSignalTracking", 17) &&
                                strncmp(control_command, "setSignalUsage", 14) &&
                                strncmp(control_command, "exeResetNavFilter", 17);
        if (control_kind == SC_GNSS_CONTROL_WRITE ||
            !strncmp(control_command, "exeResetNavFilter", 17) ||
            !strncmp(control_command, "exeCopyConfigFile", 17))
        {
            auto_manual_override = true;
            auto_retry_scheduled = false;
            mutation_hold = true;
            hybrid_probe_manual_request();
            timing_probe_break(SC_SEG_CONFIG);
            /* Caches are no longer authority for modified receiver settings.
             * Explicit resume re-queries the autonomous contract first. */
            auto_ready = false;
        }
    }
    __DMB();
    control_pending = false;
}

static bool control_complete(uint32_t now_ms)
{
    control_started_ms = now_ms; /* Bound the next stage's wait for USB acceptance. */
    if (transaction.state != SC_CMD_OK)
    {
        control_result = transaction.state == SC_CMD_TIMEOUT ? "timeout" : "rejected";
    }
    else if (control_lists.family && control_lists.bad)
    {
        control_result = "invalid_list_or_dependencies";
    }
    else if (reply_drops != control_drop_start || rx_overflow + rx_errors != control_rx_start ||
             (control_kind == SC_GNSS_CONTROL_WRITE && control_overflow))
    {
        control_result = "incomplete";
    }
    else if (control_kind == SC_GNSS_CONTROL_WRITE && (command_index == 0U || command_index == 2U))
    {
        if (control_lines != 1U)
        {
            control_result = "missing_or_ambiguous_readback";
        }
        else if (command_index == 0U)
        {
            control_preserved = control_line_hash;
            if (control_matches)
            {
                command_index = 2U;
                return true;
            } /* Skip setter, still persist Current. */
            else
            {
                return true;
            }
        }
        else if (!control_matches || control_preserved != control_line_hash)
        {
            control_result = "readback_mismatch";
        }
        else
        {
            return true; /* Verified Current; now persist to receiver Boot. */
        }
    }
    else if (control_kind == SC_GNSS_CONTROL_WRITE && command_index == 3U)
    {
        if (!strncmp(control_command, "setPPSParameters", 16))
        {
            /* Persist the expert's ownership before the normal MCU save. A
             * later boot must not silently replace a deliberate GPS/holdover
             * choice with the automatic RxClock policy. */
            pps_policy = SC_SETTINGS_PPS_MANUAL;
            pps_verified = auto_pps;
            pps_fault = false;
        }
        if (hybrid_probe_receiver_persist(true))
        {
            receiver_user_profile = true;
            control_result = "saved";
            if (!strncmp(control_command, "setElevationMask", 16))
            {
                const char *p = strrchr(control_command, ',') + 1;
                while (*p == ' ')
                {
                    ++p;
                }
                const bool negative = *p == '-';
                if (negative)
                {
                    ++p;
                }
                int degrees = 0;
                while (*p >= '0' && *p <= '9')
                {
                    degrees = degrees * 10 + (*p++ - '0');
                }
                if (negative)
                {
                    degrees = -degrees;
                }
                if (auto_csv_field(control_command, 1U, "PVT"))
                {
                    elevation_pvt = (int8_t)degrees;
                    elevation_flags |= MASK_PVT_KNOWN;
                }
                else
                {
                    elevation_tracking = (int8_t)degrees;
                    elevation_flags |= MASK_TRACKING_KNOWN;
                }
                elevation_flags = (uint8_t)((elevation_flags | MASK_SAVED) & ~MASK_FAULT);
            }
        }
        else
        {
            control_result = "mcu_save_failed";
        }
    }
    else if (control_kind == SC_GNSS_CONTROL_WRITE)
    {
        control_changed = true;
        return true;
    }
    else
    {
        control_result = control_kind == SC_GNSS_CONTROL_READ ? "complete" : "acknowledged";
    }
    command_index = command_count;
    if (!strcmp(control_result, "saved") && !control_restore_ready)
    {
        requested_profile = AUTO_RESTORE_PROFILE;
    }
    if (!strcmp(control_result, "unchanged") ||
        (control_restore_ready &&
         (!strcmp(control_result, "saved") || !strcmp(control_result, "acknowledged"))))
    {
        auto_ready = control_ready_start;
    }
    control_event = true;
    control_event_ms = now_ms;
    return false;
}
#else
bool gnss_probe_control(const char *line)
{
    (void)line;
    return false;
}
#endif

#ifdef SIMPLECLOCK_AUTO_HYBRID
static void startup_ack_service(uint32_t now_ms)
{
    const bool fresh = status_seen && status_epoch_valid && (uint32_t)(now_ms - status_ms) <= 1500U;
    if (startup_ack == STARTUP_ACK_WAIT_STATUS)
    {
        /* A command echo/prompt does not prove that the receiver recovered.
         * A lost/rejected reply can still have acknowledged the native latch.
         * Stop waiting after 5 s; normal receiver freshness/error gates still
         * inhibit discipline, without adding a permanent MCU-side fault. */
        if ((fresh && status_rx_error == 0U && status_ms != startup_ack_completed_ms &&
             (uint32_t)(status_ms - startup_ack_completed_ms) < UINT32_C(0x80000000)) ||
            (uint32_t)(now_ms - startup_ack_completed_ms) >= 5000U)
        {
            startup_ack = STARTUP_ACK_DONE;
        }
    }
    if (startup_ack != STARTUP_ACK_ARMED || !auto_ready || !auto_pvt_verified || auto_fault ||
        (elevation_flags & MASK_FAULT) || !fresh)
    {
        return;
    }
    if (status_rx_error == 0U)
    {
        startup_ack = STARTUP_ACK_DONE; /* A healthy startup needs no acknowledgement. */
        return;
    }
    /* G5 1.1.0 ReceiverStatus: bit 3 is SOFTWARE. The native Error read
     * clears that latch as a whole; it cannot select a particular PLL entry.
     * Never acknowledge while another error or a missing reference is present. */
    if (status_rx_error != 8U || !(status_rx_state & SC_SBF_RX_STATE_EXT_FREQ))
    {
        return;
    }
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (gnss_probe_commands_idle() && !ppp_restore_pending)
    {
        /* Claim and start the existing native UART owner atomically. Unlike
         * the expert host bridge, this path has no USB-report prerequisite. */
        startup_ack = STARTUP_ACK_INFLIGHT;
        command_profile = GNSS_PROFILE_STARTUP_ERROR_ACK;
        commands = health_commands;
        command_count = COMMAND_COUNT(health_commands);
        command_index = command_tx_index = 0U;
        command_active =
            sc_gnss_command_start_checked(&transaction, commands[0], SC_CMD_EXPECT_NONE, now_ms);
        if (!command_active)
        {
            startup_ack = STARTUP_ACK_DONE;
            command_index = command_count;
        }
    }
    __set_PRIMASK(mask);
}

static void pps_policy_fail(uint32_t now_ms)
{
    pps_verified = false;
    auto_pps = false;
    pps_fault = true;
    if (pps_retries < 3U)
    {
        pps_retries++;
    }
    pps_retry_due_ms = now_ms + 30000U;
    command_index = command_count;
}

static void pps_policy_service(uint32_t now_ms)
{
    if (pps_policy == SC_SETTINGS_PPS_MANUAL || pps_verified ||
        (pps_fault &&
         (pps_retries >= 3U || (uint32_t)(now_ms - pps_retry_due_ms) >= UINT32_C(0x80000000))) ||
        !gnss_probe_commands_idle())
    {
        return;
    }
    pps_fault = false;
    pps_policy_commands[1] = pps_policy_setter();
    command_profile = GNSS_PROFILE_PPS_POLICY;
    commands = pps_policy_commands;
    command_count = COMMAND_COUNT(pps_policy_commands);
    command_index = 0U;
    command_completed_ms = now_ms;
    pps_policy_rx_start = rx_overflow + rx_errors;
}

static void pps_policy_complete(uint32_t now_ms)
{
    if (transaction.state != SC_CMD_OK || rx_overflow + rx_errors != pps_policy_rx_start ||
        ((command_index == 0U || command_index == 2U) && pps_policy_rows != 1U))
    {
        if (transaction.state == SC_CMD_TIMEOUT)
        {
            command_timeouts++;
        }
        else
        {
            reply_rejects++;
            transaction.state = SC_CMD_REJECTED;
        }
        pps_policy_fail(now_ms);
        return;
    }
    reply_acks++;
    if (command_index == 0U)
    {
        pps_policy_preserved = pps_policy_row_hash;
        if (pps_policy_matches)
        {
            command_index = command_count; /* Matching fresh Current: no setter. */
            pps_retries = 0U;
            return;
        }
    }
    else if (command_index == 2U)
    {
        if (!pps_policy_matches || pps_policy_preserved != pps_policy_row_hash)
        {
            reply_rejects++;
            transaction.state = SC_CMD_REJECTED;
            pps_policy_fail(now_ms);
            return;
        }
        pps_retries = 0U;
    }
    command_index++;
}

#endif

void gnss_probe_service(uint32_t now_ms)
{
    /* Bound RX work so parsing cannot monopolize watchdog/PPS service. */
    const uint32_t gap = now_ms - last_service_ms;
    last_service_ms = now_ms;
    if (gap > max_service_gap_ms)
    {
        max_service_gap_ms = gap;
    }
#ifdef SIMPLECLOCK_AUTO_HYBRID
    receive_dma_sync(gap);
#endif
    sc_sbf_expire(&sbf, now_ms, 115200U);
    for (unsigned i = 0; i < 128U && rx_tail != rx_head; i++)
    {
        const uint8_t byte = rx[rx_tail & (RX_SIZE - 1U)];
        __DMB();
        rx_tail++;
        sc_sbf_feed(&sbf, &byte, 1, now_ms, receive_sbf_frame, receive_ascii_byte, NULL);
    }
    const uint32_t faults = rx_overflow + rx_errors + sbf.crc_errors + sbf.length_errors +
                            sbf.timeouts + sbf.structure_errors + decode_bad + decode_revision;
    if (faults != input_faults)
    {
        timing_probe_break(SC_SEG_LOSS);
        input_faults = faults;
    }
#ifdef SIMPLECLOCK_AUTO_HYBRID
    startup_ack_service(now_ms);
    pps_policy_service(now_ms);
    pps_report_service(now_ms);
#endif
    if (command_active)
    {
        if (transaction.state == SC_CMD_WAITING &&
            commands[command_index][command_tx_index] != '\0' &&
            (USART2->ISR & USART_ISR_TXE_TXFNF) != 0U)
        {
            USART2->TDR = (uint8_t)commands[command_index][command_tx_index++];
        }
        sc_gnss_command_expire(&transaction, now_ms);
#ifdef SIMPLECLOCK_AUTO_HYBRID
        if (command_profile == CONTROL_PROFILE && (uint32_t)(now_ms - control_started_ms) > 60000U)
        {
            transaction.state = SC_CMD_TIMEOUT;
        }
#endif
        if (command_profile == GNSS_PROFILE_REBOOT &&
            commands[command_index][command_tx_index] == '\0' && (USART2->ISR & USART_ISR_TC) != 0U)
        {
            reboot_sent = true;
        }
        if (transaction.state != SC_CMD_WAITING)
        {
            if (command_profile == GNSS_PROFILE_REBOOT)
            {
                reboot_failed = transaction.rejected || transaction.state == SC_CMD_REJECTED ||
                                commands[command_index][command_tx_index] != '\0';
            }
            command_active = false;
            command_completed_ms = now_ms;
#ifdef SIMPLECLOCK_AUTO_HYBRID
            if (command_profile == GNSS_PROFILE_STARTUP_ERROR_ACK)
            {
                startup_ack = STARTUP_ACK_WAIT_STATUS;
                /* Use the same clock as the status callback, after RX parsing;
                 * the service argument can precede a tick during that work. */
                startup_ack_completed_ms = HAL_GetTick();
            }
            if (command_profile == CONTROL_PROFILE)
            {
                if (control_complete(now_ms))
                {
                    command_index++;
                }
                goto control_service_done;
            }
            if (command_profile == GNSS_PROFILE_PPS_POLICY)
            {
                pps_policy_complete(now_ms);
                goto control_service_done;
            }
            if (command_profile == GNSS_PROFILE_DEFAULTS &&
                (command_index == 0U || command_index == 4U) && defaults_transport != 1U)
            {
                transaction.state = SC_CMD_REJECTED;
            }
            if (command_profile == GNSS_PROFILE_AUTO_HYBRID)
            {
                /* COM1> is not newline terminated; discard it before the
                 * next command's echoed line is accumulated. */
                auto_ascii_used = 0U;
            }
#endif
            if (transaction.state == SC_CMD_OK)
            {
                reply_acks++;
                if ((elevation_profile(command_profile) ||
                     command_profile == GNSS_PROFILE_DEFAULTS) &&
                    transaction.readback.elevation_valid)
                {
                    const bool tracking =
                        transaction.expected == SC_CMD_EXPECT_ELEVATION_TRACKING ||
                        (unsigned)transaction.expected >= SC_CMD_EXPECT_TRACKING_MASK_BASE;
                    if (tracking)
                    {
                        elevation_tracking = transaction.readback.elevation_deg;
                        elevation_flags |= MASK_TRACKING_KNOWN;
                    }
                    else
                    {
                        elevation_pvt = transaction.readback.elevation_deg;
                        elevation_flags |= MASK_PVT_KNOWN;
                    }
                }
                bool advance = true;
#ifdef SIMPLECLOCK_AUTO_HYBRID
                if (command_profile == GNSS_PROFILE_AUTO_HYBRID &&
                    !auto_command_readback_ok(command_index))
                {
                    auto_fail(now_ms);
                    command_index = command_count;
                    advance = false;
                }
#endif
                if (advance)
                {
                    command_index++;
                    if (command_index == command_count
#ifdef SIMPLECLOCK_AUTO_HYBRID
                        || (command_profile == GNSS_PROFILE_AUTO_HYBRID && receiver_user_profile &&
                            command_index == AUTO_SAVE_BOOT)
#endif
                    )
                    {
                        command_index = command_count;
#ifdef SIMPLECLOCK_AUTO_HYBRID
                        if (command_profile == GNSS_PROFILE_DEFAULTS)
                        {
                            receiver_user_profile = false;
                            pps_policy = SC_SETTINGS_PPS_CONTINUOUS;
                            pps_verified = pps_fault = false;
                            pps_retries = 0U;
                            elevation_flags = MASK_PVT_KNOWN | MASK_TRACKING_KNOWN;
                            receiver_selection = SC_SETTINGS_GNSS_AUTO;
                            requested_profile = AUTO_RESTORE_PROFILE;
                        }
#endif
                        if (elevation_mutation(command_profile))
                        {
                            elevation_flags = MASK_PVT_KNOWN | MASK_TRACKING_KNOWN | MASK_SAVED;
#ifdef SIMPLECLOCK_AUTO_HYBRID
                            if (hybrid_probe_receiver_persist(true))
                            {
                                receiver_user_profile = true;
                            }
                            else
                            {
                                elevation_flags |= MASK_FAULT;
                            }
#endif
                        }
                        else if (command_profile == GNSS_PROFILE_SAVE_BOOT)
                        {
                            elevation_flags |= MASK_SAVED;
                        }
                        else if (command_profile == GNSS_PROFILE_PPP_ENABLE)
                        {
                            ppp_configured = true;
                        }
                        else if (command_profile == GNSS_PROFILE_PPP_RESTORE)
                        {
                            ppp_restore_pending = ppp_configured = false;
                        }
#ifdef SIMPLECLOCK_AUTO_HYBRID
                        else if (bench_profile(command_profile))
                        {
                            auto_pvt_verified = transaction.readback.matched;
                            auto_static_pvt = false; /* Legacy shortcuts explicitly set Rover. */
                            receiver_selection =
                                command_profile == GNSS_PROFILE_NO_HAS ? SC_SETTINGS_GNSS_SBAS
                                : command_profile == GNSS_PROFILE_HAS  ? SC_SETTINGS_GNSS_HAS
                                                                      : SC_SETTINGS_GNSS_STANDALONE;
                            if (hybrid_probe_receiver_persist(true))
                            {
                                receiver_user_profile = true;
                            }
                            else
                            {
                                auto_pvt_verified = false;
                                auto_fault = true;
                            }
                        }
                        else if (command_profile == GNSS_PROFILE_AUTO_HYBRID)
                        {
                            auto_ready =
                                auto_pps && auto_clock_free && auto_time_source &&
                                auto_timing_auto && auto_data_sbf && auto_stream &&
                                auto_capability && auto_tracking && auto_usage && auto_ppp &&
                                (receiver_user_profile ? auto_seen_dynamics : auto_dynamics_static);
                            if (auto_ready && !receiver_user_profile)
                            {
                                if (defaults_pending)
                                {
                                    hybrid_probe_defaults_finish(true);
                                }
                                if (hybrid_probe_receiver_persist(true))
                                {
                                    receiver_user_profile = true;
                                }
                                else
                                {
                                    auto_ready = false;
                                    auto_manual_override = true;
                                }
                            }
                            if (defaults_pending && auto_ready)
                            {
                                control_seq++;
                                control_kind = SC_GNSS_CONTROL_INVALID;
                                control_event = true;
                                control_event_ms = now_ms;
                                control_result = "defaults_saved";
                                control_changed = true;
                                elevation_flags |= MASK_SAVED;
                                control_command[0] = control_query[0] = '\0';
                                control_drop_start = reply_drops;
                                control_rx_start = rx_overflow + rx_errors;
                                defaults_pending = false;
                            }
                            auto_fault = !auto_ready;
                            if (!auto_ready)
                            {
                                auto_fail(now_ms);
                            }
                            if (auto_ready)
                            {
                                auto_retry_count = 0U;
                            }
                        }
#endif
                    }
                }
            }
            else
            {
                if (transaction.state == SC_CMD_TIMEOUT)
                {
                    command_timeouts++;
                }
                else
                {
                    reply_rejects++;
                }
                command_index = command_count; /* stop the entire profile on error */
#ifdef SIMPLECLOCK_AUTO_HYBRID
                if (command_profile == GNSS_PROFILE_AUTO_HYBRID)
                {
                    auto_fail(now_ms);
                }
                else if (command_profile == GNSS_PROFILE_DEFAULTS)
                {
                    auto_fail(now_ms);
                }
#endif
            }
        }
    }
    else
    {
#ifdef SIMPLECLOCK_AUTO_HYBRID
        if (control_event)
        {
            const bool sent = control_report(now_ms, "done", control_result);
            if (sent || (uint32_t)(now_ms - control_event_ms) >= 3000U)
            {
                control_event_drops += sent ? 0U : 1U;
                control_event = false;
                control_kind = SC_GNSS_CONTROL_INVALID;
            }
            goto control_service_done;
        }
        if (control_denied != control_denied_sent && !control_pending && !requested_profile &&
            command_index >= command_count)
        {
            if (control_report(now_ms, "denied", "busy"))
            {
                control_denied_sent = control_denied;
            }
        }
        if (control_pending)
        {
            control_begin(now_ms);
        }
        if (control_event)
        {
            goto control_service_done;
        }
        if (reboot_recovering && command_profile == GNSS_PROFILE_REBOOT &&
            command_index >= command_count && (uint32_t)(now_ms - command_completed_ms) >= 3000U)
        {
            /* A reset may remove its own prompt. Never retransmit it after
             * timeout: wait, then use fresh normal configuration readbacks. */
            reboot_recovering = false;
            requested_profile = AUTO_RESTORE_PROFILE;
        }
#endif
#ifdef SIMPLECLOCK_AUTO_HYBRID
        if (!control_kind && auto_retry_scheduled && !auto_manual_override &&
            (uint32_t)(now_ms - auto_retry_due_ms) < 0x80000000U)
        {
            auto_retry_scheduled = false;
            if (receiver_user_profile || auto_retry_count < AUTO_MAX_RETRIES)
            {
                if (auto_retry_count < AUTO_MAX_RETRIES)
                {
                    auto_retry_count++;
                }
                requested_profile = AUTO_RESTORE_PROFILE;
            }
        }
#endif
        if (requested_profile && now_ms >= 2000U)
        {
            const uint32_t mask = __get_PRIMASK();
            __disable_irq();
            command_profile = requested_profile;
            requested_profile = 0U;
#ifdef SIMPLECLOCK_AUTO_HYBRID
            if (command_profile == AUTO_RESTORE_PROFILE)
            {
                command_profile = GNSS_PROFILE_AUTO_HYBRID;
            }
            else if (command_profile == GNSS_PROFILE_AUTO_HYBRID)
            {
                receiver_selection = SC_SETTINGS_GNSS_AUTO;
                receiver_user_profile = false; /* Explicit standard profile request. */
            }
#endif
            __set_PRIMASK(mask);
            switch (command_profile)
            {
            case GNSS_PROFILE_TRIAL_STREAM:
                commands = trial_stream_commands;
                command_count = COMMAND_COUNT(trial_stream_commands);
                break;
            case GNSS_PROFILE_SURVEY:
                commands = survey_commands;
                command_count = COMMAND_COUNT(survey_commands);
                break;
            case GNSS_PROFILE_ONCE:
                commands = once_commands;
                command_count = COMMAND_COUNT(once_commands);
                break;
            case GNSS_PROFILE_STREAM:
                commands = stream_commands;
                command_count = COMMAND_COUNT(stream_commands);
                break;
            case GNSS_PROFILE_STOP:
                commands = stop_commands;
                command_count = COMMAND_COUNT(stop_commands);
                break;
            case GNSS_PROFILE_HEALTH:
                commands = health_commands;
                command_count = COMMAND_COUNT(health_commands);
                break;
            case GNSS_PROFILE_E6_SURVEY:
                commands = e6_survey_commands;
                command_count = COMMAND_COUNT(e6_survey_commands);
                break;
            case GNSS_PROFILE_E6_STREAM:
                commands = e6_stream_commands;
                command_count = COMMAND_COUNT(e6_stream_commands);
                break;
            case GNSS_PROFILE_E6_STOP:
                commands = e6_stop_commands;
                command_count = COMMAND_COUNT(e6_stop_commands);
                break;
            case GNSS_PROFILE_PPS:
                commands = pps_commands;
                command_count = COMMAND_COUNT(pps_commands);
                break;
            case GNSS_PROFILE_PPS_OFF:
                commands = pps_off_commands;
                command_count = COMMAND_COUNT(pps_off_commands);
                break;
            case GNSS_PROFILE_PPP_SURVEY:
                commands = ppp_survey_commands;
                command_count = COMMAND_COUNT(ppp_survey_commands);
                break;
            case GNSS_PROFILE_PPP_ENABLE:
                commands = ppp_enable_commands;
                command_count = COMMAND_COUNT(ppp_enable_commands);
                break;
            case GNSS_PROFILE_PPP_RESTORE:
                commands = ppp_restore_commands;
                command_count = COMMAND_COUNT(ppp_restore_commands);
                break;
            case GNSS_PROFILE_NO_HAS:
                commands = no_has_commands;
                command_count = COMMAND_COUNT(no_has_commands);
                break;
            case GNSS_PROFILE_HAS:
                commands = has_commands;
                command_count = COMMAND_COUNT(has_commands);
                break;
            case GNSS_PROFILE_STANDALONE:
                commands = standalone_commands;
                command_count = COMMAND_COUNT(standalone_commands);
                break;
            case GNSS_PROFILE_ELEVATION_SURVEY:
                commands = elevation_queries;
                command_count = COMMAND_COUNT(elevation_queries);
                break;
            case GNSS_PROFILE_ELEVATION_RESET:
                commands = elevation_reset_commands;
                command_count = COMMAND_COUNT(elevation_reset_commands);
                break;
            case GNSS_PROFILE_SAVE_BOOT:
                commands = save_boot_commands;
                command_count = COMMAND_COUNT(save_boot_commands);
                break;
#ifdef SIMPLECLOCK_AUTO_HYBRID
            case GNSS_PROFILE_DEFAULTS:
                commands = defaults_commands;
                command_count = COMMAND_COUNT(defaults_commands);
                defaults_transport = false;
                defaults_pending = true;
                auto_manual_override = true;
                auto_retry_scheduled = false;
                mutation_hold = true;
                auto_ready = false;
                hybrid_probe_manual_request();
                timing_probe_break(SC_SEG_CONFIG);
                if (!hybrid_probe_receiver_change_ready())
                {
                    command_count = 0U;
                    auto_fail(now_ms);
                }
                break;
#endif
            case GNSS_PROFILE_REBOOT:
                commands = reboot_commands;
                command_count = COMMAND_COUNT(reboot_commands);
                break;
#ifdef SIMPLECLOCK_AUTO_HYBRID
            case GNSS_PROFILE_AUTO_HYBRID:
                commands = auto_hybrid_commands[receiver_selection];
                command_count = COMMAND_COUNT(auto_hybrid_commands[0]);
                auto_ready = auto_fault = false;
                auto_pps = auto_clock_free = auto_time_source = auto_timing_auto = false;
                auto_data_sbf = auto_stream = auto_capability = auto_tracking = auto_usage = false;
                auto_pvt_verified = auto_ppp = auto_static_pvt = false;
                auto_seen_pps = auto_seen_clock = auto_seen_time_source = auto_seen_timing = false;
                auto_seen_data = auto_seen_stream = auto_seen_capability = auto_seen_tracking =
                    false;
                auto_seen_usage = auto_seen_pvt = false;
                auto_seen_dynamics = auto_dynamics_static = false;
                auto_ascii_used = 0U;
                auto_ascii_overflow = false;
                auto_pps_response[0] = '\0';
                mutation_hold = false;
                auto_retry_scheduled = false;
                auto_manual_override = false;
                status_seen = false;
                status_epoch_valid = false;
                break;
#endif
            default:
                if (gnss_probe_elevation_set(command_profile))
                {
                    commands = elevation_commands;
                    command_count = COMMAND_COUNT(elevation_commands);
                }
                else
                {
                    commands = identify_commands;
                    command_count = COMMAND_COUNT(identify_commands);
                }
                break;
            }
            command_index = 0;
            command_completed_ms = now_ms;
            if (elevation_profile(command_profile))
            {
                /* Keep an interrupted mutation fault across read-only queries. */
                elevation_flags &= MASK_FAULT;
                if (elevation_mutation(command_profile))
                {
                    elevation_flags |= MASK_FAULT;
                }
            }
            else if (command_profile == GNSS_PROFILE_SAVE_BOOT)
            {
                elevation_flags &= UINT8_MAX ^ MASK_SAVED;
            }
            if ((command_profile == GNSS_PROFILE_PPP_ENABLE && ppp_restore_pending) ||
                (command_profile == GNSS_PROFILE_PPP_RESTORE && !ppp_restore_pending))
            {
                ppp_denied++;
                command_index = command_count;
                transaction.state = SC_CMD_REJECTED;
            }
        }
        /* USB diagnostics are best effort: a missing/slow host may fill reply[]
         * but must never stall receiver configuration or a checked retry.
         * receive_ascii_byte always parses command/readback state independently
         * of that queue; overflow is counted by reply_drops. */
        if (command_index < command_count && now_ms >= 2000U &&
            (uint32_t)(now_ms - command_completed_ms) >= 100U)
        {
#ifdef SIMPLECLOCK_AUTO_HYBRID
            if (command_profile == GNSS_PROFILE_PPS_POLICY &&
                (command_index == 0U || command_index == 2U))
            {
                pps_policy_rows = 0U;
                pps_policy_matches = false;
            }
            if (bench_profile(command_profile) && (elevation_flags & MASK_FAULT))
            {
                transaction.state = SC_CMD_REJECTED;
                command_index = command_count;
                goto control_service_done;
            }
            if (command_profile == GNSS_PROFILE_AUTO_HYBRID && command_index == AUTO_SAVE_BOOT &&
                (elevation_flags & MASK_FAULT))
            {
                /* Configuration checks may complete, but never persist an
                 * interrupted mask change as an unrelated setup side effect. */
                auto_ready = auto_pps && auto_clock_free && auto_time_source && auto_timing_auto &&
                             auto_data_sbf && auto_stream && auto_capability && auto_tracking &&
                             auto_usage && auto_ppp && auto_dynamics_static;
                auto_fault = !auto_ready;
                auto_retry_scheduled = false;
                command_index = command_count;
                goto control_service_done;
            }
            if (command_profile == CONTROL_PROFILE)
            {
                control_lines = 0U;
                control_matches = false;
                if (control_lists.family && (command_index == 0U || command_index == 2U))
                {
                    sc_gnss_control_lists_start(&control_lists, command_index == 0U);
                }
                if (!control_report(now_ms,
                                    command_index == 0U   ? "before"
                                    : command_index == 1U ? "write"
                                    : command_index == 2U ? "after"
                                                          : "save",
                                    "started"))
                {
                    if ((uint32_t)(now_ms - control_started_ms) >= 3000U)
                    {
                        /* Never execute a request left waiting for a vanished
                         * host. A partly applied edit remains held/incomplete;
                         * no automatic setter/save/reset retry is introduced. */
                        control_result = "host_unavailable";
                        command_index = command_count;
                        control_event = true;
                        control_event_ms = now_ms;
                    }
                    goto control_service_done;
                }
                if (command_index == 1U && control_kind == SC_GNSS_CONTROL_WRITE &&
                    !strncmp(control_command, "setElevationMask", 16))
                {
                    elevation_flags = (uint8_t)((elevation_flags | MASK_FAULT) & ~MASK_SAVED);
                }
            }
            if (command_profile == GNSS_PROFILE_DEFAULTS && command_index == 4U)
            {
                defaults_transport = false;
            }
            if (command_profile == GNSS_PROFILE_DEFAULTS && command_index == 2U)
            {
                elevation_flags = MASK_FAULT;
            }
            if (command_profile == GNSS_PROFILE_AUTO_HYBRID && auto_skip_command(command_index))
            {
                command_index++;
                command_completed_ms = now_ms;
                goto auto_command_schedule_done;
            }
#endif
            /* Recheck immediately before the PPS setter, never auto-enable on
             * startup or merely because an old FINETIME bit was observed. */
            if (command_profile == GNSS_PROFILE_PPS && command_index == 0U &&
                !timing_probe_pps_ready(now_ms))
            {
                pps_denied++;
                command_index = command_count;
                transaction.state = SC_CMD_REJECTED;
                return;
            }
            if (command_index == 0U &&
                (command_profile == GNSS_PROFILE_TRIAL_STREAM ||
                 command_profile == GNSS_PROFILE_STREAM || command_profile == GNSS_PROFILE_STOP ||
                 command_profile == GNSS_PROFILE_E6_STREAM ||
                 command_profile == GNSS_PROFILE_E6_STOP || command_profile == GNSS_PROFILE_PPS ||
                 command_profile == GNSS_PROFILE_PPS_OFF))
            {
                timing_probe_break(SC_SEG_CONFIG);
                mutation_hold = true;
#ifdef SIMPLECLOCK_AUTO_HYBRID
                hybrid_probe_manual_request();
#endif
            }
            if (bench_profile(command_profile) &&
                command_index ==
                    (command_profile == GNSS_PROFILE_STANDALONE ? 1U : BENCH_SET_PVT_INDEX))
            {
                /* Supersede legacy restore ownership only at the actual setter.
                 * A lost reply must neither resume clocks nor retry mutation. */
                ppp_restore_pending = ppp_configured = false;
                mutation_hold = true;
                timing_probe_break(SC_SEG_CONFIG);
#ifdef SIMPLECLOCK_AUTO_HYBRID
                hybrid_probe_manual_request();
#endif
            }
            if (elevation_mutation(command_profile) && command_index == 2U)
            {
                /* The two checked preflight queries have completed. The other
                 * engine is kept as the exact post-mutation readback target. */
                if (command_profile == GNSS_PROFILE_ELEVATION_RESET)
                {
                    elevation_pvt = elevation_tracking = -90;
                }
                else
                {
                    const bool tracking = gnss_probe_elevation_tracking(command_profile);
                    const int degrees = gnss_probe_elevation_degrees(command_profile);
                    if (tracking)
                    {
                        elevation_tracking = (int8_t)degrees;
                    }
                    else
                    {
                        elevation_pvt = (int8_t)degrees;
                    }
                    sc_text_t setter;
                    sc_text_init(&setter, command_scratch, sizeof(command_scratch));
                    sc_text_string(&setter, tracking ? "setElevationMask, Tracking, "
                                                     : "setElevationMask, PVT, ");
                    sc_text_i32(&setter, degrees);
                    sc_text_string(&setter, "\r\n");
                    if (!setter.ok)
                    {
                        transaction.state = SC_CMD_REJECTED;
                        command_index = command_count;
                        return;
                    }
                }
                elevation_flags = MASK_FAULT; /* Values are targets until verified. */
                mutation_hold = true;
                timing_probe_break(SC_SEG_CONFIG);
#ifdef SIMPLECLOCK_AUTO_HYBRID
                hybrid_probe_manual_request();
#endif
            }
            if ((command_profile == GNSS_PROFILE_PPP_ENABLE &&
                 command_index == PPP_ENABLE_SET_PVT_INDEX) ||
                (command_profile == GNSS_PROFILE_PPP_RESTORE &&
                 command_index == PPP_RESTORE_SET_PVT_INDEX))
            {
                ppp_restore_pending = true;
                ppp_configured = false;
                timing_probe_break(
                    SC_SEG_CONFIG); /* A PVT/TimeSystem change starts a new segment. */
                mutation_hold = true;
#ifdef SIMPLECLOCK_AUTO_HYBRID
                hybrid_probe_manual_request();
#endif
            }
#ifdef SIMPLECLOCK_AUTO_HYBRID
            if (command_profile == GNSS_PROFILE_AUTO_HYBRID && auto_mutating_command(command_index))
            {
                if (command_index == AUTO_SET_CLOCK && !auto_clock_steering)
                {
                    auto_fail(now_ms);
                    command_index = command_count;
                    goto auto_command_schedule_done;
                }
                if (command_index == AUTO_SET_PVT &&
                    !(auto_capability && auto_tracking && auto_usage && auto_pvt_verified))
                {
                    auto_fault = true;
                    mutation_hold = true;
                    command_index = command_count;
                    goto auto_command_schedule_done;
                }
                mutation_hold = true;
                timing_probe_break(SC_SEG_CONFIG);
            }
#endif
            command_tx_index = 0;
#ifdef SIMPLECLOCK_AUTO_HYBRID
            control_started_ms = now_ms;
            if (command_profile == GNSS_PROFILE_AUTO_HYBRID)
            {
                auto_ascii_used = 0U;
                auto_ascii_overflow = false;
                auto_reply_seen = auto_reply_ambiguous = false;
            }
#endif
            command_active = sc_gnss_command_start_checked(
                &transaction, commands[command_index],
                command_expectation(command_profile, command_index), now_ms);
        }
    }
#ifdef SIMPLECLOCK_AUTO_HYBRID
auto_command_schedule_done:
control_service_done:
#endif
    service_reply_queues(now_ms);
}

static void append_u32_field(sc_text_t *text, const char *key, uint32_t value)
{
    sc_text_string(text, key);
    sc_text_u32(text, value);
}

void gnss_probe_report(uint32_t now_ms)
{
    char line[SIMPLECLOCK_TMP117_USB_TX_MAX];
    sc_text_t text;
    sc_text_init(&text, line, sizeof(line));
    /* v4: pps_snapshot carries the receiver CSV without spaces. */
    sc_text_string(&text, "type=gnss_probe v=4");
    append_u32_field(&text, " uptime_ms=", now_ms);
    append_u32_field(&text, " baud=", 115200);
    append_u32_field(&text, " rx_bytes=", rx_bytes);
    append_u32_field(&text, " rx_peak=", rx_peak);
    append_u32_field(&text, " rx_overflow=", rx_overflow);
    append_u32_field(&text, " rx_errors=", rx_errors);
    append_u32_field(&text, " service_gap_ms=", max_service_gap_ms);
    append_u32_field(&text, " reply_drops=", reply_drops);
    append_u32_field(&text, " acks=", reply_acks);
    append_u32_field(&text, " rejects=", reply_rejects);
    append_u32_field(&text, " timeouts=", command_timeouts);
    append_u32_field(&text, " sbf_frames=", sbf.frames);
    append_u32_field(&text, " crc_errors=", sbf.crc_errors);
    append_u32_field(&text, " length_errors=", sbf.length_errors);
    append_u32_field(&text, " frame_timeouts=", sbf.timeouts);
    append_u32_field(&text, " fields_decoded=", decode_ok);
    append_u32_field(&text, " unknown=", decode_unknown);
    append_u32_field(&text, " unsupported_rev=", decode_revision);
    append_u32_field(&text, " invalid_length=", decode_bad);
    append_u32_field(&text, " structure_errors=", sbf.structure_errors);
    append_u32_field(&text, " record_drops=", record_drops);
    append_u32_field(&text, " command_profile=", command_profile);
    append_u32_field(&text, " command_index=", command_index);
    append_u32_field(&text, " command_state=", transaction.state);
    sc_text_string(&text, " ext_freq=");
    if (status_seen && (uint32_t)(now_ms - status_ms) <= 1500U)
    {
        sc_text_u32(&text, (status_rx_state & 4U) != 0U ? 1U : 0U);
    }
    else
    {
        sc_text_string(&text, "unknown");
    }
    append_u32_field(&text, " status_age_ms=", status_seen ? now_ms - status_ms : UINT32_MAX);
    append_u32_field(&text, " pps_denied=", pps_denied);
    append_u32_field(&text, " ppp_configured=", ppp_configured ? 1U : 0U);
    append_u32_field(&text, " ppp_restore_pending=", ppp_restore_pending ? 1U : 0U);
    append_u32_field(&text, " ppp_denied=", ppp_denied);
    append_u32_field(&text, " auto_ready=", gnss_probe_auto_ready() ? 1U : 0U);
    append_u32_field(&text, " auto_fault=", gnss_probe_auto_fault() ? 1U : 0U);
    append_u32_field(&text, " mutation_hold=", gnss_probe_mutation_hold() ? 1U : 0U);
    gnss_probe_pps_snapshot_t pps_snapshot;
    if (gnss_probe_pps_snapshot(&pps_snapshot))
    {
        /* A key=value field never contains a space or '='; the receiver's
         * ", " separators shrink to ",". Anything unprintable becomes '?'. */
        size_t used = 0U;
        for (const char *p = pps_snapshot.response; *p; p++)
        {
            const char c = *p;
            if (c != ' ')
            {
                pps_snapshot.response[used++] = (char)(c > ' ' && c < '\x7f' && c != '=' ? c : '?');
            }
        }
        pps_snapshot.response[used] = '\0';
        sc_text_string(&text, " pps_snapshot=");
        sc_text_string(&text, pps_snapshot.response);
    }
    else
    {
        sc_text_string(&text, " pps_snapshot=unknown");
    }
    append_u32_field(&text, " readback_expected=", (uint32_t)transaction.expected);
    append_u32_field(&text, " readback_ok=", transaction.readback.matched ? 1U : 0U);
    if (elevation_flags != 0U || elevation_profile(command_profile) ||
        command_profile == GNSS_PROFILE_SAVE_BOOT)
    {
        sc_text_string(&text, " pvt_mask=");
        if (elevation_flags & MASK_PVT_KNOWN)
        {
            sc_text_i32(&text, elevation_pvt);
        }
        else
        {
            sc_text_string(&text, "unknown");
        }
        sc_text_string(&text, " tracking_mask=");
        if (elevation_flags & MASK_TRACKING_KNOWN)
        {
            sc_text_i32(&text, elevation_tracking);
        }
        else
        {
            sc_text_string(&text, "unknown");
        }
        append_u32_field(&text, " mask_saved=", (elevation_flags & MASK_SAVED) != 0U);
        append_u32_field(&text, " mask_fault=", (elevation_flags & MASK_FAULT) != 0U);
    }
    sc_text_string(&text, "\r\n");
    if (text.ok)
    {
        (void)simpleclock_usb_cdc_try_send((const uint8_t *)line, (uint16_t)text.length);
    }
#ifdef SIMPLECLOCK_AUTO_HYBRID
    /* The single CDC buffer still holds the status line: send gnss_pps from a
     * later service pass instead of losing it to BUSY. */
    pps_report_due = true;
    pps_report_ms = now_ms;
#endif
}

#ifdef SIMPLECLOCK_AUTO_HYBRID
/* Separate record: the status line has no budget for policy fields, and an
 * oversized line is dropped whole rather than truncated. Retried each service
 * pass while BUSY; one still unsent after 500 ms is counted as dropped. */
static void pps_report_service(uint32_t now_ms)
{
    if (!pps_report_due)
    {
        return;
    }
    if ((uint32_t)(now_ms - pps_report_ms) > 500U)
    {
        pps_report_due = false;
        pps_report_drops++;
        return;
    }
    gnss_probe_pps_snapshot_t pps_snapshot;
    (void)gnss_probe_pps_snapshot(&pps_snapshot);
    char line[SIMPLECLOCK_TMP117_USB_TX_MAX];
    sc_text_t text;
    sc_text_init(&text, line, sizeof(line));
    sc_text_string(&text, "type=gnss_pps v=1");
    append_u32_field(&text, " uptime_ms=", pps_report_ms);
    append_u32_field(&text, " policy=", pps_snapshot.policy);
    append_u32_field(&text, " policy_valid=", pps_snapshot.policy_valid);
    append_u32_field(&text, " enabled=", pps_snapshot.enabled);
    append_u32_field(&text, " pending=", pps_snapshot.pending);
    append_u32_field(&text, " fault=", pps_snapshot.fault);
    append_u32_field(&text, " retries=", pps_retries);
    sc_text_string(&text, " time_quality=");
    sc_text_string(&text, pps_qualified        ? "GNSS_VALID"
                          : pps_ever_qualified ? "HOLDOVER"
                                               : "FREE_RUNNING");
    append_u32_field(&text, " stream1_dop=", dop_stream_ready);
    append_u32_field(&text, " report_drops=", pps_report_drops);
    sc_text_string(&text, "\r\n");
    if (!text.ok || simpleclock_usb_cdc_try_send((const uint8_t *)line, (uint16_t)text.length))
    {
        pps_report_due = false;
    }
}
#endif

bool gnss_probe_auto_ready(void)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    return auto_ready && auto_pvt_verified && !(elevation_flags & MASK_FAULT) &&
           startup_ack != STARTUP_ACK_INFLIGHT && startup_ack != STARTUP_ACK_WAIT_STATUS;
#else
    return false;
#endif
}

bool gnss_probe_auto_fault(void)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    return auto_fault || (elevation_flags & MASK_FAULT) != 0U;
#else
    return false;
#endif
}

bool gnss_probe_static_pvt(void)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    return gnss_probe_auto_ready() && auto_static_pvt;
#else
    return false;
#endif
}

bool gnss_probe_mutation_hold(void)
{
    return mutation_hold;
}

bool gnss_probe_pps_snapshot(gnss_probe_pps_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return false;
    }
    memset(snapshot, 0, sizeof(*snapshot));
#ifdef SIMPLECLOCK_AUTO_HYBRID
    snapshot->valid = auto_pps;
    auto_copy_text(snapshot->response, sizeof(snapshot->response), auto_pps_response);
    snapshot->policy = pps_policy;
    snapshot->enabled = auto_pps && pps_enabled;
    const bool active = command_profile == GNSS_PROFILE_PPS_POLICY &&
                        (command_active || command_index < command_count);
    snapshot->policy_valid = auto_pps && pps_verified && !active;
    snapshot->fault = pps_fault;
    snapshot->pending =
        active || (pps_policy != SC_SETTINGS_PPS_MANUAL && !snapshot->policy_valid && !pps_fault);
    snapshot->qualified = pps_qualified;
    snapshot->ever_qualified = pps_ever_qualified;
    return snapshot->valid;
#else
    return false;
#endif
}

bool gnss_probe_pps_policy_set(uint32_t policy)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (policy > SC_SETTINGS_PPS_MANUAL || !gnss_probe_commands_idle())
    {
        __set_PRIMASK(mask);
        return false;
    }
    pps_policy = (uint8_t)policy;
    pps_verified = false;
    pps_fault = false;
    pps_retries = 0U;
    if (auto_pps)
    {
        pps_cache(auto_pps_response);
    }
    __set_PRIMASK(mask);
    return true;
#else
    (void)policy;
    return false;
#endif
}

uint32_t gnss_probe_pps_policy(void)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    return pps_policy;
#else
    return UINT32_MAX;
#endif
}

void gnss_probe_pps_quality(bool qualified)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    if (pps_policy == SC_SETTINGS_PPS_QUALIFIED && qualified != pps_qualified)
    {
        pps_verified = false;
        pps_fault = false;
        pps_retries = 0U;
    }
    pps_qualified = qualified;
    pps_ever_qualified |= qualified;
#else
    (void)qualified;
#endif
}

bool gnss_probe_auto_recheck(void)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    /* Never turn a managed-profile retry into a provisioning operation, or
     * interrupt a pending edit/reset. Read-only checks cannot clear a HOLD. */
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!receiver_user_profile || !gnss_probe_commands_idle())
    {
        __set_PRIMASK(mask);
        return false;
    }
    auto_ready = auto_fault = false;
    auto_retry_scheduled = false;
    requested_profile = AUTO_RESTORE_PROFILE;
    __set_PRIMASK(mask);
    return true;
#else
    return false;
#endif
}

void gnss_probe_auto_restart(void)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    if (control_kind || control_pending || control_event)
    {
        return;
    }
    /* Mailbox only: gnss_probe_service performs all UART I/O. */
    if (auto_retry_count >= AUTO_MAX_RETRIES)
    {
        auto_fault = true;
        return;
    }
    auto_retry_count++;
    receiver_user_profile =
        false; /* Explicit repair API; boot/reboot use read-only managed policy. */
    auto_ready = false;
    auto_fault = false;
    mutation_hold = false;
    requested_profile = AUTO_RESTORE_PROFILE;
#endif
}

uint32_t gnss_probe_selection(void)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    return receiver_selection;
#else
    return 0U;
#endif
}

bool gnss_probe_selection_ready(void)
{
    return gnss_probe_auto_ready() && !gnss_probe_auto_fault() && !command_active &&
           requested_profile == 0U && command_index >= command_count && !ppp_restore_pending;
}

bool gnss_probe_user_profile(void)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    return receiver_user_profile;
#else
    return false;
#endif
}

bool gnss_probe_defaults_request(void)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    if (!gnss_probe_commands_idle() || !hybrid_probe_receiver_change_ready())
    {
        return false;
    }
    requested_profile = GNSS_PROFILE_DEFAULTS;
    return true;
#else
    return false;
#endif
}

bool gnss_probe_commands_idle(void)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    if (control_kind || control_pending || control_event || defaults_pending)
    {
        return false;
    }
#endif
    return !command_active && requested_profile == 0U && command_index >= command_count &&
           !reboot_recovering;
}

bool gnss_probe_reboot_request(void)
{
#ifdef SIMPLECLOCK_AUTO_HYBRID
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    if (!gnss_probe_commands_idle())
    {
        __set_PRIMASK(mask);
        return false;
    }
    reboot_sent = false;
    reboot_failed = false;
    reboot_recovering = true;
    startup_ack = STARTUP_ACK_ARMED;
    status_epoch_valid = false;
    pps_verified = pps_fault = pps_qualified = pps_ever_qualified = false;
    pps_retries = 0U;
    auto_ready = false;
    auto_retry_scheduled = false;
    mutation_hold = true;
    requested_profile = GNSS_PROFILE_REBOOT;
    __set_PRIMASK(mask);
    timing_probe_break(SC_SEG_CONFIG);
    return true;
#else
    return false;
#endif
}

bool gnss_probe_reboot_transmitted(void)
{
    return reboot_sent;
}

bool gnss_probe_reboot_failed(void)
{
    return reboot_failed;
}
