/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_GNSS_PROBE_H
#define SIMPLECLOCK_GNSS_PROBE_H
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Stable diagnostic profile IDs (also emitted in gnss_probe telemetry).
 * Keep wire values stable; do not add arbitrary receiver-command passthrough. */
enum
{
    GNSS_PROFILE_NONE = 0U,
    GNSS_PROFILE_IDENTIFY = 1U,
    GNSS_PROFILE_SURVEY = 2U,
    GNSS_PROFILE_ONCE = 3U,
    GNSS_PROFILE_STREAM = 4U,
    GNSS_PROFILE_STOP = 5U,
    GNSS_PROFILE_HEALTH = 6U,
    GNSS_PROFILE_E6_SURVEY = 7U,
    GNSS_PROFILE_E6_STREAM = 8U,
    GNSS_PROFILE_E6_STOP = 9U,
    GNSS_PROFILE_PPS = 10U,
    GNSS_PROFILE_PPS_OFF = 11U,
    GNSS_PROFILE_PPP_SURVEY = 12U,
    GNSS_PROFILE_PPP_ENABLE = 13U,
    GNSS_PROFILE_PPP_RESTORE = 14U,
    GNSS_PROFILE_TRIAL_STREAM = 15U,
    GNSS_PROFILE_AUTO_HYBRID = 16U,
    GNSS_PROFILE_NO_HAS = 17U,
    GNSS_PROFILE_HAS = 18U,
    GNSS_PROFILE_STANDALONE = 19U,
    GNSS_PROFILE_ELEVATION_SURVEY = 20U,
    GNSS_PROFILE_ELEVATION_RESET = 21U,
    GNSS_PROFILE_SAVE_BOOT = 22U,
    GNSS_PROFILE_REBOOT = 23U, /* Internal system-reboot route, no erase. */
    GNSS_PROFILE_DEFAULTS = 24U,
    GNSS_PROFILE_STARTUP_ERROR_ACK = 25U, /* Internal one-shot SOFTWARE latch acknowledgement. */
    GNSS_PROFILE_PPS_POLICY = 26U,        /* Internal serialized continuous/qualified policy. */
    /* Encoded angle is degrees + 90, covering the documented -90..90 range. */
    GNSS_PROFILE_PVT_MASK_BASE = 256U,
    GNSS_PROFILE_TRACKING_MASK_BASE = 512U,
    GNSS_PROFILE_MASK_VALUES = 181U
};

static inline bool gnss_probe_elevation_set(uint32_t profile)
{
    return (profile >= GNSS_PROFILE_PVT_MASK_BASE &&
            profile < GNSS_PROFILE_PVT_MASK_BASE + GNSS_PROFILE_MASK_VALUES) ||
           (profile >= GNSS_PROFILE_TRACKING_MASK_BASE &&
            profile < GNSS_PROFILE_TRACKING_MASK_BASE + GNSS_PROFILE_MASK_VALUES);
}

static inline bool gnss_probe_elevation_tracking(uint32_t profile)
{
    return profile >= GNSS_PROFILE_TRACKING_MASK_BASE;
}

/* Only call for an encoded setter accepted by gnss_probe_elevation_set. */
static inline int gnss_probe_elevation_degrees(uint32_t profile)
{
    const uint32_t base = gnss_probe_elevation_tracking(profile) ? GNSS_PROFILE_TRACKING_MASK_BASE
                                                                 : GNSS_PROFILE_PVT_MASK_BASE;
    return (int)(profile - base) - 90;
}

/* One command whitelist shared by clock-only and thermal CDC receivers.
 * Call only after bounding and NUL-terminating a complete input line. This pure
 * lookup is IRQ-safe; it neither changes receiver state nor performs I/O.
 * Keep explicit IDs so adding/reordering names cannot silently retarget UART
 * commands. Header-local storage lets each image discard unused routes. */
static inline uint32_t gnss_probe_command_profile(const char *line)
{
    static const struct
    {
        const char *name;
        uint32_t profile;
    } routes[] = {{"identify", GNSS_PROFILE_IDENTIFY},
                  {"survey", GNSS_PROFILE_SURVEY},
                  {"once", GNSS_PROFILE_ONCE},
                  {"stream", GNSS_PROFILE_STREAM},
                  {"stop", GNSS_PROFILE_STOP},
                  {"health", GNSS_PROFILE_HEALTH},
                  {"e6survey", GNSS_PROFILE_E6_SURVEY},
                  {"e6stream", GNSS_PROFILE_E6_STREAM},
                  {"e6stop", GNSS_PROFILE_E6_STOP},
                  {"pps", GNSS_PROFILE_PPS},
                  {"ppsoff", GNSS_PROFILE_PPS_OFF},
                  {"pppsurvey", GNSS_PROFILE_PPP_SURVEY},
                  {"ppp", GNSS_PROFILE_PPP_ENABLE},
                  {"ppprestore", GNSS_PROFILE_PPP_RESTORE},
                  {"trial", GNSS_PROFILE_TRIAL_STREAM},
                  {"auto", GNSS_PROFILE_AUTO_HYBRID},
                  {"nohas", GNSS_PROFILE_NO_HAS},
                  {"sbas", GNSS_PROFILE_NO_HAS},
                  {"has", GNSS_PROFILE_HAS},
                  {"standalone", GNSS_PROFILE_STANDALONE},
                  {"masks", GNSS_PROFILE_ELEVATION_SURVEY},
                  {"masks reset", GNSS_PROFILE_ELEVATION_RESET},
                  {"save", GNSS_PROFILE_SAVE_BOOT}
#ifdef SIMPLECLOCK_AUTO_HYBRID
                  ,
                  {"defaults", GNSS_PROFILE_DEFAULTS}
#endif
    };

    if (line != NULL && strncmp(line, "gnss ", 5) == 0)
    {
        for (unsigned i = 0; i < sizeof(routes) / sizeof(routes[0]); ++i)
        {
            if (strcmp(line + 5, routes[i].name) == 0)
            {
                return routes[i].profile;
            }
        }
    }
    if (line != NULL && strncmp(line, "gnss mask ", 10) == 0)
    {
        const char *value = line + 10;
        uint32_t base;
        if (strncmp(value, "pvt ", 4) == 0)
        {
            base = GNSS_PROFILE_PVT_MASK_BASE;
            value += 4;
        }
        else if (strncmp(value, "tracking ", 9) == 0)
        {
            base = GNSS_PROFILE_TRACKING_MASK_BASE;
            value += 9;
        }
        else
        {
            return GNSS_PROFILE_NONE;
        }
        if (strcmp(value, "off") == 0)
        {
            return base; /* -90 degrees disables exclusion over the physical elevation range. */
        }
        const bool negative = *value == '-';
        if (negative)
        {
            value++;
        }
        unsigned degrees = 0U, digits = 0U;
        while (*value >= '0' && *value <= '9' && digits < 2U)
        {
            degrees = degrees * 10U + (unsigned)(*value++ - '0');
            digits++;
        }
        if (*value || !digits || degrees > 90U)
        {
            return GNSS_PROFILE_NONE;
        }
        const int signed_degrees = negative ? -(int)degrees : (int)degrees;
        return base + (uint32_t)(signed_degrees + 90);
    }
    return GNSS_PROFILE_NONE;
}

/* init/service/report are foreground-only. Request functions may be called
 * from the USB ISR: they set a single latest-request-wins mailbox and do no I/O.
 * The active command completes before a replacement profile is selected. */
void gnss_probe_init(void);
void gnss_probe_service(uint32_t now_ms);
void gnss_probe_identify(void);
/* 1=identify, 2=survey, 3=once, 4=volatile stream, 5=stream off, 6=health;
 * 7=E6 survey, 8=E6 pages on Stream2, 9=Stream2 off, 10=gated RxClock PPS,
 * 11=PPS off; 12=PPP preflight survey, 13=checked volatile PPP enable,
 * 14=restore this session's prior Rover/StandAlone/auto mode (volatile).
 * 17=explicit no-HAS bench mode (StandAlone+SBAS); 18=explicit HAS bench mode
 * (StandAlone+PPP); 19=pure StandAlone (neither PPP nor SBAS). All query mode,
 * the augmented selections also check capability, and all hold control before the
 * sole PVT setter, verify readback and leave clock control held. Other receiver
 * Current settings are preserved; normal AUTO firmware saves verified Boot
 * and MCU selection/profile after success. These are not arbitrary-state restores.
 * 20=query both elevation masks; encoded mask profiles set only PVT or Tracking
 * within -90..90 degrees. Both masks are read before/after; the other must stay
 * unchanged. Mutations hold control and require explicit hybrid resume. off=-90.
 * 21=reset both masks to -90; 22=save the whole Current configuration to Boot.
 * Mask setters/reset automatically save Current to Boot after verified readback.
 * A failed/cancelled mask operation stays faulted until a successful mask setter
 * or reset; queries and the generic save cannot certify an interrupted change.
 * E6 page capture does not alter tracking/PVT/receiver flash. */
void gnss_probe_request(uint32_t profile);
void gnss_probe_report(uint32_t now_ms);

/* The PPS snapshot is a native getPPSParameters readback. enabled denotes a
 * verified sec1 setting, not physical pulse observation or absolute alignment.
 * policy_valid requires the actual settings to match the current policy target;
 * manual preserves explicit expert settings and never schedules a policy write. */
typedef struct
{
    bool valid;
    char response[96];
    uint32_t policy;
    bool enabled, policy_valid, pending, fault, qualified, ever_qualified;
} gnss_probe_pps_snapshot_t;

bool gnss_probe_auto_ready(void);
/* True only for verified Static PVT without PPP; not inferred from motion. */
bool gnss_probe_static_pvt(void);
bool gnss_probe_auto_fault(void);
bool gnss_probe_mutation_hold(void);
bool gnss_probe_pps_snapshot(gnss_probe_pps_snapshot_t *snapshot);
/* Foreground-only policy control. Values are SC_SETTINGS_PPS_*; acceptance
 * queues verified application, not proof of delivered pulses or persistence.
 * Explicit hybrid save persists policy; routine application never saves Boot.
 * A successful expert PPS edit selects persistent MANUAL to avoid fighting it. */
bool gnss_probe_pps_policy_set(uint32_t policy);
uint32_t gnss_probe_pps_policy(void);
/* Fresh usable GNSS quality, independent of clock mode/phase lock. */
void gnss_probe_pps_quality(bool qualified);
/* Foreground-only managed-profile requery. Preserves Current, Boot and the
 * MCU profile marker; false if unmanaged or another operation owns the link. */
bool gnss_probe_auto_recheck(void);
/* Internal explicit standard-profile repair, which can write Current/Boot.
 * Not used by normal managed startup or hybrid resume. */
void gnss_probe_auto_restart(void);
/* Verified desired solution set, using SC_SETTINGS_GNSS_* values. Save only
 * after all queued/in-flight receiver transactions have completed. */
uint32_t gnss_probe_selection(void);
bool gnss_probe_selection_ready(void);
/* Foreground-only soft restart through the existing serialized UART owner.
 * No erase/save, no automatic retry of the reset. Recovery re-queries state. */
bool gnss_probe_commands_idle(void);
bool gnss_probe_reboot_request(void);
bool gnss_probe_reboot_transmitted(void);
bool gnss_probe_reboot_failed(void);
/* AUTO_HYBRID expert bridge. IRQ-safe bounded mailbox; returns true when the
 * prefix is consumed (including a rejected/busy request). No I/O in IRQ. */
bool gnss_probe_control(const char *line);
bool gnss_probe_user_profile(void);
bool gnss_probe_defaults_request(void);
#endif
