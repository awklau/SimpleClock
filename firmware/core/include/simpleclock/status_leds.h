/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_STATUS_LEDS_H
#define SIMPLECLOCK_STATUS_LEDS_H
#include "simpleclock/thermal_control.h"
#include "simpleclock/hybrid_defaults.h"

typedef enum
{
    SC_LED_OFF,
    SC_LED_BLINK,
    SC_LED_SOLID,
    SC_LED_FAULT
} sc_led_mode_t;

typedef struct
{
    sc_led_mode_t system, gnss, thermal; /* Independent modes for the three LEDs. */
} sc_led_modes_t;

/* Separate operator indication; never changes the PI gains, duty or stricter
 * control.stable qualification. All fields are owned by foreground. */
typedef struct
{
    uint32_t since_ms, sample_ms; /* Controller update times, modulo-2^32 ms. */
    bool timing, stable, seen;
} sc_thermal_indicator_t;

/* Legacy display-only grace (the autonomous profile uses hybrid's observer).
 * After a previously locked clock loses PPP/HAS, it never
 * delays control holdover or changes the truthful relative_lock telemetry. */
typedef struct
{
    uint32_t loss_ms;
    bool was_locked, grace;
} sc_clock_indicator_t;

#define SC_LED_HOLDOVER_GRACE_MS 120000U
bool sc_clock_indicator_update(sc_clock_indicator_t *state, bool locked, bool gnss_loss,
                               bool healthy, uint32_t now_ms);

/* One specification for LED1 clock qualification and LED3 temperature status.
 * Q16 conversion rounds conservatively down; the default is exactly 0.25 C
 * and 6553 counts/min (0.099991 C/min). No floating-point work at runtime. */
#define SC_LED_THERMAL_ERROR_Q16                                                                   \
    ((int32_t)(SC_HYBRID_QUALIFICATION_TEMPERATURE_MAX_ABS_ERROR_C * SC_THERMAL_Q16))
#define SC_LED_THERMAL_SLOPE_Q16                                                                   \
    ((int32_t)(SC_HYBRID_QUALIFICATION_TEMPERATURE_ERROR_SLOPE_LIMIT_K_PER_S * 60.0 *              \
               SC_THERMAL_Q16))
#define SC_LED_THERMAL_DWELL_MS 0U /* User policy: immediate return once current criteria pass. */

/* healthy includes fresh sensor/OCXO state, no actuator fault, and active control.
 * Eligibility is <=0.25 C error and <=0.1 K/min error slope, with fresh data.
 * Recovery is immediate; previous heater saturation does not delay this display.
 * Clock qualification checks duty headroom separately. No heater deadband. */
/* Foreground-only; state is caller-owned and reset on any ineligible update.
 * control is read-only and must remain valid for the duration of the call. */
void sc_thermal_indicator_update(sc_thermal_indicator_t *state, const sc_thermal_control_t *control,
                                 bool healthy, uint32_t now_ms);
sc_led_modes_t sc_led_modes(bool fault, bool record_ready, bool gnss_fix, bool ppp_active,
                            bool thermal_available, bool thermal_stable);
/* Autonomous profile: each LED reports its own condition. Faults belonging
 * to the OCXO/heater are displayed on LED3; they never veto frequency data. */
sc_led_modes_t sc_led_independent_modes(bool frequency_settled, bool gnss_fix, bool ppp_active,
                                        bool thermal_available, bool thermal_stable,
                                        bool ocxo_thermal_fault);
/* One monotonic 1 Hz/50% blink phase. Fault: ON 0..99 and 200..299 ms/second.
 * No delays, GPIO, PPS or host connection dependencies. */
bool sc_led_on(sc_led_mode_t mode, uint32_t now_ms);
const char *sc_led_mode_name(sc_led_mode_t mode);
#endif
