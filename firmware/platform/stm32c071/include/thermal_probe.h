/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_THERMAL_PROBE_H
#define SIMPLECLOCK_THERMAL_PROBE_H
#include "simpleclock/sit5801.h"
#include <stddef.h>

/* AUTO_HYBRID starts bounded full-power slope capture on fresh sensor/OCXO
 * checks, then continuous PID with generated gains (D on measurement). Target
 * is clamped to 0..60 C with a 70 C raw trip. Legacy M5 warms for 300 s and
 * uses baseline +2 C / 45 C ceiling; M6 uses fixed 39.399 C (both trip at 50 C).
 * Uses elapsed-time operator permission,
 * not native OCXO qualification. OFF, a fault, or an accepted explicit trial
 * suppresses automatic startup until reboot. USB disconnect does not stop PI.
 * service/ocxo are foreground; tick is SysTick-only.
 * command receives complete USB lines in IRQ context, copies one pending command;
 * parsing/acknowledgment happens in foreground. OFF bypasses the mailbox. */
void thermal_probe_init(uint32_t now_ms);
void thermal_probe_service(uint32_t now_ms);
void thermal_probe_tick(uint32_t now_ms);
void thermal_probe_fault(void);
void thermal_probe_command(const char *line, size_t length);
void thermal_probe_receive(const uint8_t *bytes, uint32_t length);
void thermal_probe_disconnect(void);
void thermal_probe_ocxo(const sit5801_probe_t *probe, uint32_t now_ms);
/* Foreground-only LED snapshot. 'available' means the profile's startup policy
 * has completed, not native OCXO qualification. Stable uses +/-0.25 C and
 * 0.1 C/min without a recovery dwell once the slope window is available. */
void thermal_probe_led_status(uint32_t now_ms, bool *available, bool *stable, bool *fault);
#ifdef SIMPLECLOCK_AUTO_HYBRID
typedef struct
{
    uint32_t sample_ms, frame_sequence, frame_ms, faults;
    int32_t filtered_q16, target_q16, proportional_micro, integral_micro, derivative_micro;
    int16_t raw;
    uint16_t completed_duty_permille;
    bool safe, sensor_valid, warmed, running, completed_frame_valid;
    bool regulation_stable; /* LED3's latest error/slope verdict; read-only. */
} thermal_probe_snapshot_t;

/* Foreground snapshot; completed duty counts timer-applied output over a whole
 * modulation interval, not one sampled pin state or internal SiTime power. */
void thermal_probe_snapshot(uint32_t now_ms, thermal_probe_snapshot_t *out);
bool thermal_probe_set_target(int32_t target_q16);
/* Foreground-only bounded flash-save interlock. Force physical PB3 low before
 * any flash stall; discard the interrupted heater duty-frame evidence. */
void thermal_probe_storage_inhibit(bool inhibit);
#endif
#ifdef SIMPLECLOCK_ACTUATOR_TEST
/* Fixed target/gains and live PI health; require the LED dwell only at start. */
bool thermal_probe_trial_ready(uint32_t now_ms, bool require_stable);
#endif
#endif
