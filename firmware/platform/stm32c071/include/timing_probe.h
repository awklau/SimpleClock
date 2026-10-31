/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_TIMING_PROBE_H
#define SIMPLECLOCK_TIMING_PROBE_H
#include "simpleclock/sbf_fields.h"
#include "simpleclock/timing.h"
/* All APIs run in foreground; TIM2_IRQHandler alone produces capture records.
 * observe copies already-validated fields. now is MCU milliseconds, not GNSS
 * time. pps_ready is a prerequisite for an explicit command, never auto-enable. */
void timing_probe_init(void);
void timing_probe_observe(const sc_sbf_observation_t *o, uint32_t now);
void timing_probe_break(uint32_t reason);
void timing_probe_service(uint32_t now);
bool timing_probe_pps_ready(uint32_t now);
bool timing_probe_fix_valid(uint32_t now);
bool timing_probe_ppp_active(uint32_t now);
/* Foreground-only borrowed pointer, valid until the next service/push. */
const sc_epoch_t *timing_probe_latest(void);
#endif
