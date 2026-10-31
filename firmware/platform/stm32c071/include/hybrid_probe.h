/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_HYBRID_PROBE_H
#define SIMPLECLOCK_HYBRID_PROBE_H
#include "simpleclock/sbf_fields.h"
#include "simpleclock/sit5801.h"
#include "simpleclock/hybrid.h"
/* Foreground hooks except explicitly documented IRQ-safe command mailboxes. */
void hybrid_probe_init(void);
void hybrid_probe_service(uint32_t now_ms);
void hybrid_probe_observe(const sc_sbf_observation_t *observation, uint32_t now_ms);
void hybrid_probe_cycle(const sit5801_probe_t *probe, const uint32_t read_ms[3], uint32_t now_ms);
/* IRQ-safe bounded copy, no bus/float work. HOLD remains sticky across disconnect. */
bool hybrid_probe_command(const char *line);
void hybrid_probe_manual_request(void);
void hybrid_probe_disconnect(void);
bool hybrid_probe_locked(void); /* Relative phase lock, independently reported. */
/* Independent observer, including bounded gap retention. */
bool hybrid_probe_frequency_settled(void);
bool hybrid_probe_faulted(void);
uint32_t hybrid_probe_writes(void);

/* Thermal warning bits (LED3/USB only; never clock gates). REGULATION is
 * LED3's 0.25 C error / 0.1 K/min slope verdict after thermal startup. */
enum
{
    HYBRID_PROBE_THERMAL_SENSOR = 1U,
    HYBRID_PROBE_THERMAL_STARTUP = 2U,
    HYBRID_PROBE_THERMAL_REGULATION = 4U,
    HYBRID_PROBE_THERMAL_DUTY = 8U
};

uint32_t hybrid_probe_thermal_warning(void);
const sc_hybrid_t *hybrid_probe_state(void);
bool hybrid_probe_receiver_change_ready(void);
bool hybrid_probe_receiver_persist(bool managed);
/* Finish the receiver portion of defaults. Only system defaults resets the
 * six loop times; a failed receiver preflight leaves them unchanged. */
void hybrid_probe_defaults_finish(bool receiver_ready);
#endif
