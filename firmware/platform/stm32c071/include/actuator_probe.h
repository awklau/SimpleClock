/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_ACTUATOR_PROBE_H
#define SIMPLECLOCK_ACTUATOR_PROBE_H
#include "simpleclock/sit5801.h"
#include "simpleclock/sbf_fields.h"
#include <stddef.h>
/* Only compiled into the M6 bench image. USB IRQ submits whitelisted requests;
 * foreground alone owns I2C/state. No automatic frequency write at startup. */
bool actuator_probe_command(const char *line);
void actuator_probe_disconnect(void);
void actuator_probe_observe(const sc_sbf_observation_t *o, uint32_t now_ms);
void actuator_probe_cycle(const sit5801_probe_t *probe, const uint32_t read_ms[3], uint32_t now_ms);
void actuator_probe_service(uint32_t now_ms);
uint32_t actuator_probe_writes(void);
#endif
