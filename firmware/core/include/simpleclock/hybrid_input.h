/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_HYBRID_INPUT_H
#define SIMPLECLOCK_HYBRID_INPUT_H
#include "simpleclock/hybrid.h"
#include "simpleclock/timing.h"
/* Foreground-only adapter; retains no pointers. out must be non-NULL and
 * initialized by the caller, who supplies the thermal fields; those fields are
 * preserved. now_ms and epoch->source_ms are modulo-2^32 MCU milliseconds.
 *
 * Every call updates now_ms and clears the receiver-validity/discontinuity flags.
 * A NULL epoch then returns false, retaining previous numeric fields/timestamps;
 * never use those retained values as a fresh measurement. A non-NULL epoch fills
 * receiver fields even if rejected; conversion failure sets bias/drift to NaN.
 *
 * Returns qualified PPP availability (gnss_valid), not general GNSS validity.
 * standalone_valid and sbas_valid separately permit fallback control.
 * frequency_valid is for the independent measurement observer and does not
 * require HAS use/correction age; it alone cannot authorize clock steering.
 * All require complete numeric epochs and fresh verified RFStatus, supplied by
 * rf_status_clear from the separate stream. configuration_ready means the
 * required receiver settings have been read back successfully. static_pvt
 * separately permits mode 3 only after verified non-PPP Static configuration;
 * fixed-position timing uses the standalone loop pair, never claims SBAS/PPP.
 *
 * Rejected observations remain in SBF/epoch telemetry. coordinate_changed
 * denotes reset/jump/reference/configuration recovery. Time-system changes
 * are carried by time_system for the controller's ordinary handover check.
 * A missing packet alone does not change phase identity. */
bool sc_hybrid_input_epoch(sc_hybrid_input_t *out, const sc_epoch_t *epoch, uint32_t now_ms,
                           bool configuration_ready, bool rf_status_clear, bool static_pvt);
#endif
