/* SPDX-License-Identifier: MIT */
#include "simpleclock/hybrid_input.h"
#include <math.h>

#define RECEIVER_TIMEOUT_MS ((uint32_t)(SC_HYBRID_QUALIFICATION_RECEIVER_UPDATE_TIMEOUT_S * 1000.0))
/* MeanCorrAge is in centiseconds; the controller limit is specified in seconds. */
#define PPP_CORRECTION_AGE_LIMIT_CS                                                                \
    ((uint16_t)(SC_HYBRID_QUALIFICATION_CORRECTION_AGE_LIMIT_S * 100.0))

bool sc_hybrid_input_epoch(sc_hybrid_input_t *out, const sc_epoch_t *epoch, uint32_t now_ms,
                           bool configuration_ready, bool rf_status_clear, bool static_pvt)
{
    out->now_ms = now_ms;
    out->gnss_valid = false;
    out->coordinate_changed = false;
    out->standalone_valid = out->sbas_valid = out->frequency_valid = out->frequency_discontinuity =
        out->static_fix = false;
    if (epoch == NULL)
    {
        return false;
    }
    const sc_sbf_observation_t *p = &epoch->pvt;
    out->epoch_ms = (uint64_t)epoch->week * SC_SBF_WEEK_MS + epoch->tow_ms;
    out->received_ms = epoch->source_ms[0]; /* PVT frame completion, not epoch publication. */
    /* Time-system changes are visible directly in out->time_system. They may
     * use a continuous relative-phase handover; resets/jumps/configuration and
     * external-reference changes must still force full recovery. */
    out->coordinate_changed =
        (epoch->reasons & (SC_SEG_RESET | SC_SEG_JUMP | SC_SEG_REFERENCE | SC_SEG_CONFIG)) != 0U;
    out->frequency_discontinuity =
        (epoch->reasons & (SC_SEG_RESET | SC_SEG_JUMP | SC_SEG_REFERENCE)) != 0U;
    double bias_s = 0.0, drift_fraction = 0.0;
    const bool numbers = sc_sbf_bias_seconds(p->data.pvt.bias_bits, &bias_s) &&
                         sc_sbf_drift_fraction(p->data.pvt.drift_bits, &drift_fraction);
    out->bias_ns = numbers ? bias_s * 1e9 : (double)NAN;
    out->drift_ppb = numbers ? drift_fraction * 1e9 : (double)NAN;
    const uint16_t fields = SC_SBF_VALID_EPOCH | SC_SBF_VALID_FIX | SC_SBF_VALID_BIAS |
                            SC_SBF_VALID_DRIFT | SC_SBF_VALID_TIME_SYSTEM;
    bool fresh = true;
    for (unsigned i = 0; i < SC_EPOCH_SOURCE_COUNT; i++)
    {
        fresh = fresh && (uint32_t)(now_ms - epoch->source_ms[i]) <= RECEIVER_TIMEOUT_MS;
    }
    /* G5 v1.1 PVTGeodetic: AlertFlag 01 = successful RAIM, no HPCA/storm flags.
     * RFStatus separately supplies authenticity flags; absence is not 'clear'.
     * MeanCorrAge is 0.01s, 65535 is DNU; never substitute zero for unavailable.
     * Satellite count is diagnostic only. PPP needs HAS in use and correction
     * age; standalone has no correction-age requirement. All retain source
     * integrity/freshness, RAIM and verified external-clock requirements. */
    const bool common =
        configuration_ready && rf_status_clear && numbers && fresh && epoch->mask == SC_EPOCH_ALL &&
        epoch->duplicate == 0U && epoch->sources_valid && epoch->week != UINT16_MAX &&
        epoch->tow_ms < SC_SBF_WEEK_MS && (p->valid & fields) == fields &&
        p->data.pvt.time_system <= SC_SBF_TIME_SYSTEM_GALILEO && p->data.pvt.error == 0U &&
        p->data.pvt.alert == SC_SBF_PVT_ALERT_RAIM_OK && epoch->rx_error == 0U &&
        epoch->ext_error == 0U && (epoch->rx_state & SC_SBF_RX_STATE_EXT_FREQ) != 0U &&
        (epoch->sync_level & SC_SBF_TIME_SYNC_READY_MASK) == SC_SBF_TIME_SYNC_READY_MASK &&
        !(epoch->common_flags & SC_SBF_MEAS_CLOCK_STEERING) && epoch->jumps_valid &&
        (!static_pvt || !(p->data.pvt.mode & SC_SBF_PVT_STATIC_PENDING));
    const unsigned mode = p->data.pvt.mode & SC_SBF_PVT_MODE_MASK;
    out->time_system = p->data.pvt.time_system;
    const bool fixed = static_pvt && mode == SC_SBF_PVT_MODE_FIXED;
    const bool standalone = mode == SC_SBF_PVT_MODE_STANDALONE || fixed;
    const bool ppp = !static_pvt && mode == SC_SBF_PVT_MODE_PPP;
    out->standalone_valid = common && standalone;
    out->static_fix = out->standalone_valid && fixed;
    /* Native mode 6 indicates an SBAS solution. Require reported correction
     * age, but do not invent a protocol expiry from the bench noise records. */
    out->sbas_valid = common && mode == SC_SBF_PVT_MODE_SBAS &&
                      (p->valid & SC_SBF_VALID_CORR_AGE) != 0U &&
                      p->data.pvt.mean_corr_age != UINT16_MAX;
    out->frequency_valid = common && (standalone || mode == SC_SBF_PVT_MODE_SBAS || ppp);
    out->gnss_valid = common && ppp && p->data.pvt.time_system == SC_SBF_TIME_SYSTEM_GALILEO &&
                      (p->valid & SC_SBF_VALID_CORR_AGE) != 0U &&
                      (p->data.pvt.wa_corr_info & SC_SBF_PVT_WA_ORBIT_CLOCK_USED) != 0U &&
                      p->data.pvt.mean_corr_age <= PPP_CORRECTION_AGE_LIMIT_CS;
    return out->gnss_valid;
}
