/* SPDX-License-Identifier: MIT */
#include "simpleclock/hybrid_input.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#define CHECK(x)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(x))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #x);                                        \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

int main(void)
{
    sc_epoch_t e = {0};
    sc_hybrid_input_t out = {0};
    e.mask = SC_EPOCH_ALL;
    e.sources_valid = e.jumps_valid = true;
    e.week = 2436U;
    e.tow_ms = 604799000U;
    e.sync_level = 7U;
    e.rx_state = 116U;
    e.common_flags = 197U;
    for (unsigned i = 0; i < SC_EPOCH_SOURCE_COUNT; i++)
    {
        e.source_ms[i] = 10000U;
    }
    e.pvt.valid = SC_SBF_VALID_EPOCH | SC_SBF_VALID_FIX | SC_SBF_VALID_BIAS | SC_SBF_VALID_DRIFT |
                  SC_SBF_VALID_TIME_SYSTEM | SC_SBF_VALID_CORR_AGE;
    e.pvt.data.pvt.mode = 10U;
    e.pvt.data.pvt.time_system = 1U;
    e.pvt.data.pvt.nr_sv = 6U;
    e.pvt.data.pvt.alert = 1U;
    e.pvt.data.pvt.wa_corr_info = 1U;
    e.pvt.data.pvt.mean_corr_age = 3000U;
    double bias_ms = -0.4;
    float drift_ppm = 0.00275f;
    memcpy(&e.pvt.data.pvt.bias_bits, &bias_ms, sizeof(bias_ms));
    memcpy(&e.pvt.data.pvt.drift_bits, &drift_ppm, sizeof(drift_ppm));
    CHECK(sc_hybrid_input_epoch(&out, &e, 12500U, true, true, false));
    CHECK(fabs(out.bias_ns + 400000.0) < 1e-8 && fabs(out.drift_ppb - 2.75) < 0.000001);
    const uint64_t before = out.epoch_ms;
    e.week++;
    e.tow_ms = 0;
    CHECK(sc_hybrid_input_epoch(&out, &e, 12500U, true, true, false) &&
          out.epoch_ms - before == 1000U);
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12501U, true, true, false));
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, false, true, false));
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, false, false));
    e.pvt.data.pvt.alert = 0;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    e.pvt.data.pvt.alert = 1;
    e.pvt.data.pvt.mean_corr_age = UINT16_MAX;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    e.pvt.data.pvt.mean_corr_age = 3000;
    /* The receiver's valid solution governs acceptance. All count values,
     * including five and DNU, remain metadata rather than an extra lock gate. */
    for (unsigned count = 0U; count <= UINT8_MAX; count++)
    {
        e.pvt.data.pvt.nr_sv = (uint8_t)count;
        CHECK(sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    }
    e.pvt.data.pvt.nr_sv = 5;
    e.pvt.data.pvt.mode = 1U;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    CHECK(out.standalone_valid && out.frequency_valid);
    e.pvt.data.pvt.time_system = 0U;
    e.pvt.data.pvt.mean_corr_age = UINT16_MAX;
    e.pvt.data.pvt.wa_corr_info = 0U;
    e.pvt.valid &= (uint16_t)~SC_SBF_VALID_CORR_AGE;
    e.reasons = SC_SEG_SYSTEM;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    CHECK(out.standalone_valid && out.frequency_valid && !out.coordinate_changed &&
          !out.frequency_discontinuity); /* GPS/GST is handled by the controller's domain check. */
    e.pvt.data.pvt.mode = SC_SBF_PVT_MODE_FIXED;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    CHECK(!out.standalone_valid && !out.frequency_valid); /* Unexpected fixed PVT is rejected. */
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, true));
    CHECK(out.standalone_valid && out.frequency_valid && !out.sbas_valid && !out.gnss_valid);
    e.pvt.data.pvt.mode |= 0x40U;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, true));
    CHECK(!out.standalone_valid && !out.frequency_valid); /* Auto-base has not settled. */
    e.pvt.data.pvt.mode = SC_SBF_PVT_MODE_FIXED;
    e.pvt.data.pvt.alert = 0U;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, true));
    CHECK(!out.standalone_valid && !out.frequency_valid); /* RAIM remains required. */
    e.pvt.data.pvt.alert = 1U;
    e.rx_error = 8U;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, true));
    CHECK(!out.standalone_valid && !out.frequency_valid);
    e.rx_error = 0U;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, false, true, true));
    CHECK(!out.standalone_valid && !out.frequency_valid);
    e.pvt.data.pvt.mode = SC_SBF_PVT_MODE_STANDALONE;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12501U, true, true, false));
    CHECK(!out.standalone_valid && !out.frequency_valid);
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, false, false));
    CHECK(!out.standalone_valid && !out.frequency_valid);
    e.pvt.data.pvt.error = 1U;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    CHECK(!out.standalone_valid && !out.frequency_valid);
    e.pvt.data.pvt.error = 0U;
    e.pvt.data.pvt.time_system = 2U;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    CHECK(!out.standalone_valid && !out.frequency_valid);
    e.pvt.data.pvt.time_system = 1U;
    e.pvt.data.pvt.mode = 6U;
    e.pvt.valid |= SC_SBF_VALID_CORR_AGE;
    e.pvt.data.pvt.mean_corr_age = 300U;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    CHECK(out.sbas_valid && out.frequency_valid && !out.standalone_valid);
    CHECK(out.time_system == 1U);
    e.pvt.data.pvt.mean_corr_age = UINT16_MAX;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false) && !out.sbas_valid);
    e.pvt.data.pvt.mean_corr_age = 300U;
    e.pvt.data.pvt.alert = 2U;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    CHECK(!out.sbas_valid && !out.frequency_valid);
    e.pvt.data.pvt.alert = 1U;
    e.pvt.data.pvt.mode = 2U; /* Do not silently accept unsupported differential modes. */
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    CHECK(!out.standalone_valid && !out.frequency_valid);
    e.reasons = 0U;
    e.pvt.valid |= SC_SBF_VALID_CORR_AGE;
    e.pvt.data.pvt.mean_corr_age = 3000U;
    e.pvt.data.pvt.mode = 10U;
    e.pvt.data.pvt.wa_corr_info = 0U;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    e.pvt.data.pvt.wa_corr_info = 1U;
    e.pvt.data.pvt.error = 1U;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    e.pvt.data.pvt.error = 0U;
    CHECK(sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    e.pvt.data.pvt.nr_sv = 6;
    e.common_flags |= 8;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    e.common_flags = 197;
    bias_ms = -2e10;
    memcpy(&e.pvt.data.pvt.bias_bits, &bias_ms, sizeof(bias_ms));
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false) && isnan(out.bias_ns));
    bias_ms = 0;
    memcpy(&e.pvt.data.pvt.bias_bits, &bias_ms, sizeof(bias_ms));
    e.reasons = SC_SEG_GAP | SC_SEG_LOSS;
    CHECK(sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false) && !out.coordinate_changed);
    e.reasons = SC_SEG_JUMP;
    CHECK(sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false) && out.coordinate_changed);
    CHECK(out.frequency_discontinuity);
    e.duplicate = SC_EPOCH_TIME;
    CHECK(!sc_hybrid_input_epoch(&out, &e, 12000U, true, true, false));
    puts("Hybrid source quality, units, DNU, age and receiver-week rollover passed");
    return 0;
}
