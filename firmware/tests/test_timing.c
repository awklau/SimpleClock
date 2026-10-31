/* SPDX-License-Identifier: MIT */
#include "simpleclock/timing.h"
#include <stdio.h>
#include <string.h>
#define CHECK(e)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(e))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #e);                                        \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)
_Static_assert(sizeof(sc_epochs_t) < 1024, "Bounded epoch state exceeds 1 KiB");

static sc_sbf_observation_t obs(unsigned i, uint16_t week, uint32_t tow)
{
    static const uint16_t ids[] = {SC_SBF_PVT, SC_SBF_STATUS, SC_SBF_TIME, SC_SBF_MEAS, SC_SBF_END};
    sc_sbf_observation_t o = {0};
    o.id = ids[i];
    o.week = week;
    o.tow_ms = tow;
    o.valid = SC_SBF_VALID_EPOCH;
    if (i == 0U)
    {
        o.revision = 2;
        o.valid = 31;
        o.data.pvt.mode = 1;
        o.data.pvt.alert = 1;
    }
    if (i == 1U)
    {
        o.data.status.rx_state = 116;
        o.data.status.uptime_s = 1000 + tow / 1000;
    }
    if (i == 2U)
    {
        o.data.time.sync_level = 7;
    }
    if (i == 3U)
    {
        o.valid |= SC_SBF_VALID_JUMPS;
        o.data.meas.cum_jumps = 0;
    }
    return o;
}

static void complete(sc_epochs_t *s, uint16_t week, uint32_t tow, uint32_t now, uint8_t jump,
                     uint8_t system)
{
    /* EndOfPVT deliberately arrives before ReceiverStatus and ReceiverTime. */
    static const unsigned order[] = {4, 0, 3, 1, 2};
    for (unsigned n = 0; n < 5; n++)
    {
        unsigned i = order[n];
        sc_sbf_observation_t o = obs(i, week, tow);
        if (i == 3)
        {
            o.data.meas.cum_jumps = jump;
        }
        if (i == 0)
        {
            o.data.pvt.time_system = system;
        }
        sc_epochs_push(s, &o, now + n * 10);
    }
}

int main(void)
{
    sc_epochs_t ppp_state = {0};
    sc_sbf_observation_t ppp = obs(0, 1, 1000);
    ppp.data.pvt.mode = 10;
    ppp.data.pvt.time_system = 1;
    ppp.valid |= SC_SBF_VALID_CORR_AGE;
    ppp.data.pvt.wa_corr_info = 1;
    ppp.data.pvt.mean_corr_age = 11999;
    CHECK(sc_pvt_has_corrections(&ppp));
    ppp.data.pvt.mean_corr_age = 12000;
    CHECK(!sc_pvt_has_corrections(&ppp));
    ppp.data.pvt.mean_corr_age = 65535;
    CHECK(!sc_pvt_has_corrections(&ppp));
    /* LED policy reports native PPP mode immediately, independently of
     * correction-age policy and PPPInfo seed metadata; no 300-second dwell. */
    sc_epochs_push(&ppp_state, &ppp, UINT32_MAX - 100U);
    CHECK(sc_epochs_ppp_active(&ppp_state, UINT32_MAX - 100U));
    CHECK(sc_epochs_ppp_active(&ppp_state, 1399));
    CHECK(!sc_epochs_ppp_active(&ppp_state, 1400));
    ppp.tow_ms = 2000;
    ppp.data.pvt.mode = 1;
    sc_epochs_push(&ppp_state, &ppp, 500);
    CHECK(sc_epochs_fix_valid(&ppp_state, 500) && !sc_epochs_ppp_active(&ppp_state, 500));
    ppp.tow_ms = 3000;
    ppp.data.pvt.mode = 10;
    ppp.data.pvt.alert = 2;
    sc_epochs_push(&ppp_state, &ppp, 600);
    CHECK(!sc_epochs_ppp_active(&ppp_state, 600));
    sc_epochs_break(&ppp_state, SC_SEG_CONFIG);
    CHECK(!sc_epochs_ppp_active(&ppp_state, 600));
    ppp.data.pvt.alert = 1;
    ppp.data.pvt.mean_corr_age = 100;
    ppp.data.pvt.wa_corr_info = 0;
    CHECK(!sc_pvt_has_corrections(&ppp));
    ppp.data.pvt.wa_corr_info = 1;
    ppp.data.pvt.time_system = 0;
    CHECK(!sc_pvt_has_corrections(&ppp));
    ppp.data.pvt.time_system = 1;
    ppp.valid &= (uint16_t)~SC_SBF_VALID_CORR_AGE;
    CHECK(!sc_pvt_has_corrections(&ppp));
    sc_epochs_t fix_state = {0};
    CHECK(!sc_epochs_fix_valid(&fix_state, 0));
    sc_sbf_observation_t fix = obs(0, 1, 1000);
    sc_epochs_push(&fix_state, &fix, UINT32_MAX - 100U);
    CHECK(sc_epochs_fix_valid(&fix_state, 1399));
    CHECK(!sc_epochs_fix_valid(&fix_state, 1400));
    fix = obs(0, 1, 2000);
    fix.valid &= (uint16_t)~SC_SBF_VALID_FIX;
    sc_epochs_push(&fix_state, &fix, 100);
    CHECK(!sc_epochs_fix_valid(&fix_state, 100)); /* New invalid PVT beats old valid snapshot. */
    fix = obs(0, 1, 3000);
    sc_epochs_push(&fix_state, &fix, 200);
    CHECK(sc_epochs_fix_valid(&fix_state, 200));
    sc_epochs_push(&fix_state, &fix, 300);
    CHECK(!sc_epochs_fix_valid(&fix_state, 300)); /* Duplicate never refreshes/repairs a fix. */
    sc_epochs_break(&fix_state, SC_SEG_CONFIG);
    CHECK(!sc_epochs_fix_valid(&fix_state, 300));
    fix = obs(0, 1, 4000);
    fix.revision = 3;
    fix.data.pvt.alert = 0x20;
    sc_epochs_push(&fix_state, &fix, 400);
    CHECK(!sc_epochs_fix_valid(&fix_state, 400));
    sc_epochs_break(&fix_state, SC_SEG_LOSS);
    fix = obs(0, 1, 5000);
    sc_epochs_push(&fix_state, &fix, 500);
    fix = obs(1, 1, 5000);
    fix.data.status.rx_error = 1;
    sc_epochs_push(&fix_state, &fix, 510);
    CHECK(!sc_epochs_fix_valid(&fix_state, 510));
    sc_epochs_t s = {0};
    sc_epoch_t e;
    sc_pps_t p = {0};
    complete(&s, 2436, 1000, UINT32_MAX - 100U, 0, 0);
    CHECK(!sc_epochs_poll(&s, 100, &e));
    CHECK(sc_epochs_poll(&s, 149, &e) && e.mask == 31 && e.sources_valid &&
          e.reasons == SC_SEG_START);
    CHECK(sc_epochs_pps_ready(&s, 1000));
    CHECK(!sc_epochs_pps_ready(&s, 1400));
    sc_sbf_observation_t o = obs(2, 2436, 1000);
    sc_epochs_push(&s, &o, 200);
    CHECK(s.late == 1 && !sc_epochs_poll(&s, 800, &e));
    complete(&s, 2436, 2000, 1000, 255, 0);
    CHECK(sc_epochs_poll(&s, 1250, &e));
    CHECK((e.reasons & SC_SEG_JUMP) != 0 && e.jumps == 255); /* 0->255 retained; no +255ms guess. */
    complete(&s, 2436, 3000, 2000, 0, 1);
    CHECK(sc_epochs_poll(&s, 2250, &e));
    CHECK((e.reasons & (SC_SEG_JUMP | SC_SEG_SYSTEM)) == (SC_SEG_JUMP | SC_SEG_SYSTEM));
    o = obs(1, 2436, 4000);
    o.data.status.rx_state = 112;
    sc_epochs_push(&s, &o, 2300);
    CHECK(!sc_epochs_pps_ready(&s, 2300)); /* Immediate loss of reference veto. */
    CHECK(sc_epochs_poll(&s, 2550, &e) && !e.sources_valid && e.mask == SC_EPOCH_STATUS);
    CHECK((e.reasons & SC_SEG_INCOMPLETE) != 0);
    sc_epochs_break(&s, SC_SEG_CONFIG);
    CHECK(!sc_epochs_pps_ready(&s, 2600));
    complete(&s, 2436, 5000, 3000, 128, 0);
    o = obs(0, 2436, 5000);
    sc_epochs_push(&s, &o, 3100);
    CHECK(sc_epochs_poll(&s, 3250, &e) && !e.sources_valid && e.duplicate == 1 &&
          (e.reasons & SC_SEG_DUPLICATE));
    o = obs(1, 2436, 500);
    o.data.status.uptime_s = 1;
    sc_epochs_push(&s, &o, 4000);
    CHECK(s.receiver_boot == 1 && !sc_epochs_pps_ready(&s, 4000));
    CHECK(sc_epochs_poll(&s, 4250, &e) && (e.reasons & SC_SEG_RESET));
    memset(&s, 0, sizeof(s));
    complete(&s, 2436, 604799000, 100, 0, 0);
    CHECK(sc_epochs_poll(&s, 350, &e));
    s.have_status = false; /* Receiver uptime in helper is not a rollover model. */
    complete(&s, 2437, 0, 1100, 0, 0);
    CHECK(sc_epochs_poll(&s, 1350, &e));
    CHECK(e.reasons == 0);
    complete(&s, 2437, 3000, 2100, 128, 0);
    CHECK(sc_epochs_poll(&s, 2350, &e));
    CHECK((e.reasons & (SC_SEG_GAP | SC_SEG_JUMP)) == (SC_SEG_GAP | SC_SEG_JUMP));
    sc_epochs_break(&s, SC_SEG_LOSS);
    for (unsigned i = 0; i < 5; i++)
    {
        o = obs(0, 2437, 10000 + i * 1000);
        sc_epochs_push(&s, &o, 3000 + i);
    }
    CHECK(s.discarded >= 4 && !sc_epochs_pps_ready(&s, 3010));
    memset(&s, 0, sizeof(s));
    complete(&s, 1, 1000, 0, 0, 0);
    o = obs(0, 1, 2000);
    o.data.pvt.alert = 2;
    sc_epochs_push(&s, &o, 100);
    CHECK(sc_epochs_poll(&s, 250, &e) && e.sources_valid);
    CHECK(!sc_epochs_pps_ready(&s, 250)); /* New bad slot survives older publish. */
    sc_epochs_break(&s, SC_SEG_LOSS);
    o = obs(0, 1, 3000);
    o.valid = 0;
    sc_epochs_push(&s, &o, 300);
    CHECK(!sc_epochs_poll(&s, 1000, &e));
    sc_pps_expire(&p, 9999);
    CHECK(p.state == SC_PPS_ABSENT);
    sc_pps_edge(&p, UINT32_MAX - 500000, UINT32_MAX - 500, UINT32_MAX, false);
    CHECK(!p.interval_valid && p.state == SC_PPS_ACQUIRING);
    sc_pps_edge(&p, 499999, 499, 0, false);
    CHECK(p.interval_valid && p.interval_us == 1000000);
    sc_pps_edge(&p, 1499999, 1499, 1, false);
    sc_pps_edge(&p, 2499999, 2499, 2, false);
    CHECK(p.state == SC_PPS_PRESENT);
    sc_pps_expire(&p, 3999);
    CHECK(p.state == SC_PPS_PRESENT);
    sc_pps_expire(&p, 4000);
    CHECK(p.state == SC_PPS_LOST);
    sc_pps_edge(&p, 3499999, 5000, 3, false);
    CHECK(!p.interval_valid);
    sc_pps_edge(&p, 4499999, 6000, 5, false);
    CHECK(!p.interval_valid); /* queue loss */
    sc_pps_edge(&p, 5499999, 7000, 6, true);
    CHECK(!p.interval_valid); /* overcapture */
    sc_pps_edge(&p, 5500000, 7000, 7, false);
    CHECK(!p.interval_valid); /* extra edge */
    puts("timing tests passed");
    return 0;
}
