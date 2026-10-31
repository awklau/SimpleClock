/* SPDX-License-Identifier: MIT */
#include "simpleclock/sit5801.h"
#include <math.h>
#include <stdio.h>
#include <stddef.h>

#define CHECK(expr)                                                                                \
    do                                                                                             \
    {                                                                                              \
        if (!(expr))                                                                               \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #expr);                                     \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

int main(void)
{
    static const struct
    {
        double ppb;
        int64_t q;
        uint16_t words[3];
    } vectors[] = {{0.0, 0, {0, 0, 0}},
                   {1.0, 343597, {0, 0x0a7c, 0x5a00}},
                   {10.0, 3435974, {0, 0x68db, 0x8c00}},
                   {-10.0, -3435974, {0xffff, 0x9724, 0x7400}},
                   {3125.0, 1073741824, {0x0080, 0, 0}},
                   {-3125.0, -1073741824, {0xff80, 0, 0}}};

    uint16_t words[3];
    int64_t q;
    for (size_t i = 0; i < sizeof(vectors) / sizeof(vectors[0]); i++)
    {
        CHECK(sit5801_ppb_to_fcw(vectors[i].ppb, &q));
        CHECK(q == vectors[i].q);
        CHECK(sit5801_fcw_encode(q, words));
        for (unsigned j = 0; j < 3; j++)
        {
            CHECK(words[j] == vectors[i].words[j]);
        }
        CHECK(sit5801_fcw_decode(words) == q);
        CHECK(fabs(sit5801_fcw_to_ppb(q) - vectors[i].ppb) < 0.0000015);
    }
    static const int64_t endpoints[] = {SIT5801_FCW_MIN, SIT5801_FCW_MAX, -1, 1};
    for (unsigned i = 0; i < 4; i++)
    {
        CHECK(sit5801_fcw_encode(endpoints[i], words));
        CHECK(sit5801_fcw_decode(words) == endpoints[i]);
    }
    CHECK(!sit5801_fcw_encode(SIT5801_FCW_MIN - 1, words));
    CHECK(!sit5801_fcw_encode(SIT5801_FCW_MAX + 1, words));
    CHECK(!sit5801_ppb_to_fcw(NAN, &q));
    CHECK(!sit5801_ppb_to_fcw(INFINITY, &q));
    CHECK(!sit5801_ppb_to_fcw(800001.0, &q));
    CHECK(!sit5801_ppb_to_fcw(-800001.0, &q));
    CHECK(sit5801_clip_ppb(32) == 3125.0);
    CHECK(sit5801_clip_ppb(0) == 800000.0);
    CHECK(sit5801_power_code(0xffff, 0xc000) == 262143U);

    sit5801_probe_t p = {0};
    uint8_t reg;
    uint8_t bytes[2] = {0xd0, 0x01};
    CHECK(sit5801_probe_begin(&p, UINT32_MAX - 20U, &reg) && reg == 0x2a);
    CHECK(!sit5801_probe_begin(&p, 0, &reg));
    sit5801_probe_complete(&p, UINT32_MAX - 10U, true, bytes);
    CHECK(!sit5801_probe_begin(&p, 38U, &reg));
    CHECK(sit5801_probe_begin(&p, 39U, &reg) && reg == 0x00);
    bytes[0] = 0;
    bytes[1] = 32;
    sit5801_probe_complete(&p, 40, true, bytes);
    for (unsigned i = 2; i < 8; i++)
    {
        CHECK(sit5801_probe_begin(&p, i * 100U, &reg));
        bytes[0] = 0;
        bytes[1] = 0;
        sit5801_probe_complete(&p, i * 100U + 3U, true, bytes);
    }
    CHECK(p.sequence == 1 && sit5801_probe_word_order(&p) == 1);
    p.wire[0] = 0x01d0;
    p.wire[1] = 0x2000;
    CHECK(sit5801_probe_word_order(&p) == 2);
    CHECK(sit5801_probe_begin(&p, 800, &reg));
    sit5801_probe_complete(&p, 809, false, NULL);
    CHECK(p.errors == 1 && sit5801_probe_word_order(&p) == 0);
    CHECK(!sit5801_probe_begin(&p, 858, &reg));
    CHECK(sit5801_probe_begin(&p, 859, &reg));
    return 0;
}
