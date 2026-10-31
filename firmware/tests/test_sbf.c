/* SPDX-License-Identifier: MIT */
#include "simpleclock/sbf.h"
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

typedef struct
{
    unsigned frames, ascii;
    uint16_t length;
    uint8_t prefix[SC_SBF_PREFIX];
} capture_t;

static void frame(void *ctx, const uint8_t *p, uint16_t length, bool structure_valid)
{
    (void)structure_valid;
    capture_t *c = ctx;
    c->frames++;
    c->length = length;
    memcpy(c->prefix, p, length < SC_SBF_PREFIX ? length : SC_SBF_PREFIX);
}

static void ascii(void *ctx, uint8_t byte)
{
    capture_t *c = ctx;
    (void)byte;
    c->ascii++;
}

static void make_frame(uint8_t *data, uint16_t length)
{
    memset(data, 0, length);
    data[0] = '$';
    data[1] = '@';
    data[4] = 0xff;
    data[5] = 0x0f;
    data[6] = (uint8_t)length;
    data[7] = (uint8_t)(length >> 8);
    const uint16_t crc = simpleclock_crc16_ccitt(data + 4, (size_t)length - 4U);
    data[2] = (uint8_t)crc;
    data[3] = (uint8_t)(crc >> 8);
}

int main(void)
{
    uint8_t data[4096];
    make_frame(data, 4096);
    /* Unknown block ID: this test checks framing independently of body schema.
     * The protocol decoder tests exercise structurally valid large MeasEpochs. */
    CHECK(data[2] == 0x43 && data[3] == 0xee); /* independent Python crc_hqx oracle */
    for (unsigned split = 0; split <= 4096; split++)
    {
        sc_sbf_t s = {0};
        capture_t c = {0};
        sc_sbf_feed(&s, data, split, 0, frame, ascii, &c);
        sc_sbf_feed(&s, data + split, 4096 - split, 1, frame, ascii, &c);
        CHECK(c.frames == 1 && c.length == 4096 && c.ascii == 0 && s.frames == 1);
        CHECK(memcmp(c.prefix, data, SC_SBF_PREFIX) == 0);
    }
    sc_sbf_t s = {0};
    capture_t c = {0};
    const uint8_t reply[] = "$R: example\r\nCOM1>";
    sc_sbf_feed(&s, reply, sizeof(reply) - 1U, 0, frame, ascii, &c);
    CHECK(c.ascii == sizeof(reply) - 1U);
    data[3000] ^= 1U;
    sc_sbf_feed(&s, data, 4096, 1, frame, ascii, &c);
    CHECK(s.crc_errors == 1 && c.frames == 0);
    data[3000] ^= 1U;
    sc_sbf_feed(&s, data, 8, UINT32_MAX - 100U, frame, ascii, &c);
    sc_sbf_expire(&s, 101U, 115200);
    CHECK(s.timeouts == 1 && s.used == 0);
    sc_sbf_feed(&s, data, 4096, 200, frame, ascii, &c);
    CHECK(c.frames == 1);
    data[6] = 9;
    data[7] = 0;
    sc_sbf_feed(&s, data, 8, 201, frame, ascii, &c);
    CHECK(s.length_errors == 1);
    make_frame(data, 8);
    sc_sbf_feed(&s, data, 8, 202, frame, ascii, &c);
    CHECK(c.frames == 2);
    _Static_assert(sizeof(sc_sbf_t) < 256, "wire length must not become a RAM allocation");
    return 0;
}
