/* SPDX-License-Identifier: MIT */
#include "simpleclock/bridge.h"
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
    uint8_t frame[SC_BRIDGE_FRAME_MAX], data[512];
    sc_bridge_parser_t p = {0};
    sc_bridge_packet_t out = {0};
    for (unsigned i = 0; i < 512; i++)
    {
        data[i] = (uint8_t)i;
    }
    const unsigned sizes[] = {0, 1, 63, 64, 65, 511, 512};
    for (unsigned k = 0; k < sizeof(sizes) / sizeof(sizes[0]); k++)
    {
        const uint16_t n = (uint16_t)sizes[k];
        size_t size = sc_bridge_encode(frame, sizeof(frame), 4, 0, UINT32_MAX, data, n);
        CHECK(size == (size_t)n + 14U);
        for (size_t i = 0; i < size; i++)
        {
            CHECK(sc_bridge_feed(&p, frame[i], &out) == (i + 1U == size ? 1 : 0));
        }
        CHECK(out.sequence == UINT32_MAX && out.type == 4 && out.length == n &&
              !memcmp(out.payload, data, n));
        frame[size - 1U] ^= 1;
        for (size_t i = 0; i < size; i++)
        {
            CHECK(sc_bridge_feed(&p, frame[i], &out) == (i + 1U == size ? -1 : 0));
        }
    }
    CHECK(!sc_bridge_encode(frame, sizeof(frame), 1, 0, 0, data, 513));
    CHECK(!sc_bridge_encode(frame, 13, 1, 0, 0, NULL, 0));
    CHECK(sc_bridge_feed(&p, 0, &out) == -1);
    (void)sc_bridge_encode(frame, sizeof(frame), 1, 0, 0, NULL, 0);
    frame[6] = 1;
    frame[7] = 2;
    for (unsigned i = 0; i < 12; i++)
    {
        CHECK(sc_bridge_feed(&p, frame[i], &out) == (i == 11 ? -1 : 0));
    }
    CHECK(p.used == 0 && p.total == 0);
    /* Cross-language wire vector (Python binascii CRC, all little endian). */
    const uint8_t golden[] = {0x53, 0x43, 0x42, 0x31, 0x02, 0x00, 0x04, 0x00, 0x78,
                              0x56, 0x34, 0x12, 0x00, 0x11, 0x13, 0xff, 0x4d, 0x36};
    CHECK(sc_bridge_encode(frame, sizeof(frame), 2, 0, 0x12345678U, golden + 12, 4) ==
          sizeof(golden));
    CHECK(memcmp(frame, golden, sizeof(golden)) == 0);
    return 0;
}
