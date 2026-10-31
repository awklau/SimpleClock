/* SPDX-License-Identifier: MIT */
/* Portable SCB1 framing only. The platform owns sequencing, fault latching and I/O. */
#include "simpleclock/bridge.h"
#include "simpleclock/crc16.h"
#include <string.h>

uint16_t sc_bridge_u16(const uint8_t *bytes)
{
    return (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8));
}

uint32_t sc_bridge_u32(const uint8_t *bytes)
{
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) | ((uint32_t)bytes[2] << 16) |
           ((uint32_t)bytes[3] << 24);
}

void sc_bridge_put32(uint8_t *bytes, uint32_t value)
{
    for (unsigned i = 0; i < 4U; i++)
    {
        bytes[i] = (uint8_t)(value >> (8U * i));
    }
}

size_t sc_bridge_encode(uint8_t *frame, size_t capacity, uint8_t type, uint8_t flags,
                        uint32_t sequence, const uint8_t *payload, uint16_t payload_length)
{
    const size_t frame_length = (size_t)payload_length + SC_BRIDGE_HEADER_SIZE + SC_BRIDGE_CRC_SIZE;
    if (payload_length > SC_BRIDGE_PAYLOAD_MAX || capacity < frame_length ||
        (payload_length && payload == NULL))
    {
        return 0;
    }
    /* Explicit byte layout avoids host padding/alignment and endian assumptions. */
    memcpy(frame, "SCB1", 4);
    frame[4] = type;
    frame[5] = flags;
    frame[6] = (uint8_t)payload_length;
    frame[7] = (uint8_t)(payload_length >> 8);
    sc_bridge_put32(frame + 8, sequence);
    if (payload_length)
    {
        memcpy(frame + SC_BRIDGE_HEADER_SIZE, payload, payload_length);
    }
    uint16_t crc = simpleclock_crc16_ccitt(frame, frame_length - SC_BRIDGE_CRC_SIZE);
    frame[frame_length - 2U] = (uint8_t)crc;
    frame[frame_length - 1U] = (uint8_t)(crc >> 8);
    return frame_length;
}

int sc_bridge_feed(sc_bridge_parser_t *parser, uint8_t byte, sc_bridge_packet_t *packet)
{
    static const uint8_t magic[] = {'S', 'C', 'B', '1'};
    if (parser->used < sizeof(magic) && byte != magic[parser->used])
    {
        parser->used = 0;
        parser->total = 0;
        return -1;
    }
    parser->data[parser->used++] = byte;
    if (parser->used == SC_BRIDGE_HEADER_SIZE)
    {
        uint16_t payload_length = sc_bridge_u16(parser->data + 6);
        if (payload_length > SC_BRIDGE_PAYLOAD_MAX)
        {
            parser->used = 0;
            parser->total = 0;
            return -1;
        }
        parser->total = (uint16_t)(payload_length + SC_BRIDGE_HEADER_SIZE + SC_BRIDGE_CRC_SIZE);
    }
    if (!parser->total || parser->used != parser->total)
    {
        return 0;
    }
    /* Reset partial-frame state on either CRC outcome. Data remains available
     * to the returned packet until the caller supplies another byte. */
    const uint16_t frame_length = parser->total;
    parser->used = 0;
    parser->total = 0;
    if (simpleclock_crc16_ccitt(parser->data, (size_t)frame_length - SC_BRIDGE_CRC_SIZE) !=
        sc_bridge_u16(parser->data + frame_length - SC_BRIDGE_CRC_SIZE))
    {
        return -1;
    }
    packet->type = parser->data[4];
    packet->flags = parser->data[5];
    packet->length = sc_bridge_u16(parser->data + 6);
    packet->sequence = sc_bridge_u32(parser->data + 8);
    packet->payload = parser->data + SC_BRIDGE_HEADER_SIZE;
    return 1;
}
