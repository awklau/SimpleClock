/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_BRIDGE_H
#define SIMPLECLOCK_BRIDGE_H
#include <stddef.h>
#include <stdint.h>
#define SC_BRIDGE_PAYLOAD_MAX 512U
#define SC_BRIDGE_PROTOCOL_VERSION 1U
#define SC_BRIDGE_HEADER_SIZE 12U
#define SC_BRIDGE_CRC_SIZE 2U
#define SC_BRIDGE_FRAME_MAX (SC_BRIDGE_PAYLOAD_MAX + SC_BRIDGE_HEADER_SIZE + SC_BRIDGE_CRC_SIZE)

/* Wire values shared with tools/gnss_bridge.py.
 * Keep existing numbers stable: host and maintenance image are deployed separately. */
enum
{
    SC_BRIDGE_HELLO = 1U,
    SC_BRIDGE_UART_TX = 2U,
    SC_BRIDGE_STATUS = 3U,
    SC_BRIDGE_ECHO = 4U,
    SC_BRIDGE_HELLO_REPLY = 129U,
    SC_BRIDGE_UART_TX_REPLY = 130U,
    SC_BRIDGE_STATUS_REPLY = 131U,
    SC_BRIDGE_ECHO_REPLY = 132U,
    SC_BRIDGE_UART_RX = 144U,
    SC_BRIDGE_ERROR = 255U
};

enum
{
    SC_BRIDGE_FAULT_UART_RX = 1U,
    SC_BRIDGE_FAULT_USB_RX = 2U,
    SC_BRIDGE_FAULT_PROTOCOL = 4U,
    SC_BRIDGE_FAULT_DISCONNECT = 8U,
    SC_BRIDGE_FAULT_HEALTH = 16U
};

/* Zero-initialize before first use. This codec is caller-owned and has no I/O,
 * allocation, retries or resynchronization policy. Invalid input resets its
 * partial frame; the platform must latch the error before accepting more TX. */

/* SCB1, type:u8, flags:u8, length:u16LE, sequence:u32LE, payload, CRC16LE.
 * CRC covers header and payload. No USB framing bytes go to the receiver. */
typedef struct
{
    uint8_t data[SC_BRIDGE_FRAME_MAX];
    uint16_t used, total;
} sc_bridge_parser_t;

typedef struct
{
    uint8_t type, flags;
    uint16_t length;
    uint32_t sequence;
    const uint8_t *payload;
} sc_bridge_packet_t;

uint16_t sc_bridge_u16(const uint8_t *p);
uint32_t sc_bridge_u32(const uint8_t *p);
void sc_bridge_put32(uint8_t *p, uint32_t v);
/* 0 incomplete, 1 complete (payload lives until next feed), -1 invalid. */
int sc_bridge_feed(sc_bridge_parser_t *parser, uint8_t byte, sc_bridge_packet_t *out);
/* Returns frame length, or zero for insufficient capacity/invalid payload size.
 * out must reference capacity writable bytes; payload may be NULL only at length zero.
 * Input/output buffers must not overlap. Multi-byte wire fields are little endian. */
size_t sc_bridge_encode(uint8_t *out, size_t capacity, uint8_t type, uint8_t flags,
                        uint32_t sequence, const uint8_t *payload, uint16_t length);
#endif
