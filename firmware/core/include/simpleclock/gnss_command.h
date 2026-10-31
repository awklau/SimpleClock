/* SPDX-License-Identifier: MIT */
#ifndef SIMPLECLOCK_GNSS_COMMAND_H
#define SIMPLECLOCK_GNSS_COMMAND_H
#include <stdbool.h>
#include <stdint.h>

/* One foreground-owned transaction. An echoed command plus a following COM1>
 * prompt completes it; an unrelated asynchronous message cannot acknowledge it.
 * State/line storage must be zero-initialized before first use. */

typedef enum
{
    SC_CMD_IDLE,
    SC_CMD_WAITING,
    SC_CMD_OK,
    SC_CMD_REJECTED,
    SC_CMD_TIMEOUT
} sc_cmd_state_t;

/* Optional readback gates for the supplied G5 1.1.0 PPP bench workflow.
 * Tokens are checked in their documented comma-separated fields, even when a
 * capability/signal list is longer than the ordinary echo line buffer. */
typedef enum
{
    SC_CMD_EXPECT_NONE,
    SC_CMD_EXPECT_HAS_CAPABILITY,
    SC_CMD_EXPECT_E6_TRACKING,
    SC_CMD_EXPECT_E6_NAVIGATION,
    SC_CMD_EXPECT_STANDALONE,
    SC_CMD_EXPECT_STANDALONE_PPP,
    SC_CMD_EXPECT_PPP_TEST_MODE,
    SC_CMD_EXPECT_SBAS_CAPABILITY,
    SC_CMD_EXPECT_STANDALONE_SBAS,
    SC_CMD_EXPECT_BENCH_PVT_MODE,
    SC_CMD_EXPECT_MULTIMODE,
    SC_CMD_EXPECT_ELEVATION_PVT,
    SC_CMD_EXPECT_ELEVATION_TRACKING,
    SC_CMD_EXPECT_SAVE_BOOT,
    /* Exact expectations encode degrees + 90, without extra transaction storage. */
    SC_CMD_EXPECT_PVT_MASK_BASE = 128,
    SC_CMD_EXPECT_TRACKING_MASK_BASE = 384
} sc_cmd_expect_t;

typedef struct
{
    char word[32];
    uint8_t used, field, bits;
    bool selected, bad, overflow, seen, matched;
    int8_t elevation_deg;
    bool elevation_valid;
} sc_cmd_readback_t;

typedef struct
{
    const char *command;
    char line[192];
    uint32_t started_ms;
    unsigned used;
    sc_cmd_state_t state;
    bool echo, rejected, overflow;
    sc_cmd_expect_t expected;
    sc_cmd_readback_t readback;
} sc_gnss_command_t;

/* command includes CR/LF and remains alive until the transaction ends. This
 * helper only recognizes replies: the caller owns UART TX and timeout polling.
 * start returns false for a busy transaction or an overlong/NULL command. */
bool sc_gnss_command_start(sc_gnss_command_t *transaction, const char *command, uint32_t now_ms);
/* A matching echo and prompt without the required readback is REJECTED.
 * PPP_TEST_MODE accepts only Rover/(StandAlone or StandAlone+PPP)/auto, for
 * restoration after an uncertain setter; it never accepts other settings. */
bool sc_gnss_command_start_checked(sc_gnss_command_t *transaction, const char *command,
                                   sc_cmd_expect_t expected, uint32_t now_ms);
void sc_gnss_command_byte(sc_gnss_command_t *transaction, uint8_t byte);
void sc_gnss_command_expire(sc_gnss_command_t *transaction, uint32_t now_ms);
#endif
