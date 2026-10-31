/* SPDX-License-Identifier: MIT */
/* USB telemetry line contract, checked on every line a test's fake CDC sees.
 * Host tools split each line on single spaces into key=value fields, so a
 * value with a space or '=' would silently lose data there. Contract: printable
 * ASCII, one CRLF at the end, first field type=<name>, single-space separated
 * key=value fields with [A-Za-z0-9_] keys, unique keys, values without spaces
 * or '=' (empty values are allowed). A violation ends the test process. */
#ifndef SIMPLECLOCK_TEST_TELEMETRY_CHECK_H
#define SIMPLECLOCK_TEST_TELEMETRY_CHECK_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static void telemetry_fail(const uint8_t *data, uint16_t length, const char *why)
{
    fprintf(stderr, "Malformed USB telemetry (%s): %.*s\n", why, (int)length, (const char *)data);
    exit(EXIT_FAILURE);
}

static void telemetry_check(const uint8_t *data, uint16_t length)
{
    if (length < 7U || data[length - 2U] != '\r' || data[length - 1U] != '\n')
    {
        telemetry_fail(data, length, "no CRLF end");
    }
    const uint16_t end = (uint16_t)(length - 2U);
    if (memcmp(data, "type=", 5U) != 0)
    {
        telemetry_fail(data, length, "first field is not type=");
    }
    uint16_t starts[96];
    uint16_t key_lengths[96];
    unsigned fields = 0U;
    uint16_t i = 0U;
    while (i < end)
    {
        const uint16_t start = i;
        while (i < end && data[i] != '=')
        {
            const uint8_t c = data[i];
            if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') ||
                  c == '_'))
            {
                telemetry_fail(data, length, "bad key character or field without '='");
            }
            i++;
        }
        if (i == start || i == end)
        {
            telemetry_fail(data, length, "empty key or missing '='");
        }
        const uint16_t key_length = (uint16_t)(i - start);
        for (unsigned k = 0U; k < fields; k++)
        {
            if (key_lengths[k] == key_length &&
                memcmp(data + starts[k], data + start, key_length) == 0)
            {
                telemetry_fail(data, length, "duplicate key");
            }
        }
        if (fields == sizeof(starts) / sizeof(starts[0]))
        {
            telemetry_fail(data, length, "too many fields for the checker");
        }
        starts[fields] = start;
        key_lengths[fields++] = key_length;
        i++; /* '=' */
        while (i < end && data[i] != ' ')
        {
            if (data[i] < 0x21U || data[i] > 0x7EU || data[i] == '=')
            {
                telemetry_fail(data, length, "value contains a space, '=' or non-printable byte");
            }
            i++;
        }
        if (i < end)
        {
            i++; /* single separating space */
            if (i == end || data[i] == ' ')
            {
                telemetry_fail(data, length, "empty field");
            }
        }
    }
}

#endif
