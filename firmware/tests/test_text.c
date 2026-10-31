/* SPDX-License-Identifier: MIT */
#include "simpleclock/text.h"
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

int main(void)
{
    char buffer[32];
    sc_text_t text;
    sc_text_init(&text, buffer, sizeof(buffer));
    sc_text_u32(&text, 0);
    sc_text_string(&text, " ");
    sc_text_u32(&text, UINT32_MAX);
    sc_text_string(&text, " ");
    sc_text_hex(&text, UINT32_MAX, 8);
    CHECK(text.ok && strcmp(buffer, "0 4294967295 FFFFFFFF") == 0);
    sc_text_init(&text, buffer, 4);
    sc_text_string(&text, "abc");
    CHECK(text.ok && text.length == 3);
    sc_text_string(&text, "def");
    CHECK(!text.ok && strcmp(buffer, "abc") == 0);
    sc_text_init(&text, buffer, 1);
    sc_text_u32(&text, 1);
    CHECK(!text.ok && buffer[0] == '\0');
    sc_text_init(&text, NULL, 0);
    sc_text_string(&text, "x");
    CHECK(!text.ok);
    sc_text_init(&text, buffer, sizeof(buffer));
    sc_text_i32(&text, INT32_MIN);
    CHECK(text.ok && strcmp(buffer, "-2147483648") == 0);
    sc_text_init(&text, buffer, sizeof(buffer));
    sc_text_i64(&text, INT64_MIN);
    CHECK(text.ok && strcmp(buffer, "-9223372036854775808") == 0);
    sc_text_init(&text, buffer, sizeof(buffer));
    sc_text_u64(&text, UINT64_MAX);
    CHECK(text.ok && strcmp(buffer, "18446744073709551615") == 0);
    sc_text_init(&text, buffer, 20);
    sc_text_u64(&text, UINT64_MAX);
    CHECK(!text.ok);
    sc_text_init(&text, buffer, sizeof(buffer));
    sc_text_q16(&text, -65535);
    CHECK(text.ok && strcmp(buffer, "-1.000") == 0);
    sc_text_init(&text, buffer, sizeof(buffer));
    sc_text_q16(&text, 65535);
    CHECK(text.ok && strcmp(buffer, "1.000") == 0);
    sc_text_init(&text, buffer, sizeof(buffer));
    sc_text_q16(&text, -1);
    CHECK(text.ok && strcmp(buffer, "0.000") == 0);
    sc_text_init(&text, buffer, sizeof(buffer));
    sc_text_hex(&text, 0, 9);
    CHECK(!text.ok);
    return 0;
}
