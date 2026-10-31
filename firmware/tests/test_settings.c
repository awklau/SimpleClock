/* SPDX-License-Identifier: MIT */
#include "simpleclock/settings.h"
#include "simpleclock/hybrid.h"
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
static uint8_t pages[2][SC_SETTINGS_RECORD_BYTES];
static unsigned erases, programs, fail_program, partial_bytes;
static bool fail_read, fail_erase, corrupt_readback;

static bool read_slot(void *context, unsigned slot, uint8_t *out)
{
    (void)context;
    if (fail_read)
    {
        return false;
    }
    memcpy(out, pages[slot], SC_SETTINGS_RECORD_BYTES);
    if (corrupt_readback && slot == 1U)
    {
        out[17] ^= 1U;
    }
    return true;
}

static bool erase_slot(void *context, unsigned slot)
{
    (void)context;
    erases++;
    memset(pages[slot], UINT8_MAX, SC_SETTINGS_RECORD_BYTES);
    return !fail_erase;
}

static bool program_slot(void *context, unsigned slot, unsigned offset, const uint8_t *bytes)
{
    (void)context;
    programs++;
    const unsigned count = programs == fail_program ? partial_bytes : 8U;
    for (unsigned i = 0U; i < count; i++)
    {
        pages[slot][offset + i] &= bytes[i];
    }
    return programs != fail_program;
}

static const sc_settings_io_t io = {NULL, read_slot, erase_slot, program_slot};

/* Golden v1 records generated independently with Python struct/zlib. */
static const uint8_t legacy[3][64] = {
    {0x46, 0x43, 0x43, 0x53, 0x01, 0x00, 0x00, 0x00, 0x2a, 0x00, 0x00, 0x00, 0x40,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64, 0x00,
     0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0xd0, 0x07, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x3e, 0x29, 0x14, 0x0f,
     0xc1, 0xd6, 0xeb, 0xf0, 0x45, 0x56, 0x41, 0x53, 0xba, 0xa9, 0xbe, 0xac},
    {0x46, 0x43, 0x43, 0x53, 0x01, 0x00, 0x00, 0x00, 0x2a, 0x00, 0x00, 0x00, 0x40,
     0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64, 0x00,
     0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0xd0, 0x07, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x94, 0x2c, 0xce, 0xfe,
     0x6b, 0xd3, 0x31, 0x01, 0x45, 0x56, 0x41, 0x53, 0xba, 0xa9, 0xbe, 0xac},
    {0x46, 0x43, 0x43, 0x53, 0x01, 0x00, 0x00, 0x00, 0x2a, 0x00, 0x00, 0x00, 0x40,
     0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x64, 0x00,
     0x00, 0x00, 0xe8, 0x03, 0x00, 0x00, 0xc8, 0x00, 0x00, 0x00, 0xd0, 0x07, 0x00,
     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2b, 0x24, 0xd1, 0x37,
     0xd4, 0xdb, 0x2e, 0xc8, 0x45, 0x56, 0x41, 0x53, 0xba, 0xa9, 0xbe, 0xac}};

static int test_legacy_migration(void)
{
    for (unsigned mode = 0U; mode < 3U; mode++)
    {
        memset(pages, UINT8_MAX, sizeof(pages));
        memcpy(pages[0], legacy[mode], sizeof(legacy[mode]));
        sc_settings_t loaded;
        unsigned slot;
        uint32_t generation;
        erases = programs = 0U;
        CHECK(sc_settings_load(&io, &loaded, &generation, &slot) == SC_SETTINGS_VALID);
        CHECK(generation == 42U && slot == 0U && loaded.gnss_selection == SC_SETTINGS_GNSS_SBAS);
        CHECK(loaded.ppp_frequency_s == (mode == SC_HYBRID_LOOP_PHASE ? 0U : 100U));
        CHECK(loaded.ppp_phase_s == (mode == SC_HYBRID_LOOP_FREQUENCY ? 0U : 1000U));
        CHECK(loaded.sbas_frequency_s == (mode == SC_HYBRID_LOOP_PHASE ? 0U : 200U));
        CHECK(loaded.sbas_phase_s == (mode == SC_HYBRID_LOOP_FREQUENCY ? 0U : 2000U));
        CHECK(loaded.standalone_frequency_s == loaded.sbas_frequency_s &&
              loaded.standalone_phase_s == loaded.sbas_phase_s);
        CHECK(erases == 0U && programs == 0U);
        loaded.gnss_selection = SC_SETTINGS_GNSS_AUTO;
        CHECK(sc_settings_save(&io, &loaded));
        CHECK(pages[1][4] == 5U && erases == 1U && programs == 9U);
        CHECK(loaded.static_frequency_s == loaded.standalone_frequency_s &&
              loaded.static_phase_s == loaded.standalone_phase_s);
        sc_settings_t restored;
        CHECK(sc_settings_load(&io, &restored, &generation, &slot) == SC_SETTINGS_VALID);
        CHECK(slot == 1U && generation == 43U && sc_settings_equal(&loaded, &restored));
    }
    erases = programs = 0U;
    return 0;
}

static int test_v2_profile_migration(void)
{
    /* Independently packed pre-profile v2 record, generation 1. */
    const uint8_t v2[SC_SETTINGS_RECORD_BYTES] =
        "\x46\x43\x43\x53\x02\x00\x00\x00\x01\x00\x00\x00\x40\x00\x00\x00"
        "\x01\x00\x00\x00\x00\x00\x00\x00\x0a\x00\x00\x00\x00\x00\x00\x00"
        "\x0a\x00\x00\x00\x64\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00"
        "\x9b\x6a\xc1\x88\x64\x95\x3e\x77\x45\x56\x41\x53\xba\xa9\xbe\xac";
    sc_settings_t loaded, again;
    uint32_t generation;
    CHECK(sc_settings_decode(v2, &loaded, &generation));
    CHECK(generation == 1U && loaded.receiver_user_profile == 0U &&
          loaded.gnss_selection == SC_SETTINGS_GNSS_SBAS);
    CHECK(loaded.ppp_frequency_s == 0U && loaded.ppp_phase_s == 10U &&
          loaded.standalone_frequency_s == 100U);
    /* Options absent from older schemas take the compiled defaults. */
    CHECK(loaded.pps_policy == SC_SETTINGS_PPS_CONTINUOUS &&
          loaded.bias_clip_ps == SC_HYBRID_BIAS_CLIP_DEFAULT_PS);
    /* Static fixed-position solutions used the standalone pair before v5. */
    CHECK(loaded.static_frequency_s == 100U && loaded.static_phase_s == 0U);
    loaded.receiver_user_profile = 1U;
    uint8_t v5[SC_SETTINGS_RECORD_BYTES];
    /* Word 44: profile bit 0 | policy << 1 | reserved bits 3..6 | clip ps << 7. */
    CHECK(sc_settings_encode(v5, &loaded, 2U) && v5[4] == 5U && v5[12] == 72U);
    CHECK(v5[44] == 0x01U && v5[45] == 0xe8U && v5[46] == 0x03U && v5[47] == 0U);
    CHECK(v5[48] == 100U && v5[52] == 0U); /* Static pair, words 48/52. */
    CHECK(sc_settings_decode(v5, &again, &generation) && sc_settings_equal(&loaded, &again));
    loaded.receiver_user_profile = 2U;
    CHECK(!sc_settings_encode(v5, &loaded, 3U));
    return 0;
}

/* Independent Python struct/zlib.crc32 records: a v3 record (profile bit only)
 * and v4 records with every packed option. The `v4` and `reserved` records set
 * reserved bits 3..6 (a removed filter option): ignored on read, written 0.
 * Out-of-range clips with a correct CRC must still be rejected. */
static int test_v3_v4_records(void)
{
    const uint8_t v3[SC_SETTINGS_RECORD_BYTES] =
        "\x46\x43\x43\x53\x03\x00\x00\x00\x07\x00\x00\x00\x40\x00\x00\x00"
        "\x00\x00\x00\x00\x3c\x00\x00\x00\x58\x02\x00\x00\x78\x00\x00\x00"
        "\xb8\x0b\x00\x00\x78\x00\x00\x00\xb8\x0b\x00\x00\x01\x00\x00\x00"
        "\x31\x3a\xda\x5d\xce\xc5\x25\xa2\x45\x56\x41\x53\xba\xa9\xbe\xac";
    const uint8_t v4[SC_SETTINGS_RECORD_BYTES] =
        "\x46\x43\x43\x53\x04\x00\x00\x00\x09\x00\x00\x00\x40\x00\x00\x00"
        "\x02\x00\x00\x00\x0a\x00\x00\x00\x00\x00\x00\x00\x0a\x00\x00\x00"
        "\x64\x00\x00\x00\x0a\x00\x00\x00\x64\x00\x00\x00\x3b\xc4\x09\x00"
        "\xc4\xa0\xf5\x79\x3b\x5f\x0a\x86\x45\x56\x41\x53\xba\xa9\xbe\xac";
    const uint8_t reserved[SC_SETTINGS_RECORD_BYTES] =
        "\x46\x43\x43\x53\x04\x00\x00\x00\x09\x00\x00\x00\x40\x00\x00\x00"
        "\x02\x00\x00\x00\x0a\x00\x00\x00\x00\x00\x00\x00\x0a\x00\x00\x00"
        "\x64\x00\x00\x00\x0a\x00\x00\x00\x64\x00\x00\x00\x58\xe8\x03\x00"
        "\x27\xaf\xde\x9b\xd8\x50\x21\x64\x45\x56\x41\x53\xba\xa9\xbe\xac";
    const uint8_t bad[2][SC_SETTINGS_RECORD_BYTES] = {
        "\x46\x43\x43\x53\x04\x00\x00\x00\x09\x00\x00\x00\x40\x00\x00\x00"
        "\x02\x00\x00\x00\x0a\x00\x00\x00\x00\x00\x00\x00\x0a\x00\x00\x00"
        "\x64\x00\x00\x00\x0a\x00\x00\x00\x64\x00\x00\x00\x00\x00\x00\x00"
        "\xb1\x5a\xbb\x19\x4e\xa5\x44\xe6\x45\x56\x41\x53\xba\xa9\xbe\xac",
        "\x46\x43\x43\x53\x04\x00\x00\x00\x09\x00\x00\x00\x40\x00\x00\x00"
        "\x02\x00\x00\x00\x0a\x00\x00\x00\x00\x00\x00\x00\x0a\x00\x00\x00"
        "\x64\x00\x00\x00\x0a\x00\x00\x00\x64\x00\x00\x00\x80\xc4\x09\x00"
        "\x5f\x69\x86\xb3\xa0\x96\x79\x4c\x45\x56\x41\x53\xba\xa9\xbe\xac"};
    sc_settings_t loaded, again;
    uint32_t generation;
    CHECK(sc_settings_decode(v3, &loaded, &generation) && generation == 7U);
    CHECK(loaded.gnss_selection == SC_SETTINGS_GNSS_AUTO && loaded.receiver_user_profile == 1U);
    CHECK(loaded.ppp_frequency_s == 60U && loaded.ppp_phase_s == 600U &&
          loaded.standalone_phase_s == 3000U);
    CHECK(loaded.pps_policy == SC_SETTINGS_PPS_CONTINUOUS &&
          loaded.bias_clip_ps == SC_HYBRID_BIAS_CLIP_DEFAULT_PS);
    CHECK(sc_settings_decode(v4, &loaded, &generation) && generation == 9U);
    CHECK(loaded.gnss_selection == SC_SETTINGS_GNSS_HAS && loaded.receiver_user_profile == 1U);
    CHECK(loaded.pps_policy == SC_SETTINGS_PPS_QUALIFIED && loaded.bias_clip_ps == 5000U);
    uint8_t encoded[SC_SETTINGS_RECORD_BYTES];
    CHECK(loaded.static_frequency_s == 10U && loaded.static_phase_s == 100U);
    CHECK(sc_settings_encode(encoded, &loaded, 9U) && (encoded[44] & 0x78U) == 0U);
    CHECK(encoded[4] == 5U && memcmp(encoded + 16U, v4 + 16U, 28U) == 0 && encoded[45] == v4[45] &&
          encoded[46] == v4[46]);
    CHECK(sc_settings_decode(encoded, &again, &generation) && sc_settings_equal(&loaded, &again));
    CHECK(sc_settings_decode(reserved, &again, &generation) && again.bias_clip_ps == 2000U);
    for (unsigned i = 0U; i < 2U; i++)
    {
        CHECK(!sc_settings_decode(bad[i], &again, &generation)); /* clip 0, clip 5001 */
    }
    return 0;
}

int main(void)
{
    CHECK(test_legacy_migration() == 0);
    CHECK(test_v2_profile_migration() == 0);
    CHECK(test_v3_v4_records() == 0);
    memset(pages, UINT8_MAX, sizeof(pages));
    sc_settings_t settings, loaded;
    sc_settings_defaults(&settings);
    unsigned slot;
    uint32_t generation;
    CHECK(sc_settings_load(&io, &loaded, &generation, &slot) == SC_SETTINGS_EMPTY);
    CHECK(sc_settings_equal(&loaded, &settings));
    settings.gnss_selection = SC_SETTINGS_GNSS_SBAS;
    settings.ppp_frequency_s = 0U;
    settings.ppp_phase_s = 10U;
    settings.sbas_frequency_s = 0U;
    settings.sbas_phase_s = 10U;
    settings.standalone_frequency_s = 100U;
    settings.standalone_phase_s = 0U;
    CHECK(sc_settings_save(&io, &settings));
    CHECK(sc_settings_load(&io, &loaded, &generation, &slot) == SC_SETTINGS_VALID);
    CHECK(sc_settings_equal(&loaded, &settings) && generation == 1U && slot == 0U);
    CHECK(erases == 1U && programs == 9U);
    /* Independent Python zlib.crc32 vector for this v5 payload (default 2 ns
     * clip, continuous PPS, default static pair 10/100 s): 0x7c4ee069. */
    CHECK(pages[0][56] == 0x69U && pages[0][57] == 0xe0U && pages[0][58] == 0x4eU &&
          pages[0][59] == 0x7cU);
    CHECK(sc_settings_save(&io, &settings) && erases == 1U && programs == 9U);
    settings.sbas_phase_s = 9U;
    CHECK(!sc_settings_save(&io, &settings) && erases == 1U);
    settings.sbas_phase_s = 10U;
    settings.ppp_frequency_s = 10U;
    CHECK(sc_settings_valid(&settings)); /* Ratio is only a recommendation. */
    settings.ppp_frequency_s = 86400U;
    uint8_t independent[SC_SETTINGS_RECORD_BYTES];
    CHECK(sc_settings_encode(independent, &settings, 50U));
    CHECK(sc_settings_decode(independent, &loaded, &generation));
    CHECK(loaded.ppp_frequency_s == 86400U && loaded.ppp_phase_s == 10U);
    settings.ppp_phase_s = settings.sbas_phase_s = 25U;
    CHECK(sc_settings_valid(&settings));
    settings.ppp_frequency_s = 0U;
    settings.ppp_phase_s = settings.sbas_phase_s = 10U;
    const sc_settings_t original = settings;
    settings.sbas_phase_s = 100U;
    uint8_t baseline[sizeof(pages)];
    memcpy(baseline, pages, sizeof(pages));
    /* Simulated power failure at every byte boundary of every programmed
     * double word, including the final commit word. Old slot stays intact. */
    for (unsigned word = 1U; word <= 9U; word++)
    {
        for (unsigned bytes = 0U; bytes <= 8U; bytes++)
        {
            memcpy(pages, baseline, sizeof(pages));
            programs = 0U;
            fail_program = word;
            partial_bytes = bytes;
            CHECK(!sc_settings_save(&io, &settings));
            CHECK(memcmp(pages[0], baseline, SC_SETTINGS_RECORD_BYTES) == 0);
            CHECK(sc_settings_load(&io, &loaded, &generation, &slot) == SC_SETTINGS_VALID);
            CHECK(sc_settings_equal(&loaded, word == 9U && bytes == 8U ? &settings : &original));
        }
    }
    fail_program = 0U;
    memcpy(pages, baseline, sizeof(pages));
    fail_erase = true;
    CHECK(!sc_settings_save(&io, &settings));
    fail_erase = false;
    fail_read = true;
    CHECK(!sc_settings_save(&io, &settings));
    fail_read = false;
    corrupt_readback = true;
    CHECK(!sc_settings_save(&io, &settings));
    corrupt_readback = false;
    CHECK(sc_settings_save(&io, &settings));
    CHECK(sc_settings_load(&io, &loaded, &generation, &slot) == SC_SETTINGS_VALID);
    CHECK(generation == 2U && slot == 1U && sc_settings_equal(&loaded, &settings));
    /* Any one-bit record corruption rejects the newer slot and recovers old. */
    uint8_t newest[SC_SETTINGS_RECORD_BYTES];
    memcpy(newest, pages[1], sizeof(newest));
    for (unsigned byte = 0U; byte < SC_SETTINGS_RECORD_BYTES; byte++)
    {
        for (unsigned bit = 0U; bit < 8U; bit++)
        {
            memcpy(pages[1], newest, sizeof(newest));
            pages[1][byte] ^= (uint8_t)(1U << bit);
            CHECK(sc_settings_load(&io, &loaded, &generation, &slot) == SC_SETTINGS_VALID);
            CHECK(slot == 0U && sc_settings_equal(&loaded, &original));
        }
    }
    CHECK(sc_settings_encode(pages[0], &original, UINT32_MAX));
    CHECK(sc_settings_encode(pages[1], &settings, 0U));
    CHECK(sc_settings_load(&io, &loaded, &generation, &slot) == SC_SETTINGS_VALID);
    CHECK(slot == 1U && generation == 0U && sc_settings_equal(&loaded, &settings));
    memset(pages, 0U, sizeof(pages));
    CHECK(sc_settings_load(&io, &loaded, &generation, &slot) == SC_SETTINGS_INVALID);
    sc_settings_defaults(&settings);
    CHECK(sc_settings_equal(&loaded, &settings));
    return 0;
}
