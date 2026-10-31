/* SPDX-License-Identifier: MIT */
/* Real foreground sequencer + fake UART peer; no hardware timing claim. */
#include "gnss_probe.h"
#include "main.h"
#include "telemetry_check.h"
#include "timing_probe.h"
#include <stdio.h>
#include <stdlib.h>
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
GPIO_TypeDef test_gpio_a;
USART_TypeDef test_usart2;
static uint32_t tick = 2000;
static bool navigation = true, static_mode, receiver_ppp, timeout_setter;
static bool receiver_sbas, sbas_capability = true, ignore_setter;
static unsigned commands, setters, breaks;
static unsigned trial_setters, unexpected_trial_setters;
static bool trial_profile;
static char command[192];
static size_t used;
static int pvt_mask = 5, tracking_mask = 5, boot_pvt = 5, boot_tracking = 5;
static unsigned mask_setters, saves;
static bool ignore_mask, timeout_mask, corrupt_other_mask, reject_save, missing_mask;
static char mask_report[1024];
void USART2_IRQHandler(void);

uint32_t HAL_GetTick(void)
{
    return tick;
}

uint32_t __get_PRIMASK(void)
{
    return 0;
}

void __disable_irq(void) {}

void __set_PRIMASK(uint32_t mask)
{
    (void)mask;
}

void __DMB(void) {}

void HAL_GPIO_Init(GPIO_TypeDef *gpio, const GPIO_InitTypeDef *config)
{
    (void)gpio;
    (void)config;
}

void HAL_NVIC_SetPriority(unsigned irq, unsigned priority, unsigned subpriority)
{
    (void)irq;
    (void)priority;
    (void)subpriority;
}

void HAL_NVIC_EnableIRQ(unsigned irq)
{
    (void)irq;
}

bool simpleclock_usb_cdc_try_send(const uint8_t *data, uint16_t length)
{
    telemetry_check(data, length);
    if (length >= 16U && memcmp(data, "type=gnss_probe ", 16U) == 0 && length < sizeof(mask_report))
    {
        memcpy(mask_report, data, length);
        mask_report[length] = '\0';
    }
    return true;
}

bool simpleclock_usb_cdc_try_send_queued(const uint8_t *data, uint16_t length)
{
    telemetry_check(data, length);
    (void)data;
    (void)length;
    return true;
}

void timing_probe_observe(const sc_sbf_observation_t *o, uint32_t now)
{
    (void)o;
    (void)now;
}

void timing_probe_break(uint32_t reason)
{
    (void)reason;
    breaks++;
}

bool timing_probe_pps_ready(uint32_t now)
{
    (void)now;
    return false;
}

static void receive(const char *text)
{
    while (*text)
    {
        test_usart2.ISR = USART_ISR_RXNE_RXFNE | USART_ISR_TXE_TXFNF;
        test_usart2.RDR = (uint8_t)*text++;
        USART2_IRQHandler();
    }
    test_usart2.ISR = USART_ISR_TXE_TXFNF;
}

static void reply(void)
{
    commands++;
    const bool mask_setter = strncmp(command, "setElevationMask, ", 18) == 0;
    const bool mask_getter = strncmp(command, "getElevationMask, ", 18) == 0;
    if (mask_setter || mask_getter)
    {
        const bool tracking = strstr(command, "Tracking") != NULL;
        if (mask_setter)
        {
            const char *value = strrchr(command, ',');
            const int degrees = (int)strtol(value + 1, NULL, 10);
            mask_setters++;
            if (!ignore_mask)
            {
                if (tracking)
                {
                    tracking_mask = degrees;
                }
                else
                {
                    pvt_mask = degrees;
                }
            }
            if (corrupt_other_mask)
            {
                if (tracking)
                {
                    pvt_mask++;
                }
                else
                {
                    tracking_mask++;
                }
            }
            if (timeout_mask)
            {
                return;
            }
        }
        receive("$R: ");
        receive(command);
        char line[80];
        (void)snprintf(line, sizeof(line), "ElevationMask, %s, %d.000\r\nCOM1>",
                       tracking ? "Tracking" : "PVT", tracking ? tracking_mask : pvt_mask);
        receive(missing_mask ? "COM1>" : line);
        return;
    }
    if (strcmp(command, "exeCopyConfigFile, Current, Boot\r\n") == 0)
    {
        saves++;
        if (reject_save)
        {
            receive("$R? save failed\r\nCOM1>");
        }
        else
        {
            boot_pvt = pvt_mask;
            boot_tracking = tracking_mask;
            receive("$R: exeCopyConfigFile, Current, Boot\r\n"
                    "CopyConfigFile, Current, Boot\r\nCOM1>");
        }
        return;
    }
    if (trial_profile && strncmp(command, "set", 3) == 0)
    {
        trial_setters++;
        if (strcmp(command, "setDataInOut, COM1, ,+SBF\r\n") != 0 &&
            strcmp(command, "setSBFOutput, Stream1, COM1, "
                            "PVTGeodetic+ReceiverTime+ReceiverStatus+MeasEpoch+EndOfPVT"
                            "+PosCovCartesian+VelCovCartesian, sec1\r\n") != 0)
        {
            unexpected_trial_setters++;
        }
    }
    const bool setter = strncmp(command, "setPVTMode", 10) == 0;
    if (setter)
    {
        setters++;
        if (!ignore_setter)
        {
            receiver_ppp = strstr(command, "+PPP") != NULL;
            receiver_sbas = strstr(command, "+SBAS") != NULL;
        }
        if (timeout_setter)
        {
            return; /* It applied, but its entire reply was lost. */
        }
    }
    receive("$R: ");
    receive(command);
    if (strstr(command, "getReceiverCapabilities"))
    {
        receive("ReceiverCapabilities, Main, GPSL1CA+GALE6BC, COM1, ");
        receive(sbas_capability ? "SBAS+PPPGalileoHAS-SIS, 200, 200\r\n"
                                : "PPPGalileoHAS-SIS, 200, 200\r\n");
    }
    else if (strstr(command, "getSignalTracking"))
    {
        receive("SignalTracking, GPSL1CA+GALE6BC\r\n");
    }
    else if (strstr(command, "getSignalUsage"))
    {
        receive(navigation ? "SignalUsage, GPSL1CA, GPSL1CA+GALE6BC\r\n"
                           : "SignalUsage, GALE6BC, GPSL1CA\r\n");
    }
    else if (strstr(command, "PVTMode"))
    {
        receive(static_mode     ? "PVTMode, Static, StandAlone, auto\r\n"
                : receiver_ppp  ? "PVTMode, Rover, StandAlone+PPP, auto\r\n"
                : receiver_sbas ? "PVTMode, Rover, StandAlone+SBAS, auto\r\n"
                                : "PVTMode, Rover, StandAlone, auto\r\n");
    }
    else if (strstr(command, "getSBASCorrections"))
    {
        receive("SBASCorrections, auto, Operational\r\n");
    }
    receive("COM1>");
}

static void service_ticks(unsigned count)
{
    for (unsigned i = 0; i < count; ++i)
    {
        test_usart2.TDR = 0;
        gnss_probe_service(tick++);
        if (test_usart2.TDR)
        {
            if (used + 1 >= sizeof(command))
            {
                puts("command overflow");
                return;
            }
            command[used++] = (char)test_usart2.TDR;
            if (test_usart2.TDR == '\n')
            {
                command[used] = '\0';
                reply();
                used = 0;
            }
        }
    }
}

static void run(unsigned profile)
{
    gnss_probe_request(profile);
    service_ticks(6500U);
}

int main(void)
{
    test_usart2.ISR = USART_ISR_TXE_TXFNF;
    gnss_probe_init();
    run(GNSS_PROFILE_PPP_RESTORE);
    CHECK(commands == 0 && setters == 0); /* No ownership proof after boot. */
    navigation = false;
    run(GNSS_PROFILE_PPP_ENABLE);
    CHECK(commands == 3 && setters == 0); /* Stop before mutation on bad preflight. */
    navigation = true;
    static_mode = true;
    run(GNSS_PROFILE_PPP_ENABLE);
    CHECK(commands == 7 && setters == 0); /* Never overwrite another PVT configuration. */
    static_mode = false;
    run(GNSS_PROFILE_PPP_ENABLE);
    CHECK(commands == 13 && setters == 1 && receiver_ppp && breaks == 1);
    run(GNSS_PROFILE_PPP_ENABLE);
    CHECK(commands == 13 && setters == 1); /* Repeated enable does not overwrite baseline. */
    static_mode = true;
    run(GNSS_PROFILE_PPP_RESTORE);
    CHECK(commands == 14 && setters == 1); /* Changed external settings veto restore. */
    static_mode = false;
    run(GNSS_PROFILE_PPP_RESTORE);
    CHECK(commands == 17 && setters == 2 && !receiver_ppp);
    run(GNSS_PROFILE_PPP_RESTORE);
    CHECK(commands == 17 && setters == 2);
    timeout_setter = true;
    run(GNSS_PROFILE_PPP_ENABLE);
    CHECK(commands == 22 && setters == 3 && receiver_ppp); /* No automatic mutation retry. */
    timeout_setter = false;
    run(GNSS_PROFILE_PPP_RESTORE);
    CHECK(commands == 25 && setters == 4 && !receiver_ppp); /* Recover uncertain completion. */
    trial_profile = true;
    run(GNSS_PROFILE_TRIAL_STREAM);
    CHECK(commands == 33 && trial_setters == 2 && unexpected_trial_setters == 0);
    trial_profile = false;

    /* AUTO enabled PPP without legacy test ownership. The explicit bench
     * profiles can switch it; the old restore guard must remain intact. */
    receiver_ppp = true;
    CHECK(gnss_probe_command_profile("gnss nohas") == GNSS_PROFILE_NO_HAS);
    CHECK(gnss_probe_command_profile("gnss has") == GNSS_PROFILE_HAS);
    CHECK(gnss_probe_command_profile("gnss nohas extra") == 0U);
    CHECK(gnss_probe_command_profile("gnss sbas") == GNSS_PROFILE_NO_HAS);
    CHECK(gnss_probe_command_profile("gnss auto") == GNSS_PROFILE_AUTO_HYBRID);
    run(GNSS_PROFILE_NO_HAS);
    CHECK(commands == 38 && setters == 5 && !receiver_ppp && receiver_sbas);
    run(GNSS_PROFILE_PPP_RESTORE);
    CHECK(commands == 38 && setters == 5);
    run(GNSS_PROFILE_HAS);
    CHECK(commands == 43 && setters == 6 && receiver_ppp && !receiver_sbas);

    sbas_capability = false;
    run(GNSS_PROFILE_NO_HAS);
    CHECK(commands == 44 && setters == 6 && receiver_ppp);
    sbas_capability = true;
    static_mode = true;
    run(GNSS_PROFILE_NO_HAS);
    CHECK(commands == 46 && setters == 6);
    static_mode = false;
    ignore_setter = true;
    run(GNSS_PROFILE_NO_HAS);
    CHECK(commands == 50 && setters == 7 && receiver_ppp); /* Bad setter readback stops. */
    ignore_setter = false;
    timeout_setter = true;
    run(GNSS_PROFILE_NO_HAS);
    CHECK(commands == 54 && setters == 8 && receiver_sbas && !receiver_ppp);
    timeout_setter = false;
    run(GNSS_PROFILE_NO_HAS);
    CHECK(commands == 59 && setters == 9 && receiver_sbas); /* Explicit retry only. */
    CHECK(gnss_probe_command_profile("gnss standalone") == GNSS_PROFILE_STANDALONE);
    CHECK(gnss_probe_command_profile("gnss standalone extra") == 0U);
    sbas_capability = false; /* Pure standalone must not require SBAS licensing. */
    run(GNSS_PROFILE_STANDALONE);
    CHECK(commands == 62 && setters == 10 && !receiver_sbas && !receiver_ppp);
    run(GNSS_PROFILE_PPP_RESTORE);
    CHECK(commands == 62 && setters == 10); /* The legacy ownership guard remains. */
    receiver_ppp = true;
    run(GNSS_PROFILE_STANDALONE);
    CHECK(commands == 65 && setters == 11 && !receiver_ppp && !receiver_sbas);
    receiver_sbas = true;
    ignore_setter = true;
    run(GNSS_PROFILE_STANDALONE);
    CHECK(commands == 67 && setters == 12 && receiver_sbas); /* SBAS left on is rejected. */
    ignore_setter = false;
    timeout_setter = true;
    run(GNSS_PROFILE_STANDALONE);
    CHECK(commands == 69 && setters == 13 && !receiver_sbas && !receiver_ppp);
    timeout_setter = false;
    run(GNSS_PROFILE_STANDALONE);
    CHECK(commands == 72 && setters == 14 && !receiver_sbas && !receiver_ppp);
    static_mode = true;
    run(GNSS_PROFILE_STANDALONE);
    CHECK(commands == 73 && setters == 14); /* Unexpected baseline vetoes mutation. */
    const unsigned before_masks = commands, before_breaks = breaks;
    CHECK(gnss_probe_command_profile("gnss masks") == GNSS_PROFILE_ELEVATION_SURVEY);
    CHECK(gnss_probe_command_profile("gnss masks reset") == GNSS_PROFILE_ELEVATION_RESET);
    CHECK(gnss_probe_command_profile("gnss save") == GNSS_PROFILE_SAVE_BOOT);
    CHECK(gnss_probe_command_profile("gnss mask pvt 15") == GNSS_PROFILE_PVT_MASK_BASE + 105U);
    CHECK(gnss_probe_command_profile("gnss mask tracking off") == GNSS_PROFILE_TRACKING_MASK_BASE);
    CHECK(gnss_probe_command_profile("gnss mask pvt -90") == GNSS_PROFILE_PVT_MASK_BASE);
    CHECK(gnss_probe_command_profile("gnss mask tracking 90") ==
          GNSS_PROFILE_TRACKING_MASK_BASE + 180U);
    const char *invalid[] = {"gnss mask pvt 91",
                             "gnss mask tracking -91",
                             "gnss mask all 15",
                             "gnss mask pvt 15 extra",
                             "gnss mask pvt +15",
                             "gnss mask pvt 15.5",
                             "gnss mask pvt 15\r\nsetPVTMode",
                             "gnss masks reset extra",
                             "gnss save extra",
                             "gnss mask pvt -",
                             "gnss mask pvt ",
                             "gnss mask tracking 1000000000000000000"};
    for (unsigned i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i)
    {
        CHECK(gnss_probe_command_profile(invalid[i]) == GNSS_PROFILE_NONE);
    }
    CHECK(gnss_probe_command_profile(NULL) == GNSS_PROFILE_NONE);
    run(GNSS_PROFILE_ELEVATION_SURVEY);
    CHECK(commands == before_masks + 2U && mask_setters == 0U && saves == 0U);
    CHECK(breaks == before_breaks);
    run(gnss_probe_command_profile("gnss mask pvt 15"));
    CHECK(pvt_mask == 15 && tracking_mask == 5 && mask_setters == 1U && saves == 1U);
    CHECK(boot_pvt == 15 && boot_tracking == 5 && breaks == before_breaks + 1U);
    gnss_probe_report(tick);
    CHECK(strstr(mask_report, "pvt_mask=15 tracking_mask=5 mask_saved=1 mask_fault=0") != NULL);
    run(gnss_probe_command_profile("gnss mask tracking 20"));
    CHECK(pvt_mask == 15 && tracking_mask == 20 && boot_pvt == 15 && boot_tracking == 20);
    run(gnss_probe_command_profile("gnss mask pvt off"));
    CHECK(pvt_mask == -90 && tracking_mask == 20 && boot_pvt == -90 && boot_tracking == 20);
    run(GNSS_PROFILE_ELEVATION_RESET);
    CHECK(pvt_mask == -90 && tracking_mask == -90 && boot_pvt == -90 && boot_tracking == -90);
    const unsigned successful_saves = saves;
    ignore_mask = true;
    run(gnss_probe_command_profile("gnss mask pvt 15"));
    CHECK(pvt_mask == -90 && saves == successful_saves);
    ignore_mask = false;
    run(GNSS_PROFILE_ELEVATION_SURVEY);
    gnss_probe_report(tick);
    CHECK(strstr(mask_report, "mask_saved=0 mask_fault=1") != NULL);
    timeout_mask = true;
    run(gnss_probe_command_profile("gnss mask pvt 15"));
    CHECK(pvt_mask == 15 && saves == successful_saves); /* Applied, completion uncertain. */
    timeout_mask = false;
    corrupt_other_mask = true;
    run(gnss_probe_command_profile("gnss mask pvt 15"));
    CHECK(tracking_mask == -89 && saves == successful_saves); /* Other engine must stay put. */
    corrupt_other_mask = false;
    reject_save = true;
    run(GNSS_PROFILE_ELEVATION_RESET);
    gnss_probe_report(tick);
    CHECK(strstr(mask_report, "mask_saved=0 mask_fault=1") != NULL);
    CHECK(saves == successful_saves + 1U && boot_pvt == -90 && boot_tracking == -90);
    reject_save = false;
    run(GNSS_PROFILE_ELEVATION_RESET);
    gnss_probe_report(tick);
    CHECK(strstr(mask_report, "mask_saved=1 mask_fault=0") != NULL);
    missing_mask = true;
    const unsigned setters_before_missing = mask_setters, saves_before_missing = saves;
    run(gnss_probe_command_profile("gnss mask tracking 90"));
    CHECK(mask_setters == setters_before_missing && saves == saves_before_missing);
    missing_mask = false;
    run(gnss_probe_command_profile("gnss mask tracking 90"));
    CHECK(boot_tracking == 90 && boot_pvt == -90);
    pvt_mask = boot_pvt;
    tracking_mask = boot_tracking; /* Simulated receiver reboot. */
    run(GNSS_PROFILE_ELEVATION_SURVEY);
    CHECK(pvt_mask == -90 && tracking_mask == 90 && saves == saves_before_missing + 1U);
    run(GNSS_PROFILE_SAVE_BOOT);
    CHECK(saves == saves_before_missing + 2U);
    const unsigned before_cancel = mask_setters, saves_before_cancel = saves;
    gnss_probe_request(gnss_probe_command_profile("gnss mask pvt 25"));
    for (unsigned i = 0; i < 3000U && mask_setters == before_cancel; ++i)
    {
        service_ticks(1U);
    }
    CHECK(mask_setters == before_cancel + 1U && pvt_mask == 25);
    run(GNSS_PROFILE_IDENTIFY); /* Cancel after the setter, before verification/save. */
    CHECK(saves == saves_before_cancel);
    gnss_probe_report(tick);
    CHECK(strstr(mask_report, "mask_saved=0 mask_fault=1") != NULL);
    run(gnss_probe_command_profile("gnss mask pvt 15"));
    CHECK(boot_pvt == 15 && boot_tracking == 90 && saves == saves_before_cancel + 1U);
    puts("GNSS preflight, independent persistent masks, reset and failure paths passed");
    return 0;
}
