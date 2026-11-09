/* SPDX-License-Identifier: MIT */
/* Host exercise of the autonomous GNSS sequencer. No USB input is provided. */
#include "gnss_probe.h"
#include "main.h"
#include "telemetry_check.h"
#include "timing_probe.h"
#include "settings_flash.h"
#include "simpleclock/crc16.h"
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CHECK(e)                                                                                   \
    do                                                                                             \
    {                                                                                              \
        if (!(e))                                                                                  \
        {                                                                                          \
            fprintf(stderr, "line %d: %s\n", __LINE__, #e);                                        \
            fprintf(stderr, "last control event: %s\n", control_event_line);                       \
            fprintf(stderr, "last receiver command: %s\n", command);                               \
            return 1;                                                                              \
        }                                                                                          \
    } while (0)

GPIO_TypeDef test_gpio_a;
USART_TypeDef test_usart2;
DMA_TypeDef test_dma1;
DMA_Channel_TypeDef test_dma1_channel1;
DMAMUX_Channel_TypeDef test_dmamux1_channel0;
static uint32_t irq_mask;
static char gnss_status_line[2048];
void DMA1_Channel1_IRQHandler(void);

static void dma_dispatch(void)
{
    if (!irq_mask && test_dma1.ISR)
    {
        DMA1_Channel1_IRQHandler();
        test_dma1.ISR = 0U; /* Model write-one-to-clear IFCR. */
    }
}

static void receive_byte(uint8_t byte)
{
    if (!(test_dma1_channel1.CCR & DMA_CCR_EN))
    {
        abort();
    }
    ((uint8_t *)test_dma1_channel1.CMAR)[2048U - test_dma1_channel1.CNDTR] = byte;
    if (--test_dma1_channel1.CNDTR == 0U)
    {
        test_dma1_channel1.CNDTR = 2048U;
        test_dma1.ISR |= DMA_ISR_TCIF1;
    }
    dma_dispatch();
}

static uint32_t tick = 2000U;
static char command[192];
static size_t command_used;
static unsigned command_count, setter_count, pvt_setter_count, manual_holds;
static bool lose_reply, wrong_stream, overflow_stream, clock_steering;
static bool lose_queries;
static bool receiver_ppp, receiver_stream, receiver_data_sbf, receiver_timing_auto;
static bool receiver_sbas;
static bool usb_available = true;
static unsigned usb_rejected, usb_accepted;
static const char *fault_query, *duplicate_line, *pre_echo_line;
static unsigned fault_query_count;
static bool missing_readback, duplicate_readback, pre_echo_readback;
static bool early_reply, early_reply_sent;
static bool cr_reply;
static int pvt_mask = 5, tracking_mask = 5;
static unsigned mask_setters, boot_saves;
static bool fail_save;
static unsigned reboot_count;
static bool reject_reboot;
static bool receiver_low_static;
static bool receiver_saved_managed;
static bool receiver_high_static, boot_high_static, control_ignore, control_corrupt, fail_mcu_save;
static bool receiver_pps_off;
static bool pps_custom_shape, pps_foreign_scale, pps_missing, pps_duplicate;
static bool pps_ignore_setter, pps_corrupt_shape;
static unsigned pps_setters;
static char control_event_line[1024];
static unsigned control_events;
static bool lists_mode, lists_e6, lists_corrupt;
static bool diagnostic_test, diagnostic_capture, diagnostic_bad, diagnostic_headless;
static unsigned diagnostic_busy_ms, diagnostic_started, diagnostic_credit;
static size_t diagnostic_position, diagnostic_length, diagnostic_captured;
static uint32_t diagnostic_next_offset;
static char diagnostic_reply[8192], diagnostic_received[8192];
static bool diagnostic_mixed;
static unsigned diagnostic_sbf_injected, diagnostic_sbf_observed, diagnostic_sbf_exported;
static bool latch_test, latch_status_enabled, latch_status_valid = true, latch_reference = true;
static bool latch_persistent, latch_timeout, latch_reject, latch_pause_after_read;
static uint32_t latch_error = 8U;
static unsigned latch_reads;
static bool receiver_static, receiver_actuator_busy, usb_block_after_reply, usb_block_after_set;
static unsigned defaults_fixture;
static const char *receiver_ref = "auto";
static const char lists_pvt[] =
    "GPSL1CA+GPSL2PY+GPSL2C+GPSL5+GPSL1C+GLOL1CA+GLOL2P+GLOL2CA+GLOL3+"
    "GALE1BC+GALE6BC+GALE5a+GALE5b+GALE5+BDSB1I+BDSB2I+BDSB3I+BDSB1C+"
    "BDSB2a+BDSB2b+QZSL1CA+QZSL2C+QZSL5+QZSL6+QZSL1C+QZSL1S+QZSL1CB+NAVICL5";
static const char lists_nav[] =
    "GPSL1CA+GPSL2C+GPSL5+GLOL1CA+GLOL2CA+GALE1BC+GALE5a+GALE5b+GALE5+GEOL1+"
    "BDSB1I+BDSB2I+BDSB3I+BDSB1C+BDSB2a+BDSB2b+QZSL1CA+QZSL2C+QZSL5+QZSL6+QZSL1CB+NAVICL5";

bool hybrid_probe_receiver_change_ready(void)
{
    return !receiver_actuator_busy;
}

bool hybrid_probe_receiver_persist(bool managed)
{
    if (fail_mcu_save || receiver_actuator_busy) /* As firmware: refused while an FCW is pending. */
    {
        return false;
    }
    sc_settings_t saved;
    uint32_t generation;
    if (settings_flash_load(&saved, &generation) != SC_SETTINGS_VALID)
    {
        sc_settings_defaults(&saved);
    }
    saved.receiver_user_profile = managed ? 1U : 0U;
    saved.gnss_selection = gnss_probe_selection();
    saved.pps_policy = gnss_probe_pps_policy();
    receiver_saved_managed = managed;
    return settings_flash_save(&saved);
}

uint32_t HAL_GetTick(void)
{
    return tick;
}

uint32_t __get_PRIMASK(void)
{
    return irq_mask;
}

void __disable_irq(void)
{
    irq_mask = 1U;
}

void __set_PRIMASK(uint32_t mask)
{
    irq_mask = mask;
    dma_dispatch();
}

void __DMB(void) {}

void HAL_GPIO_Init(GPIO_TypeDef *g, const GPIO_InitTypeDef *c)
{
    (void)g;
    (void)c;
}

void HAL_NVIC_SetPriority(unsigned i, unsigned p, unsigned s)
{
    (void)i;
    (void)p;
    (void)s;
}

void HAL_NVIC_EnableIRQ(unsigned i)
{
    (void)i;
}

/* cdc_single_buffer models the one CDC transmit buffer: a send occupies it
 * until the test clears cdc_busy (transfer complete). */
static bool cdc_single_buffer, cdc_busy;
static char gnss_pps_line[512];

bool simpleclock_usb_cdc_try_send(const uint8_t *d, uint16_t n)
{
    telemetry_check(d, n);
    if (cdc_single_buffer && cdc_busy)
    {
        return false;
    }
    cdc_busy = cdc_single_buffer;
    if (n > 16U && !memcmp(d, "type=gnss_probe ", 16U) && n < sizeof(gnss_status_line))
    {
        memcpy(gnss_status_line, d, n);
        gnss_status_line[n] = '\0';
    }
    if (n > 14U && !memcmp(d, "type=gnss_pps ", 14U) && n < sizeof(gnss_pps_line))
    {
        memcpy(gnss_pps_line, d, n);
        gnss_pps_line[n] = '\0';
    }
    return true;
}

bool simpleclock_usb_cdc_try_send_queued(const uint8_t *d, uint16_t n)
{
    telemetry_check(d, n);
    (void)d;
    (void)n;
    if (usb_available)
    {
        if (diagnostic_capture && n > 16U && !memcmp(d, "type=sbf_prefix ", 16U))
        {
            diagnostic_sbf_exported++;
        }
        if (diagnostic_capture && n > 16U && !memcmp(d, "type=gnss_ascii ", 16U))
        {
            char line[256];
            if (n >= sizeof(line))
            {
                diagnostic_bad = true;
                return true;
            }
            memcpy(line, d, n);
            line[n] = '\0';
            const char *offset = strstr(line, " offset="), *hex = strstr(line, " hex=");
            if (!offset || !hex)
            {
                diagnostic_bad = true;
                return true;
            }
            const uint32_t start = (uint32_t)strtoul(offset + 8U, NULL, 10);
            if (diagnostic_captured && start != diagnostic_next_offset)
            {
                diagnostic_bad = true;
            }
            hex += 5U;
            unsigned kept = 0U;
            while (isxdigit((unsigned char)hex[0]) && isxdigit((unsigned char)hex[1]))
            {
                if (diagnostic_captured >= sizeof(diagnostic_received))
                {
                    diagnostic_bad = true;
                    break;
                }
                const char pair[3] = {hex[0], hex[1], '\0'};
                diagnostic_received[diagnostic_captured++] = (char)strtoul(pair, NULL, 16);
                kept++;
                hex += 2U;
            }
            diagnostic_next_offset = start + kept;
        }
        if (n > 18U && !memcmp(d, "type=gnss_control ", 18U) && n < sizeof(control_event_line))
        {
            memcpy(control_event_line, d, n);
            control_event_line[n] = '\0';
            control_events++;
        }
        usb_accepted++;
    }
    else
    {
        usb_rejected++;
    }
    return usb_available;
}

void timing_probe_observe(const sc_sbf_observation_t *o, uint32_t n)
{
    (void)o;
    (void)n;
    if (diagnostic_capture)
    {
        diagnostic_sbf_observed++;
    }
}

void timing_probe_break(uint32_t reason)
{
    (void)reason;
}

bool timing_probe_pps_ready(uint32_t n)
{
    (void)n;
    return false;
}

void hybrid_probe_observe(const sc_sbf_observation_t *o, uint32_t n)
{
    (void)o;
    (void)n;
}

void hybrid_probe_manual_request(void)
{
    manual_holds++;
}

void hybrid_probe_defaults_finish(bool receiver_ready)
{
    (void)receiver_ready;
}

void USART2_IRQHandler(void);

static void receive(const char *s)
{
    while (*s != '\0')
    {
        receive_byte((uint8_t)*s++);
    }
    test_usart2.ISR = USART_ISR_TXE_TXFNF;
}

static void receive_status(void);

static void reply(void)
{
    command_count++;
    if (latch_test && strcmp(command, "lstInternalFile, Error\r\n") == 0)
    {
        latch_reads++;
        if (latch_pause_after_read)
        {
            latch_status_enabled = false;
        }
        if (latch_timeout)
        {
            return;
        }
        if (latch_reject)
        {
            receive("$R? acknowledgement rejected\r\nCOM1>");
            return;
        }
        if (!latch_persistent)
        {
            latch_error = 0U;
        }
        if (latch_pause_after_read)
        {
            receive_status(); /* A zero status before the command completes is insufficient. */
        }
        receive("$R; lstInternalFile, Error\r\nStart up: after PLL unlocked issue\r\nCOM1>");
        return;
    }
    if (diagnostic_test && strcmp(command, "lstInternalFile,Error\r\n") == 0)
    {
        diagnostic_length = (size_t)snprintf(diagnostic_reply, sizeof(diagnostic_reply),
                                             "$R: lstInternalFile,Error\r\n");
        for (unsigned i = 0U; i < 70U; i++)
        {
            diagnostic_length += (size_t)snprintf(
                diagnostic_reply + diagnostic_length, sizeof(diagnostic_reply) - diagnostic_length,
                "%02u Diagnostic trace: retain every byte across thermal telemetry windows.\r\n",
                i);
        }
        diagnostic_length += (size_t)snprintf(diagnostic_reply + diagnostic_length,
                                              sizeof(diagnostic_reply) - diagnostic_length,
                                              "Start up: after PLL unlocked issue\r\nCOM1>");
        diagnostic_started = tick;
        return; /* The receiver streams this reply at 115200 baud below. */
    }
    if (lose_queries && strncmp(command, "get", 3U) == 0)
    {
        return; /* Receiver booted slowly or a read-only query reply was lost. */
    }
    if (strncmp(command, "setElevationMask, ", 18) == 0 ||
        strncmp(command, "getElevationMask, ", 18) == 0)
    {
        const bool tracking = strstr(command, "Tracking") != NULL;
        if (command[0] == 's')
        {
            const int degrees = (int)strtol(strrchr(command, ',') + 1, NULL, 10);
            if (tracking)
            {
                tracking_mask = degrees;
            }
            else
            {
                pvt_mask = degrees;
            }
            mask_setters++;
            if (lose_reply)
            {
                return;
            }
        }
        receive("$R: ");
        receive(command);
        char line[80];
        (void)snprintf(line, sizeof(line), "ElevationMask, %s, %d\r\nCOM1>",
                       tracking ? "Tracking" : "PVT", tracking ? tracking_mask : pvt_mask);
        receive(line);
        return;
    }
    if (strcmp(command, "exeCopyConfigFile, Current, Boot\r\n") == 0)
    {
        boot_saves++;
        if (!fail_save)
        {
            boot_high_static = receiver_high_static;
        }
        receive(
            fail_save
                ? "$R? save failed\r\nCOM1>"
                : "$R: exeCopyConfigFile, Current, Boot\r\nCopyConfigFile, Current, Boot\r\nCOM1>");
        return;
    }
    if (strcmp(command, "exeCopyConfigFile, RxDefault, Current\r\n") == 0)
    {
        receiver_low_static = receiver_high_static = false;
        receiver_ppp = receiver_sbas = receiver_stream = receiver_data_sbf = false;
        pvt_mask = tracking_mask = 5;
        receive("$R: ");
        receive(command);
        receive("CopyConfigFile, RxDefault, Current\r\nCOM1>");
        return;
    }
    if (strcmp(command, "exeResetReceiver, Soft, none\r\n") == 0)
    {
        reboot_count++;
        if (latch_test && !reject_reboot)
        {
            latch_error = 8U;
        }
        receive(reject_reboot ? "$R? reboot rejected\r\nSTOP>"
                              : "$R: exeResetReceiver, Soft, none\r\n"
                                "ResetReceiver, Soft, none\r\nSTOP>\r\n$TE ResetReceiver Soft\r\n");
        return; /* Real reset has no COM1 prompt; exercise the timeout path. */
    }
    if (strncmp(command, "set", 3) == 0)
    {
        setter_count++;
        if (strncmp(command, "setPVTMode", 10) == 0)
        {
            if (strstr(command, "Static"))
            {
                receiver_static = true;
            }
            else if (strstr(command, "Rover"))
            {
                receiver_static = false;
            }
            receiver_ppp = strstr(command, "+PPP") != NULL;
            receiver_sbas = strstr(command, "+SBAS") != NULL;
            pvt_setter_count++;
        }
        else if (strncmp(command, "setClockSyncThreshold", 21) == 0)
        {
            clock_steering = false;
        }
        else if (strncmp(command, "setSBFOutput", 12) == 0)
        {
            receiver_stream = true;
        }
        else if (strncmp(command, "setDataInOut", 12) == 0)
        {
            receiver_data_sbf = true;
        }
        else if (strncmp(command, "setTimingSystem", 15) == 0)
        {
            receiver_timing_auto = true;
        }
        else if (strncmp(command, "setReceiverDynamics", 19) == 0 && !control_ignore)
        {
            receiver_high_static = strstr(command, "High") != NULL;
            receiver_low_static = !receiver_high_static;
        }
        else if (!strncmp(command, "setPPSParameters", 16))
        {
            pps_setters++;
            if (!pps_ignore_setter)
            {
                receiver_pps_off = strstr(command, "off") != NULL;
                if (strstr(command, "RxClock"))
                {
                    pps_foreign_scale = false;
                }
                if (pps_corrupt_shape)
                {
                    pps_custom_shape = false;
                }
            }
        }
        else if (lists_mode && !strncmp(command, "setSignalUsage", 14))
        {
            lists_e6 = strstr(command, "+GALE6BC") != NULL;
        }
        if (lose_reply)
        {
            return;
        }
    }
    const bool fault =
        fault_query != NULL && strcmp(command, fault_query) == 0 && ++fault_query_count == 2U;
    if (fault && pre_echo_readback)
    {
        receive(pre_echo_line); /* Data before the matching echo is not current readback. */
    }
    receive("$R: ");
    receive(command);
    if (fault)
    {
        if (missing_readback || pre_echo_readback)
        {
            receive("COM1>"); /* An echo alone must not reuse the first query's value. */
            return;
        }
        if (duplicate_readback)
        {
            receive(duplicate_line); /* A later good line cannot repair an ambiguous reply. */
        }
    }
    if (!strncmp(command, "getReceiverDynamics", 19) ||
        !strncmp(command, "setReceiverDynamics", 19))
    {
        receive(receiver_high_static  ? (control_corrupt ? "ReceiverDynamics, High, Automotive\r\n"
                                                         : "ReceiverDynamics, High, Static\r\n")
                : receiver_low_static ? "ReceiverDynamics, Low, Static\r\n"
                                      : "ReceiverDynamics, Moderate, Automotive\r\n");
    }
    else if (!strcmp(command, "getCOMSettings, COM1\r\n"))
    {
        if (defaults_fixture != 1U)
        {
            receive(defaults_fixture == 2U
                        ? "  COMSettings, COM1, baud9600, bits8, No, bit1, none\r\n"
                        : " \tCOMSettings, COM1, baud115200, bits8, No, bit1, none\r\n");
        }
        if (defaults_fixture == 3U)
        {
            receive("  COMSettings, COM1, baud115200, bits8, No, bit1, none\r\n");
        }
    }
    else if (strcmp(command, "getPPSParameters\r\n") == 0)
    {
        if (!pps_missing)
        {
            char pps_line[128];
            (void)snprintf(
                pps_line, sizeof(pps_line), "  PPSParameters, %s, %s, %s, %s, %s, %s\r\n",
                receiver_pps_off ? "off" : "sec1", pps_custom_shape ? "High2Low" : "Low2High",
                pps_custom_shape ? "23.40" : "0", pps_foreign_scale ? "GPS" : "RxClock",
                pps_foreign_scale ? "60" : "0", pps_custom_shape ? "0.100000" : "5");
            receive(pps_line);
            if (pps_duplicate)
            {
                receive(pps_line);
            }
        }
        if (receiver_pps_off && defaults_fixture == 4U)
        {
            receive("  PPSParameters, off, Low2High, 0, RxClock, 0, 5\r\n");
        }
    }
    else if (strcmp(command, "getClockSyncThreshold\r\n") == 0)
    {
        receive(clock_steering ? "  ClockSyncThreshold, ClockSteering\r\n"
                               : "  ClockSyncThreshold, msec1, on\r\n");
    }
    else if (strcmp(command, "getTimeSyncSource\r\n") == 0)
    {
        receive("  TimeSyncSource, none\r\n");
    }
    else if (strcmp(command, "getTimingSystem\r\n") == 0)
    {
        receive(receiver_timing_auto ? "  TimingSystem, auto\r\n" : "  TimingSystem, GPS\r\n");
    }
    else if (strcmp(command, "getDataInOut, COM1\r\n") == 0)
    {
        receive(receiver_data_sbf ? "  DataInOut, COM1, auto, SBF+NMEA, (on)\r\n"
                                  : "  DataInOut, COM1, auto, NMEA, (on)\r\n");
    }
    else if (strcmp(command, "getSBFOutput, Stream1\r\n") == 0)
    {
        if (overflow_stream)
        {
            receive("  SBFOutput, Stream1, COM1, "
                    "PVTGeodetic+ReceiverTime+ReceiverStatus+MeasEpoch+EndOfPVT+PosCovCartesian+"
                    "VelCovCartesian+RFStatus+DOP, sec1, "
                    "this-line-is-intentionally-overlong-to-exercise-bounded-discard\r\n");
        }
        else if (wrong_stream)
        {
            receive("  SBFOutput, Stream1, UART2, "
                    "PVTGeodetic+ReceiverTime+ReceiverStatus+MeasEpoch+EndOfPVT+PosCovCartesian+"
                    "VelCovCartesian+RFStatus+DOP, sec1\r\n");
        }
        else if (receiver_stream)
        {
            receive("  SBFOutput, Stream1, COM1, "
                    "PVTGeodetic+ReceiverTime+ReceiverStatus+MeasEpoch+EndOfPVT+PosCovCartesian+"
                    "VelCovCartesian+RFStatus+DOP, sec1\r\n");
        }
        else
        {
            receive("  SBFOutput, Stream1, none, none, off\r\n");
        }
    }
    else if (strcmp(command, "getReceiverCapabilities\r\n") == 0)
    {
        receive("  ReceiverCapabilities, Main, "
                "GPSL1CA+GPSL2PY+GPSL2C+GPSL5+GPSL1C+GLOL1CA+GLOL2P+GLOL2CA+GLOL3+GALE1BC+GALE6BC+"
                "GALE5a+GALE5b+GEOL1+GEOL5+BDSB1I+BDSB2I+BDSB3I+BDSB1C+BDSB2a+BDSB2b+QZSL1CA+"
                "QZSL2C+QZSL5+QZSL1C+QZSL1S+QZSL1CB+LBAND+NAVICL5, DSK1+COM1+COM2+USB1+USB2, "
                "SBAS+TimeSync+xPPSOutput+TimedEvent+InternalLogging+APME+RAIM+IM+DSK1+FreqSync+"
                "GalOSNMA+PPPGalileoHAS-SIS, 200, 200\r\n");
    }
    else if (strcmp(command, "getSignalTracking\r\n") == 0 ||
             !strncmp(command, "setSignalTracking", 17))
    {
        receive("  SignalTracking, "
                "GPSL1CA+GPSL2PY+GPSL2C+GPSL5+GLOL1CA+GLOL2CA+GALE1BC+GALE6BC+GALE5a+GALE5b+GEOL1+"
                "BDSB1I+BDSB2I+BDSB3I+BDSB1C+BDSB2a+QZSL1CA+QZSL2C+QZSL5+NAVICL5\r\n");
    }
    else if (strcmp(command, "getSignalUsage\r\n") == 0 || !strncmp(command, "setSignalUsage", 14))
    {
        if (lists_mode)
        {
            receive("SignalUsage,");
            receive(lists_corrupt && lists_e6 ? "GPSL1CA" : lists_pvt);
            receive(",");
            receive(lists_nav);
            if (lists_e6)
            {
                receive("+GALE6BC");
            }
            receive("\r\nCOM1>");
            return;
        }
        receive(
            "  SignalUsage, "
            "GPSL1CA+GPSL2PY+GPSL2C+GPSL5+GLOL1CA+GLOL2P+GLOL2CA+GALE1BC+GALE5a+GALE5b+GALE5+"
            "BDSB1I+BDSB2I+BDSB3I+BDSB1C+BDSB2a+BDSB2b+QZSL1CA+QZSL2C+QZSL5+QZSL1CB, "
            "GPSL1CA+GPSL2C+GPSL5+GLOL1CA+GLOL2CA+GALE1BC+GALE6BC+GALE5a+GALE5b+GALE5+GEOL1+BDSB1I+"
            "BDSB2I+BDSB3I+BDSB1C+BDSB2a+BDSB2b+QZSL1CA+QZSL2C+QZSL5+QZSL6+QZSL1CB+NAVICL5\r\n");
    }
    else if (strstr(command, "PVTMode") != NULL)
    {
        receive(receiver_static ? "  PVTMode, Static, " : "  PVTMode, Rover, ");
        receive(receiver_ppp && receiver_sbas ? "StandAlone+SBAS+PPP, "
                : receiver_ppp                ? "StandAlone+PPP, "
                : receiver_sbas               ? "StandAlone+SBAS, "
                                              : "StandAlone, ");
        receive(receiver_ref);
        receive("\r\n");
    }
    else if (strcmp(command, "getSBASCorrections\r\n") == 0)
    {
        receive("  SBASCorrections, auto, Operational\r\n");
    }
    receive("COM1>");
    if (usb_block_after_reply || (usb_block_after_set && !strncmp(command, "set", 3U)))
    {
        usb_available = false;
    }
}

static void receive_status(void)
{
    /* ReceiverStatus rev0, no subblocks; valid CRC and epoch are essential. */
    uint8_t frame[32] = {'$', '@', 0U, 0U, 0xAEU, 0x0FU, 32U, 0U};
    const uint32_t uptime = tick / 1000U;
    for (unsigned byte = 0U; byte < 4U; byte++)
    {
        frame[8U + byte] = (uint8_t)(tick >> (8U * byte));
        frame[16U + byte] = (uint8_t)(uptime >> (8U * byte));
        frame[24U + byte] = (uint8_t)(latch_error >> (8U * byte));
    }
    frame[12] = latch_status_valid ? 1U : 0xFFU;
    frame[13] = latch_status_valid ? 0U : 0xFFU;
    frame[20] = latch_reference ? SC_SBF_RX_STATE_EXT_FREQ : 0U;
    const uint16_t crc = simpleclock_crc16_ccitt(frame + 4U, 28U);
    frame[2] = (uint8_t)crc;
    frame[3] = (uint8_t)(crc >> 8U);
    for (unsigned byte = 0U; byte < sizeof(frame); byte++)
    {
        receive_byte(frame[byte]);
    }
    test_usart2.ISR = USART_ISR_TXE_TXFNF;
}

static void run(unsigned ticks)
{
    for (unsigned i = 0; i < ticks; ++i)
    {
        if (latch_test && latch_status_enabled && tick % 1000U == 0U)
        {
            receive_status();
        }
        if (diagnostic_started && diagnostic_position < diagnostic_length)
        {
            usb_available = !diagnostic_headless && tick - diagnostic_started >= diagnostic_busy_ms;
            /* 115200 baud / 10 bits per 8N1 byte = 11520 bytes/s. */
            diagnostic_credit += 11520U;
            unsigned count = diagnostic_credit / 1000U;
            diagnostic_credit %= 1000U;
            char chunk[13];
            if (count > diagnostic_length - diagnostic_position)
            {
                count = (unsigned)(diagnostic_length - diagnostic_position);
            }
            memcpy(chunk, diagnostic_reply + diagnostic_position, count);
            chunk[count] = '\0';
            diagnostic_position += count;
            receive(chunk);
            if (diagnostic_mixed && (tick - diagnostic_started) % 50U == 0U)
            {
                /* Mixed native ASCII/SBF uses the same UART parser. A small
                 * valid EndOfPVT frame must be consumed and eventually exported
                 * even when an ASCII burst takes temporary output priority. */
                uint8_t frame[16] = {'$', '@', 0U, 0U, 0x21U, 0x17U, 16U, 0U,
                                     0U,  0U,  0U, 0U, 1U,    0U,    0U,  0U};
                const uint16_t crc = simpleclock_crc16_ccitt(frame + 4U, 12U);
                frame[2] = (uint8_t)crc;
                frame[3] = (uint8_t)(crc >> 8U);
                for (unsigned byte = 0U; byte < sizeof(frame); byte++)
                {
                    receive_byte(frame[byte]);
                }
                test_usart2.ISR = USART_ISR_TXE_TXFNF;
                diagnostic_sbf_injected++;
            }
        }
        test_usart2.TDR = 0U;
        gnss_probe_service(tick++);
        if (test_usart2.TDR != 0U)
        {
            if (cr_reply && command_used == 0U && test_usart2.TDR == '\n')
            {
                continue; /* Optional LF after a CR-terminated command. */
            }
            if (command_used + 1U >= sizeof(command))
            {
                command_used = 0U;
                continue;
            }
            command[command_used++] = (char)test_usart2.TDR;
            if (early_reply && !early_reply_sent && command_used == 8U)
            {
                early_reply_sent = true;
                receive("$R: getPPSParameters\r\n"
                        "  PPSParameters, sec1, Low2High, 0, RxClock, 0, 5\r\nCOM1>");
            }
            if (cr_reply && test_usart2.TDR == '\r')
            {
                command[command_used++] = '\n'; /* Normalize for the existing fake peer. */
                command[command_used] = '\0';
                reply();
                command_used = 0U;
            }
            else if (test_usart2.TDR == '\n')
            {
                command[command_used] = '\0';
                reply();
                command_used = 0U;
            }
        }
    }
}

extern void (*settings_flash_test_pause)(void);
static unsigned flash_pauses;

static void flash_pause(void)
{
    settings_flash_test_pause = NULL;
    flash_pauses++;
    const uint32_t mask = __get_PRIMASK();
    __disable_irq();
    /* 448 bytes fit within the 40 ms maximum erase interval's
     * available ring space. No foreground or ISR consumes a byte here. */
    for (unsigned i = 0U; i < 14U; i++)
    {
        receive_status();
    }
    tick += 40U;
    __set_PRIMASK(mask);
}

static unsigned status_counter(const char *name)
{
    gnss_probe_report(tick);
    const char *field = strstr(gnss_status_line, name);
    if (!field)
    {
        fprintf(stderr, "Missing status field: %s\n", name);
        abort();
    }
    return (unsigned)strtoul(field + strlen(name), NULL, 10);
}

static unsigned mask_fault_flag(void)
{
    const bool usb = usb_available;
    usb_available = true;
    const unsigned value = status_counter(" mask_fault=");
    usb_available = usb;
    return value;
}

static uint32_t saved_selection(void)
{
    sc_settings_t saved;
    uint32_t generation;
    return settings_flash_load(&saved, &generation) == SC_SETTINGS_VALID ? saved.gnss_selection
                                                                         : UINT32_MAX;
}

static int pps_policy_test(const char *name)
{
    const bool cold = !strcmp(name, "cold");
    const bool saved_off = !strcmp(name, "saved-off");
    const bool shape = !strcmp(name, "shape");
    CHECK(cold || saved_off || shape || !strcmp(name, "qualified") || !strcmp(name, "manual") ||
          !strcmp(name, "retry") || !strcmp(name, "duplicate") || !strcmp(name, "ignored") ||
          !strcmp(name, "report"));
    if (cold)
    {
        receiver_pps_off = pps_foreign_scale = pps_custom_shape = true;
    }
    if (shape)
    {
        pps_custom_shape = true;
    }
    receiver_stream = receiver_data_sbf = receiver_timing_auto = true;
    if (saved_off)
    {
        sc_settings_t saved;
        sc_settings_defaults(&saved);
        saved.pps_policy = SC_SETTINGS_PPS_OFF;
        saved.receiver_user_profile = 1U;
        CHECK(settings_flash_save(&saved));
        receiver_ppp = receiver_sbas = receiver_low_static = true;
    }
    test_usart2.ISR = USART_ISR_TXE_TXFNF;
    gnss_probe_init();
    CHECK(!gnss_probe_pps_policy_set(SC_SETTINGS_PPS_CONTINUOUS)); /* UART owner busy. */
    usb_available = false; /* No host acknowledgement is required for PPS policy. */
    run(10000U);
    CHECK(gnss_probe_auto_ready() && manual_holds == 0U);
    gnss_probe_pps_snapshot_t pps;
    CHECK(gnss_probe_pps_snapshot(&pps) && pps.policy_valid && !pps.pending && !pps.fault);
    CHECK(!pps.qualified && !pps.ever_qualified);
    if (saved_off)
    {
        CHECK(pps.policy == SC_SETTINGS_PPS_OFF && !pps.enabled && receiver_pps_off);
        CHECK(pps_setters == 1U && boot_saves == 0U);
        return 0;
    }
    CHECK(pps.policy == SC_SETTINGS_PPS_CONTINUOUS && pps.enabled && !receiver_pps_off);
    if (cold)
    {
        CHECK(!pps_foreign_scale && pps_custom_shape && pps_setters == 1U);
        CHECK(strstr(pps.response, "High2Low, 23.40, RxClock, 0, 0.100000"));
        const unsigned writes = pps_setters, saves = boot_saves;
        run(70000U);
        CHECK(pps_setters == writes && boot_saves == saves && !receiver_pps_off);
        return 0;
    }
    if (!strcmp(name, "report"))
    {
        /* The status line occupies the single CDC buffer: gnss_pps follows on
         * a later service pass instead of being lost to BUSY. */
        cdc_single_buffer = true;
        cdc_busy = false;
        gnss_pps_line[0] = '\0';
        gnss_probe_report(tick);
        CHECK(strstr(gnss_status_line, "type=gnss_probe") && !gnss_pps_line[0]);
        cdc_busy = false;
        gnss_probe_service(tick++);
        CHECK(strstr(gnss_pps_line, " policy_valid=1") && strstr(gnss_pps_line, " enabled=1"));
        CHECK(strstr(gnss_pps_line, " report_drops=0"));
        /* A host that never drains: dropped after 500 ms and counted. */
        cdc_busy = true;
        gnss_pps_line[0] = '\0';
        gnss_probe_report(tick);
        run(600U);
        CHECK(!gnss_pps_line[0]);
        cdc_busy = false;
        gnss_probe_report(tick);
        gnss_probe_service(tick++);
        cdc_busy = false;
        gnss_probe_service(tick++);
        CHECK(strstr(gnss_pps_line, " report_drops=1"));
        cdc_single_buffer = false;
        return 0;
    }
    const unsigned saves = boot_saves, holds = manual_holds;
    if (!strcmp(name, "qualified"))
    {
        CHECK(gnss_probe_pps_policy_set(SC_SETTINGS_PPS_QUALIFIED));
        run(2000U);
        CHECK(gnss_probe_pps_snapshot(&pps) && pps.policy_valid && !pps.enabled);
        gnss_probe_pps_quality(true);
        run(2000U);
        CHECK(gnss_probe_pps_snapshot(&pps) && pps.policy_valid && pps.enabled && pps.qualified);
        gnss_probe_pps_quality(false);
        run(2000U);
        CHECK(gnss_probe_pps_snapshot(&pps) && pps.policy_valid && !pps.enabled && !pps.qualified &&
              pps.ever_qualified);
        CHECK(boot_saves == saves && manual_holds == holds && gnss_probe_auto_ready());
        return 0;
    }
    if (!strcmp(name, "manual"))
    {
        usb_available = true;
        CHECK(gnss_probe_control("gnss write setPPSParameters,off"));
        run(7000U);
        CHECK(strstr(control_event_line, "status=saved"));
        CHECK(gnss_probe_pps_snapshot(&pps) && pps.policy == SC_SETTINGS_PPS_MANUAL &&
              pps.policy_valid && !pps.enabled);
        sc_settings_t saved;
        uint32_t generation;
        CHECK(settings_flash_load(&saved, &generation) == SC_SETTINGS_VALID &&
              saved.pps_policy == SC_SETTINGS_PPS_MANUAL);
        const unsigned writes = pps_setters;
        run(8000U);
        CHECK(pps_setters == writes && receiver_pps_off);
        CHECK(gnss_probe_pps_policy_set(SC_SETTINGS_PPS_CONTINUOUS));
        run(2000U);
        CHECK(gnss_probe_pps_snapshot(&pps) && pps.policy_valid && pps.enabled);
        CHECK(boot_saves == saves + 1U && manual_holds == holds + 1U);
        return 0;
    }
    pps_missing = !strcmp(name, "retry");
    pps_duplicate = !strcmp(name, "duplicate");
    pps_ignore_setter = !strcmp(name, "ignored");
    pps_corrupt_shape = shape;
    CHECK(gnss_probe_pps_policy_set(SC_SETTINGS_PPS_OFF));
    run(2000U);
    (void)gnss_probe_pps_snapshot(&pps);
    CHECK(pps.fault && !pps.policy_valid && !pps.pending);
    CHECK(gnss_probe_auto_ready() && manual_holds == holds && boot_saves == saves);
    pps_missing = pps_duplicate = pps_ignore_setter = pps_corrupt_shape = false;
    if (!strcmp(name, "retry"))
    {
        run(31000U); /* Bounded query-first retry, not an unpaced setter loop. */
    }
    else
    {
        CHECK(gnss_probe_pps_policy_set(SC_SETTINGS_PPS_OFF));
        run(2000U);
    }
    CHECK(gnss_probe_pps_snapshot(&pps) && pps.policy_valid && !pps.fault && !pps.enabled);
    CHECK(boot_saves == saves && manual_holds == holds);
    return 0;
}

int main(int argc, char **argv)
{
#ifdef _MSC_VER
    /* A failed fixture abort()s; report through the exit code, not a modal
     * Debug CRT/Windows Error Reporting dialog that blocks unattended ctest. */
    _set_abort_behavior(0, _WRITE_ABORT_MSG | _CALL_REPORTFAULT);
#endif
    if (argc == 2 && !strncmp(argv[1], "pps-policy-", 11U))
    {
        return pps_policy_test(argv[1] + 11U);
    }
    if (argc == 2 && !strncmp(argv[1], "defaults-", 9U))
    {
        test_usart2.ISR = USART_ISR_TXE_TXFNF;
        gnss_probe_init();
        run(7000U);
        CHECK(gnss_probe_auto_ready());
        const unsigned before = setter_count;
        defaults_fixture = !strcmp(argv[1], "defaults-missing")         ? 1U
                           : !strcmp(argv[1], "defaults-baud")          ? 2U
                           : !strcmp(argv[1], "defaults-duplicate")     ? 3U
                           : !strcmp(argv[1], "defaults-pps-duplicate") ? 4U
                                                                        : 0U;
        if (!strcmp(argv[1], "defaults-direct"))
        {
            gnss_probe_request(GNSS_PROFILE_DEFAULTS); /* USB gnss defaults route. */
        }
        else
        {
            CHECK(gnss_probe_defaults_request()); /* system defaults route. */
        }
        run(15000U);
        if (defaults_fixture)
        {
            CHECK(strstr(control_event_line, "status=defaults_failed"));
            CHECK(!gnss_probe_auto_ready());
            if (defaults_fixture <= 3U)
            {
                CHECK(setter_count == before); /* COM preflight precedes all mutations. */
            }
        }
        else
        {
            CHECK(strstr(control_event_line, "status=defaults_saved"));
            CHECK(gnss_probe_auto_ready() && receiver_low_static && !receiver_pps_off);
        }
        return 0;
    }
    if (argc == 2 && !strncmp(argv[1], "dma-", 4U))
    {
        test_usart2.ISR = USART_ISR_TXE_TXFNF;
        gnss_probe_init();
        run(7000U);
        CHECK(gnss_probe_auto_ready());
        CHECK(test_dmamux1_channel0.CCR == 52U && (test_usart2.CR3 & USART_CR3_DMAR) &&
              !(test_usart2.CR1 & USART_CR1_RXNEIE_RXFNEIE));
        CHECK(status_counter(" rx_errors=") == 0U);
        diagnostic_capture = true;
        if (!strcmp(argv[1], "dma-save"))
        {
            settings_flash_test_pause = flash_pause;
            CHECK(gnss_probe_control("gnss write setPVTMode,Rover,StandAlone,auto"));
            run(7000U);
            CHECK(flash_pauses == 1U && diagnostic_sbf_observed == 14U);
            CHECK(strstr(control_event_line, "status=saved") &&
                  strstr(control_event_line, "rx_loss=0"));
        }
        else if (!strcmp(argv[1], "dma-wrap"))
        {
            for (unsigned i = 0U; i < 320U; i++)
            {
                receive_status();
                run(1U);
            }
            CHECK(diagnostic_sbf_observed == 320U);
        }
        else if (!strcmp(argv[1], "dma-pending") || !strcmp(argv[1], "dma-terminal"))
        {
            /* Place producer exactly 64 bytes before a wrap, then hold its
             * TC IRQ pending through a foreground snapshot. */
            while (test_dma1_channel1.CNDTR != 64U)
            {
                receive("\n");
                run(1U);
            }
            __disable_irq();
            const bool terminal = !strcmp(argv[1], "dma-terminal");
            const unsigned frames = terminal ? 2U : 4U;
            for (unsigned i = 0U; i < frames; i++)
            {
                receive_status();
            }
            CHECK(test_dma1.ISR & DMA_ISR_TCIF1);
            if (terminal)
            {
                test_dma1_channel1.CNDTR = 0U; /* Count complete, before automatic reload. */
            }
            run(1U);
            if (terminal)
            {
                test_dma1_channel1.CNDTR = 2048U;
            }
            __set_PRIMASK(0U);
            run(10U);
            CHECK(diagnostic_sbf_observed == frames);
        }
        else if (!strcmp(argv[1], "dma-overflow") || !strcmp(argv[1], "dma-stall"))
        {
            __disable_irq();
            for (unsigned i = 0U; i < 2400U; i++)
            {
                receive("\n");
            }
            if (!strcmp(argv[1], "dma-stall"))
            {
                tick += 210U;
            }
            __set_PRIMASK(0U);
            run(1U);
            CHECK(status_counter(!strcmp(argv[1], "dma-stall") ? " rx_errors=" : " rx_overflow=") >
                  0U);
            return 0;
        }
        else if (!strcmp(argv[1], "dma-error"))
        {
            test_dma1.ISR |= DMA_ISR_TEIF1;
            dma_dispatch();
            CHECK(status_counter(" rx_errors=") == 1U);
            return 0;
        }
        else
        {
            CHECK(false);
        }
        CHECK(status_counter(" rx_errors=") == 0U);
        CHECK(status_counter(" rx_overflow=") == 0U);
        CHECK(status_counter(" crc_errors=") == 0U);
        return 0;
    }
    if (argc == 2 && (!strncmp(argv[1], "static-", 7U) || !strncmp(argv[1], "usb-", 4U)))
    {
        const bool static_auto = !strcmp(argv[1], "static-auto");
        const bool static_fixed = !strcmp(argv[1], "static-fixed");
        const bool static_has = !strcmp(argv[1], "static-has");
        const bool static_sbas = !strcmp(argv[1], "static-sbas");
        const bool static_edit = !strcmp(argv[1], "static-edit");
        const bool static_sbas_edit = !strcmp(argv[1], "static-sbas-edit");
        const bool usb_before = !strcmp(argv[1], "usb-before");
        const bool usb_completion = !strcmp(argv[1], "usb-completion");
        const bool usb_read_busy = !strcmp(argv[1], "usb-read-busy");
        const bool usb_partial = !strcmp(argv[1], "usb-partial-write");
        CHECK(static_auto || static_fixed || static_has || static_sbas || static_edit ||
              static_sbas_edit || usb_before || usb_completion || usb_read_busy || usb_partial);
        sc_settings_t saved;
        sc_settings_defaults(&saved);
        saved.gnss_selection = static_has || static_edit || static_sbas_edit ? SC_SETTINGS_GNSS_HAS
                               : static_sbas                                 ? SC_SETTINGS_GNSS_SBAS
                                             : SC_SETTINGS_GNSS_STANDALONE;
        saved.receiver_user_profile = 1U;
        CHECK(settings_flash_save(&saved));
        receiver_ppp = static_has || static_edit || static_sbas_edit;
        receiver_sbas = static_sbas;
        receiver_static = static_auto || static_fixed || static_has || static_sbas;
        receiver_ref = static_fixed ? "Geodetic1" : "auto";
        receiver_high_static = receiver_stream = receiver_data_sbf = receiver_timing_auto = true;
        test_usart2.ISR = USART_ISR_TXE_TXFNF;
        gnss_probe_init();
        run(9000U);
        if (static_has)
        {
            CHECK(!gnss_probe_auto_ready() && gnss_probe_auto_fault());
            CHECK(setter_count == 0U && boot_saves == 0U);
            return 0;
        }
        CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault());
        CHECK(gnss_probe_static_pvt() == (static_auto || static_fixed || static_sbas));
        if (static_edit || static_sbas_edit)
        {
            CHECK(gnss_probe_control(static_sbas_edit
                                         ? "gnss write setPVTMode,Static,StandAlone+SBAS,auto"
                                         : "gnss write setPVTMode,Static,StandAlone,auto"));
            run(9000U);
            CHECK(strstr(control_event_line, "status=saved") && receiver_static &&
                  gnss_probe_selection() == (uint32_t)(static_sbas_edit
                                                           ? SC_SETTINGS_GNSS_SBAS
                                                           : SC_SETTINGS_GNSS_STANDALONE));
            CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault() && gnss_probe_static_pvt());
            gnss_probe_init();
            run(9000U);
            CHECK(gnss_probe_auto_ready() && gnss_probe_static_pvt() && setter_count == 1U &&
                  boot_saves == 1U);
            return 0;
        }
        if (usb_partial)
        {
            usb_block_after_set = true;
            CHECK(gnss_probe_control("gnss write setReceiverDynamics,Low,Static"));
            run(9000U);
            CHECK(gnss_probe_commands_idle() && setter_count == 1U && boot_saves == 0U &&
                  receiver_low_static && manual_holds == 1U && !gnss_probe_auto_ready());
            usb_available = true;
            usb_block_after_set = false;
            CHECK(gnss_probe_control("gnss read getReceiverDynamics"));
            run(2000U);
            CHECK(strstr(control_event_line, "status=complete") && boot_saves == 0U &&
                  !gnss_probe_auto_ready()); /* No implicit save or resume after partial edit. */
            return 0;
        }
        if (usb_before || usb_completion || usb_read_busy)
        {
            const unsigned before = command_count;
            usb_available = !usb_before;
            usb_block_after_reply = usb_completion;
            receiver_actuator_busy = usb_read_busy;
            CHECK(gnss_probe_control("gnss read getPVTMode"));
            run(9000U);
            CHECK(gnss_probe_commands_idle()); /* No indefinite USB event owner. */
            CHECK(command_count == before + (usb_before ? 0U : 1U));
            usb_available = true;
            usb_block_after_reply = false;
            CHECK(gnss_probe_control("gnss read getReceiverDynamics"));
            run(2000U);
            CHECK(strstr(control_event_line, "status=complete"));
            CHECK(gnss_probe_commands_idle());
        }
        CHECK(setter_count == 0U && boot_saves == 0U);
        if (static_auto || static_fixed || static_sbas)
        {
            gnss_probe_init();
            run(9000U);
            CHECK(gnss_probe_auto_ready() && gnss_probe_static_pvt() && setter_count == 0U &&
                  boot_saves == 0U);
        }
        return 0;
    }
    if (argc == 2 && strncmp(argv[1], "latch-", 6U) == 0)
    {
        const char *scenario = argv[1] + 6U;
        const bool headless = strcmp(scenario, "headless") == 0;
        const bool clear = strcmp(scenario, "clear") == 0;
        const bool other = strcmp(scenario, "other-error") == 0;
        const bool stale = strcmp(scenario, "stale") == 0;
        const bool invalid = strcmp(scenario, "invalid-epoch") == 0;
        const bool no_ref = strcmp(scenario, "no-ref") == 0;
        const bool fresh = strcmp(scenario, "fresh") == 0;
        const bool reboot = strcmp(scenario, "reboot") == 0;
        const bool config_fault = strcmp(scenario, "config-fault") == 0;
        const bool busy = strcmp(scenario, "busy") == 0;
        latch_persistent = strcmp(scenario, "persistent") == 0;
        latch_timeout = strcmp(scenario, "timeout") == 0;
        latch_reject = strcmp(scenario, "reject") == 0;
        CHECK(headless || clear || other || stale || invalid || no_ref || fresh || reboot ||
              config_fault || busy || latch_persistent || latch_timeout || latch_reject);
        sc_settings_t saved, restored;
        uint32_t generation, restored_generation;
        sc_settings_defaults(&saved);
        saved.gnss_selection = SC_SETTINGS_GNSS_HAS;
        saved.receiver_user_profile = 1U;
        CHECK(settings_flash_save(&saved));
        CHECK(settings_flash_load(&restored, &generation) == SC_SETTINGS_VALID);
        receiver_ppp = receiver_high_static = true;
        receiver_stream = receiver_data_sbf = receiver_timing_auto = true;
        usb_available = false; /* No host may be required for recovery. */
        latch_test = true;
        latch_status_enabled = !stale && !busy;
        latch_status_valid = !invalid;
        latch_reference = !no_ref;
        latch_pause_after_read = fresh;
        clock_steering = config_fault;
        latch_error = clear ? 0U : other ? 2056U : 8U;
        test_usart2.ISR = USART_ISR_TXE_TXFNF;
        gnss_probe_init();
        if (stale)
        {
            run(1U);          /* Let AUTO invalidate the previous session's status. */
            receive_status(); /* This lone report will be stale when configuration is ready. */
        }
        run(9000U);
        if (latch_timeout)
        {
            run(5000U); /* Command timeout followed by the bounded status wait. */
        }
        if (config_fault)
        {
            CHECK(latch_reads == 0U && gnss_probe_auto_fault() && !gnss_probe_auto_ready());
            clock_steering = false;
            CHECK(gnss_probe_auto_recheck());
            run(9000U);
        }
        if (busy)
        {
            CHECK(latch_reads == 0U && gnss_probe_auto_ready());
            CHECK(gnss_probe_control("gnss read getPVTMode"));
            latch_status_enabled = true;
            run(3000U);
            CHECK(latch_reads == 0U); /* The queued host transaction owns UART. */
            usb_available = true;
            run(4000U);
            CHECK(strstr(control_event_line, "status=complete"));
            usb_available = false;
        }
        if (clear)
        {
            CHECK(latch_reads == 0U && gnss_probe_auto_ready());
            latch_error = 8U; /* A warning appearing later must stay latched. */
            run(10000U);
            CHECK(latch_reads == 0U && latch_error == 8U);
        }
        else
        {
            if (other || stale || invalid || no_ref)
            {
                CHECK(latch_reads == 0U);
                latch_error = 8U;
                latch_status_enabled = latch_status_valid = latch_reference = true;
                run(3000U);
            }
            CHECK(latch_reads == 1U);
            if (fresh)
            {
                /* Stop at the next acknowledgement: a pre-completion zero
                 * status cannot certify recovery. The wait also has a bound. */
                CHECK(gnss_probe_auto_ready()); /* First wait expired after 5 s. */
                CHECK(gnss_probe_reboot_request());
                latch_status_enabled = true;
                for (unsigned i = 0U; i < 15000U && latch_reads < 2U; i++)
                {
                    run(1U);
                }
                CHECK(latch_reads == 2U);
                run(10U);
                CHECK(!gnss_probe_auto_ready());
                receive_status(); /* Post-completion status with no error. */
                run(2U);
                CHECK(gnss_probe_auto_ready());
            }
            else
            {
                CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault());
            }
            const unsigned reads = latch_reads;
            latch_error = 8U;
            latch_status_enabled = true;
            run(10000U);
            CHECK(latch_reads == reads); /* Never repeatedly clear a warning. */
            CHECK(gnss_probe_auto_recheck());
            run(9000U);
            CHECK(latch_reads == reads); /* Resume/requery does not rearm. */
            if (reboot)
            {
                CHECK(gnss_probe_reboot_request());
                run(16000U);
                CHECK(reboot_count == 1U && latch_reads == reads + 1U && latch_error == 0U);
                CHECK(gnss_probe_auto_ready());
            }
        }
        CHECK(setter_count == 0U && boot_saves == 0U && manual_holds == 0U);
        CHECK(settings_flash_load(&restored, &restored_generation) == SC_SETTINGS_VALID &&
              generation == restored_generation && sc_settings_equal(&saved, &restored));
        return 0;
    }
    if (argc == 2 && strncmp(argv[1], "reply-burst", 11U) == 0)
    {
        const bool ordinary = strcmp(argv[1], "reply-burst") == 0;
        diagnostic_mixed = strcmp(argv[1], "reply-burst-mixed") == 0;
        diagnostic_headless = strcmp(argv[1], "reply-burst-headless") == 0;
        diagnostic_busy_ms = strcmp(argv[1], "reply-burst-busy") == 0 ? 40U : 0U;
        CHECK(ordinary || diagnostic_headless || diagnostic_busy_ms || diagnostic_mixed);
        test_usart2.ISR = USART_ISR_TXE_TXFNF;
        receiver_stream = receiver_data_sbf = receiver_timing_auto = true;
        gnss_probe_init();
        run(9000U);
        CHECK(gnss_probe_auto_ready());
        const unsigned before = command_count, saves = boot_saves, setters = setter_count;
        diagnostic_test = diagnostic_capture = true;
        CHECK(gnss_probe_control("gnss read lstInternalFile,Error"));
        run(2000U);
        usb_available = true;
        run(1000U);
        CHECK(diagnostic_position == diagnostic_length && diagnostic_length > 4096U);
        CHECK(command_count == before + 1U && boot_saves == saves && setter_count == setters);
        CHECK(gnss_probe_auto_ready());
        if (diagnostic_headless)
        {
            CHECK(strstr(control_event_line, "status=incomplete"));
            CHECK(diagnostic_captured < diagnostic_length);
        }
        else
        {
            CHECK(strstr(control_event_line, "status=complete"));
            CHECK(strstr(control_event_line, " drops=0 rx_loss=0"));
            CHECK(!diagnostic_bad && diagnostic_captured == diagnostic_length);
            CHECK(memcmp(diagnostic_received, diagnostic_reply, diagnostic_length) == 0);
            if (diagnostic_mixed)
            {
                CHECK(diagnostic_sbf_injected > 5U);
                CHECK(diagnostic_sbf_observed == diagnostic_sbf_injected);
                CHECK(diagnostic_sbf_exported == diagnostic_sbf_injected);
            }
        }
        return 0;
    }
    if (argc == 2 && strncmp(argv[1], "managed-", 8U) == 0)
    {
        const bool delayed = strcmp(argv[1], "managed-timeout") == 0;
        const bool recheck = strcmp(argv[1], "managed-recheck") == 0;
        const bool incompatible = strcmp(argv[1], "managed-config-fault") == 0;
        const bool busy = strcmp(argv[1], "managed-busy") == 0;
        CHECK(delayed || recheck || incompatible || busy);
        sc_settings_t saved, restored;
        uint32_t generation, restored_generation;
        sc_settings_defaults(&saved);
        saved.gnss_selection = SC_SETTINGS_GNSS_HAS;
        saved.receiver_user_profile = 1U;
        CHECK(settings_flash_save(&saved));
        CHECK(settings_flash_load(&restored, &generation) == SC_SETTINGS_VALID);
        receiver_ppp = receiver_high_static = true;
        receiver_sbas = receiver_low_static = false;
        receiver_stream = receiver_data_sbf = receiver_timing_auto = true;
        usb_available = false;
        test_usart2.ISR = USART_ISR_TXE_TXFNF;
        lose_queries = delayed || recheck;
        clock_steering = incompatible;
        gnss_probe_init();
        run(7000U);
        if (busy)
        {
            CHECK(gnss_probe_auto_ready());
            CHECK(gnss_probe_control("gnss read getPVTMode"));
            CHECK(!gnss_probe_auto_recheck()); /* Do not replace a queued query. */
            usb_available = true;
            run(2000U);
            CHECK(strstr(control_event_line, "status=complete"));
        }
        else
        {
            CHECK(gnss_probe_auto_fault() && !gnss_probe_auto_ready());
            if (delayed)
            {
                run(150000U); /* Longer than the old three-retry limit. */
                CHECK(command_count >= 4U && gnss_probe_auto_fault());
                CHECK(setter_count == 0U && boot_saves == 0U);
                lose_queries = false;
                run(37000U);
            }
            else
            {
                if (incompatible)
                {
                    const unsigned commands = command_count;
                    run(70000U);
                    CHECK(command_count == commands && clock_steering);
                    clock_steering = false; /* Model an intentional external repair. */
                }
                lose_queries = false;
                CHECK(gnss_probe_auto_recheck());
                run(7000U);
            }
        }
        CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault());
        CHECK(receiver_ppp && !receiver_sbas && receiver_high_static && !receiver_low_static &&
              setter_count == 0U && boot_saves == 0U);
        CHECK(settings_flash_load(&restored, &restored_generation) == SC_SETTINGS_VALID &&
              generation == restored_generation && sc_settings_equal(&saved, &restored));
        const unsigned healthy_commands = command_count;
        run(37000U);
        CHECK(command_count == healthy_commands); /* No settings polling once healthy. */
        return 0;
    }
    if (argc == 2 && !strncmp(argv[1], "lists", 5))
    {
        test_usart2.ISR = USART_ISR_TXE_TXFNF;
        receiver_stream = receiver_data_sbf = receiver_timing_auto = true;
        gnss_probe_init();
        run(7000U);
        CHECK(gnss_probe_auto_ready() && boot_saves == 1U);
        lists_mode = true;
        lists_corrupt = !strcmp(argv[1], "lists-corrupt");
        const bool dependency = !strcmp(argv[1], "lists-dependency");
        CHECK(strlen(lists_pvt) > 192U);
        const unsigned quiet_commands = command_count, quiet_events = control_events;
        run(10000U);
        CHECK(command_count == quiet_commands && control_events == quiet_events);
        CHECK(gnss_probe_control("gnss read getSignalUsage"));
        run(2000U);
        CHECK(command_count == quiet_commands + 1U);
        CHECK(strstr(control_event_line, "status=complete"));
        const unsigned writes = setter_count, saves = boot_saves;
        CHECK(gnss_probe_control(dependency ? "gnss write setSignalTracking,-GPSL1CA"
                                            : "gnss write setSignalUsage,,+GALE6BC"));
        run(6000U);
        if (dependency || lists_corrupt)
        {
            CHECK(strstr(control_event_line, dependency ? "status=invalid_list_or_dependencies"
                                                        : "status=readback_mismatch"));
            CHECK(setter_count == writes + (dependency ? 0U : 1U) && boot_saves == saves);
            CHECK(!gnss_probe_auto_ready());
            return 0;
        }
        CHECK(strstr(control_event_line, "status=saved") &&
              strstr(control_event_line, "truncated=0"));
        CHECK(lists_e6 && setter_count == writes + 1U && boot_saves == saves + 1U);
        CHECK(gnss_probe_auto_ready() && manual_holds == 1U);
        CHECK(gnss_probe_control("gnss write setSignalUsage,,+GALE6BC"));
        run(6000U);
        CHECK(strstr(control_event_line, "status=saved") &&
              strstr(control_event_line, "changed=0"));
        CHECK(setter_count == writes + 1U && boot_saves == saves + 2U);
        const unsigned final_commands = command_count, final_events = control_events;
        run(37000U);
        CHECK(command_count == final_commands && control_events == final_events);
        return 0;
    }
    if (argc == 2 && !strncmp(argv[1], "control", 7))
    {
        test_usart2.ISR = USART_ISR_TXE_TXFNF;
        receiver_stream = receiver_data_sbf = receiver_timing_auto = true;
        gnss_probe_init();
        run(7000U);
        CHECK(gnss_probe_auto_ready() && boot_saves == 1U);
        if (!strcmp(argv[1], "control-noop-cache"))
        {
            /* Current already differs from the MCU's old desired selection,
             * e.g. an earlier volatile edit. A satisfied setter still saves it. */
            receiver_ppp = false;
            receiver_sbas = true;
            const unsigned writes = setter_count;
            CHECK(gnss_probe_control("gnss write setPVTMode,Rover,StandAlone+SBAS,auto"));
            run(6000U);
            CHECK(strstr(control_event_line, "status=saved") &&
                  strstr(control_event_line, "changed=0"));
            CHECK(setter_count == writes && boot_saves == 2U && gnss_probe_auto_ready());
            CHECK(gnss_probe_selection() == SC_SETTINGS_GNSS_SBAS);
            sc_settings_t saved;
            uint32_t generation;
            CHECK(settings_flash_load(&saved, &generation) == SC_SETTINGS_VALID);
            CHECK(saved.gnss_selection == SC_SETTINGS_GNSS_SBAS &&
                  saved.receiver_user_profile == 1U);
            gnss_probe_init();
            run(7000U);
            CHECK(gnss_probe_auto_ready() && setter_count == writes && boot_saves == 2U);
            receiver_pps_off = true;
            CHECK(gnss_probe_control("gnss write setPPSParameters,off"));
            run(2000U);
            CHECK(strstr(control_event_line, "status=saved") &&
                  strstr(control_event_line, "changed=0"));
            gnss_probe_pps_snapshot_t pps;
            CHECK(gnss_probe_pps_snapshot(&pps) && strstr(pps.response, "PPSParameters, off,"));
            CHECK(setter_count == writes && boot_saves == 3U);
            return 0;
        }
        if (!strcmp(argv[1], "control-busy"))
        {
            /* A steering clock nearly always has an FCW write awaiting readback:
             * the write holds first and waits; it is refused only if that never
             * settles, with nothing sent to the receiver. */
            receiver_actuator_busy = true;
            const unsigned before_busy = command_count, holds = manual_holds;
            CHECK(gnss_probe_control("gnss write setReceiverDynamics,High"));
            run(1500U);
            CHECK(command_count == before_busy && manual_holds == holds + 1U);
            receiver_actuator_busy = false;
            run(5000U);
            CHECK(strstr(control_event_line, "status=saved") && receiver_high_static);
            receiver_actuator_busy = true;
            const unsigned before_refused = command_count;
            CHECK(gnss_probe_control("gnss write setReceiverDynamics,Moderate"));
            run(5000U);
            CHECK(strstr(control_event_line, "status=busy") && command_count == before_refused);
            receiver_actuator_busy = false;
            return 0;
        }
        control_ignore = !strcmp(argv[1], "control-ignore");
        control_corrupt = !strcmp(argv[1], "control-corrupt");
        fail_save = !strcmp(argv[1], "control-save-fail");
        fail_mcu_save = !strcmp(argv[1], "control-mcu-fail");
        const unsigned before = command_count;
        usb_available = false;
        CHECK(gnss_probe_control("gnss write setReceiverDynamics,High"));
        run(500U);
        CHECK(command_count == before); /* No transmission before USB accepts event. */
        usb_available = true;
        run(5000U);
        if (control_ignore || control_corrupt || fail_save || fail_mcu_save)
        {
            CHECK(strstr(control_event_line, "stage=done") &&
                  !strstr(control_event_line, "status=saved"));
            CHECK(!gnss_probe_auto_ready() && manual_holds == 1U);
            CHECK(boot_saves == (control_ignore || control_corrupt ? 1U : 2U));
            return 0;
        }
        CHECK(strstr(control_event_line, "status=saved") && strstr(control_event_line, "saved=1"));
        CHECK(receiver_high_static && boot_high_static && gnss_probe_user_profile());
        const unsigned writes = setter_count, saves = boot_saves;
        CHECK(gnss_probe_control("gnss write setReceiverDynamics,High"));
        run(2000U);
        CHECK(strstr(control_event_line, "status=saved") &&
              strstr(control_event_line, "changed=0") && setter_count == writes &&
              boot_saves == saves + 1U);
        /* Power-cycle model: Current reloads receiver Boot; MCU reloads its own record. */
        receiver_high_static = boot_high_static;
        receiver_low_static = !receiver_high_static;
        gnss_probe_init();
        run(7000U);
        CHECK(gnss_probe_auto_ready() && receiver_high_static && setter_count == writes &&
              boot_saves == saves + 1U);
        lose_reply = true;
        gnss_probe_request(gnss_probe_command_profile("gnss mask pvt 15"));
        run(6500U);
        CHECK(!gnss_probe_auto_fault() && mask_fault_flag() == 1U); /* Re-read, reported. */
        lose_reply = false;
        CHECK(gnss_probe_defaults_request());
        run(500U);
        gnss_probe_identify();
        gnss_probe_request(GNSS_PROFILE_STANDALONE); /* Cannot interrupt defaults ownership. */
        run(12000U);
        CHECK(strstr(control_event_line, "status=defaults_saved") && receiver_low_static &&
              !receiver_high_static);
        CHECK(!receiver_pps_off && gnss_probe_auto_ready() && !gnss_probe_auto_fault() &&
              pvt_mask == 5 && tracking_mask == 5);
        return 0;
    }
    if (argc == 2 && strncmp(argv[1], "saved-", 6U) == 0)
    {
        const char *name = argv[1] + 6U;
        sc_settings_t settings;
        sc_settings_defaults(&settings);
        settings.gnss_selection = strcmp(name, "sbas") == 0         ? SC_SETTINGS_GNSS_SBAS
                                  : strcmp(name, "has") == 0        ? SC_SETTINGS_GNSS_HAS
                                  : strcmp(name, "standalone") == 0 ? SC_SETTINGS_GNSS_STANDALONE
                                                                    : SC_SETTINGS_GNSS_AUTO;
        CHECK(settings_flash_save(&settings));
        test_usart2.ISR = USART_ISR_TXE_TXFNF;
        receiver_stream = receiver_data_sbf = receiver_timing_auto = true;
        gnss_probe_init();
        CHECK(!gnss_probe_selection_ready());
        run(7000U);
        CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault() && gnss_probe_selection_ready());
        CHECK(gnss_probe_selection() == settings.gnss_selection);
        CHECK(receiver_sbas == (settings.gnss_selection == SC_SETTINGS_GNSS_AUTO ||
                                settings.gnss_selection == SC_SETTINGS_GNSS_SBAS));
        CHECK(receiver_ppp == (settings.gnss_selection == SC_SETTINGS_GNSS_AUTO ||
                               settings.gnss_selection == SC_SETTINGS_GNSS_HAS));
        const unsigned previous_setters = setter_count;
        gnss_probe_auto_restart();
        run(7000U);
        CHECK(gnss_probe_auto_ready() && setter_count == previous_setters &&
              gnss_probe_selection() == settings.gnss_selection);
        gnss_probe_request(GNSS_PROFILE_AUTO_HYBRID); /* Explicit auto restores multimode. */
        CHECK(!gnss_probe_selection_ready());
        run(7000U);
        CHECK(gnss_probe_auto_ready() && receiver_sbas && receiver_ppp &&
              gnss_probe_selection() == SC_SETTINGS_GNSS_AUTO);
        sc_settings_t still_saved;
        uint32_t generation;
        CHECK(settings_flash_load(&still_saved, &generation) == SC_SETTINGS_VALID &&
              still_saved.gnss_selection == SC_SETTINGS_GNSS_AUTO &&
              still_saved.receiver_user_profile == 1U);
        return 0;
    }
    const bool reboot_test =
        argc == 2 && (strcmp(argv[1], "reboot") == 0 || strcmp(argv[1], "reboot-reject") == 0);
    reject_reboot = reboot_test && strcmp(argv[1], "reboot-reject") == 0;
    const bool headless = argc == 2 && strcmp(argv[1], "headless") == 0;
    const bool reconnect = argc == 2 && strcmp(argv[1], "reconnect") == 0;
    early_reply = argc == 2 && strcmp(argv[1], "early-reply") == 0;
    cr_reply = argc == 2 && strcmp(argv[1], "cr-reply") == 0;
    missing_readback = argc == 3 && strcmp(argv[1], "missing") == 0;
    duplicate_readback = argc == 3 && strcmp(argv[1], "duplicate") == 0;
    pre_echo_readback = argc == 3 && strcmp(argv[1], "pre-echo") == 0;
    CHECK(argc == 1 || reboot_test || headless || reconnect || early_reply || cr_reply ||
          missing_readback || duplicate_readback || pre_echo_readback);
    if (missing_readback || duplicate_readback || pre_echo_readback)
    {
        static const char *const names[] = {"clock", "timing", "data", "stream"};
        static const char *const queries[] = {"getClockSyncThreshold\r\n", "getTimingSystem\r\n",
                                              "getDataInOut, COM1\r\n",
                                              "getSBFOutput, Stream1\r\n"};
        static const char *const bad_lines[] = {
            "  ClockSyncThreshold, ClockSteering\r\n", "  TimingSystem, GPS\r\n",
            "  DataInOut, COM1, auto, NMEA, (on)\r\n", "  SBFOutput, Stream1, none, none, off\r\n"};
        static const char *const good_lines[] = {
            "  ClockSyncThreshold, msec1, on\r\n", "  TimingSystem, auto\r\n",
            "  DataInOut, COM1, auto, SBF+NMEA, (on)\r\n",
            ("  SBFOutput, Stream1, COM1, "
             "PVTGeodetic+ReceiverTime+ReceiverStatus+MeasEpoch+EndOfPVT"
             "+PosCovCartesian+VelCovCartesian+RFStatus+DOP, sec1\r\n")};
        for (unsigned i = 0U; i < 4U; ++i)
        {
            if (strcmp(argv[2], names[i]) == 0)
            {
                fault_query = queries[i];
                duplicate_line = bad_lines[i];
                pre_echo_line = good_lines[i];
            }
        }
        CHECK(fault_query != NULL);
    }
    usb_available = !(headless || reconnect);
    test_usart2.ISR = USART_ISR_TXE_TXFNF;
    receiver_stream = receiver_data_sbf = receiver_timing_auto = true;
    gnss_probe_init();
    run(6000U);
    if (missing_readback || duplicate_readback || pre_echo_readback)
    {
        CHECK(fault_query_count == 2U);
        CHECK(gnss_probe_auto_fault() && !gnss_probe_auto_ready());
        CHECK(setter_count == 0U); /* Stop before any later setter can run. */
        return 0;
    }
    CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault());
    CHECK(receiver_ppp && receiver_sbas);
    CHECK(manual_holds == 0U);
    if (reboot_test)
    {
        const unsigned previous_setters = setter_count;
        test_usart2.ISR |= USART_ISR_TC;
        CHECK(gnss_probe_reboot_request());
        CHECK(!gnss_probe_reboot_request() && !gnss_probe_auto_ready());
        gnss_probe_request(GNSS_PROFILE_STANDALONE); /* Cannot replace the reset. */
        run(16000U);
        CHECK(reboot_count == 1U && gnss_probe_reboot_transmitted());
        CHECK(gnss_probe_reboot_failed() == reject_reboot);
        CHECK(gnss_probe_selection_ready() && setter_count == previous_setters && boot_saves == 1U);
        run(10000U);
        CHECK(reboot_count == 1U);
        return 0;
    }
    CHECK(setter_count == 2U && pvt_setter_count == 1U && receiver_low_static &&
          receiver_saved_managed);
    gnss_probe_pps_snapshot_t snapshot;
    CHECK(gnss_probe_pps_snapshot(&snapshot) && strstr(snapshot.response, "RxClock") != NULL);

    if (headless || reconnect)
    {
        CHECK(usb_rejected != 0U && usb_accepted == 0U);
    }
    if (reconnect)
    {
        const unsigned before_connect = command_count, before_setters = setter_count;
        usb_available = true;
        run(6000U);
        CHECK(usb_accepted != 0U);
        CHECK(command_count == before_connect && setter_count == before_setters);
        CHECK(gnss_probe_auto_ready() && manual_holds == 0U);
        usb_available = false; /* Later manual selections and retries must also finish. */
    }

    const unsigned before_nohas = setter_count;
    gnss_probe_request(GNSS_PROFILE_NO_HAS);
    run(6500U);
    CHECK(!receiver_ppp && receiver_sbas && manual_holds == 1U);
    CHECK(gnss_probe_auto_ready());
    CHECK(setter_count == before_nohas + 1U && pvt_setter_count + 1U == setter_count);
    const unsigned after_nohas_commands = command_count;
    run(37000U);
    CHECK(command_count == after_nohas_commands && !receiver_ppp);
    CHECK(gnss_probe_mutation_hold()); /* Manual mutation holds automatic control. */
    gnss_probe_request(GNSS_PROFILE_HAS);
    run(6500U);
    CHECK(receiver_ppp && !receiver_sbas && manual_holds == 2U);
    CHECK(setter_count == before_nohas + 2U && pvt_setter_count + 1U == setter_count);
    CHECK(gnss_probe_mutation_hold()); /* Restoring HAS never resumes the servo. */

    receiver_ppp = false;
    gnss_probe_auto_restart();
    lose_reply = true;
    run(6500U); /* Reach the PVT setter, then let its readback timeout. */
    CHECK(gnss_probe_auto_fault() && !gnss_probe_auto_ready());
    lose_reply = false;
    run(37000U); /* bounded autonomous retry after the 30 s quiet interval */
    CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault());

    clock_steering = true;
    gnss_probe_auto_restart();
    const unsigned setters_before_steering_repair = setter_count;
    run(6000U);
    CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault());
    CHECK(setter_count == setters_before_steering_repair + 1U && !clock_steering);

    wrong_stream = true;
    gnss_probe_auto_restart();
    run(6000U);
    CHECK(gnss_probe_auto_fault() && !gnss_probe_auto_ready());
    wrong_stream = false;
    run(30500U);
    CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault());

    overflow_stream = true;
    gnss_probe_auto_restart();
    run(6000U);
    CHECK(gnss_probe_auto_fault() && !gnss_probe_auto_ready());
    overflow_stream = false;
    gnss_probe_request(GNSS_PROFILE_NO_HAS);
    run(6500U);
    CHECK(receiver_sbas && !receiver_ppp && gnss_probe_mutation_hold());
    const unsigned before_quiet = command_count;
    run(37000U); /* Explicit no-HAS cancels the pending automatic retry too. */
    CHECK(command_count == before_quiet && !receiver_ppp);
    const unsigned before_standalone = setter_count;
    const unsigned holds_before_standalone = manual_holds;
    gnss_probe_request(GNSS_PROFILE_STANDALONE);
    run(6500U);
    CHECK(!receiver_sbas && !receiver_ppp && gnss_probe_mutation_hold());
    CHECK(manual_holds == holds_before_standalone + 1U);
    CHECK(setter_count == before_standalone + 1U); /* Only PVT eligibility changed. */
    const unsigned standalone_commands = command_count;
    run(37000U);
    CHECK(command_count == standalone_commands && !receiver_ppp && !receiver_sbas);
    /* Returning from a manual pure-standalone selection verifies the full
     * automatic allowed set, then a repeat auto request makes no extra writes. */
    gnss_probe_request(GNSS_PROFILE_AUTO_HYBRID);
    run(6500U);
    CHECK(gnss_probe_auto_ready() && receiver_ppp && receiver_sbas);
    const unsigned auto_setters = setter_count;
    gnss_probe_request(GNSS_PROFILE_AUTO_HYBRID);
    run(6500U);
    CHECK(gnss_probe_auto_ready() && setter_count == auto_setters);
    /* A selection setter whose reply is lost leaves the receiver state unknown.
     * The firmware reads it back, adopts the mode the receiver actually runs
     * and saves it: a known selection, never a latched fault. */
    lose_reply = true;
    gnss_probe_request(GNSS_PROFILE_STANDALONE);
    run(12000U);
    CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault() && !receiver_ppp && !receiver_sbas);
    CHECK(gnss_probe_selection() == SC_SETTINGS_GNSS_STANDALONE &&
          saved_selection() == SC_SETTINGS_GNSS_STANDALONE);
    lose_reply = false;
    /* A steering clock nearly always has an FCW write awaiting readback. The
     * switch holds first and sends the setter only once that has settled. */
    const unsigned pvt_before_busy = pvt_setter_count, holds_before_busy = manual_holds;
    receiver_actuator_busy = true;
    gnss_probe_request(GNSS_PROFILE_NO_HAS);
    run(1500U);
    CHECK(pvt_setter_count == pvt_before_busy && manual_holds == holds_before_busy + 1U);
    CHECK(gnss_probe_auto_ready()); /* Nothing has changed yet. */
    receiver_actuator_busy = false;
    run(6500U);
    CHECK(pvt_setter_count == pvt_before_busy + 1U && receiver_sbas && !receiver_ppp);
    CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault() &&
          saved_selection() == SC_SETTINGS_GNSS_SBAS);
    /* If it never settles, the switch is refused with the receiver untouched. */
    receiver_actuator_busy = true;
    gnss_probe_request(GNSS_PROFILE_STANDALONE);
    run(6500U);
    CHECK(pvt_setter_count == pvt_before_busy + 1U && receiver_sbas);
    CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault() &&
          gnss_probe_selection() == SC_SETTINGS_GNSS_SBAS && gnss_probe_commands_idle());
    receiver_actuator_busy = false;
    /* A failed MCU save after a verified switch: the receiver is re-read and
     * adopted, and the save is retried by the next recheck (hybrid resume). */
    fail_mcu_save = true;
    gnss_probe_request(GNSS_PROFILE_STANDALONE);
    run(12000U);
    CHECK(gnss_probe_auto_fault() && !gnss_probe_auto_ready() && !receiver_sbas);
    CHECK(gnss_probe_selection() == SC_SETTINGS_GNSS_STANDALONE &&
          saved_selection() == SC_SETTINGS_GNSS_SBAS);
    fail_mcu_save = false;
    CHECK(gnss_probe_auto_recheck());
    run(6500U);
    CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault() &&
          saved_selection() == SC_SETTINGS_GNSS_STANDALONE);
    gnss_probe_request(GNSS_PROFILE_PPS_OFF);
    run(1200U);
    CHECK(manual_holds != 0U);
    CHECK(mask_setters == 0U); /* Profile saves preserve the existing mask values. */
    boot_saves = 0U;           /* The remaining checks count only mask-operation saves. */
    const unsigned holds_before_masks = manual_holds;
    gnss_probe_request(GNSS_PROFILE_ELEVATION_SURVEY);
    run(1500U);
    CHECK(mask_setters == 0U && boot_saves == 0U && manual_holds == holds_before_masks);
    /* A mask setter whose reply is lost: the masks are read back right away.
     * mask_fault=1 reports the failed change; known values block nothing. */
    lose_reply = true;
    gnss_probe_request(gnss_probe_command_profile("gnss mask pvt 15"));
    run(6500U);
    CHECK(pvt_mask == 15 && tracking_mask == 5 && mask_setters == 1U && boot_saves == 0U);
    CHECK(!gnss_probe_auto_fault() && gnss_probe_auto_ready() && mask_fault_flag() == 1U);
    lose_reply = false;
    const unsigned before_selection = pvt_setter_count;
    gnss_probe_request(GNSS_PROFILE_NO_HAS);
    run(6500U);
    CHECK(pvt_setter_count == before_selection + 1U && receiver_sbas && boot_saves == 1U);
    CHECK(gnss_probe_auto_ready() && !gnss_probe_auto_fault());
    gnss_probe_request(gnss_probe_command_profile("gnss mask pvt 15"));
    run(6500U);
    CHECK(!gnss_probe_auto_fault() && gnss_probe_auto_ready() && mask_fault_flag() == 0U);
    CHECK(mask_setters == 2U && boot_saves == 2U && manual_holds == holds_before_masks + 3U);
    /* Verified values whose Boot save failed: known, reported, not blocking. */
    fail_save = true;
    gnss_probe_request(GNSS_PROFILE_ELEVATION_RESET);
    run(6500U);
    CHECK(pvt_mask == -90 && tracking_mask == -90 && boot_saves == 3U);
    CHECK(!gnss_probe_auto_fault() && gnss_probe_auto_ready() && mask_fault_flag() == 1U);
    fail_save = false;
    gnss_probe_request(GNSS_PROFILE_ELEVATION_RESET);
    run(6500U);
    CHECK(!gnss_probe_auto_fault() && gnss_probe_auto_ready() && mask_fault_flag() == 0U);
    CHECK(gnss_probe_mutation_hold());
    const unsigned writes_after_reset = mask_setters, saves_after_reset = boot_saves;
    run(37000U);
    CHECK(mask_setters == writes_after_reset && boot_saves == saves_after_reset);
    puts("autonomous GNSS query, skip, readback failure and PPS preservation paths passed");
    return 0;
}
