/* SPDX-License-Identifier: MIT */
/* TIM2 capture ISR plus foreground epoch/PPS telemetry.
 * Records preserve source timestamps; USB arrival never becomes a measurement epoch. */
#include "timing_probe.h"
#include "main.h"
#include "simpleclock/text.h"

typedef struct
{
    uint32_t tick, ms, sequence, overcapture;
} capture_t;

/* TIM2 ISR owns capture_head/sequence; foreground owns capture_tail. Records
 * are copied before publication (DMB), and queue loss invalidates PPS intervals.
 * All epoch/output queues below belong to foreground, never the capture ISR. */
static capture_t captures[8];
static volatile uint32_t capture_head, capture_tail, capture_sequence;
static volatile uint32_t capture_drops, overcaptures, max_isr_body_us;
static sc_epochs_t epochs;
/* PPS telemetry retains one pending snapshot; replacing it counts a drop.
 * Epoch telemetry keeps four records. USB queues do not alter source validity. */
static sc_pps_t pps, pending_pps;
static sc_epoch_t epoch_records[4];
static uint32_t epoch_head, epoch_tail, epoch_drops, pps_drops;
static uint32_t last_status_ms, last_send_ms, observed_capture_drops;
static uint32_t status_pps_sequence;
static bool status_pps_seen;
static bool pps_pending;
static uint32_t e6_pages, e6_crc_passed, e6_last_ms;

void timing_probe_init(void)
{
    GPIO_InitTypeDef gpio = {0};
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM2_FORCE_RESET();
    __HAL_RCC_TIM2_RELEASE_RESET();
    gpio.Pin = GPIO_PIN_5;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF3_TIM2;
    HAL_GPIO_Init(GPIOA, &gpio); /* PPS_ref input: PA5 / TIM2_CH1, after U6. */
    /* APB1 prescaler is 1; TIM2 clock is 48 MHz HSI. Capture every rising edge
     * at nominal 1 MHz, no input filter, no prescaling, no output compare. */
    TIM2->PSC = 47U;
    TIM2->ARR = UINT32_MAX;
    TIM2->EGR = TIM_EGR_UG;
    TIM2->CCMR1 = TIM_CCMR1_CC1S_0;
    TIM2->CCER = TIM_CCER_CC1E;
    TIM2->SR = 0U;
    TIM2->DIER = TIM_DIER_CC1IE;
    TIM2->CR1 = TIM_CR1_CEN;
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    epochs.pending_reasons = SC_SEG_START;
}

void TIM2_IRQHandler(void)
{
    const uint32_t start = TIM2->CNT, status = TIM2->SR;
    if (status & TIM_SR_CC1IF)
    {
        /* Reading CCR1 clears CC1IF. Never clear it afterward: a later capture
         * must stay pending. RC_W0 clears only the overcapture flag. */
        const uint32_t tick = TIM2->CCR1;
        const uint32_t over = (status | TIM2->SR) & TIM_SR_CC1OF;
        if (over)
        {
            overcaptures++;
            TIM2->SR = TIM_SR_CC1IF | TIM_SR_UIF;
        }
        const uint32_t sequence = capture_sequence++;
        if (capture_head - capture_tail < 8U)
        {
            captures[capture_head & 7U] = (capture_t){tick, HAL_GetTick(), sequence, over};
            __DMB();
            capture_head++;
        }
        else
        {
            capture_drops++;
        }
    }
    else if (status & TIM_SR_CC1OF)
    {
        overcaptures++;
        TIM2->SR = TIM_SR_CC1IF | TIM_SR_UIF;
    }
    /* Excludes interrupt entry/exit and latency; resolution is one HSI-based us. */
    const uint32_t elapsed = TIM2->CNT - start;
    if (elapsed > max_isr_body_us)
    {
        max_isr_body_us = elapsed;
    }
}

void timing_probe_observe(const sc_sbf_observation_t *observation, uint32_t now_ms)
{
    if (observation->id == SC_SBF_GAL_CNAV)
    {
        e6_pages++;
        if (observation->valid & SC_SBF_VALID_E6_PAGE)
        {
            e6_crc_passed++;
            e6_last_ms = now_ms;
        }
    }
    sc_epochs_push(&epochs, observation, now_ms);
}

void timing_probe_break(uint32_t reason)
{
    sc_epochs_break(&epochs, reason);
}

bool timing_probe_pps_ready(uint32_t now_ms)
{
    return sc_epochs_pps_ready(&epochs, now_ms);
}

bool timing_probe_fix_valid(uint32_t now_ms)
{
    return sc_epochs_fix_valid(&epochs, now_ms);
}

bool timing_probe_ppp_active(uint32_t now_ms)
{
    return sc_epochs_ppp_active(&epochs, now_ms);
}

const sc_epoch_t *timing_probe_latest(void)
{
    return epochs.have_last ? &epochs.last : NULL;
}

static void append_u32_field(sc_text_t *text, const char *key, uint32_t n)
{
    sc_text_string(text, key);
    sc_text_u32(text, n);
}

static const char *state_name(sc_pps_state_t state)
{
    static const char *const names[] = {"ABSENT", "ACQUIRING", "PRESENT", "LOST"};
    return names[state];
}

static void append_epoch_fields(sc_text_t *text, const sc_epoch_t *epoch, uint32_t now_ms)
{
    sc_text_string(text, "type=epoch v=1");
    append_u32_field(text, " uptime_ms=", now_ms);
    append_u32_field(text, " seq=", epoch->sequence);
    append_u32_field(text, " segment=", epoch->segment);
    append_u32_field(text, " reasons=", epoch->reasons);
    append_u32_field(text, " week=", epoch->week);
    append_u32_field(text, " tow_ms=", epoch->tow_ms);
    append_u32_field(text, " present=", epoch->mask);
    append_u32_field(text, " duplicate=", epoch->duplicate);
    append_u32_field(text, " sources_valid_at_assembly=", epoch->sources_valid ? 1U : 0U);
    static const char *const ages[] = {
        " pvt_age_ms=", " status_age_ms=", " time_age_ms=", " meas_age_ms=", " end_age_ms="};
    for (unsigned i = 0; i < 5U; i++)
    {
        append_u32_field(text, ages[i],
                         (epoch->mask & (1U << i)) ? now_ms - epoch->source_ms[i] : UINT32_MAX);
    }
    append_u32_field(text, " pvt_valid=", epoch->pvt.valid);
    append_u32_field(text, " mode=", epoch->pvt.data.pvt.mode);
    append_u32_field(text, " time_system=", epoch->pvt.data.pvt.time_system);
    append_u32_field(text, " wa_corr_info=", epoch->pvt.data.pvt.wa_corr_info);
    append_u32_field(text, " mean_corr_age_cs=", epoch->pvt.data.pvt.mean_corr_age);
    append_u32_field(text, " has_ppp_corr=", sc_pvt_has_corrections(&epoch->pvt) ? 1U : 0U);
    sc_text_string(text, " bias_bits=");
    sc_text_hex(text, (uint32_t)(epoch->pvt.data.pvt.bias_bits >> 32), 8);
    sc_text_hex(text, (uint32_t)epoch->pvt.data.pvt.bias_bits, 8);
    sc_text_string(text, " drift_bits=");
    sc_text_hex(text, epoch->pvt.data.pvt.drift_bits, 8);
    append_u32_field(text, " rx_uptime_s=", epoch->rx_uptime);
    append_u32_field(text, " rx_state=", epoch->rx_state);
    append_u32_field(text, " rx_error=", epoch->rx_error);
    append_u32_field(text, " sync=", epoch->sync_level);
    append_u32_field(text, " meas_flags=", epoch->common_flags);
    append_u32_field(text, " jumps_raw=", epoch->jumps);
    append_u32_field(text, " jumps_valid=", epoch->jumps_valid ? 1U : 0U);
    sc_text_string(
        text, " jump_unwrap=unknown pps_association=unknown ppp_qualified=0 time_calibrated=0");
}

void timing_probe_service(uint32_t now_ms)
{
    for (unsigned i = 0; i < 8U && capture_tail != capture_head; i++)
    {
        __DMB();
        const capture_t capture = captures[capture_tail & 7U];
        __DMB();
        capture_tail++;
        const bool lost = capture.overcapture != 0U || observed_capture_drops != capture_drops;
        observed_capture_drops = capture_drops;
        sc_pps_edge(&pps, capture.tick, capture.ms, capture.sequence, lost);
        if (pps_pending)
        {
            pps_drops++;
        }
        pending_pps = pps;
        pps_pending = true;
    }
    sc_pps_expire(&pps, now_ms);
    for (unsigned i = 0; i < 4U; i++)
    {
        sc_epoch_t epoch;
        if (!sc_epochs_poll(&epochs, now_ms, &epoch))
        {
            break;
        }
        if (epoch_head - epoch_tail == 4U)
        {
            epoch_drops++;
        }
        else
        {
            epoch_records[epoch_head & 3U] = epoch;
            epoch_head++;
        }
    }
    const uint32_t phase = now_ms % 1000U;
    if (now_ms == last_send_ms || phase < 30U || (phase > 300U && phase < 400U) ||
        (phase > 650U && phase < 760U) || phase > 940U)
    {
        return;
    }
    last_send_ms = now_ms;
    char line[SIMPLECLOCK_TMP117_USB_TX_MAX];
    sc_text_t text;
    sc_text_init(&text, line, sizeof(line));
    unsigned kind = 0;
    const bool pps_paced = pps.state == SC_PPS_PRESENT;
    const bool status_due = pps_paced
                                ? ((uint32_t)(now_ms - pps.ms) >= 250U &&
                                   (!status_pps_seen || status_pps_sequence != pps.sequence) &&
                                   (uint32_t)(now_ms - last_status_ms) >= 500U)
                                : ((uint32_t)(now_ms - last_status_ms) >= 1000U && phase >= 500U);
    if (status_due)
    {
        kind = 1;
        sc_text_string(&text, "type=timing_status v=1");
        append_u32_field(&text, " uptime_ms=", now_ms);
        sc_text_string(&text, pps_paced ? " report_source=PPS" : " report_source=LOCAL");
        sc_text_string(&text, " pps_state=");
        sc_text_string(&text, state_name(pps.state));
        append_u32_field(&text, " pps_age_ms=", pps.seen ? now_ms - pps.ms : UINT32_MAX);
        append_u32_field(&text, " pps_edges=", capture_sequence);
        append_u32_field(&text, " overcaptures=", overcaptures);
        append_u32_field(&text, " capture_drops=", capture_drops);
        append_u32_field(&text, " pps_record_drops=", pps_drops);
        append_u32_field(&text, " isr_body_us_max=", max_isr_body_us);
        append_u32_field(&text, " epoch_record_drops=", epoch_drops);
        append_u32_field(&text, " segment=", epochs.segment);
        append_u32_field(&text, " rx_boot=", epochs.receiver_boot);
        append_u32_field(&text, " epoch_late=", epochs.late);
        append_u32_field(&text, " epoch_duplicates=", epochs.duplicates);
        append_u32_field(&text, " epoch_discarded=", epochs.discarded);
        append_u32_field(&text,
                         " pps_enable_ready=", sc_epochs_pps_ready(&epochs, now_ms) ? 1U : 0U);
        append_u32_field(&text, " e6_pages=", e6_pages);
        append_u32_field(&text, " e6_crc_passed=", e6_crc_passed);
        append_u32_field(&text, " e6_age_ms=", e6_crc_passed ? now_ms - e6_last_ms : UINT32_MAX);
        append_u32_field(&text, " ppp_active=", timing_probe_ppp_active(now_ms) ? 1U : 0U);
        sc_text_string(
            &text, " timer_hz=1000000 timer_source=HSI boot_id=host_required association=unknown");
    }
    else if (pps_pending)
    {
        kind = 2;
        sc_text_string(&text, "type=pps_edge v=1");
        append_u32_field(&text, " uptime_ms=", now_ms);
        append_u32_field(&text, " isr_ms=", pending_pps.ms);
        append_u32_field(&text, " seq=", pending_pps.sequence);
        append_u32_field(&text, " tick_us32=", pending_pps.tick);
        append_u32_field(&text, " interval_us=", pending_pps.interval_us);
        append_u32_field(&text, " interval_valid=", pending_pps.interval_valid ? 1U : 0U);
        sc_text_string(&text, " state_at_capture=");
        sc_text_string(&text, state_name(pending_pps.state));
        sc_text_string(&text, " pps_association=unknown time_calibrated=0");
    }
    else if (epoch_tail != epoch_head)
    {
        kind = 3;
        append_epoch_fields(&text, &epoch_records[epoch_tail & 3U], now_ms);
    }
    else
    {
        return;
    }
    sc_text_string(&text, "\r\n");
    if (text.ok &&
        simpleclock_usb_cdc_try_send_queued((const uint8_t *)line, (uint16_t)text.length))
    {
        if (kind == 1U)
        {
            last_status_ms = now_ms;
            status_pps_seen = pps.seen;
            status_pps_sequence = pps.sequence;
        }
        else if (kind == 2U)
        {
            pps_pending = false;
        }
        else
        {
            epoch_tail++;
        }
    }
}
