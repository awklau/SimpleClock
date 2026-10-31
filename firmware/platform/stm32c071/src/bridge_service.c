/* SPDX-License-Identifier: MIT */
/* USB SCB1 requests -> one acknowledged UART chunk; raw receiver bytes -> SCB1 RX.
 * Protocol errors stop transmission. The host owns SUF selection and upgrade sequencing. */
#include "bridge_service.h"
#include "main.h"
#include "usbd_cdc_if.h"
#include "simpleclock/bridge.h"
#include <string.h>
#define BRIDGE_QUEUE_SIZE 2048U
#define BRIDGE_QUEUE_MASK (BRIDGE_QUEUE_SIZE - 1U)
#define BRIDGE_USB_PACKET_SIZE 64U
#define BRIDGE_SERVICE_BUDGET 1024U
#define BRIDGE_MAX_SERVICE_GAP_MS 500U
_Static_assert((BRIDGE_QUEUE_SIZE & BRIDGE_QUEUE_MASK) == 0U, "Ring size must be a power of two");

/* Each RX ring has one interrupt producer and one foreground consumer:
 * USART2 publishes uart_rx_head; USB publishes usb_rx_head. Main advances tails.
 * Monotonic unsigned indices permit wrap; their difference never exceeds capacity.
 * DMB publishes bytes before a head update. volatile alone is not that ordering.
 * UART TX is a separate single chunk, immutable until TC and its host ACK. */
static uint8_t uart_rx[BRIDGE_QUEUE_SIZE], usb_rx[BRIDGE_QUEUE_SIZE],
    uart_tx[SC_BRIDGE_PAYLOAD_MAX];
static volatile uint32_t uart_rx_head, uart_rx_tail, usb_rx_head, usb_rx_tail, uart_rx_bytes,
    usb_rx_bytes, uart_rx_dropped, usb_rx_dropped, uart_rx_peak, usb_rx_peak;
static volatile uint32_t uart_tx_position, uart_tx_length, uart_tx_bytes;
static volatile bool usb_configured, usb_rx_armed, session_active;
volatile uint32_t bridge_faults;
static sc_bridge_parser_t usb_frame_parser;
static uint8_t response[SC_BRIDGE_FRAME_MAX];
static uint16_t response_length;
static uint32_t session_id, next_request_sequence, uart_rx_sequence, uart_tx_sequence,
    protocol_errors, last_service_ms, max_service_gap_ms;
static bool uart_tx_pending;

/* Both IRQs and main latch faults; protect the read/modify/write from nesting. */
static void latch_fault(uint32_t bits)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    bridge_faults |= bits;
    __set_PRIMASK(mask);
}

void bridge_usb_state(bool state)
{
    if (!state && session_active)
    {
        latch_fault(SC_BRIDGE_FAULT_DISCONNECT);
    }
    usb_configured = state;
    usb_rx_armed = state;
}

void bridge_usb_received(const uint8_t *data, uint32_t length)
{
    usb_rx_armed = false;
    usb_rx_bytes += length;
    if (length > BRIDGE_USB_PACKET_SIZE || usb_rx_head - usb_rx_tail > BRIDGE_QUEUE_SIZE - length)
    {
        usb_rx_dropped += length;
        latch_fault(SC_BRIDGE_FAULT_USB_RX);
        return;
    }
    for (uint32_t i = 0; i < length; i++)
    {
        usb_rx[(usb_rx_head + i) & BRIDGE_QUEUE_MASK] = data[i];
    }
    __DMB();
    usb_rx_head += length;
    if (usb_rx_head - usb_rx_tail > usb_rx_peak)
    {
        usb_rx_peak = usb_rx_head - usb_rx_tail;
    }
}

void bridge_init(void)
{
    GPIO_InitTypeDef pin = {0};
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_USART2_FORCE_RESET();
    __HAL_RCC_USART2_RELEASE_RESET();
    pin.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    pin.Mode = GPIO_MODE_AF_PP;
    pin.Speed = GPIO_SPEED_FREQ_LOW;
    pin.Pull = GPIO_NOPULL;
    pin.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(GPIOA, &pin);
    USART2->BRR = 417U;
    USART2->CR3 = USART_CR3_EIE;
    USART2->CR1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE | USART_CR1_RXNEIE_RXFNEIE;
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
    last_service_ms = HAL_GetTick();
}

void USART2_IRQHandler(void)
{
    uint32_t status = USART2->ISR;
    if (status & (USART_ISR_ORE | USART_ISR_FE | USART_ISR_NE | USART_ISR_PE))
    {
        USART2->ICR = USART_ICR_ORECF | USART_ICR_FECF | USART_ICR_NECF | USART_ICR_PECF;
        if (session_active)
        {
            latch_fault(SC_BRIDGE_FAULT_UART_RX);
        }
    }
    if (status & USART_ISR_RXNE_RXFNE)
    {
        uint8_t byte = (uint8_t)USART2->RDR;
        if (session_active)
        {
            uart_rx_bytes++;
            if (uart_rx_head - uart_rx_tail < BRIDGE_QUEUE_SIZE)
            {
                uart_rx[uart_rx_head & BRIDGE_QUEUE_MASK] = byte;
                __DMB();
                uart_rx_head++;
                if (uart_rx_head - uart_rx_tail > uart_rx_peak)
                {
                    uart_rx_peak = uart_rx_head - uart_rx_tail;
                }
            }
            else
            {
                uart_rx_dropped++;
                latch_fault(SC_BRIDGE_FAULT_UART_RX);
            }
        }
    }
    if ((USART2->CR1 & USART_CR1_TXEIE_TXFNFIE) && (status & USART_ISR_TXE_TXFNF))
    {
        if (uart_tx_position < uart_tx_length && !bridge_faults)
        {
            USART2->TDR = uart_tx[uart_tx_position++];
            uart_tx_bytes++;
        }
        if (uart_tx_position == uart_tx_length || bridge_faults)
        {
            USART2->CR1 &= ~USART_CR1_TXEIE_TXFNFIE;
        }
    }
}

static void queue_response(uint8_t type, uint32_t sequence, const uint8_t *data, uint16_t length)
{
    response_length =
        (uint16_t)sc_bridge_encode(response, sizeof(response), type,
                                   (uint8_t)(bridge_faults ? 1U : 0U), sequence, data, length);
}

static void queue_status(uint8_t type, uint32_t sequence)
{
    /* Fixed v1 wire order: keep aligned with INFO_KEYS in tools/gnss_bridge.py. */
    uint32_t values[] = {SC_BRIDGE_PROTOCOL_VERSION,
                         HAL_GetTick(),
                         session_id,
                         bridge_faults,
                         uart_tx_bytes,
                         uart_rx_bytes,
                         uart_rx_dropped,
                         usb_rx_bytes,
                         usb_rx_dropped,
                         protocol_errors,
                         uart_rx_peak,
                         usb_rx_peak,
                         max_service_gap_ms,
                         bridge_stack_peak(),
                         bridge_guard_ok() ? 1U : 0U};
    uint8_t data[sizeof(values)];
    for (unsigned i = 0; i < sizeof(values) / sizeof(values[0]); i++)
    {
        sc_bridge_put32(data + 4U * i, values[i]);
    }
    queue_response(type, sequence, data, (uint16_t)sizeof(data));
}

static void handle_request(const sc_bridge_packet_t *packet)
{
    if (packet->flags || (packet->type == SC_BRIDGE_HELLO && packet->length) ||
        (packet->type != SC_BRIDGE_HELLO &&
         (!session_active || packet->sequence != next_request_sequence)))
    {
        latch_fault(SC_BRIDGE_FAULT_PROTOCOL);
        queue_status(SC_BRIDGE_ERROR, packet->sequence);
        return;
    }
    next_request_sequence = packet->sequence + 1U;
    /* HELLO discards old RX bytes but never clears a sticky fault or retries TX. */
    if (packet->type == SC_BRIDGE_HELLO)
    {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        uart_rx_tail = uart_rx_head;
        uart_rx_sequence = 0;
        session_id++;
        session_active = true;
        __set_PRIMASK(mask);
        queue_status(SC_BRIDGE_HELLO_REPLY, packet->sequence);
    }
    else if (packet->type == SC_BRIDGE_STATUS && packet->length == 0U)
    {
        queue_status(SC_BRIDGE_STATUS_REPLY, packet->sequence);
    }
    else if (packet->type == SC_BRIDGE_ECHO)
    {
        queue_response(SC_BRIDGE_ECHO_REPLY, packet->sequence, packet->payload,
                       packet->length); /* Never reaches UART. */
    }
    else if (packet->type == SC_BRIDGE_UART_TX && packet->length && !bridge_faults)
    {
        memcpy(uart_tx, packet->payload, packet->length);
        uart_tx_position = 0;
        uart_tx_length = packet->length;
        uart_tx_sequence = packet->sequence;
        uart_tx_pending = true;
        USART2->ICR = USART_ICR_TCCF;
        __DMB();
        USART2->CR1 |= USART_CR1_TXEIE_TXFNFIE;
    }
    else
    {
        latch_fault(SC_BRIDGE_FAULT_PROTOCOL);
        queue_status(SC_BRIDGE_ERROR, packet->sequence);
    }
}

void bridge_service(void)
{
    uint32_t now = HAL_GetTick(), gap = now - last_service_ms;
    last_service_ms = now;
    if (gap > max_service_gap_ms)
    {
        max_service_gap_ms = gap;
    }
    if (!bridge_guard_ok() || gap > BRIDGE_MAX_SERVICE_GAP_MS)
    {
        latch_fault(SC_BRIDGE_FAULT_HEALTH);
    }
    /* TXE only means TDR has room. TC proves the final stop bit left the UART.
     * A fault ACK carries the error flag; it must never be treated as success. */
    if (uart_tx_pending &&
        (bridge_faults || (uart_tx_position == uart_tx_length && (USART2->ISR & USART_ISR_TC))))
    {
        uart_tx_pending = false;
        uint8_t total[4];
        sc_bridge_put32(total, uart_tx_bytes);
        queue_response(SC_BRIDGE_UART_TX_REPLY, uart_tx_sequence, total, 4);
    }
    /* Stop-and-wait: consume one complete request only when its response slot
     * and UART chunk are free. USB OUT is re-armed only with room for 64 bytes. */
    if (!uart_tx_pending && !response_length)
    {
        for (unsigned i = 0; i < BRIDGE_SERVICE_BUDGET && usb_rx_tail != usb_rx_head; i++)
        {
            __DMB();
            uint8_t byte = usb_rx[usb_rx_tail & BRIDGE_QUEUE_MASK];
            usb_rx_tail++;
            sc_bridge_packet_t packet;
            int result = sc_bridge_feed(&usb_frame_parser, byte, &packet);
            if (result < 0)
            {
                protocol_errors++;
                latch_fault(SC_BRIDGE_FAULT_PROTOCOL);
                queue_status(SC_BRIDGE_ERROR, 0);
                break;
            }
            if (result > 0)
            {
                handle_request(&packet);
                break;
            }
        }
    }
    if (usb_configured && !usb_rx_armed &&
        usb_rx_head - usb_rx_tail <= BRIDGE_QUEUE_SIZE - BRIDGE_USB_PACKET_SIZE)
    {
        uint32_t mask = __get_PRIMASK();
        __disable_irq();
        usb_rx_armed = true;
        bridge_usb_rearm();
        __set_PRIMASK(mask);
    }
    /* Control replies take priority. CDC copies on OK; retain bytes on BUSY. */
    if (response_length)
    {
        if (CDC_Transmit_FS(response, response_length) == USBD_OK)
        {
            response_length = 0;
        }
    }
    else if (session_active && uart_rx_tail != uart_rx_head)
    {
        uint8_t data[128];
        uint32_t n = uart_rx_head - uart_rx_tail;
        if (n > sizeof(data))
        {
            n = sizeof(data);
        }
        __DMB();
        for (uint32_t i = 0; i < n; i++)
        {
            data[i] = uart_rx[(uart_rx_tail + i) & BRIDGE_QUEUE_MASK];
        }
        size_t length = sc_bridge_encode(response, sizeof(response), SC_BRIDGE_UART_RX,
                                         (uint8_t)(bridge_faults ? 1U : 0U), uart_rx_sequence, data,
                                         (uint16_t)n);
        if (CDC_Transmit_FS(response, (uint16_t)length) == USBD_OK)
        {
            uart_rx_tail += n;
            uart_rx_sequence++;
        }
    }
}
