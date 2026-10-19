#include "rc_radio.h"
#include "rc_frame.h"
#include "usart.h"
#include <string.h>

/*
 * ESP-NOW bridge on USART1 (PA9 → bridge RX, PA10 ← bridge TX, common GND).
 *
 * RX: DMA writes into a circular buffer forever; each poll reads up to the
 * DMA write position — no interrupts, no callbacks. At 115200 baud a 10 ms
 * tick brings at most ~115 bytes, well inside the buffer.
 *
 * TX: one RcAck frame per call, interrupt driven, skipped while the
 * previous one is still going out (at 50 Hz it never is).
 */

#define RX_BUF_SIZE 256u

static uint8_t       s_rx[RX_BUF_SIZE];
static uint16_t      s_rd;
static RcFrameParser s_parser;
static uint8_t       s_tx[RC_FRAME_MAX];

static bool start_rx(void)
{
    s_rd = 0;
    if (HAL_UART_Receive_DMA(&huart1, s_rx, RX_BUF_SIZE) != HAL_OK) {
        return false;
    }
    /* Line noise (bridge unplugged, rebooting) must not abort the DMA: HAL
     * stops reception on any error while EIE/PEIE are set. The frame CRC
     * catches the damage instead. */
    CLEAR_BIT(huart1.Instance->CR3, USART_CR3_EIE);
    CLEAR_BIT(huart1.Instance->CR1, USART_CR1_PEIE);
    /* Polled — the half/full-buffer interrupts would only call empty hooks. */
    __HAL_DMA_DISABLE_IT(huart1.hdmarx, DMA_IT_HT | DMA_IT_TC);
    return true;
}

static bool rx_pop(uint8_t *b)
{
    uint16_t wr = (uint16_t)(RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx));
    if (wr >= RX_BUF_SIZE) {
        wr = 0;
    }
    if (s_rd == wr) {
        return false;
    }
    *b = s_rx[s_rd];
    s_rd = (uint16_t)((s_rd + 1u) % RX_BUF_SIZE);
    return true;
}

bool RcRadio_Init(void)
{
    if (huart1.Init.BaudRate != RC_BRIDGE_BAUD) {
        huart1.Init.BaudRate = RC_BRIDGE_BAUD;
        if (HAL_UART_Init(&huart1) != HAL_OK) {
            return false;
        }
    }
    rc_frame_parser_init(&s_parser);
    return start_rx();
}

bool RcRadio_Receive(RcPacket *pkt)
{
    if (huart1.RxState != HAL_UART_STATE_BUSY_RX) {
        (void)start_rx();                 /* recover if HAL ever stopped it */
        return false;
    }

    uint8_t b;
    while (rx_pop(&b)) {
        if (rc_frame_parse_byte(&s_parser, b)
            && s_parser.type == RC_FRAME_PACKET
            && s_parser.len == sizeof(RcPacket)) {
            memcpy(pkt, s_parser.payload, sizeof(*pkt));
            return true;
        }
    }
    return false;
}

void RcRadio_Reply(const RcAck *ack)
{
    if (huart1.gState != HAL_UART_STATE_READY) {
        return;
    }
    const size_t n = rc_frame_encode(RC_FRAME_ACK, ack, sizeof(*ack), s_tx, sizeof(s_tx));
    if (n > 0) {
        (void)HAL_UART_Transmit_IT(&huart1, s_tx, (uint16_t)n);
    }
}

const char *RcRadio_Name(void)
{
    return "bridge";
}

const char *RcRadio_MissingHint(void)
{
    return "USART1 for the bridge failed to start";
}
