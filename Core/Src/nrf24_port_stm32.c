#include "nrf24_port_stm32.h"
#include "main.h"
#include "spi.h"

#define SPI_TIMEOUT_MS 5u   /* 33 bytes at 5.6 MHz take ~50 us */

static void spi_transfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)ctx;
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
    /* On a timeout rx keeps garbage; the driver's presence check and
     * payload-width check catch a chip that stops answering. */
    (void)HAL_SPI_TransmitReceive(&hspi2, (uint8_t *)tx, rx, (uint16_t)len, SPI_TIMEOUT_MS);
    HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

static void ce_write(void *ctx, bool high)
{
    (void)ctx;
    HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Busy-wait on the DWT cycle counter (180 cycles per us). Only short
 * datasheet delays (15 us CE pulse) run in flight; the 5 ms power-up wait
 * happens once in nrf24_init(). */
static void delay_us(void *ctx, uint32_t us)
{
    (void)ctx;
    const uint32_t start = DWT->CYCCNT;
    const uint32_t ticks = us * (SystemCoreClock / 1000000u);
    while ((DWT->CYCCNT - start) < ticks) {
    }
}

static const Nrf24Port s_port = {
    .spi_transfer = spi_transfer,
    .ce_write     = ce_write,
    .delay_us     = delay_us,
    .ctx          = NULL,
};

const Nrf24Port *Nrf24Port_Stm32(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
    return &s_port;
}
