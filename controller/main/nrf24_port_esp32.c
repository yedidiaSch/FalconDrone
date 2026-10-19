#include "nrf24_port_esp32.h"
#include "board.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_rom_sys.h"

#define NRF_SPI_HOST   SPI3_HOST
#define NRF_SPI_HZ     (4 * 1000 * 1000)

static spi_device_handle_t s_dev;

/* One transaction = CSN low .. high (the SPI driver drives CS). Without DMA
 * a transaction is limited to 64 bytes; the longest nRF24 one is 33. */
static void spi_transfer(void *ctx, const uint8_t *tx, uint8_t *rx, size_t len)
{
    (void)ctx;
    spi_transaction_t t = {
        .length    = len * 8,
        .tx_buffer = tx,
        .rx_buffer = rx,
    };
    ESP_ERROR_CHECK(spi_device_polling_transmit(s_dev, &t));
}

static void ce_write(void *ctx, bool high)
{
    (void)ctx;
    gpio_set_level(PIN_NRF_CE, high ? 1 : 0);
}

static void delay_us(void *ctx, uint32_t us)
{
    (void)ctx;
    esp_rom_delay_us(us);
}

esp_err_t nrf24_port_esp32_init(Nrf24Port *port)
{
    const gpio_config_t ce = {
        .pin_bit_mask = 1ULL << PIN_NRF_CE,
        .mode         = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = gpio_config(&ce);
    if (err != ESP_OK) return err;
    gpio_set_level(PIN_NRF_CE, 0);

    const spi_bus_config_t bus = {
        .mosi_io_num   = PIN_NRF_MOSI,
        .miso_io_num   = PIN_NRF_MISO,
        .sclk_io_num   = PIN_NRF_SCK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    err = spi_bus_initialize(NRF_SPI_HOST, &bus, SPI_DMA_DISABLED);
    if (err != ESP_OK) return err;

    const spi_device_interface_config_t dev = {
        .clock_speed_hz = NRF_SPI_HZ,
        .mode           = 0,
        .spics_io_num   = PIN_NRF_CSN,
        .queue_size     = 1,
    };
    err = spi_bus_add_device(NRF_SPI_HOST, &dev, &s_dev);
    if (err != ESP_OK) return err;

    *port = (Nrf24Port){
        .spi_transfer = spi_transfer,
        .ce_write     = ce_write,
        .delay_us     = delay_us,
        .ctx          = NULL,
    };
    return ESP_OK;
}
