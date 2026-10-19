#include "oled.h"
#include "board.h"
#include "driver/i2c_master.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_log.h"

#define OLED_I2C_HZ      (400 * 1000)
#define OLED_TIMEOUT_MS  50
#define OLED_ADDR_ALT    0x3D

/* Some modules are mounted upside down relative to the text; set to 1 if
 * the picture appears rotated by 180 degrees. */
#define OLED_ROTATE_180  1

static const char *TAG = "oled";
static esp_lcd_panel_handle_t s_panel;

esp_err_t oled_init(void)
{
    i2c_master_bus_handle_t bus;
    const i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = PIN_OLED_SDA,
        .scl_io_num        = PIN_OLED_SCL,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,   /* modules usually have their own too */
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &bus);
    if (err != ESP_OK) return err;

    uint8_t addr = OLED_I2C_ADDR;
    if (i2c_master_probe(bus, addr, OLED_TIMEOUT_MS) != ESP_OK) {
        addr = OLED_ADDR_ALT;
        if (i2c_master_probe(bus, addr, OLED_TIMEOUT_MS) != ESP_OK) {
            ESP_LOGW(TAG, "no display at 0x%02X or 0x%02X — check SDA=%d SCL=%d, 3V3",
                     OLED_I2C_ADDR, OLED_ADDR_ALT, PIN_OLED_SDA, PIN_OLED_SCL);
            i2c_del_master_bus(bus);
            return ESP_ERR_NOT_FOUND;
        }
    }

    esp_lcd_panel_io_handle_t io;
    const esp_lcd_panel_io_i2c_config_t io_cfg = {
        .dev_addr               = addr,
        .scl_speed_hz           = OLED_I2C_HZ,
        .control_phase_bytes    = 1,     /* SSD1306 datasheet */
        .dc_bit_offset          = 6,
        .lcd_cmd_bits           = 8,
        .lcd_param_bits         = 8,
        .transaction_timeout_ms = OLED_TIMEOUT_MS,
    };
    err = esp_lcd_new_panel_io_i2c(bus, &io_cfg, &io);
    if (err != ESP_OK) return err;

    esp_lcd_panel_ssd1306_config_t ssd_cfg = { .height = GFX_H };
    const esp_lcd_panel_dev_config_t panel_cfg = {
        .bits_per_pixel = 1,
        .reset_gpio_num = -1,
        .vendor_config  = &ssd_cfg,
    };
    err = esp_lcd_new_panel_ssd1306(io, &panel_cfg, &s_panel);
    if (err == ESP_OK) err = esp_lcd_panel_reset(s_panel);
    if (err == ESP_OK) err = esp_lcd_panel_init(s_panel);
    if (err == ESP_OK && OLED_ROTATE_180) err = esp_lcd_panel_mirror(s_panel, true, true);
    if (err == ESP_OK) err = esp_lcd_panel_disp_on_off(s_panel, true);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "display init failed: %s", esp_err_to_name(err));
        return err;
    }
    ESP_LOGI(TAG, "SSD1306 at 0x%02X", addr);
    return ESP_OK;
}

esp_err_t oled_flush(const Gfx *g)
{
    return esp_lcd_panel_draw_bitmap(s_panel, 0, 0, GFX_W, GFX_H, g->buf);
}
