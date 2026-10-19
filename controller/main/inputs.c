#include "inputs.h"
#include "board.h"
#include "rc_protocol.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdlib.h>

/* ---- Calibration — adjust after watching the raw values in the log ------ */
#define SLIDER_RAW_MIN     300    /* raw at the "zero" end (and below → 0)    */
#define SLIDER_RAW_MAX    3950    /* raw at the "full" end (and above → max)  */
#define SLIDER_INVERT        0    /* 1 if pushing the slider up lowers raw    */

#define STICK_DEADBAND     120    /* raw counts around center that read as 0  */
#define STICK_EDGE_MARGIN   60    /* raw counts short of the rail = full      */
#define STICK_INVERT_X       0    /* set so stick right → roll > 0            */
#define STICK_INVERT_Y       0    /* set so stick back (toward you) → pitch > 0 */

#define ARM_SW_ON_LEVEL      1    /* GPIO level of the ARM switch when ON     */
#define MODE_ANGLE_LEVEL     1    /* GPIO level of the MODE switch on ANGLE   */

#define ADC_RAW_MAX       4095
#define OVERSAMPLE           8    /* per read; the highest and lowest dropped */
#define CENTER_SAMPLES      32

static const char *TAG = "inputs";

static adc_oneshot_unit_handle_t s_adc;
static adc_channel_t s_ch_throttle, s_ch_x, s_ch_y;
static int s_center_x, s_center_y;
/* Accept a new level only when two reads in a row agree (20 ms apart). */
typedef struct {
    bool last_raw;
    bool state;
} Debounce;

static Debounce s_btn, s_arm, s_mode;

static bool debounce(Debounce *d, bool raw)
{
    if (raw == d->last_raw) {
        d->state = raw;
    }
    d->last_raw = raw;
    return d->state;
}

static esp_err_t add_channel(gpio_num_t pin, adc_channel_t *ch)
{
    adc_unit_t unit;
    esp_err_t err = adc_oneshot_io_to_channel(pin, &unit, ch);
    if (err != ESP_OK) return err;
    if (unit != ADC_UNIT_1) return ESP_ERR_INVALID_ARG;   /* see board.h */

    const adc_oneshot_chan_cfg_t cfg = {
        .atten    = ADC_ATTEN_DB_12,       /* full 0 .. 3.3 V range */
        .bitwidth = ADC_BITWIDTH_12,
    };
    return adc_oneshot_config_channel(s_adc, *ch, &cfg);
}

static int read_avg(adc_channel_t ch, int n)
{
    int sum = 0;
    for (int i = 0; i < n; i++) {
        int raw = 0;
        adc_oneshot_read(s_adc, ch, &raw);
        sum += raw;
    }
    return sum / n;
}

/* OVERSAMPLE reads, highest and lowest dropped: the ESP32 ADC throws
 * occasional spikes of a few hundred counts that a plain average lets through. */
static int read_trimmed(adc_channel_t ch)
{
    int sum = 0, lo = ADC_RAW_MAX, hi = 0;
    for (int i = 0; i < OVERSAMPLE; i++) {
        int raw = 0;
        adc_oneshot_read(s_adc, ch, &raw);
        sum += raw;
        if (raw < lo) lo = raw;
        if (raw > hi) hi = raw;
    }
    return (sum - lo - hi) / (OVERSAMPLE - 2);
}

static int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/* raw → -RC_AXIS_MAX..+RC_AXIS_MAX, with a center deadband. The two halves
 * are scaled separately because the center is rarely at 2048. */
static int16_t stick_axis(int raw, int center, bool invert)
{
    int out = 0;
    if (raw > center + STICK_DEADBAND) {
        const int span = (ADC_RAW_MAX - STICK_EDGE_MARGIN) - (center + STICK_DEADBAND);
        out = (raw - center - STICK_DEADBAND) * RC_AXIS_MAX / (span > 0 ? span : 1);
    } else if (raw < center - STICK_DEADBAND) {
        const int span = (center - STICK_DEADBAND) - STICK_EDGE_MARGIN;
        out = -(center - STICK_DEADBAND - raw) * RC_AXIS_MAX / (span > 0 ? span : 1);
    }
    out = clampi(out, -RC_AXIS_MAX, RC_AXIS_MAX);
    return (int16_t)(invert ? -out : out);
}

static uint16_t slider(int raw)
{
    int out = (raw - SLIDER_RAW_MIN) * RC_THROTTLE_MAX / (SLIDER_RAW_MAX - SLIDER_RAW_MIN);
    out = clampi(out, 0, RC_THROTTLE_MAX);
    return (uint16_t)(SLIDER_INVERT ? RC_THROTTLE_MAX - out : out);
}

esp_err_t inputs_init(void)
{
    const adc_oneshot_unit_init_cfg_t unit = { .unit_id = ADC_UNIT_1 };
    esp_err_t err = adc_oneshot_new_unit(&unit, &s_adc);
    if (err == ESP_OK) err = add_channel(PIN_THROTTLE, &s_ch_throttle);
    if (err == ESP_OK) err = add_channel(PIN_STICK_X,  &s_ch_x);
    if (err == ESP_OK) err = add_channel(PIN_STICK_Y,  &s_ch_y);
    if (err != ESP_OK) return err;

    const gpio_config_t sw = {
        .pin_bit_mask = (1ULL << PIN_STICK_SW) | (1ULL << PIN_ARM_SW) | (1ULL << PIN_MODE_SW),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
    };
    err = gpio_config(&sw);
    if (err != ESP_OK) return err;

    /* Stick center: average while nobody touches it. */
    vTaskDelay(pdMS_TO_TICKS(50));
    s_center_x = read_avg(s_ch_x, CENTER_SAMPLES);
    s_center_y = read_avg(s_ch_y, CENTER_SAMPLES);
    ESP_LOGI(TAG, "stick center x=%d y=%d", s_center_x, s_center_y);

    /* A center far from mid-scale usually means the stick was held or the
     * module is wired to 5 V / not at all. */
    if (abs(s_center_x - ADC_RAW_MAX / 2) > 600 || abs(s_center_y - ADC_RAW_MAX / 2) > 600) {
        ESP_LOGW(TAG, "stick center looks wrong — check wiring, keep the stick still at boot");
    }
    return ESP_OK;
}

void inputs_read(Inputs *in)
{
    in->raw_throttle = read_trimmed(s_ch_throttle);
    in->raw_x        = read_trimmed(s_ch_x);
    in->raw_y        = read_trimmed(s_ch_y);

    in->throttle = slider(in->raw_throttle);
    in->roll     = stick_axis(in->raw_x, s_center_x, STICK_INVERT_X);
    in->pitch    = stick_axis(in->raw_y, s_center_y, STICK_INVERT_Y);

    in->raw_arm  = gpio_get_level(PIN_ARM_SW);
    in->raw_mode = gpio_get_level(PIN_MODE_SW);

    in->button = debounce(&s_btn,  gpio_get_level(PIN_STICK_SW) == 0);   /* active low */
    in->arm    = debounce(&s_arm,  in->raw_arm  == ARM_SW_ON_LEVEL);
    in->angle  = debounce(&s_mode, in->raw_mode == MODE_ANGLE_LEVEL);
}
