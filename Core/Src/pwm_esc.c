#include "pwm_esc.h"
#include "tim.h"

/*
 * Static tables map motor index → timer handle + channel.
 * Indexed as: M1=0, M2=1, M3=2, M4=3  (see pwm_esc.h for layout).
 */
static TIM_HandleTypeDef * const s_tim[ESC_MOTOR_COUNT] = {
    &htim2, /* M1 TIM2_CH1 PA0 */
    &htim2, /* M2 TIM2_CH2 PA1 */
    &htim3, /* M3 TIM3_CH1 PA6 */
    &htim3, /* M4 TIM3_CH2 PA7 */
};

static const uint32_t s_ch[ESC_MOTOR_COUNT] = {
    TIM_CHANNEL_1, /* M1 */
    TIM_CHANNEL_2, /* M2 */
    TIM_CHANNEL_1, /* M3 */
    TIM_CHANNEL_2, /* M4 */
};

/* ------------------------------------------------------------------ */

void ESC_Init(void)
{
    for (uint8_t i = 0; i < ESC_MOTOR_COUNT; i++) {
        /* Pre-load minimum before starting so the first pulse is safe. */
        __HAL_TIM_SET_COMPARE(s_tim[i], s_ch[i], ESC_MIN_US);
        HAL_TIM_PWM_Start(s_tim[i], s_ch[i]);
    }
}

void ESC_Arm(void)
{
    ESC_SetAllMotors(ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US);
}

void ESC_Disarm(void)
{
    ESC_SetAllMotors(ESC_OFF_US, ESC_OFF_US, ESC_OFF_US, ESC_OFF_US);
}

void ESC_SetMotor(uint8_t motor, uint16_t throttle_us)
{
    if (motor >= ESC_MOTOR_COUNT) return;

    if (throttle_us < ESC_MIN_US) throttle_us = ESC_MIN_US;
    if (throttle_us > ESC_MAX_US) throttle_us = ESC_MAX_US;

    __HAL_TIM_SET_COMPARE(s_tim[motor], s_ch[motor], throttle_us);
}

void ESC_SetAllMotors(uint16_t m1_us, uint16_t m2_us,
                      uint16_t m3_us, uint16_t m4_us)
{
    __HAL_TIM_SET_COMPARE(s_tim[0], s_ch[0], m1_us);
    __HAL_TIM_SET_COMPARE(s_tim[1], s_ch[1], m2_us);
    __HAL_TIM_SET_COMPARE(s_tim[2], s_ch[2], m3_us);
    __HAL_TIM_SET_COMPARE(s_tim[3], s_ch[3], m4_us);
}
