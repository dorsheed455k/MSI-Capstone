#include "ms_switch.h"

/* ===== Your pin mapping =====
   PB4 -> Q1_CTRL
   PB5 -> Q2_CTRL
   PC8 -> Q3_CTRL
*/

static inline void ms_write_pin(GPIO_TypeDef *port, uint16_t pin, GPIO_PinState st)
{
    HAL_GPIO_WritePin(port, pin, st);
}

static inline void ms_write_raw(MS_Switch_t *ms, uint8_t q1, uint8_t q2, uint8_t q3)
{
    ms_write_pin(ms->q1_port, ms->q1_pin, q1 ? ms->on_level : ms->off_level);
    ms_write_pin(ms->q2_port, ms->q2_pin, q2 ? ms->on_level : ms->off_level);
    ms_write_pin(ms->q3_port, ms->q3_pin, q3 ? ms->on_level : ms->off_level);
}

void MS_Init(MS_Switch_t *ms)
{
    if (!ms) return;

    /* Fill defaults */
    ms->q1_port = GPIOB; ms->q1_pin = GPIO_PIN_4;
    ms->q2_port = GPIOB; ms->q2_pin = GPIO_PIN_5;
    ms->q3_port = GPIOC; ms->q3_pin = GPIO_PIN_8;

    ms->on_level  = GPIO_PIN_SET;    /* adjust if active-low */
    ms->off_level = GPIO_PIN_RESET;

    ms->deadtime_ms = 2;
    ms->active_mode = 0;
    ms->mode3_variant = MS_MODE3_VAR_A;

    /* Ensure clocks are on (safe even if already enabled elsewhere) */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure GPIOs (in case CubeMX didn't) */
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    GPIO_InitStruct.Pin = ms->q1_pin | ms->q2_pin;
    HAL_GPIO_Init(ms->q1_port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = ms->q3_pin;
    HAL_GPIO_Init(ms->q3_port, &GPIO_InitStruct);

    /* Default OFF */
    MS_AllOff(ms);
}

void MS_AllOff(MS_Switch_t *ms)
{
    if (!ms) return;
    ms_write_raw(ms, 0, 0, 0);
    ms->active_mode = 0;
}

void MS_SetRaw(MS_Switch_t *ms, uint8_t q1, uint8_t q2, uint8_t q3)
{
    if (!ms) return;
    ms_write_raw(ms, q1, q2, q3);
}

void MS_SetMode(MS_Switch_t *ms, MS_Mode_t mode, MS_Mode3Variant_t var3)
{
    if (!ms) return;

    /* Break-before-make */
    MS_AllOff(ms);

    /* Only call from main loop / slow task (NOT in 20kHz ISR) */
    if (ms->deadtime_ms > 0) HAL_Delay(ms->deadtime_ms);

    switch (mode)
    {
        case MS_MODE_1_12V:
            /* Low Voltage Mode (12V logic) */
            ms_write_raw(ms, 0, 1, 0); 
            break;
        case MS_MODE_2_24V:
            /* Mid Voltage Mode (24V logic) */
            if (var3 == MS_MODE3_VAR_A) ms_write_raw(ms, 1, 0, 0);
            else ms_write_raw(ms, 0, 1, 1);
            ms->mode3_variant = var3;
            break;
        case MS_MODE_3_36V:
            /* High Voltage Mode (36V logic) */
            ms_write_raw(ms, 1, 1, 0);
            break;
        default:
            MS_AllOff(ms);
            mode = 0;
            break;
    }
    ms->active_mode = mode;
}

float MS_ModeToVbus(MS_Mode_t mode)
{
    switch (mode)
    {
        case MS_MODE_1_12V: return 12.0f;
        case MS_MODE_2_24V: return 24.0f;
        case MS_MODE_3_36V: return 36.0f;
        default: return 0.0f;
    }
}
