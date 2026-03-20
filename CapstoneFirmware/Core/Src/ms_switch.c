#include "ms_switch.h"

/*
 * Pin mapping: PB4=Q1, PB5=Q2, PC8=Q3.
 *
 * Combined behaviour (ms_switch + msifunc):
 * - MS_SetMode: direct hardware change (break-before-make).
 * - MS_RequestMode: request a mode; upshift = apply now, downshift = wait for safe RPM.
 * - MS_SafetyTask + MS_ApplyPending: when RPM is below threshold, apply pending mode.
 */

/* Max RPM allowed for each mode before we allow a downshift *into* that mode.
   Below these, it's safe to switch to the lower bus voltage. Tune to your motor/PSIM. */
#define RPM_THRESHOLD_MODE1_12V  1500.0f   /* downshift to 12V only when rpm < this */
#define RPM_THRESHOLD_MODE2_24V  3000.0f   /* downshift to 24V only when rpm < this */
#define RPM_THRESHOLD_MODE3_36V  5200.0f   /* downshift to 36V only when rpm < this */

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
    ms->mode2_variant = MS_MODE2_VAR_A;
    ms->target_mode = MS_MODE_1_12V;
    ms->shift_pending = 0;

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

void MS_SetMode(MS_Switch_t *ms, MS_Mode_t mode, MS_Mode2Variant_t var2)
{
    if (!ms) return;

    /* Break-before-make: all Q off before changing taps */
    MS_AllOff(ms);

    /* Only call from main loop / slow task (NOT in 25kHz ISR) */
    if (ms->deadtime_ms > 0) HAL_Delay(ms->deadtime_ms);

    switch (mode)
    {
        case MS_MODE_1_12V:
            /* 12V 5A 1800rpm — Q1=0 Q3=0 Q2=1 */
            ms_write_raw(ms, 0, 1, 0);
            break;

        case MS_MODE_2_24V:
            /* 24V 2.5A 3600rpm — two possible patterns */
            if (var2 == MS_MODE2_VAR_A)
                ms_write_raw(ms, 1, 0, 0);   /* Q1=1 Q2=0 Q3=0 */
            else
                ms_write_raw(ms, 0, 1, 1);   /* Q1=0 Q2=1 Q3=1 */
            ms->mode2_variant = var2;
            break;

        case MS_MODE_3_36V:
            /* 36V 1.66A 5500rpm — Q1=1 Q3=0 Q2=1 */
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

/* ----- Request/safety API (ideas from msifunc) ----- */

void MS_RequestMode(MS_Switch_t *ms, MS_Mode_t mode)
{
    if (!ms) return;

    /* Upshift: apply immediately (same as before). */
    if (mode >= ms->active_mode)
    {
        MS_SetMode(ms, mode, ms->mode2_variant);
        ms->shift_pending = 0;
        return;
    }

    /* Downshift: store target and set pending. Caller must set speed/RPM to 0 and
       poll MS_SafetyTask; when it returns 1, call MS_ApplyPending then update FOC. */
    ms->target_mode = mode;
    ms->shift_pending = 1;
}

uint8_t MS_SafetyTask(MS_Switch_t *ms, float current_rpm)
{
    if (!ms || !ms->shift_pending) return 0;

    /* Check if RPM is below the safe threshold for the target mode. */
    float thresh = 0.0f;
    switch (ms->target_mode)
    {
        case MS_MODE_1_12V: thresh = RPM_THRESHOLD_MODE1_12V; break;
        case MS_MODE_2_24V: thresh = RPM_THRESHOLD_MODE2_24V; break;
        case MS_MODE_3_36V: thresh = RPM_THRESHOLD_MODE3_36V; break;
        default: return 0;
    }

    if (current_rpm >= thresh)
        return 0;   /* Not safe yet; keep braking. */

    /* Safe to apply: caller will call MS_ApplyPending then FOC_SetVdc / FOC_ApplyMSModePI. */
    return 1;
}

void MS_ApplyPending(MS_Switch_t *ms)
{
    if (!ms || !ms->shift_pending) return;

    MS_SetMode(ms, ms->target_mode, ms->mode2_variant);
    ms->shift_pending = 0;
}

uint8_t MS_IsShiftPending(const MS_Switch_t *ms)
{
    return ms ? ms->shift_pending : 0;
}
