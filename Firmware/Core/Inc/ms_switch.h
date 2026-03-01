#ifndef MS_SWITCH_H
#define MS_SWITCH_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Modes from your table */
typedef enum
{
    MS_MODE_1_9V  = 1,   // Q1=0 Q3=0 Q2=1
    MS_MODE_2_15V = 2,   // Q1=0 Q3=1 Q2=0
    MS_MODE_3_24V = 3,   // two possible states (variant selects)
    MS_MODE_4_33V = 4    // Q1=1 Q3=0 Q2=1
} MS_Mode_t;

/* Variant for Mode 3 (24V) */
typedef enum
{
    MS_MODE3_VAR_A = 0,  // Q1=1 Q2=0 Q3=0
    MS_MODE3_VAR_B = 1   // Q1=0 Q2=1 Q3=1
} MS_Mode3Variant_t;

typedef struct
{
    GPIO_TypeDef *q1_port; uint16_t q1_pin; // PB4
    GPIO_TypeDef *q2_port; uint16_t q2_pin; // PB5
    GPIO_TypeDef *q3_port; uint16_t q3_pin; // PC8

    GPIO_PinState on_level;   // usually GPIO_PIN_SET
    GPIO_PinState off_level;  // usually GPIO_PIN_RESET

    uint32_t deadtime_ms;     // break-before-make delay
    MS_Mode_t active_mode;
    MS_Mode3Variant_t mode3_variant;
} MS_Switch_t;

/* Initialize pins + default off */
void MS_Init(MS_Switch_t *ms);

/* Turn everything off */
void MS_AllOff(MS_Switch_t *ms);

/* Set raw states: q1/q2/q3 are 0 or 1 */
void MS_SetRaw(MS_Switch_t *ms, uint8_t q1, uint8_t q2, uint8_t q3);

/* Apply an operating mode (with safe break-before-make) */
void MS_SetMode(MS_Switch_t *ms, MS_Mode_t mode, MS_Mode3Variant_t var3);

/* Optional helper: returns target bus voltage for a mode */
float MS_ModeToVbus(MS_Mode_t mode);

#ifdef __cplusplus
}
#endif

#endif /* MS_SWITCH_H */
