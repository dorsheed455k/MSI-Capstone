#include "stm32g4xx_hal.h"
#include <stdint.h>

// current_sense.h
#ifndef AMC1302_H
#define AMC1302_H

typedef struct {
  float iu, iv, iw;
  float offset_u, offset_v, offset_w;
} CurrentSense_t;

void CurrentSense_Init(CurrentSense_t *cs, ADC_HandleTypeDef *hadc);
void CurrentSense_Start(CurrentSense_t *cs);
void CurrentSense_OnDmaComplete(CurrentSense_t *cs); // call from ADC DMA callback

void CurrentSense_CalibrateOffsets(CurrentSense_t *cs, uint32_t samples);

/* Set these to your hardware */
void CurrentSense_SetParams(float vref, uint16_t adc_bits, float amc_gain,
                            float rshunt);

#endif
