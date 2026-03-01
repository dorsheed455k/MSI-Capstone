// current_sense.c
#include "amc1302.h"

// User set parameters

static float g_vref = 3.3f;
static uint16_t g_adc_bits = 12;
static float g_gain = 41.0f; // AMC1302 gain
static float g_rshunt = 0.002f;

// ADC + DMA buffer
static ADC_HandleTypeDef *g_hadc = 0;
static int16_t g_adc_buf[3];

static inline float counts_to_vdiff(int16_t n) {

	const float full = (float) (1u << (g_adc_bits - 1));
	return ((float)n/full) * g_vref;
}

void CurrentSense_SetParams(float vref, uint16_t adc_bits, float amc_gain, float rshunt)
{
  g_vref = vref;
  g_adc_bits = adc_bits;
  g_gain = amc_gain;
  g_rshunt = rshunt;
}

void CurrentSense_Init(CurrentSense_t *cs, ADC_HandleTypeDef *hadc)
{
  g_hadc = hadc;
  cs->iu = cs->iv = cs->iw = 0.0f;
  cs->offset_u = cs->offset_v = cs->offset_w = 0.0f;
}

void CurrentSense_Start(CurrentSense_t *cs)
{
  (void)cs;
  /* Differential calibration */
  HAL_ADCEx_Calibration_Start(g_hadc, ADC_DIFFERENTIAL_ENDED);

  /* Start DMA for 3 ranks */
  HAL_ADC_Start_DMA(g_hadc, (uint32_t*)g_adc_buf, 3);
}

void CurrentSense_OnDmaComplete(CurrentSense_t *cs)
{
  float vdu = counts_to_vdiff(g_adc_buf[0]) - cs->offset_u;
  float vdv = counts_to_vdiff(g_adc_buf[1]) - cs->offset_v;
  float vdw = counts_to_vdiff(g_adc_buf[2]) - cs->offset_w;

  /* I = Vdiff / (Gain * Rshunt) */
  cs->iu = vdu / (g_gain * g_rshunt);
  cs->iv = vdv / (g_gain * g_rshunt);
  cs->iw = vdw / (g_gain * g_rshunt);
}

void CurrentSense_CalibrateOffsets(CurrentSense_t *cs, uint32_t samples)
{
  /* Call this at startup with motor OFF / zero current */
  float sum_u = 0, sum_v = 0, sum_w = 0;

  for (uint32_t i = 0; i < samples; i++)
  {
    /* wait for one DMA update (simple blocking approach) */
    HAL_Delay(1);

    sum_u += counts_to_vdiff(g_adc_buf[0]);
    sum_v += counts_to_vdiff(g_adc_buf[1]);
    sum_w += counts_to_vdiff(g_adc_buf[2]);
  }

  cs->offset_u = sum_u / (float)samples;
  cs->offset_v = sum_v / (float)samples;
  cs->offset_w = sum_w / (float)samples;
}
