/*
 * foc.h
 *
 *  Created on: Feb 15, 2026
 *      Author: jack2
 */

/*
 * foc.h
 *
 *  Created on: Feb 15, 2026
 *      Author: jack2
 */

#ifndef FOC_H
#define FOC_H

#include "main.h"
#include <math.h>
#include <stdint.h>

/* PI controller struct */
typedef struct {
    float Kp;
    float Ki;
    float integral;
    float limit;     // output clamp (e.g., volts)
} PI_Controller;

/* FOC state struct */
typedef struct {
  float id_ref, iq_ref;     // current references
  float d_u, d_v, d_w;      // PWM duties (0..1)
  float vdc;                // DC bus voltage for SVM scaling

  PI_Controller pi_d;       // d-axis current PI
  PI_Controller pi_q;       // q-axis current PI
} FOC_t;

void FOC_Init(FOC_t *f);
void FOC_SetVdc(FOC_t *f, float vdc);
void FOC_ApplyMSModePI(FOC_t *f, uint8_t mode);

/* Main control step: computes new duties based on currents + angle */
void FOC_Run(FOC_t *f, float iu, float iv, float iw, float theta, float dt);

/* Clarke Transform */
void Clarke(float ia, float ib, float ic, float* alpha, float* beta);

/* Park Transform */
void Park(float alpha, float beta, float theta, float* d, float* q);

/* Inverse Park Transform */
void InvPark(float d, float q, float theta, float* alpha, float* beta);

/* Space Vector Modulation (pure function: computes duty only) */
void SVM(float alpha, float beta, float vdc,
         float* dutyA, float* dutyB, float* dutyC);

#endif
