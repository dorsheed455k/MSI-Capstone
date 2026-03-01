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

/* MOVE THIS UP ABOVE FOC_t */
typedef struct {
    float Kp;
    float Ki;
    float integral;
    float limit;
} PI_Controller;

/* Then define FOC_t */
typedef struct {
  float id_ref, iq_ref;
  float d_u, d_v, d_w;

  float vdc;            // if you added this
  PI_Controller pi_d;   // now this compiles
  PI_Controller pi_q;
} FOC_t;

void FOC_Init(FOC_t *f);
void FOC_Run(FOC_t *f, float iu, float iv, float iw, float theta, float dt);
void FOC_SetVdc(FOC_t *f, float vdc);

/* Clarke / Park / InvPark / SVM prototypes unchanged */
void Clarke(float ia, float ib, float ic, float* alpha, float* beta);
void Park(float alpha, float beta, float theta, float* d, float* q);
void InvPark(float d, float q, float theta, float* alpha, float* beta);
void SVM(float alpha, float beta, float vdc, float* dutyA, float* dutyB, float* dutyC);

#endif
