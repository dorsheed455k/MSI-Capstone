/*
 * foc.c
 *
 *  Created on: Feb 15, 2026
 *      Author: jack2
 */

#include "foc.h"
#include "main.h"
#include "tim.h"
#include <math.h>

/* ADC buffer coming from DMA */
extern uint16_t adc_buffer[3];

float ia, ib, ic;
float ialpha, ibeta;
float id, iq;
float vd, vq;
float valpha, vbeta;

float theta_e = 0.0f;

float iq_ref = 0.0f;      // comes from speed loop
float speed_ref = 100.0f; // desired speed (rad/s)
float measured_speed = 0.0f;

float Ts_current = 0.00004f; // 25kHz
float Ts_speed   = 0.001f;   // 1kHz

PI_Controller pi_d     = {1.0f, 100.0f, 0.0f, 6.0f};
PI_Controller pi_q     = {1.0f, 100.0f, 0.0f, 6.0f};
PI_Controller pi_speed = {0.1f, 5.0f, 0.0f, 10.0f};

static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float PI_Update(PI_Controller *pi, float err, float dt)
{
    pi->integral += err * pi->Ki * dt;
    pi->integral = clampf(pi->integral, -pi->limit, pi->limit);

    float out = pi->Kp * err + pi->integral;
    out = clampf(out, -pi->limit, pi->limit);
    return out;
}

void FOC_SetVdc(FOC_t *f, float vdc)
{
    f->vdc = vdc;
}

void FOC_Init(FOC_t *f)
{
	   f->id_ref = 0.0f;
	    f->iq_ref = 0.0f;

	    f->d_u = f->d_v = f->d_w = 0.5f;

	    f->vdc = 9.0f; // default, change with MS mode

	    /* Example PI defaults (YOU will tune these) */
	    f->pi_d.Kp = 0.2f;
	    f->pi_d.Ki = 50.0f;
	    f->pi_d.integral = 0.0f;
	    f->pi_d.limit = 5.0f;   // volts (or alpha/beta units)

	    f->pi_q.Kp = 0.2f;
	    f->pi_q.Ki = 50.0f;
	    f->pi_q.integral = 0.0f;
	    f->pi_q.limit = 5.0f;
}

void FOC_Run(FOC_t *f, float iu, float iv, float iw, float theta, float dt) {
	 /* 1) Clarke: abc -> alpha/beta */
	    float i_alpha = 0.0f, i_beta = 0.0f;
	    Clarke(iu, iv, iw, &i_alpha, &i_beta);

	    /* 2) Park: alpha/beta -> d/q */
	    float id = 0.0f, iq = 0.0f;
	    Park(i_alpha, i_beta, theta, &id, &iq);

	    /* 3) Current control -> vd/vq */
	    float err_d = f->id_ref - id;
	    float err_q = f->iq_ref - iq;

	    float vd = PI_Update(&f->pi_d, err_d, dt);
	    float vq = PI_Update(&f->pi_q, err_q, dt);

	    /* 4) Inverse Park: vd/vq -> v_alpha/v_beta */
	    float v_alpha = 0.0f, v_beta = 0.0f;
	    InvPark(vd, vq, theta, &v_alpha, &v_beta);

	    /* 5) SVM: v_alpha/v_beta + vdc -> duty */
	    /* Make sure f->vdc is set to 9/15/24/33 based on MS mode */
	    SVM(v_alpha, v_beta, f->vdc, &f->d_u, &f->d_v, &f->d_w);

	    /* Optional safety clamp if your SVM doesn’t clamp */
	    f->d_u = clampf(f->d_u, 0.02f, 0.98f);
	    f->d_v = clampf(f->d_v, 0.02f, 0.98f);
	    f->d_w = clampf(f->d_w, 0.02f, 0.98f);
}


float PI_Run(PI_Controller *pi, float error, float Ts)
{
    pi->integral += pi->Ki * error * Ts;

    if(pi->integral > pi->limit)
        pi->integral = pi->limit;
    if(pi->integral < -pi->limit)
        pi->integral = -pi->limit;

    float output = pi->Kp * error + pi->integral;

    if(output > pi->limit)
        output = pi->limit;
    if(output < -pi->limit)
        output = -pi->limit;

    return output;
}

void FOC_CurrentLoop(void)
{
	float dutyA, dutyB, dutyC;

    /* ---- 1. Read currents from ADC ---- */
    ia = (float)adc_buffer[0];
    ib = (float)adc_buffer[1];
    ic = (float)adc_buffer[2];

    /* ---- 2. Clarke ---- */
    Clarke(ia, ib, ic, &ialpha, &ibeta);

    /* ---- 3. Simulated angle ---- */
    theta_e += measured_speed * Ts_current;
    if(theta_e > 2.0f * 3.1415926f)
        theta_e = 0.0f;

    /* ---- 4. Park ---- */
    Park(ialpha, ibeta, theta_e, &id, &iq);

    /* ---- 5. Test control voltages */
    vd = 0.0f;
    vq = 0.5f;

    /* ---- 6. Inverse Park ---- */
    InvPark(vd, vq, theta_e, &valpha, &vbeta);

    /* ---- 7. SVM ---- */
    SVM(valpha, vbeta, 12.0f, &dutyA, &dutyB, &dutyC);
}

void FOC_SpeedLoop(void)
{
    /* 1. Measure mechanical speed (encoder later) */
    // For now simulate:
    measured_speed = 100.0f;

    /* 2. Speed PI controller */
    float speed_error = speed_ref - measured_speed;

    iq_ref = PI_Run(&pi_speed, speed_error, Ts_speed);
}

void Clarke(float ia, float ib, float ic, float* alpha, float* beta)
{
    *alpha = ia;
    *beta  = (ia + 2.0f * ib) * 0.577350269f; // 1/sqrt(3)
}

void Park(float alpha, float beta, float theta,
          float* d, float* q)
{
    *d =  alpha * cosf(theta) + beta * sinf(theta);
    *q = -alpha * sinf(theta) + beta * cosf(theta);
}

void InvPark(float d, float q, float theta,
             float* alpha, float* beta)
{
    *alpha = d * cosf(theta) - q * sinf(theta);
    *beta  = d * sinf(theta) + q * cosf(theta);
}

void SVM(float alpha, float beta,
         float vdc,
         float* dutyA, float* dutyB, float* dutyC)
{
    /* Turn alpha and beta into 3 Phase voltages */
    float Va = alpha;
    float Vb = -0.5f * alpha + 0.8660254f * beta;
    float Vc = -0.5f * alpha - 0.8660254f * beta;

    /* Find max/min to shift the waveforms to use the full DC bus */
    float Vmax = fmaxf(Va, fmaxf(Vb, Vc));
    float Vmin = fminf(Va, fminf(Vb, Vc));

    /* Centering offset (Zero-Sequence Injection) */
    float Voffset = 0.5f * (Vmax + Vmin);

    Va -= Voffset;
    Vb -= Voffset;
    Vc -= Voffset;

    /* Get PWM Timer Period */
    uint16_t period = __HAL_TIM_GET_AUTORELOAD(&htim1);

    /* Normalize duty to 0–1 */
    *dutyA = (Va / vdc + 0.5f);
    *dutyB = (Vb / vdc + 0.5f);
    *dutyC = (Vc / vdc + 0.5f);

    /* Convert to timer counts */
    uint16_t cmpA = (uint16_t)(*dutyA * period);
    uint16_t cmpB = (uint16_t)(*dutyB * period);
    uint16_t cmpC = (uint16_t)(*dutyC * period);

    /* Adjust PWM Outputs to Gate Drivers */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, cmpA);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, cmpB);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, cmpC);
}
