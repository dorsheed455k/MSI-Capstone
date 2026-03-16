/*
 * foc.c
 *
 *  Created on: Feb 15, 2026
 *      Author: jack2
 */

/*
 * foc.c
 *
 * Refactored:
 *  - Uses FOC_t object (no globals)
 *  - FOC_Run() produces duties only (no TIM writes here)
 *  - vdc comes from MS switching via FOC_SetVdc()
 *  - PI gains can be loaded from PSIM results per MS mode
 */

#include "foc.h"
#include <math.h>

/* ----------------- small helpers ----------------- */
static float clampf(float x, float lo, float hi)
{
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

static float PI_Update(PI_Controller *pi, float err, float dt)
{
    /* Integrator */
    pi->integral += (pi->Ki * err * dt);
    pi->integral = clampf(pi->integral, -pi->limit, pi->limit);

    /* Output */
    float out = (pi->Kp * err) + pi->integral;
    out = clampf(out, -pi->limit, pi->limit);
    return out;
}

/* ----------------- public API ----------------- */
void FOC_SetVdc(FOC_t *f, float vdc)
{
    if (!f) return;
    f->vdc = vdc;
}

/* PSIM-derived PI parameters per MS mode:
 * PSIM gives K and T, for PI form: Kp=K, Ki=K/T
 * Also using Vs_max as PI limit (volts)
 */
void FOC_ApplyMSModePI(FOC_t *f, uint8_t mode)
{
    if (!f) return;
    if (mode < 1 || mode > 3) return;

    /* Updated per PI control parameters (1).pdf
       Mode:   1(12V)       2(24V)       3(36V)
       Kp:      1.183332f    0.591666f    0.394444f
       Ki:      1047.196f    523.598f     349.065f
       Vs_max: 6.928203f    13.856406f   20.784609f
    */
    static const float Kp_tbl[4]    = {0.0f, 1.183332f, 0.591666f, 0.394444f};
    static const float Ki_tbl[4]    = {0.0f, 1047.196f, 523.598f,  349.065f}; 
    static const float lim_tbl[4]   = {0.0f, 6.928203f, 13.856406f, 20.784609f};

    f->pi_d.Kp = Kp_tbl[mode];
    f->pi_d.Ki = Ki_tbl[mode];
    f->pi_d.limit = lim_tbl[mode];
    f->pi_d.integral = 0.0f;

    f->pi_q.Kp = Kp_tbl[mode];
    f->pi_q.Ki = Ki_tbl[mode];
    f->pi_q.limit = lim_tbl[mode];
    f->pi_q.integral = 0.0f;
}

void FOC_Init(FOC_t *f)
{
    if (!f) return;

    f->id_ref = 0.0f;
    f->iq_ref = 0.0f;

    f->d_u = 0.5f;
    f->d_v = 0.5f;
    f->d_w = 0.5f;

    /* Default bus voltage until MS picks a mode */
    f->vdc = 12.0f;

    /* Default PI values (will be overwritten by FOC_ApplyMSModePI if used) */
    f->pi_d.Kp = 0.2f;
    f->pi_d.Ki = 50.0f;
    f->pi_d.integral = 0.0f;
    f->pi_d.limit = 5.0f;

    f->pi_q.Kp = 0.2f;
    f->pi_q.Ki = 50.0f;
    f->pi_q.integral = 0.0f;
    f->pi_q.limit = 5.0f;
}

void FOC_Run(FOC_t *f, float iu, float iv, float iw, float theta, float dt)
{
    if (!f) return;

    /* 1) Clarke: abc -> alpha/beta */
    float i_alpha = 0.0f, i_beta = 0.0f;
    Clarke(iu, iv, iw, &i_alpha, &i_beta);

    /* 2) Park: alpha/beta -> d/q */
    float id = 0.0f, iq = 0.0f;
    Park(i_alpha, i_beta, theta, &id, &iq);

    /* 3) PI current control -> vd/vq */
    float err_d = f->id_ref - id;
    float err_q = f->iq_ref - iq;

    float vd = PI_Update(&f->pi_d, err_d, dt);
    float vq = PI_Update(&f->pi_q, err_q, dt);

    /* 4) Inverse Park: vd/vq -> v_alpha/v_beta */
    float v_alpha = 0.0f, v_beta = 0.0f;
    InvPark(vd, vq, theta, &v_alpha, &v_beta);

    /* 5) SVM: v_alpha/v_beta + vdc -> duty */
    float du=0.5f, dv=0.5f, dw=0.5f;
    SVM(v_alpha, v_beta, f->vdc, &du, &dv, &dw);

    /* 6) Clamp duties to safe range */
    f->d_u = clampf(du, 0.02f, 0.98f);
    f->d_v = clampf(dv, 0.02f, 0.98f);
    f->d_w = clampf(dw, 0.02f, 0.98f);
}

/* ----------------- Transforms ----------------- */

void Clarke(float ia, float ib, float ic, float* alpha, float* beta)
{
    (void)ic; /* optional if only using ia/ib */
    *alpha = ia;
    *beta  = (ia + 2.0f * ib) * 0.577350269f; // 1/sqrt(3)
}

void Park(float alpha, float beta, float theta, float* d, float* q)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *d =  alpha * c + beta * s;
    *q = -alpha * s + beta * c;
}

void InvPark(float d, float q, float theta, float* alpha, float* beta)
{
    float c = cosf(theta);
    float s = sinf(theta);
    *alpha = d * c - q * s;
    *beta  = d * s + q * c;
}

/* ----------------- SVM -----------------
 * This outputs duty cycles only.
 * Your main.c (or PWM module) should write duties to TIM1 CCR registers.
 */
void SVM(float alpha, float beta, float vdc,
         float* dutyA, float* dutyB, float* dutyC)
{
    /* Convert alpha/beta voltage into three-phase voltages */
    float Va = alpha;
    float Vb = -0.5f * alpha + 0.8660254f * beta;
    float Vc = -0.5f * alpha - 0.8660254f * beta;

    /* Zero-sequence injection to maximize bus utilization */
    float Vmax = fmaxf(Va, fmaxf(Vb, Vc));
    float Vmin = fminf(Va, fminf(Vb, Vc));
    float Voffset = 0.5f * (Vmax + Vmin);

    Va -= Voffset;
    Vb -= Voffset;
    Vc -= Voffset;

    /* Normalize to duty. Assumes alpha/beta are in VOLTS. */
    if (vdc <= 0.1f) vdc = 0.1f;

    *dutyA = (Va / vdc) + 0.5f;
    *dutyB = (Vb / vdc) + 0.5f;
    *dutyC = (Vc / vdc) + 0.5f;
}
