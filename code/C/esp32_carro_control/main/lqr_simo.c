#include "lqr_simo.h"
#include "lqr_tablas.h"
#include <math.h>
#include <stdbool.h>

#define PI_F 3.14159265f

// Estados del controlador (integradores del bloque Controlador_SIMO del .slx)
static float s_thr;   // referencia prefiltrada [rad]          (Int_theta_ref)
static float s_zw;    // estado del filtro de omega [rad]       (Omega_filtrada)
static float s_vf;    // v_fan estimado [%]                     (Int_vf_est)
static float s_xi;    // integral del error [rad*s]             (Int_xi)
static float s_dn;    // punto de operacion de distancia [m]    (Int_dnom)

static float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

void lqr_simo_reset(float theta_meas) {
    s_thr = theta_meas;  // la referencia arranca donde esta la compuerta: sin escalon
    s_zw = theta_meas;   // omega = 0
    s_vf = 0.0f;         // ventilador apagado
    s_xi = 0.0f;
    s_dn = LQR_D_INI_M;
}

// Ganancia K(2x5) interpolada bilinealmente en (theta_ref, d_nom), como el bloque LQR_SIMO
static void ganancia(float thr, float dn, float K[2][5]) {
    float thd = clampf(thr * 180.0f / PI_F, LQR_THG_DEG[0], LQR_THG_DEG[LQR_N_TH - 1]);
    float dcm = clampf(dn * 100.0f, LQR_DG_CM[0], LQR_DG_CM[LQR_N_D - 1]);
    int i = 0, j = 0;
    for (int k = 0; k < LQR_N_TH - 1; k++) if (thd >= LQR_THG_DEG[k]) i = k;
    for (int k = 0; k < LQR_N_D - 1; k++) if (dcm >= LQR_DG_CM[k]) j = k;
    float wt = (thd - LQR_THG_DEG[i]) / (LQR_THG_DEG[i + 1] - LQR_THG_DEG[i]);
    float wd = (dcm - LQR_DG_CM[j]) / (LQR_DG_CM[j + 1] - LQR_DG_CM[j]);
    const float *k11 = LQR_KG[j * LQR_N_TH + i], *k21 = LQR_KG[j * LQR_N_TH + i + 1];
    const float *k12 = LQR_KG[(j + 1) * LQR_N_TH + i], *k22 = LQR_KG[(j + 1) * LQR_N_TH + i + 1];
    for (int n = 0; n < 10; n++) {
        float kv = (1 - wd) * ((1 - wt) * k11[n] + wt * k21[n]) + wd * ((1 - wt) * k12[n] + wt * k22[n]);
        K[n % 2][n / 2] = kv;  // MATLAB guarda K(2x5) por columnas
    }
}

lqr_simo_out_t lqr_simo_step(float theta_cmd, float theta_meas, float d_est, float dt) {
    lqr_simo_out_t out;
    float th = theta_meas, thr = s_thr, dn = s_dn;

    // Estimadores: omega = s/(tau_w s + 1) * theta;  v_fan sigue al PWM con tau_fan
    float om = (th - s_zw) / LQR_TAU_W;

    // Ley de control LQR_SIMO: x = [th-thr; om; vf-veq; (d-dn)*100; xi], u = -K*x
    float K[2][5];
    ganancia(thr, dn, K);
    float veq = LQR_MGL * sinf(thr) / (LQR_K_M * (LQR_D0_M / dn) * (LQR_D0_M / dn));
    float x[5] = {th - thr, om, s_vf - veq, (d_est - dn) * 100.0f, s_xi};
    float u1 = 0.0f, u2 = 0.0f;
    for (int c = 0; c < 5; c++) {
        u1 -= K[0][c] * x[c];
        u2 -= K[1][c] * x[c];
    }
    out.p_raw = veq + u1;
    out.u_pwm = clampf(out.p_raw, LQR_PWM_MIN, LQR_PWM_MAX);
    out.d_ref_m = clampf(dn + u2 / 100.0f, LQR_D_MIN_M, LQR_D_MAX_M);
    out.theta_ref = thr;

    // AntiWindup: no integra con el PWM saturado empujando en la misma direccion ni con |e| grande
    float e = th - thr;
    bool satura = (out.p_raw >= LQR_PWM_MAX && e < 0) || (out.p_raw <= LQR_PWM_MIN && e > 0);
    float e_int = (satura || fabsf(e) > LQR_XI_BAND_RAD) ? 0.0f : e;

    // Supervisor_d: acerca si el ventilador no alcanza, aleja si sobra, regresa hacia d_ini
    float r = 0.0f;
    if (out.p_raw > LQR_SUP_V_HI) {
        r = -LQR_SUP_ADV * fminf(1.0f, (out.p_raw - LQR_SUP_V_HI) / 20.0f);
    } else if (out.p_raw < LQR_PWM_MIN - 3.0f) {
        r = LQR_SUP_ADV * fminf(1.0f, (LQR_PWM_MIN - 3.0f - out.p_raw) / 20.0f);
    } else if (out.p_raw < LQR_SUP_V_LO && dn < LQR_D_INI_M) {
        r = LQR_SUP_RET * fminf(1.0f, (LQR_SUP_V_LO - out.p_raw) / 20.0f);
    }

    // Referencia: gobernador + prefiltro de primer orden con limite de rampa
    float r_gov = fminf(theta_cmd, th + LQR_GOV_RAD);
    float dthr = clampf((r_gov - thr) / LQR_TAU_REF, -LQR_REF_RATE, LQR_REF_RATE);

    // Integracion de Euler con los mismos limites que los integradores del .slx
    s_thr = thr + dt * dthr;
    s_zw += dt * om;
    s_vf = clampf(s_vf + dt * (out.u_pwm - s_vf) / LQR_TAU_FAN, 0.0f, LQR_PWM_MAX);
    s_xi = clampf(s_xi + dt * e_int, -LQR_XI_LIM, LQR_XI_LIM);
    s_dn = clampf(dn + dt * r, LQR_D_MIN_M, LQR_D_MAX_M);
    return out;
}
