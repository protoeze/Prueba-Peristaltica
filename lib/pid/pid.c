//=======================================================================================================
// pid.c - PID discreto: P sobre el error, I con anti-windup, D sobre la medición con filtro
//=======================================================================================================
// Las ganancias kp, ki, kd que usa pid_calcular() son las que se cambian por el puerto serie
// ('p', 'i', 'd', 'a'); el cambio entra en el próximo cálculo (cada TS_PID_MS = 20 ms).

#include "pid.h"

// recorta x al rango [lo, hi]
static float limitar(float x, float lo, float hi)
{
    return (x < lo) ? lo : ((x > hi) ? hi : x);
}

// Configuración inicial (desde control_init con KP_INICIAL, KI_INICIAL, KD_INICIAL)
void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float ts, float out_min, float out_max)
{
    p->ts = ts;                   // período de muestreo [s]
    p->tf = 5.0f * ts;            // filtro de la derivada por defecto (después se pisa con PID_TF_S)
    p->out_min = out_min;         // U_MIN
    p->out_max = out_max;         // U_MAX
    pid_set_ganancias(p, kp, ki, kd);
    pid_reset(p, 0.0f, 0.0f, 0.0f);
}

// Cambia las ganancias (comandos 'p', 'i', 'd' y 'a' por el puerto serie)
void pid_set_ganancias(pid_ctrl_t *p, float kp, float ki, float kd)
{
    // i_term acumula Ki*e*dt, por eso cambiar Ki en caliente no produce saltos
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}

// Constante del filtro pasa-bajos de la derivada [s] (PID_TF_S en config.h)
void pid_set_filtro_d(pid_ctrl_t *p, float tf)
{
    p->tf = tf;
}

// Transferencia sin salto: la integral arranca en (salida actual - término P),
// así la primera salida del PID coincide con la que tenía el motor.
// (D arranca en 0 porque pv_ant = pv.)
void pid_reset(pid_ctrl_t *p, float sp, float pv, float out_actual)
{
    out_actual = limitar(out_actual, p->out_min, p->out_max);
    p->p_term = p->kp * (sp - pv);
    p->i_term = limitar(out_actual - p->p_term, p->out_min, p->out_max);
    p->pv_ant = pv;
    p->d_filt = 0.0f;
    p->d_term = 0.0f;
    p->out = out_actual;
}

// Un paso del PID (cada TS_PID_MS). sp y pv en rpm, devuelve la salida en % (U_MIN..U_MAX)
float pid_calcular(pid_ctrl_t *p, float sp, float pv)
{
    float e = sp - pv; // error [rpm]: positivo = el motor va más lento que el setpoint

    // P: actúa sobre el error presente
    p->p_term = p->kp * e; // Kp <- comando 'p'

    // D: sobre la medición (con SP constante de/dt = -dpv/dt). Evita el golpe
    // derivativo al cambiar el SP. Filtro pasa-bajos de 1er orden contra el ruido.
    float dpv = (pv - p->pv_ant) / p->ts;                    // aceleración medida [rpm/s]
    p->pv_ant = pv;
    p->d_filt += (p->ts / (p->tf + p->ts)) * ((-dpv) - p->d_filt); // derivada filtrada
    p->d_term = p->kd * p->d_filt;                           // Kd <- comando 'd'

    // I: acumula el error pasado. Anti-windup: si la salida satura y el error
    // sigue empujando hacia afuera, no se integra.
    // OJO: mientras el error no es cero la integral sigue creciendo. Si se frena el eje con
    // la mano, i_term sube para compensar; al soltar, la velocidad pasa el setpoint hasta que
    // i_term vuelve a bajar (sobrepaso). Más Kp o menos Ki lo achican.
    float i_nuevo = p->i_term + p->ki * e * p->ts;           // Ki <- comando 'i'
    float u = p->p_term + i_nuevo + p->d_term;               // salida sin limitar [%]
    if (u > p->out_max) // satura arriba (100 %)
    {
        u = p->out_max;
        if (e > 0.0f)
            i_nuevo = p->i_term; // no seguir integrando hacia arriba
    }
    else if (u < p->out_min) // satura abajo (0 %)
    {
        u = p->out_min;
        if (e < 0.0f)
            i_nuevo = p->i_term; // no seguir integrando hacia abajo
    }
    p->i_term = limitar(i_nuevo, p->out_min, p->out_max); // la integral sola tampoco pasa los límites
    p->out = u;
    return u;
}
