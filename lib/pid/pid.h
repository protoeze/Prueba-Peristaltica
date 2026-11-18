//=======================================================================================================
// pid.h - Controlador PID discreto (forma paralela)
//=======================================================================================================
// kp, ki, kd se cambian por el puerto serie con 'p <v>', 'i <v>', 'd <v>' o 'a z|c|k'.
// ts, tf, out_min y out_max son [FIJO] (TS_PID_S, PID_TF_S, U_MIN, U_MAX en config.h).

#ifndef PID_H // guarda de inclusión
#define PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

// u = Kp*e + Ki*Integral(e dt) + Kd*de/dt     (forma paralela)
typedef struct
{
    float kp, ki, kd;    // [SERIE: p / i / d] ganancias: [%/rpm], [%/(rpm*s)], [%*s/rpm]
    float ts;            // período de muestreo [s]
    float tf;            // constante del filtro de la derivada [s]
    float out_min, out_max; // límites de la salida [%]
    float i_term;        // integral ya multiplicada por Ki
    float pv_ant;        // medición anterior (para la derivada)
    float d_filt;        // derivada de la medición filtrada
    float p_term, d_term, out; // última contribución P, D y salida total (para depurar)
} pid_ctrl_t;

void pid_init(pid_ctrl_t *p, float kp, float ki, float kd, float ts, float out_min, float out_max);
void pid_set_ganancias(pid_ctrl_t *p, float kp, float ki, float kd); // la usan 'p', 'i', 'd', 'a'
void pid_set_filtro_d(pid_ctrl_t *p, float tf);
void pid_reset(pid_ctrl_t *p, float sp, float pv, float out_actual); // arranque sin salto
float pid_calcular(pid_ctrl_t *p, float sp, float pv);               // devuelve la salida [%]

#ifdef __cplusplus
}
#endif

#endif // PID_H
