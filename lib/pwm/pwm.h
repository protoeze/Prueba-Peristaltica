//=======================================================================================================
// pwm.h - Salida PWM al MOSFET (D9)
//=======================================================================================================

#ifndef PWM_H // guarda de inclusión
#define PWM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

void pwm_init(void);                // Timer1 en fast PWM 10 bits, 15,6 kHz
void pwm_set(uint16_t duty);        // 0..1023
void pwm_set_porcentaje(float pct); // 0..100 %

#ifdef __cplusplus
}
#endif

#endif // PWM_H
