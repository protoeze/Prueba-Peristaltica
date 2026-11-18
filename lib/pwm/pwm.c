//=======================================================================================================
// pwm.c - PWM por hardware en D9 (OC1A, Timer1) hacia el gate del MOSFET
//=======================================================================================================
// La frecuencia y la resolución son [FIJO]. El % que se aplica sale del PID, de 'm' o de 't'.

#include "pwm.h"
#include "config.h" // MOSFET_PIN, set_bit / clear_bit

// Timer1, Fast PWM 10 bits (modo 7), prescaler 1 -> 16 MHz / 1024 = 15,6 kHz en D9 (OC1A)
void pwm_init(void)
{
    TCCR1A = (1 << WGM11) | (1 << WGM10); // WGM11:10 = 11 -> fast PWM 10 bits (TOP = 1023)
    TCCR1B = (1 << WGM12) | (1 << CS10);  // WGM12 = 1 (completa el modo 7), CS10 = prescaler 1
    OCR1A = 0;                            // ciclo de trabajo inicial 0
}

// duty de 0 a 1023 (0 = apagado, 1023 = 100 %)
void pwm_set(uint16_t duty)
{
    if (duty > 1023)
        duty = 1023;
    if (duty == 0)
    {
        // OCR=0 en fast PWM deja un pico: se desconecta el pin y se fuerza a 0
        clear_bit(TCCR1A, COM1A1);    // D9 vuelve a ser un pin común
        clear_bit(PORTB, MOSFET_PIN); // en 0 -> MOSFET cortado
    }
    else
    {
        OCR1A = duty;            // ancho del pulso
        set_bit(TCCR1A, COM1A1); // PWM no invertido
    }
}

// pct de 0 a 100 % -> duty de 0 a 1023
void pwm_set_porcentaje(float pct)
{
    if (pct < 0.0f)
        pct = 0.0f;
    if (pct > 100.0f)
        pct = 100.0f;
    pwm_set((uint16_t)(pct * 10.23f + 0.5f)); // +0,5 para redondear
}
