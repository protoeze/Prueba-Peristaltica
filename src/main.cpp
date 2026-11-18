
#include <Arduino.h>  // registros del ATmega328P (no se usan setup()/loop() de Arduino)
#include "config.h"   // pines, tiempos y ganancias iniciales
#include "uart.h"     // puerto serie
#include "pwm.h"      // salida al MOSFET
#include "control.h"  // lazo PID
#include "comandos.h" // comandos por el puerto serie + encoder + botón
#include "encoder.h"  // encoder rotativo

//---------- Control PID de velocidad de un motor DC 12V 5A ----------//
// Actuador : MOSFET en D9 (Timer1, PWM 15,6 kHz)
// Sensor   : AS5600 por I2C (SDA = A4, SCL = A5) -> velocidad en rpm
// Encoder  : A = D4, B = D5 -> setpoint (también con 's <rpm>' por la terminal)
//            botón = D3 -> PID ON / OFF (también con 'g' / 'x')
// Terminal : 115200 baud. Con '?' se ve la ayuda.
//
// Qué cambia cada comando del puerto serie (tabla completa al principio de lib/comandos/comandos.c):
//   p <v>  Kp   i <v>  Ki   d <v>  Kd        (valores iniciales KP/KI/KD_INICIAL en config.h)
//   s <rpm> setpoint   m <%> salida manual   g PID ON   x parar   l log ON/OFF   ? estado
//   l <ms> una línea del log cada <ms> (inicial LOG_PERIODO_MS = 500 ms; 20 = todas)
//   e diagnóstico del AS5600 (con el motor quieto)
// Lo cambiado por serie NO se guarda: al resetear vuelve a lo que dice config.h.
// En config.h, [SERIE: x] = se puede cambiar con el comando x; [FIJO] = solo recompilando.
//
// Sintonía (punto 3 de la consigna):
//   1) t <u0> <u1>  -> ensayo al escalón: K, tau, theta + ZN y CHR  (a z | a c)
//   2) o ZN lazo cerrado: i 0, d 0, subir p hasta oscilar -> z <Ku> <Tu> (a k)
//   3) ajuste fino manual con p / i / d mientras se mira el log (t_ms, sp, rpm, u_pct)
// Requiere timer.c / timer.h (los tuyos; "tiempo" pasó a ser volatile).
int main() // reemplaza al main() de Arduino: no hay setup()/loop()
{
    config_IO();     // D9 (MOSFET) y D13 (LED) como salidas en 0
    config_timer0(); // base de tiempo de 1 ms -> get_time()
    uart_init();     // puerto serie a 115200
    pwm_init();      // Timer1: PWM de 10 bits en D9, arranca en 0
    encoder_init(); // pines del encoder con pull-up
    pcint_init();   // interrupción por cambio en PD4 / PD5
    sei();           // habilita las interrupciones globales
    control_init(); // I2C + AS5600 + PID

    while (1) // lazo principal: nunca bloquea, cada tarea mira su propio reloj
    {
        asm("NOP"); // ILUVIT
        comandos_tarea(); // terminal, encoder y botón
        control_tarea();  // sensor cada 2 ms, PID cada 20 ms, LED
    }
}
