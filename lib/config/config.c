//=======================================================================================================
// config.c - Configuración del Timer0 (base de tiempo) y de los pines de salida
//=======================================================================================================
// Nada de este archivo se cambia por el puerto serie. [FIJO]

#include "config.h" // MOSFET_PIN, LED_PIN, set_bit / clear_bit
#include "timer.h"  // get_time(): contador de ms que incrementa la ISR del Timer0

// Timer0 en modo CTC: interrumpe cada 1 ms y la ISR (timer.c) suma 1 al contador "tiempo"
void config_timer0(void)
{
    TCCR0A = 0b00000010; // WGM01 = 1 -> modo CTC (cuenta de 0 a OCR0A y vuelve a 0)
    TIMSK0 = 0b00000010; // OCIE0A = 1 -> habilita la interrupción TIMER0_COMPA
    OCR0A = 249;         // 16 MHz / 64 / 250 = 1 kHz -> 1 ms
    TCCR0B = 0b00000011; // CS01 | CS00 -> prescaler 64 (arranca el timer)
}

// Pines que maneja el programa directamente (no por periférico)
void config_IO(void)
{
    set_bit(DDRB, MOSFET_PIN);    // D9 salida
    clear_bit(PORTB, MOSFET_PIN); // motor apagado
    set_bit(DDRB, LED_PIN);       // LED de estado
    clear_bit(PORTB, LED_PIN);    // LED apagado
}

// get_time() es de 32 bits y el micro de 8: hay que leerlo sin que la ISR lo cambie
uint32_t tiempo_ms(void)
{
    uint8_t sreg = SREG;     // guarda el estado de las interrupciones
    cli();                   // las deshabilita mientras se copian los 4 bytes
    uint32_t t = get_time(); // copia del contador de ms
    SREG = sreg;             // las vuelve a dejar como estaban
    return t;                // milisegundos desde el encendido
}
