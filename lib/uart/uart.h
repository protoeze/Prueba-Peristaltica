//=======================================================================================================
// uart.h - Puerto serie por interrupciones (lo que se ve en el monitor serie de PlatformIO)
//=======================================================================================================
// La velocidad es [FIJO]: si se cambia UART_BAUD hay que cambiar también monitor_speed
// en platformio.ini.

#ifndef UART_H // guarda de inclusión
#define UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>
#include <avr/pgmspace.h> // PSTR(): textos guardados en flash, no en RAM

#define UART_BAUD 115200UL // [FIJO] baudios

void uart_init(void);
void uart_putc(char c);           // envía un carácter (espera si el buffer está lleno)
void uart_puts(const char *s);    // string en RAM
void uart_puts_P(const char *s); // string en flash
void uart_put_i(int32_t v);       // número entero
void uart_put_f(float v, uint8_t decimales); // número con decimales
int16_t uart_getc(void); // -1 si no hay datos
// función que se llama mientras uart_putc espera lugar en el buffer
void uart_set_espera(void (*f)(void));

#define UART_PRINT(s) uart_puts_P(PSTR(s)) // imprime un texto fijo sin gastar RAM

#ifdef __cplusplus
}
#endif

#endif // UART_H
