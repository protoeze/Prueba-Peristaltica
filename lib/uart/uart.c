//=======================================================================================================
// uart.c - USART0 con buffers circulares: TX y RX por interrupción (no frena el lazo de control)
//=======================================================================================================
// Lo que se escribe en la terminal llega a rx_buf y lo procesa comandos_tarea() (comandos.c).

#include "uart.h"
#include <stdlib.h> // ltoa(), dtostrf()
#include <string.h> // strlen()

#define TX_SIZE 256 // potencias de 2 (máx. 256: índices de 8 bits)
#define RX_SIZE 32  // alcanza para un comando (LINEA_MAX = 32)
#define UART_UBRR ((F_CPU + 4UL * UART_BAUD) / (8UL * UART_BAUD) - 1UL) // divisor redondeado (modo U2X)

static volatile uint8_t tx_buf[TX_SIZE];          // lo que falta enviar
static volatile uint8_t tx_head = 0, tx_tail = 0; // head = donde se escribe, tail = donde se lee
static volatile uint8_t rx_buf[RX_SIZE];          // lo recibido y todavía no leído
static volatile uint8_t rx_head = 0, rx_tail = 0;
static void (*espera_cb)(void) = 0;               // se llama mientras el buffer TX está lleno

// registra la función a llamar mientras se espera (control.c pasa tarea_sensor)
void uart_set_espera(void (*f)(void))
{
    espera_cb = f;
}

// 115200 baud, 8 bits, sin paridad, 1 stop; habilita RX, TX e interrupción de recepción
void uart_init(void)
{
    UCSR0A = (1 << U2X0);                     // doble velocidad: menos error a 115200
    UBRR0H = (uint8_t)(UART_UBRR >> 8);
    UBRR0L = (uint8_t)(UART_UBRR & 0xFF);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); // 8N1
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
}

// "registro de transmisión vacío": manda el próximo byte del buffer
ISR(USART_UDRE_vect)
{
    if (tx_head == tx_tail)
    {
        UCSR0B &= ~(1 << UDRIE0); // nada más para enviar
        return;
    }
    UDR0 = tx_buf[tx_tail];
    tx_tail = (tx_tail + 1) & (TX_SIZE - 1); // avanza y da la vuelta al llegar al final
}

// llegó un byte por el puerto serie: se guarda en rx_buf
ISR(USART_RX_vect)
{
    uint8_t d = UDR0;
    uint8_t sig = (rx_head + 1) & (RX_SIZE - 1);
    if (sig != rx_tail) // si el buffer está lleno el byte se pierde
    {
        rx_buf[rx_head] = d;
        rx_head = sig;
    }
}

void uart_putc(char c)
{
    uint8_t h, sig;
    for (;;)
    {
        h = tx_head;
        sig = (uint8_t)((h + 1) & (TX_SIZE - 1));
        if (sig != tx_tail) // hay lugar
            break;
        // buffer lleno: mientras la ISR lo vacía se sigue atendiendo el sensor,
        // si no el AS5600 deja de leerse y se pierde la cuenta de vueltas
        if (espera_cb)
            espera_cb();
    }
    tx_buf[h] = (uint8_t)c;
    tx_head = sig;
    UCSR0B |= (1 << UDRIE0); // la ISR se encarga de mandarlo
}

void uart_puts(const char *s)
{
    while (*s)
        uart_putc(*s++);
}

void uart_puts_P(const char *s)
{
    char c;
    while ((c = pgm_read_byte(s++))) // lee de la flash
        uart_putc(c);
}

// imprime el texto b precedido de los espacios que falten para ocupar 'ancho' caracteres
static void uart_puts_ancho(const char *b, uint8_t ancho)
{
    uint8_t n = (uint8_t)strlen(b);
    while (n < ancho) // relleno a la izquierda: el número queda alineado a la derecha
    {
        uart_putc(' ');
        n++;
    }
    uart_puts(b);
}

void uart_put_i_ancho(int32_t v, uint8_t ancho)
{
    char b[14];
    ltoa(v, b, 10); // entero -> texto en base 10
    uart_puts_ancho(b, ancho);
}

void uart_put_f_ancho(float v, uint8_t ancho, uint8_t decimales)
{
    char b[16];
    if (v > 99999.0f) // límite para que entre en el buffer
        v = 99999.0f;
    if (v < -99999.0f)
        v = -99999.0f;
    dtostrf(v, 1, decimales, b); // float -> texto
    uart_puts_ancho(b, ancho);
}

void uart_put_i(int32_t v) { uart_put_i_ancho(v, 0); }                         // sin relleno
void uart_put_f(float v, uint8_t decimales) { uart_put_f_ancho(v, 0, decimales); } // sin relleno

// devuelve el próximo byte recibido o -1 si no hay nada
int16_t uart_getc(void)
{
    if (rx_head == rx_tail)
        return -1;
    uint8_t d = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) & (RX_SIZE - 1);
    return d;
}
