#include "i2c.h"
#include "config.h"
#include <util/delay.h>

#define I2C_TIMEOUT 4000U // iteraciones (~1,5 ms)

static uint8_t i2c_esperar(void)
{
    uint16_t n = I2C_TIMEOUT;
    while (!(TWCR & (1 << TWINT)))
        if (--n == 0)
            return I2C_ERR_TIMEOUT;
    return I2C_OK;
}

static uint8_t i2c_start(uint8_t estado_ok)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (i2c_esperar())
        return I2C_ERR_TIMEOUT;
    return ((TWSR & 0xF8) == estado_ok) ? I2C_OK : I2C_ERR_START;
}

static uint8_t i2c_tx(uint8_t dato, uint8_t estado_ok)
{
    TWDR = dato;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (i2c_esperar())
        return I2C_ERR_TIMEOUT;
    return ((TWSR & 0xF8) == estado_ok) ? I2C_OK : I2C_ERR_NACK;
}

static uint8_t i2c_rx(uint8_t *dato, uint8_t ack)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);
    if (i2c_esperar())
        return I2C_ERR_TIMEOUT;
    if ((TWSR & 0xF8) != (ack ? 0x50 : 0x58))
        return I2C_ERR_NACK;
    *dato = TWDR;
    return I2C_OK;
}

static void i2c_stop(void)
{
    TWCR = (1 << TWINT) | (1 << TWSTO) | (1 << TWEN);
    uint16_t n = I2C_TIMEOUT;
    while ((TWCR & (1 << TWSTO)) && --n)
        ;
}

// Si un ruido corta una transacción a la mitad, el AS5600 puede quedar
// sosteniendo SDA en 0 y el bus queda trabado para siempre (todas las lecturas
// fallan). Se libera dando hasta 9 pulsos de reloj a mano en SCL.
static void i2c_liberar_bus(void)
{
    TWCR = 0; // el TWI suelta los pines
    for (uint8_t i = 0; i < 9 && !(PINC & (1 << PC4)); i++)
    {
        clear_bit(PORTC, PC5); // SCL = 0
        set_bit(DDRC, PC5);
        _delay_us(5);
        clear_bit(DDRC, PC5); // SCL suelto (pull-up)
        set_bit(PORTC, PC5);
        _delay_us(5);
    }
    TWCR = (1 << TWEN);
}

void i2c_init(void)
{
    set_bit(PORTC, PC4); // pull-ups internos de respaldo (SDA)
    set_bit(PORTC, PC5); // (SCL)
    TWSR = 0;            // prescaler 1
    TWBR = ((F_CPU / I2C_FREC) - 16) / 2; // 100 kHz -> 72
    TWCR = (1 << TWEN);
}

static uint8_t transaccion(uint8_t dir, uint8_t reg, uint8_t *dato, uint8_t n)
{
    uint8_t e;
    if ((e = i2c_start(0x08)))
        return e;
    if ((e = i2c_tx((uint8_t)(dir << 1), 0x18)))
        return e;
    if ((e = i2c_tx(reg, 0x28)))
        return e;
    if ((e = i2c_start(0x10))) // start repetido
        return e;
    if ((e = i2c_tx((uint8_t)((dir << 1) | 1), 0x40)))
        return e;
    for (uint8_t i = 0; i < n; i++)
        if ((e = i2c_rx(&dato[i], i < (uint8_t)(n - 1))))
            return e;
    return I2C_OK;
}

uint8_t i2c_leer_reg(uint8_t dir, uint8_t reg, uint8_t *dato, uint8_t n)
{
    uint8_t e = transaccion(dir, reg, dato, n);
    i2c_stop();
    if (e == I2C_ERR_TIMEOUT || e == I2C_ERR_START)
        i2c_liberar_bus(); // también resetea el periférico
    return e;
}

uint8_t i2c_probar(uint8_t dir)
{
    uint8_t e = i2c_start(0x08);
    if (!e)
        e = i2c_tx((uint8_t)(dir << 1), 0x18);
    i2c_stop();
    if (e == I2C_ERR_TIMEOUT || e == I2C_ERR_START)
        i2c_liberar_bus();
    return e;
}
