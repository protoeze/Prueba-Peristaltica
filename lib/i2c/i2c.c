//=======================================================================================================
// i2c.c - Lectura de registros por I2C con timeout (no se cuelga si se desconecta el sensor)
//=======================================================================================================
// Los números 0x08, 0x10, 0x18, ... son los códigos de estado del registro TWSR que
// indica la hoja de datos del ATmega328P para cada paso de la comunicación. [FIJO]

#include "i2c.h"
#include "config.h"      // set_bit / clear_bit
#include <util/delay.h>  // _delay_us() para los pulsos manuales de SCL

#define I2C_TIMEOUT 4000U // iteraciones (~1,5 ms)

// espera a que el periférico termine el paso actual (bit TWINT = 1)
static uint8_t i2c_esperar(void)
{
    uint16_t n = I2C_TIMEOUT;
    while (!(TWCR & (1 << TWINT)))
        if (--n == 0)
            return I2C_ERR_TIMEOUT;
    return I2C_OK;
}

// genera START (o START repetido) y verifica el estado esperado
static uint8_t i2c_start(uint8_t estado_ok)
{
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    if (i2c_esperar())
        return I2C_ERR_TIMEOUT;
    return ((TWSR & 0xF8) == estado_ok) ? I2C_OK : I2C_ERR_START;
}

// envía un byte (dirección o dato) y verifica que el esclavo responda ACK
static uint8_t i2c_tx(uint8_t dato, uint8_t estado_ok)
{
    TWDR = dato;
    TWCR = (1 << TWINT) | (1 << TWEN);
    if (i2c_esperar())
        return I2C_ERR_TIMEOUT;
    return ((TWSR & 0xF8) == estado_ok) ? I2C_OK : I2C_ERR_NACK;
}

// recibe un byte; ack = 1 pide más bytes, ack = 0 indica que es el último
static uint8_t i2c_rx(uint8_t *dato, uint8_t ack)
{
    TWCR = (1 << TWINT) | (1 << TWEN) | (ack ? (1 << TWEA) : 0);
    if (i2c_esperar())
        return I2C_ERR_TIMEOUT;
    if ((TWSR & 0xF8) != (ack ? 0x50 : 0x58)) // 0x50 = dato + ACK, 0x58 = dato + NACK
        return I2C_ERR_NACK;
    *dato = TWDR;
    return I2C_OK;
}

// genera STOP y espera (con límite) a que se complete
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
    for (uint8_t i = 0; i < 9 && !(PINC & (1 << PC4)); i++) // mientras SDA siga en 0
    {
        clear_bit(PORTC, PC5); // SCL = 0
        set_bit(DDRC, PC5);    // SCL como salida (lo baja)
        _delay_us(5);
        clear_bit(DDRC, PC5); // SCL suelto (pull-up)
        set_bit(PORTC, PC5);
        _delay_us(5);
    }
    TWCR = (1 << TWEN); // el TWI vuelve a tomar los pines
}

// configura el TWI a I2C_FREC
void i2c_init(void)
{
    set_bit(PORTC, PC4); // pull-ups internos de respaldo (SDA)
    set_bit(PORTC, PC5); // (SCL)  (los de verdad son R4 y R5 de 4k7 en la PCB)
    TWSR = 0;            // prescaler 1
    TWBR = ((F_CPU / I2C_FREC) - 16) / 2; // 100 kHz -> 72
    TWCR = (1 << TWEN);  // habilita el periférico
}

// START, dirección+W, registro, START repetido, dirección+R, n bytes
static uint8_t transaccion(uint8_t dir, uint8_t reg, uint8_t *dato, uint8_t n)
{
    uint8_t e;
    if ((e = i2c_start(0x08)))                         // 0x08 = START enviado
        return e;
    if ((e = i2c_tx((uint8_t)(dir << 1), 0x18)))      // 0x18 = dirección+W con ACK
        return e;
    if ((e = i2c_tx(reg, 0x28)))                       // 0x28 = dato enviado con ACK
        return e;
    if ((e = i2c_start(0x10))) // start repetido          0x10 = START repetido enviado
        return e;
    if ((e = i2c_tx((uint8_t)((dir << 1) | 1), 0x40))) // 0x40 = dirección+R con ACK
        return e;
    for (uint8_t i = 0; i < n; i++)
        if ((e = i2c_rx(&dato[i], i < (uint8_t)(n - 1)))) // ACK en todos menos el último
            return e;
    return I2C_OK;
}

// lectura completa; si el bus quedó trabado lo libera y reinicia el periférico
uint8_t i2c_leer_reg(uint8_t dir, uint8_t reg, uint8_t *dato, uint8_t n)
{
    uint8_t e = transaccion(dir, reg, dato, n);
    i2c_stop();
    if (e == I2C_ERR_TIMEOUT || e == I2C_ERR_START)
        i2c_liberar_bus(); // también resetea el periférico
    return e;
}

// Comando 'e': lee el nivel de SDA y SCL en reposo (los dos tienen que estar en 1)
void i2c_niveles(uint8_t *sda, uint8_t *scl)
{
    TWCR = 0; // pines como entradas con pull-up
    _delay_us(100);           // tiempo para que suban por los pull-ups
    *sda = (PINC >> PC4) & 1; // A4
    *scl = (PINC >> PC5) & 1; // A5
    TWCR = (1 << TWEN);
}

// Comando 'e': 0 si algún dispositivo responde (ACK) en la dirección dir (barrido del bus)
uint8_t i2c_probar(uint8_t dir)
{
    uint8_t e = i2c_start(0x08);                   // START
    if (!e)
        e = i2c_tx((uint8_t)(dir << 1), 0x18);     // dirección + W, espera ACK
    i2c_stop();
    if (e == I2C_ERR_TIMEOUT || e == I2C_ERR_START)
        i2c_liberar_bus();
    return e;
}
