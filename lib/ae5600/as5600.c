//=======================================================================================================
// as5600.c - Lee el ángulo del AS5600 y acumula la posición (vueltas * 4096 + ángulo)
//=======================================================================================================
// La velocidad se calcula en control.c a partir de as5600_posicion(). [FIJO]

#include "as5600.h"
#include "i2c.h"    // i2c_leer_reg()
#include "config.h" // SENTIDO_SENSOR

static uint16_t ang_ant = 0; // ángulo de la lectura anterior (0..4095)
static uint32_t pos = 0;     // posición acumulada [cuentas]
static uint8_t primera = 1;  // 1 = la próxima lectura solo toma referencia
static uint8_t estado = 0;   // último registro STATUS

// Arranca el I2C y verifica que el sensor responda y vea el imán
uint8_t as5600_init(void)
{
    i2c_init();
    uint8_t e = as5600_leer_estado();
    if (e)                       // no respondió por I2C
        return e;
    if (!(estado & AS5600_MD))   // respondió pero no detecta el imán
        return AS5600_ERR_IMAN;
    primera = 1;
    return as5600_actualizar();
}

// Lee el ángulo crudo y suma a pos lo que giró desde la lectura anterior
uint8_t as5600_actualizar(void)
{
    uint8_t b[2];
    uint8_t e = i2c_leer_reg(AS5600_DIR, AS5600_REG_RAW_ANGLE, b, 2);
    if (e)
        return e;

    uint16_t ang = ((uint16_t)(b[0] & 0x0F) << 8) | b[1]; // 12 bits: 0..4095
    if (primera)
    {
        ang_ant = ang;
        primera = 0;
        return 0;
    }
    int16_t d = (int16_t)ang - (int16_t)ang_ant; // cuánto giró desde la lectura anterior
    if (d > 2048)
        d -= 4096; // cruzó el 0 hacia atrás
    else if (d < -2048)
        d += 4096; // cruzó el 0 hacia adelante
    pos += (uint32_t)(int32_t)(d * SENTIDO_SENSOR); // acumula con el signo elegido en config.h
    ang_ant = ang;
    return 0;
}

// Lee el registro STATUS (imán detectado / débil / fuerte)
uint8_t as5600_leer_estado(void)
{
    return i2c_leer_reg(AS5600_DIR, AS5600_REG_STATUS, &estado, 1);
}

// Comando 'e': STATUS + AGC + magnitud + ángulo crudo, para revisar el imán
uint8_t as5600_leer_diag(uint8_t *agc, uint16_t *mag, uint16_t *ang)
{
    uint8_t b[3], e;
    if ((e = as5600_leer_estado()))
        return e;
    if ((e = i2c_leer_reg(AS5600_DIR, AS5600_REG_AGC, b, 3))) // AGC, MAGNITUDE alto, bajo
        return e;
    *agc = b[0];                                    // ganancia automática (0..255 a 5 V)
    *mag = ((uint16_t)(b[1] & 0x0F) << 8) | b[2];   // magnitud, 12 bits
    if ((e = i2c_leer_reg(AS5600_DIR, AS5600_REG_RAW_ANGLE, b, 2)))
        return e;
    *ang = ((uint16_t)(b[0] & 0x0F) << 8) | b[1];   // ángulo crudo 0..4095
    return 0;
}

uint32_t as5600_posicion(void) { return pos; }  // cuentas acumuladas
uint16_t as5600_angulo(void) { return ang_ant; } // último ángulo leído 0..4095 (columna ang_grad del log)
uint8_t as5600_estado(void) { return estado; }  // último STATUS leído
