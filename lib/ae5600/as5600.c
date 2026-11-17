#include "as5600.h"
#include "i2c.h"
#include "config.h"

static uint16_t ang_ant = 0;
static uint32_t pos = 0;
static uint8_t primera = 1;
static uint8_t estado = 0;

uint8_t as5600_init(void)
{
    i2c_init();
    uint8_t e = as5600_leer_estado();
    if (e)
        return e;
    if (!(estado & AS5600_MD))
        return AS5600_ERR_IMAN;
    primera = 1;
    return as5600_actualizar();
}

uint8_t as5600_actualizar(void)
{
    uint8_t b[2];
    uint8_t e = i2c_leer_reg(AS5600_DIR, AS5600_REG_RAW_ANGLE, b, 2);
    if (e)
        return e;

    uint16_t ang = ((uint16_t)(b[0] & 0x0F) << 8) | b[1];
    if (primera)
    {
        ang_ant = ang;
        primera = 0;
        return 0;
    }
    int16_t d = (int16_t)ang - (int16_t)ang_ant;
    if (d > 2048)
        d -= 4096; // cruzó el 0 hacia atrás
    else if (d < -2048)
        d += 4096; // cruzó el 0 hacia adelante
    pos += (uint32_t)(int32_t)(d * SENTIDO_SENSOR);
    ang_ant = ang;
    return 0;
}

uint8_t as5600_leer_estado(void)
{
    return i2c_leer_reg(AS5600_DIR, AS5600_REG_STATUS, &estado, 1);
}

uint8_t as5600_leer_diag(uint8_t *agc, uint16_t *mag, uint16_t *ang)
{
    uint8_t b[3], e;
    if ((e = as5600_leer_estado()))
        return e;
    if ((e = i2c_leer_reg(AS5600_DIR, AS5600_REG_AGC, b, 3))) // AGC, MAGNITUDE alto, bajo
        return e;
    *agc = b[0];
    *mag = ((uint16_t)(b[1] & 0x0F) << 8) | b[2];
    if ((e = i2c_leer_reg(AS5600_DIR, AS5600_REG_RAW_ANGLE, b, 2)))
        return e;
    *ang = ((uint16_t)(b[0] & 0x0F) << 8) | b[1];
    return 0;
}

uint32_t as5600_posicion(void) { return pos; }
uint8_t as5600_estado(void) { return estado; }
