//=======================================================================================================
// i2c.h - Maestro I2C (TWI por hardware del ATmega328P) en A4 (SDA) / A5 (SCL)
//=======================================================================================================
// Nada se cambia por el puerto serie. [FIJO]

#ifndef I2C_H // guarda de inclusión
#define I2C_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

#define I2C_FREC 100000UL // [FIJO] 100 kHz: más tolerante a cables largos, pull-ups débiles y ruido del motor
#define I2C_OK 0            // sin error
#define I2C_ERR_TIMEOUT 1   // el periférico no terminó a tiempo (bus trabado o sin pull-ups)
#define I2C_ERR_START 2     // no se pudo generar la condición de START
#define I2C_ERR_NACK 3      // el esclavo no respondió (dirección o cable)

void i2c_init(void);
// lee n bytes desde el registro reg del dispositivo dir (7 bits)
uint8_t i2c_leer_reg(uint8_t dir, uint8_t reg, uint8_t *dato, uint8_t n);
// 0 si hay un dispositivo que responde (ACK) en la dirección dir
uint8_t i2c_probar(uint8_t dir);
// nivel de SDA y SCL en reposo (con el TWI apagado y pull-ups): 1 = alto, 0 = bajo
void i2c_niveles(uint8_t *sda, uint8_t *scl);

#ifdef __cplusplus
}
#endif

#endif // I2C_H
