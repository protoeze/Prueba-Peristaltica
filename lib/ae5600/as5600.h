//=======================================================================================================
// as5600.h - Sensor magnético de ángulo AS5600 (12 bits) por I2C
//=======================================================================================================
// Nada se cambia por el puerto serie. [FIJO]

#ifndef AS5600_H // guarda de inclusión
#define AS5600_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>

#define AS5600_DIR 0x36 // dirección I2C de 7 bits. AS5600L: 0x40 (usar el comando e para ver la dirección)
#define AS5600_REG_STATUS 0x0B    // registro de estado del imán
#define AS5600_REG_RAW_ANGLE 0x0C // ángulo crudo (2 bytes, 12 bits)
#define AS5600_REG_AGC 0x1A       // ganancia automática (comando 'e')
#define AS5600_REG_MAGNITUDE 0x1B // magnitud del campo magnético (comando 'e')
#define AS5600_CUENTAS 4096.0f // 12 bits por vuelta

#define AS5600_MH (1 << 3) // imán muy fuerte
#define AS5600_ML (1 << 4) // imán muy débil
#define AS5600_MD (1 << 5) // imán detectado
#define AS5600_ERR_IMAN 0x10 // código de error propio: no hay imán

uint8_t as5600_init(void);       // 0 = OK
uint8_t as5600_actualizar(void); // lee el ángulo y acumula la posición (0 = OK)
uint32_t as5600_posicion(void);  // cuentas acumuladas (usar la resta con signo)
uint16_t as5600_angulo(void);    // último ángulo crudo leído, 0..4095 (= 0..360 grados)
uint8_t as5600_leer_estado(void); // lee STATUS (0 = OK)
uint8_t as5600_estado(void);     // último registro STATUS leído
// diagnóstico: ganancia automática, magnitud del campo y ángulo crudo (0 = OK)
uint8_t as5600_leer_diag(uint8_t *agc, uint16_t *mag, uint16_t *ang);

#ifdef __cplusplus
}
#endif

#endif // AS5600_H
