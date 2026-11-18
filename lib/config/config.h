//=======================================================================================================
// config.h - Pines, tiempos y constantes de todo el proyecto
//=======================================================================================================
// LEYENDA de las etiquetas que aparecen en los comentarios de TODO el repositorio
// (buscar "[SERIE" en VS Code con Ctrl+Shift+F para ver todo lo que se cambia por el puerto serie):
//
//   [FIJO]          solo cambia editando este archivo y volviendo a compilar/cargar.
//   [SERIE: x]      es el valor INICIAL; en funcionamiento se cambia con el comando 'x' por el
//                   puerto serie (115200 baud). OJO: lo cambiado por serie NO se guarda en EEPROM,
//                   al resetear o desenchufar el Nano vuelve al valor de este archivo.
//
// Resumen de comandos por el puerto serie (detalle en lib/comandos/comandos.c):
//   p <v>   Kp  [% / rpm]          -> pid.kp   (control.c)   valor inicial KP_INICIAL
//   i <v>   Ki  [% / (rpm*s)]      -> pid.ki   (control.c)   valor inicial KI_INICIAL
//   d <v>   Kd  [% * s / rpm]      -> pid.kd   (control.c)   valor inicial KD_INICIAL
//   s <rpm> setpoint               -> sp       (control.c)   0 .. RPM_SP_MAX (también con el encoder)
//   m <%>   salida manual          -> u_manual (control.c)   U_MIN .. U_MAX (lazo abierto)
//   a z|c|k aplica ganancias sugeridas por 't' o 'z' -> pid.kp/ki/kd
//   g / x   PID ON / parar (también el botón del encoder)
//   l <ms>  una línea del log cada <ms> -> log_periodo (control.c)  valor inicial LOG_PERIODO_MS
//   t, z, l, ?  ensayo, Ziegler-Nichols, log ON/OFF, ayuda (no cambian constantes por sí solos)
//   e           diagnóstico del AS5600 / bus I2C (no cambia nada; con el motor quieto)
//=======================================================================================================

#ifndef CONFIG_H // guarda de inclusión: evita que el archivo se procese dos veces
#define CONFIG_H

#ifdef __cplusplus
extern "C" { // main.cpp es C++ y el resto es C: así los nombres se enlazan bien
#endif

#include <Arduino.h> // registros del ATmega328P (DDRB, PORTB, ...) y tipos uint8_t, etc.

//--------------------//
#ifndef set_bit
#define set_bit(SFR, bit) SFR |= (1 << bit)    // pone en 1 el bit 'bit' del registro SFR
#define clear_bit(SFR, bit) SFR &= ~(1 << bit) // pone en 0 el bit 'bit' del registro SFR
#endif
//--------------------//

//-----------PINES-----------// [FIJO] (dependen de la PCB)
#define MOSFET_PIN PB1 // D9 = OC1A (Timer1 PWM) -> gate del IRLZ44N a través de R2 (470 ohm)
#define LED_PIN PB5    // D13, LED de la placa: fijo = motor habilitado, parpadea = falla
// SDA = A4 (PC4), SCL = A5 (PC5) -> TWI por hardware (AS5600 en J3 y LCD en J6)
// Encoder: A (CLK) = D4 (PD4, PCINT20), B (DT) = D5 (PD5, PCINT21), botón (SW) = D3 (PD3) -> ver encoder.h

//--------TEMPORIZACIÓN-------// [FIJO]
#define TS_PID_MS 20                  // período del lazo PID [ms] (cada cuánto se calcula la salida)
#define TS_PID_S (TS_PID_MS / 1000.0f) // el mismo período en segundos, lo usa el PID para integrar/derivar
#define TS_SENSOR_MS 2                // período de lectura del AS5600 [ms]
// OJO: entre dos lecturas el eje no puede girar más de media vuelta:
// rpm_max = 30000 / TS_SENSOR_MS  (2 ms -> 15000 rpm)

//----------SENSOR-----------// [FIJO]
#define SENTIDO_SENSOR 1           // poner -1 si la velocidad da negativa (sensor montado al revés)
#define RPM_FILTRO_ALFA 0.5f       // filtro de la medición: pv += alfa*(rpm - pv) (1.0 = sin filtro)
#define RPM_SP_MAX 10000.0f        // máximo setpoint aceptado [rpm] (límite de 's' y del encoder)
#define RPM_MAX_SEGURIDAD 12000.0f // si la velocidad medida lo pasa, corta el motor (falla) [rpm]
#define MAX_ERRORES_SENSOR 10      // errores I2C seguidos -> falla "sensor AS5600"

//----------ENCODER----------// [FIJO]
#define ENC_SENTIDO 1                // poner -1 si el setpoint baja al girar en horario
#define ENC_TRANSICIONES_POR_PASO 4  // KY-040: 4 (1 ciclo por detent). Algunos EC11: 2
#define ENC_PASO_RPM 10              // cambio de setpoint por paso girando despacio [rpm]
#define ENC_PASO_RAPIDO_RPM 100      // cambio por paso girando rápido [rpm]
#define ENC_T_RAPIDO_MS 40           // pasos más seguidos que esto = giro rápido [ms]
#define ENC_T_AVISO_MS 250           // como mucho un "# sp=..." cada este tiempo [ms]
#define ENC_BTN_ANTIRREBOTE_MS 30    // el botón tiene que quedar estable este tiempo [ms]

//------------LOG------------//
// El lazo calcula cada TS_PID_MS (20 ms), pero imprimir 50 líneas por segundo no se puede leer:
// el log muestra una línea cada LOG_PERIODO_MS. No afecta al control ni al ensayo 't'.
#define LOG_PERIODO_MS 500     // [SERIE: l <ms>] una línea del log cada tantos ms (20 = todas, máx. 60000)
#define LOG_ENCABEZADO_CADA 20 // [FIJO] repite el encabezado cada tantas líneas (0 = solo al cambiar de modo)

//----------ARRANQUE---------// [FIJO]
// 1 = al encender queda en lazo cerrado con sp = 0: el motor arranca al girar el
// encoder, sin necesidad de la terminal. 0 = arranca parado (esperar 'g' o 'm').
#define ARRANQUE_EN_PID 1

//------------PID------------//
#define U_MIN 0.0f     // [FIJO] salida mínima [%] (límite del PID y de 'm' / 't')
#define U_MAX 100.0f   // [FIJO] salida máxima [%] (límite del PID y de 'm' / 't')
#define PID_TF_S 0.05f // [FIJO] constante del filtro pasa-bajos de la derivada [s]
#define KP_INICIAL 0.02f // [SERIE: p <v>] Kp al encender [% / rpm]: 1000 rpm de error -> 20 % de PWM
#define KI_INICIAL 0.02f // [SERIE: i <v>] Ki al encender [% / (rpm*s)] sin integral el motor no vence el roce de la bomba
#define KD_INICIAL 0.0f  // [SERIE: d <v>] Kd al encender [% * s / rpm]

void config_timer0(void); // Timer0 como base de tiempo de 1 ms (ver config.c)
void config_IO(void);     // pines del MOSFET y del LED como salidas, en 0
uint32_t tiempo_ms(void); // get_time() leído de forma atómica [ms desde el encendido]

#ifdef __cplusplus
}
#endif

#endif // CONFIG_H
