//=======================================================================================================
// control.h - Interfaz del lazo de control (lo que llaman comandos.c y main.cpp)
//=======================================================================================================

#ifndef CONTROL_H // guarda de inclusión
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <Arduino.h>
#include "sintonia.h" // ganancias_t

// Modo de funcionamiento (se ve como número en el comando '?': "modo=...")
typedef enum
{
    CTRL_PARADO, // 0: motor apagado                       <- 'x' / botón
    CTRL_MANUAL, // 1: lazo abierto, salida = u_manual     <- 'm <%>'
    CTRL_PID,    // 2: lazo cerrado                        <- 'g' / botón / al encender
    CTRL_ENSAYO, // 3: escalón para identificar la planta  <- 't <u0> <u1>'
    CTRL_FALLA   // 4: motor apagado por una falla; salir con 'x' o el botón
} ctrl_modo_t;

void control_init(void);  // llamar una vez al arrancar
void control_tarea(void); // llamar en el while(1)

void control_parar(void); // también borra una falla                   <- 'x'
ctrl_modo_t control_get_modo(void);
uint8_t control_iniciar_pid(void);                    // PID ON         <- 'g'
uint8_t control_set_sp(float rpm);                    // setpoint       <- 's' / encoder
float control_get_sp(void);
uint8_t control_manual(float pct);                    // lazo abierto   <- 'm'
uint8_t control_ensayo(float u0, float u1);           // ensayo         <- 't'
void control_set_ganancias(const ganancias_t *g);     // Kp, Ki, Kd     <- 'p', 'i', 'd'
void control_get_ganancias(ganancias_t *g);
void control_calcular_zn_cerrado(float ku, float tu); // sugerencia ZN  <- 'z'
uint8_t control_aplicar_sugerencia(char tipo); // 'z', 'c' o 'k'        <- 'a'
uint8_t control_log_toggle(void);                     // log ON / OFF   <- 'l'
uint8_t control_diagnostico(void); // 0 si el motor está girando          <- 'e'
void control_estado(void);                            // imprime estado <- '?'

#ifdef __cplusplus
}
#endif

#endif // CONTROL_H
