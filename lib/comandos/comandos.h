//=======================================================================================================
// comandos.h - Comandos por el puerto serie, encoder y botón
//=======================================================================================================
// La tabla de qué variable cambia cada comando (p, i, d, s, g, x, m, t, z, a, l) está al
// principio de comandos.c.

#ifndef COMANDOS_H // guarda de inclusión
#define COMANDOS_H

#ifdef __cplusplus
extern "C" { // para poder llamarla desde main.cpp (C++)
#endif

#include <Arduino.h>

void comandos_tarea(void); // llamar en el while(1): lee el puerto serie, el encoder y el botón

#ifdef __cplusplus
}
#endif

#endif // COMANDOS_H
