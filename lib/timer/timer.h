#ifndef __TIMER_H // guarda de inclusión
#define __TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************************************************************************
    INCLUDES
*********************************************************************************************************/
    #include <Arduino.h>

/*********************************************************************************************************
    DEFINES
*********************************************************************************************************/
    // Nada de este módulo se cambia por el puerto serie. [FIJO]

/*********************************************************************************************************
    PROTOTIPOS DE FUNCIONES
*********************************************************************************************************/
    /* devuelve el tiempo en milisegundos(32bit) desde que arranco el timer0 */
    uint32_t get_time(void);           // OJO: leerlo con tiempo_ms() (config.c), que lo hace de forma atómica
    void set_time(uint32_t time);      // pisa el contador (no se usa)

    uint8_t get_valor(void);           // getter genérico (no se usa en este proyecto)
    void set_valor(uint8_t new_valor); // setter genérico (no se usa en este proyecto)

#ifdef __cplusplus
}
#endif
#endif // __TIMER_H
