//=======================================================================================================
// comandos.c - Intérprete de comandos del puerto serie + encoder (setpoint) + botón (PID ON/OFF)
//=======================================================================================================
// Terminal: 115200 baud, 8N1. Cada comando es UNA línea que termina en Enter (\r o \n).
// Mayúsculas o minúsculas da igual. Se acepta coma o punto decimal ("p 0,05" = "p 0.05").
// Respuestas: "# OK", "# ERROR" o el estado actual. Las líneas que empiezan con '#' son
// comentarios; el resto es el log CSV: t_ms,sp,rpm,u_pct
//
// ¿QUÉ CAMBIA CADA COMANDO?  (variable que modifica -> dónde vive -> rango / unidades)
// ---------------------------------------------------------------------------------------------------
//  ? o h        no cambia nada: imprime la ayuda y el estado (modo, sp, rpm, u, Kp, Ki, Kd)
//  p <v>        Kp  -> pid.kp (control.c, struct pid_ctrl_t)  v >= 0  [% / rpm]
//               Ej: "p 0.05": 100 rpm de error -> 5 % más de PWM.        Valor inicial: KP_INICIAL
//  i <v>        Ki  -> pid.ki                                 v >= 0  [% / (rpm*s)]
//               Ej: "i 0.02": 100 rpm de error durante 1 s -> 2 % más.   Valor inicial: KI_INICIAL
//  d <v>        Kd  -> pid.kd                                 v >= 0  [% * s / rpm]
//               Ej: "d 0.001". Actúa sobre la velocidad medida.          Valor inicial: KD_INICIAL
//  s <rpm>      setpoint -> sp (control.c) y valor_sp (encoder.c)  0 .. RPM_SP_MAX [rpm]
//               El encoder sigue sumando/restando desde este valor.
//  g            modo -> CTRL_PID (lazo cerrado). Si hay una falla responde ERROR (usar 'x').
//  x            modo -> CTRL_PARADO, PWM = 0, u_manual = 0 y borra la falla (rearma).
//  m <%>        u_manual (control.c) y modo -> CTRL_MANUAL (lazo abierto)  U_MIN .. U_MAX [%]
//  t <u0> <u1>  ens.u0 / ens.u1 y modo -> CTRL_ENSAYO: escalón de u0 a u1 [%]. Al terminar
//               calcula ganancias sugeridas (ZN y CHR) pero NO las aplica: usar "a z" o "a c".
//  z <Ku> <Tu>  calcula ganancias ZN de lazo cerrado (Ku [%/rpm], Tu [s]); NO las aplica: "a k".
//  a z|c|k      copia las ganancias sugeridas a pid.kp / pid.ki / pid.kd
//  l            log_on (control.c): prende / apaga el log CSV
//  e            no cambia nada: diagnóstico del AS5600 (niveles de SDA/SCL, barrido del bus I2C,
//               estado del imán). Solo con el motor quieto, si no responde ERROR (usar 'x').
//  encoder      girar = cambia sp (igual que 's'); pasos ENC_PASO_RPM / ENC_PASO_RAPIDO_RPM
//  botón enc.   parado -> PID ON ('g'); en marcha -> parar ('x'); con falla -> rearma ('x')
//
// IMPORTANTE: nada de esto se guarda. Al resetear el Nano las ganancias vuelven a KP_INICIAL,
// KI_INICIAL y KD_INICIAL de config.h, y el setpoint vuelve a 0. Para que un ajuste quede
// fijo hay que copiar el valor en config.h y volver a cargar el programa.
// Lo que NO tiene comando (tiempos, filtros, límites) está marcado [FIJO] en config.h.
//=======================================================================================================

#include "comandos.h" // prototipo de comandos_tarea()
#include "control.h"  // funciones que realmente cambian ganancias, setpoint y modo
#include "uart.h"     // uart_getc() para leer lo que llega y UART_PRINT() para responder
#include "config.h"   // tiempo_ms(), ENC_T_AVISO_MS
#include "encoder.h"  // enco_get_sp(), enco_set_sp(), enco_boton_presionado()
#include <stdlib.h>   // strtod(): convierte texto a número

#define LINEA_MAX 32 // [FIJO] largo máximo de un comando (incluido el 0 final)

static uint16_t sp_encoder = 0; // último setpoint tomado del encoder [rpm] (para detectar cambios)

static void ok(void) { UART_PRINT("# OK\r\n"); }       // respuesta a un comando aceptado
static void err(void) { UART_PRINT("# ERROR\r\n"); }   // comando desconocido o valor fuera de rango

// Texto que se ve al mandar '?' o 'h'
static void ayuda(void)
{
    UART_PRINT("# p/i/d <v>   Kp, Ki, Kd\r\n");
    UART_PRINT("# s <rpm>     setpoint\r\n");
    UART_PRINT("# g            PID ON (lazo cerrado)\r\n");
    UART_PRINT("# x            parar / rearmar\r\n");
    UART_PRINT("# m <%>        salida manual (lazo abierto)\r\n");
    UART_PRINT("# t <u0> <u1>  ensayo al escalon u0->u1 [%]\r\n");
    UART_PRINT("# z <Ku> <Tu>  Ziegler-Nichols lazo cerrado\r\n");
    UART_PRINT("# a <z|c|k>    aplicar ganancias sugeridas\r\n");
    UART_PRINT("# l            log ON/OFF\r\n");
    UART_PRINT("# e            diagnostico del AS5600 (motor quieto)\r\n");
    UART_PRINT("# encoder      girar = setpoint (rapido = pasos grandes)\r\n");
    UART_PRINT("# boton enc.   PID ON / parar (con falla: rearma)\r\n");
}

// lee un float del texto; avanza el puntero
static uint8_t leer_f(char **p, float *v)
{
    char *fin;                         // strtod deja acá dónde terminó el número
    float x = (float)strtod(*p, &fin); // saltea espacios y convierte "0.05", "-3", "1e-3", ...
    if (fin == *p)                     // no había ningún número
        return 0;                      // -> el comando responde ERROR
    *v = x;                            // valor leído
    *p = fin;                          // el próximo número se busca a partir de acá
    return 1;
}

// Ejecuta una línea completa recibida por el puerto serie (ver la tabla del principio)
static void ejecutar(char *l)
{
    while (*l == ' ' || *l == '\t') // saltea espacios al principio
        l++;
    if (!*l)                        // línea vacía: no hace nada
        return;
    char c = l[0];                  // la primera letra es el comando
    if (c >= 'A' && c <= 'Z')       // mayúscula -> minúscula
        c += 32;
    char *arg = l + 1;              // los argumentos vienen después de la letra
    float a, b;                     // primer y segundo argumento numérico
    ganancias_t g;                  // copia de Kp, Ki, Kd para modificar una sola

    switch (c)
    {
    case '?':
    case 'h':
        ayuda();          // lista de comandos
        control_estado(); // modo, sp, rpm, u y las ganancias que se están usando
        break;

    // --- p / i / d: CAMBIAN LAS GANANCIAS DEL PID (pid.kp, pid.ki, pid.kd en control.c) ---
    case 'p':
    case 'i':
    case 'd':
        if (!leer_f(&arg, &a) || a < 0.0f) // falta el número o es negativo
        {
            err();
            break;
        }
        control_get_ganancias(&g); // lee las 3 ganancias actuales
        if (c == 'p')
            g.kp = a;              // 'p <v>' -> Kp nuevo [% / rpm]
        else if (c == 'i')
            g.ki = a;              // 'i <v>' -> Ki nuevo [% / (rpm*s)]
        else
            g.kd = a;              // 'd <v>' -> Kd nuevo [% * s / rpm]
        control_set_ganancias(&g); // las aplica en el acto (pid_set_ganancias en pid.c), sin salto en la salida
        control_estado();          // muestra cómo quedaron
        break;

    // --- s: CAMBIA EL SETPOINT (sp en control.c y valor_sp en encoder.c) ---
    case 's':
        if (leer_f(&arg, &a) && control_set_sp(a)) // control_set_sp rechaza < 0 o > RPM_SP_MAX
        {
            // el encoder sigue desde el valor escrito por la terminal
            sp_encoder = (uint16_t)(a + 0.5f); // redondeo a rpm enteras
            enco_set_sp(sp_encoder);           // sincroniza el setpoint interno del encoder
            ok();
        }
        else
            err();
        break;

    // --- g: PID ON (modo = CTRL_PID) ---
    case 'g':
        control_iniciar_pid() ? ok() : err(); // ERROR si hay una falla activa
        break;

    // --- x: PARAR / REARMAR (modo = CTRL_PARADO, PWM = 0, borra fallas) ---
    case 'x':
        control_parar();
        ok();
        break;

    // --- m: SALIDA MANUAL, LAZO ABIERTO (u_manual en control.c) ---
    case 'm':
        (leer_f(&arg, &a) && control_manual(a)) ? ok() : err(); // a = % de PWM entre U_MIN y U_MAX
        break;

    // --- t: ENSAYO AL ESCALÓN (identifica K, tau, theta y sugiere ganancias) ---
    case 't':
        (leer_f(&arg, &a) && leer_f(&arg, &b) && control_ensayo(a, b)) ? ok() : err(); // a = u0, b = u1 [%]
        break;

    // --- z: ZIEGLER-NICHOLS DE LAZO CERRADO (solo calcula; aplicar con "a k") ---
    case 'z':
        if (leer_f(&arg, &a) && leer_f(&arg, &b) && a > 0.0f && b > 0.0f) // a = Ku, b = Tu [s]
            control_calcular_zn_cerrado(a, b);
        else
            err();
        break;

    // --- a: APLICA GANANCIAS SUGERIDAS (pisa pid.kp, pid.ki, pid.kd) ---
    case 'a':
        while (*arg == ' ')         // saltea espacios hasta la letra
            arg++;
        if (*arg >= 'A' && *arg <= 'Z')
            *arg += 32;             // mayúscula -> minúscula
        if (control_aplicar_sugerencia(*arg)) // 'z' = ZN abierto, 'c' = CHR, 'k' = ZN cerrado
            control_estado();       // muestra las ganancias nuevas
        else
            err();                  // letra inválida o todavía no se calculó esa sugerencia
        break;

    // --- e: DIAGNÓSTICO DEL AS5600 (no cambia nada; solo con el motor quieto) ---
    case 'e':
        if (!control_diagnostico()) // devuelve 0 si el motor está girando
            UART_PRINT("# ERROR: parar el motor primero (x)\r\n");
        break;

    // --- l: LOG CSV ON / OFF (log_on en control.c) ---
    case 'l':
        control_log_toggle() ? UART_PRINT("# log ON\r\n") : UART_PRINT("# log OFF\r\n");
        break;

    default: // cualquier otra letra
        err();
        break;
    }
}

// Setpoint con el encoder (encoder.c): cada clic suma/resta ENC_PASO_RPM, o
// ENC_PASO_RAPIDO_RPM si se gira rápido, entre 0 y RPM_SP_MAX.
// Es el mismo setpoint que el del comando 's'.
static void encoder_tarea(void)
{
    static uint32_t t_ult_aviso = 0; // cuándo se imprimió el último "# sp=..."
    static uint8_t aviso_pend = 0;   // 1 = hay un cambio de setpoint sin informar

    uint16_t v = enco_get_sp();      // procesa los clics acumulados por la ISR
    if (v != sp_encoder)             // se giró el encoder
    {
        sp_encoder = v;
        control_set_sp((float)v);    // CAMBIA sp en control.c (en tiempo real)
        aviso_pend = 1;
    }
    // se informa el setpoint, sin inundar la terminal mientras se gira
    uint32_t ahora = tiempo_ms();
    if (aviso_pend && (uint32_t)(ahora - t_ult_aviso) >= ENC_T_AVISO_MS)
    {
        aviso_pend = 0;
        t_ult_aviso = ahora;
        UART_PRINT("# sp=");
        uart_put_i(sp_encoder);
        UART_PRINT(" rpm\r\n");
    }
}

// Botón del encoder (D3): alterna PID ON / OFF.
//   parado              -> PID ON con el setpoint actual (igual que 'g')
//   PID, manual o ensayo -> para el motor (igual que 'x')
//   falla               -> la borra y queda parado; otra pulsación arranca el PID
static void boton_tarea(void)
{
    if (!enco_boton_presionado()) // 1 sola vez por pulsación (antirrebote en encoder.c)
        return;

    switch (control_get_modo())
    {
    case CTRL_PARADO:
        if (control_iniciar_pid())
        {
            UART_PRINT("# boton: PID ON sp=");
            uart_put_i((int32_t)(control_get_sp() + 0.5f));
            UART_PRINT(" rpm\r\n");
        }
        break;
    case CTRL_FALLA:
        control_parar();
        UART_PRINT("# boton: falla borrada, motor parado\r\n");
        break;
    default: // CTRL_PID, CTRL_MANUAL, CTRL_ENSAYO
        control_parar();
        UART_PRINT("# boton: PID OFF, motor parado\r\n");
        break;
    }
}

// Se llama en cada vuelta del while(1) de main.cpp
void comandos_tarea(void)
{
    encoder_tarea(); // setpoint desde el encoder
    boton_tarea();   // PID ON / OFF desde el botón

    static char linea[LINEA_MAX]; // línea que se está armando
    static uint8_t n = 0;         // caracteres recibidos de la línea actual
    static uint8_t desborde = 0;  // 1 = la línea superó LINEA_MAX
    int16_t c;

    while ((c = uart_getc()) >= 0) // procesa todo lo que llegó por el puerto serie
    {
        if (c == '\r' || c == '\n') // fin de línea: ejecutar el comando
        {
            if (desborde)
                err(); // línea demasiado larga: se descarta entera
            else if (n)
            {
                linea[n] = 0;     // termina el string
                ejecutar(linea);
            }
            n = 0;                // empieza una línea nueva
            desborde = 0;
        }
        else if (n < LINEA_MAX - 1)
            linea[n++] = (c == ',') ? '.' : (char)c; // acepta coma decimal
        else
            desborde = 1;
    }
}
