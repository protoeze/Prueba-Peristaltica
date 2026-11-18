//=======================================================================================================
// control.c - Lazo de control: lee el AS5600, calcula la velocidad, corre el PID y maneja el PWM
//=======================================================================================================
// Acá viven las variables que cambian los comandos del puerto serie (ver tabla en comandos.c):
//   pid.kp / pid.ki / pid.kd  <- 'p', 'i', 'd' y 'a z|c|k'
//   sp                        <- 's' y el encoder
//   u_manual                  <- 'm'
//   modo                      <- 'g', 'x', 'm', 't' y el botón del encoder
//   log_on                    <- 'l'
//   ens.u0 / ens.u1           <- 't'
// El comando 'e' (control_diagnostico) no cambia nada: solo muestra el estado del bus y del imán.
//=======================================================================================================

#include "control.h" // tipos ctrl_modo_t y prototipos de este archivo
#include "config.h"  // TS_PID_MS, límites, ganancias iniciales, tiempo_ms()
#include "uart.h"    // UART_PRINT, uart_put_i, uart_put_f
#include "pwm.h"     // pwm_set(), pwm_set_porcentaje(): salida al MOSFET
#include "as5600.h"  // lectura del sensor magnético de posición
#include "i2c.h"     // i2c_niveles(), i2c_probar() para el diagnóstico 'e'
#include "pid.h"     // pid_ctrl_t, pid_calcular()
#include <math.h>    // fabsf()

//-------ENSAYO AL ESCALÓN-------// [FIJO]
#define ENSAYO_N 250                         // muestras (250 * 20 ms = 5 s)
#define ENS_VENTANA_TICKS (500 / TS_PID_MS)  // cada 0,5 s revisa si se estabilizó
#define ENS_VENTANAS_MAX 20                  // espera máxima 10 s

#define MAX_SIN_IMAN 3 // [FIJO] lecturas de STATUS seguidas sin imán -> falla

typedef enum
{
    ENS_ESPERA,  // aplicando u0 hasta que la velocidad se estabilice
    ENS_ESCALON  // aplicó u1 y está grabando la respuesta
} ens_fase_t;

static struct
{
    ens_fase_t fase;      // en qué parte del ensayo está
    float u0, u1, pv_ref; // u0, u1 <- comando 't <u0> <u1>' [%]; pv_ref = velocidad de la ventana anterior
    uint16_t cnt, ventanas, n; // contadores: ticks de la ventana, ventanas esperadas, muestras grabadas
    int16_t y[ENSAYO_N];  // velocidad grabada después del escalón [rpm]
    ident_t id;           // resultado: K, tau, theta
} ens;

//-------------ESTADO-------------//
static ctrl_modo_t modo = CTRL_PARADO; // <- 'g', 'x', 'm', 't' y botón del encoder
static pid_ctrl_t pid;                 // pid.kp, pid.ki, pid.kd <- 'p', 'i', 'd' y 'a' (iniciales en config.h)
static ganancias_t sug_zn, sug_chr, sug_zc; // sugerencias calculadas por 't' (zn, chr) y 'z' (zc)
static uint8_t sug_ok = 0; // bit0 = z, bit1 = c, bit2 = k
static float sp = 0.0f,       // setpoint [rpm]          <- 's' y encoder
             pv = 0.0f,       // velocidad medida filtrada [rpm]
             u_actual = 0.0f, // salida aplicada al PWM [%]
             u_manual = 0.0f; // salida del modo manual [%] <- 'm'
static uint8_t log_on = 1;            // 1 = imprime el CSV t_ms,sp,rpm,u_pct <- 'l'
static uint8_t errores_sensor = 0;    // errores I2C seguidos
static uint8_t sin_iman = 0;          // lecturas seguidas sin imán
static const char *falla_pend = 0; // mensaje (en flash) pendiente de imprimir
static uint32_t pos_ant, t_ant, t_inicio_log; // posición y tiempo de la medición anterior; inicio del log
static uint32_t t_pid_prox, t_sensor_prox;    // próximo instante de ejecución del PID y del sensor [ms]

//------------AUXILIARES------------//
// true cuando venció el período; si el programa se atrasó, resincroniza
static uint8_t vencio(uint32_t *prox, uint16_t periodo)
{
    uint32_t ahora = tiempo_ms();
    if ((int32_t)(ahora - *prox) < 0) // todavía no llegó (resta con signo: aguanta el desborde)
        return 0;
    *prox += periodo;                 // próximo vencimiento, sin acumular atraso
    if ((int32_t)(ahora - *prox) >= 0) // se atrasó más de un período entero
        *prox = ahora + periodo;      // se resincroniza en vez de ejecutar varias veces seguidas
    return 1;
}

// espera activa (solo se usa en el arranque)
static void esperar_ms(uint16_t ms)
{
    uint32_t t = tiempo_ms();
    while ((uint32_t)(tiempo_ms() - t) < ms)
        ;
}

// imprime "Kp=... Ki=... Kd=..."
static void imprimir_gan(const ganancias_t *g)
{
    UART_PRINT("Kp=");
    uart_put_f(g->kp, 5);
    UART_PRINT(" Ki=");
    uart_put_f(g->ki, 5);
    UART_PRINT(" Kd=");
    uart_put_f(g->kd, 5);
    UART_PRINT("\r\n");
}

// cambia de modo y, si el log está prendido, imprime el encabezado del CSV
static void entrar_modo(ctrl_modo_t m)
{
    modo = m;
    t_inicio_log = tiempo_ms(); // la columna t_ms arranca en 0
    if (log_on && (m == CTRL_PID || m == CTRL_MANUAL || m == CTRL_ENSAYO))
        UART_PRINT("t_ms   -   sp   -   rpm  -  u_pct\r\n");
}

// Apaga el motor en el acto. El mensaje se imprime después, desde control_tarea():
// esta función puede ejecutarse dentro de uart_putc (lectura del sensor mientras
// espera lugar en el buffer) y ahí no se debe imprimir.
static void declarar_falla(const char *msg_P)
{
    if (modo == CTRL_FALLA) // ya estaba en falla: se queda con el primer mensaje
        return;
    modo = CTRL_FALLA;      // bloquea 'g', 'm' y 't' hasta 'x' o el botón
    pwm_set(0);             // motor apagado
    u_actual = 0.0f;
    falla_pend = msg_P;
}

// imprime la falla pendiente (fuera de uart_putc)
static void reportar_falla(void)
{
    if (!falla_pend)
        return;
    const char *m = falla_pend;
    falla_pend = 0;
    UART_PRINT("# FALLA: ");
    uart_puts_P(m);
    UART_PRINT(" (x para rearmar)\r\n");
}

// lee el ángulo del AS5600; tras MAX_ERRORES_SENSOR errores seguidos declara falla
static uint8_t leer_sensor(void)
{
    if (as5600_actualizar() == 0) // lectura correcta
    {
        errores_sensor = 0;
        return 1;
    }
    if (++errores_sensor >= MAX_ERRORES_SENSOR)
        declarar_falla(PSTR("se perdio la comunicacion I2C con el AS5600 (cables / ruido del motor)"));
    return 0;
}

// Lectura periódica del sensor (cada TS_SENSOR_MS). Se llama desde el while(1) y
// también desde uart_putc cuando el buffer TX está lleno, así una impresión larga
// no deja al AS5600 sin leer (más de media vuelta entre lecturas = error de cuenta).
static void tarea_sensor(void)
{
    static uint8_t ocupado = 0; // evita que se llame a sí misma (reentrada)
    if (ocupado)
        return;
    ocupado = 1;
    if (vencio(&t_sensor_prox, TS_SENSOR_MS))
        leer_sensor();
    ocupado = 0;
}

// Verifica que el imán siga presente (bit MD del registro STATUS)
static void chequear_iman(void)
{
    if (as5600_leer_estado() != 0)
        return; // los errores de I2C ya los cuenta leer_sensor()
    if (as5600_estado() & AS5600_MD)
        sin_iman = 0;
    else if (++sin_iman >= MAX_SIN_IMAN)
        declarar_falla(PSTR("se perdio el iman del AS5600"));
}

// una línea del CSV: t_ms,sp,rpm,u_pct
static void log_linea(uint32_t ahora)
{
    uart_put_i((int32_t)(ahora - t_inicio_log)); // t_ms desde que se entró al modo
    uart_putc(',');
    uart_put_f(modo == CTRL_PID ? sp : 0.0f, 1); // sp (solo tiene sentido en PID)
    uart_putc(',');
    uart_put_f(pv, 1);                           // rpm medidas
    uart_putc(',');
    uart_put_f(u_actual, 1);                     // % de PWM aplicado
    UART_PRINT("\r\n");
}

//------------ENSAYO------------//
// Al terminar el ensayo: identifica el modelo y calcula ganancias ZN y CHR (no las aplica)
static void ensayo_reportar(void)
{
    uint8_t r = sint_identificar(ens.y, ENSAYO_N, TS_PID_S, ens.u1 - ens.u0, &ens.id);
    UART_PRINT("# --- ENSAYO AL ESCALON ---\r\n");
    if (r == SINT_ERR_DATOS || r == SINT_ERR_SIN_RESP)
    {
        UART_PRINT("# ERROR: el motor casi no respondio, subir el escalon\r\n");
        return;
    }
    if (r == SINT_ERR_SIGNO)
    {
        UART_PRINT("# ERROR: ganancia negativa, invertir SENTIDO_SENSOR\r\n");
        return;
    }
    if (r == SINT_AVISO_NO_ESTAC)
        UART_PRINT("# AVISO: no llego al estacionario, resultados dudosos\r\n");

    UART_PRINT("# K=");
    uart_put_f(ens.id.K, 3);
    UART_PRINT(" rpm/%  tau=");
    uart_put_f(ens.id.tau, 3);
    UART_PRINT(" s  theta=");
    uart_put_f(ens.id.theta, 3);
    UART_PRINT(" s\r\n");

    sint_zn_abierto(&ens.id, &sug_zn); // sugerencia "a z"
    sint_chr_0(&ens.id, &sug_chr);     // sugerencia "a c"
    sug_ok |= 0x03;                    // habilita "a z" y "a c"
    UART_PRINT("# ZN  : ");
    imprimir_gan(&sug_zn);
    UART_PRINT("# CHR : ");
    imprimir_gan(&sug_chr);
    UART_PRINT("# usar: a z | a c\r\n");
}

// un paso del ensayo (cada TS_PID_MS); devuelve la salida a aplicar [%]
static float ensayo_tick(void)
{
    if (ens.fase == ENS_ESPERA) // 1ra parte: u0 hasta que la velocidad se estabilice
    {
        if (++ens.cnt >= ENS_VENTANA_TICKS) // cada 0,5 s
        {
            ens.cnt = 0;
            float tol = 0.01f * fabsf(pv); // estable = cambió menos del 1 %...
            if (tol < 2.0f)
                tol = 2.0f;                // ...o de 2 rpm
            uint8_t estable = (ens.ventanas > 0) && (fabsf(pv - ens.pv_ref) <= tol);
            ens.pv_ref = pv;
            ens.ventanas++;
            if (estable || ens.ventanas >= ENS_VENTANAS_MAX)
            {
                ens.fase = ENS_ESCALON; // pasa a aplicar u1
                ens.n = 0;
            }
        }
        return ens.u0;
    }

    // el primer y[0] es la velocidad justo antes de aplicar el escalón
    ens.y[ens.n++] = (int16_t)(pv < 0.0f ? 0.0f : pv);
    if (ens.n >= ENSAYO_N) // grabó las 250 muestras (5 s)
    {
        // primero se cambia de modo: si aparece una falla mientras se imprime
        // el reporte, no tiene que quedar pisada por CTRL_PARADO
        entrar_modo(CTRL_PARADO);
        ensayo_reportar();
        return 0.0f;
    }
    return ens.u1;
}

//--------------LAZO--------------//
// Se ejecuta cada TS_PID_MS (20 ms): mide la velocidad, calcula la salida y la aplica
static void tick_pid(void)
{
    uint32_t ahora = tiempo_ms();
    if (!leer_sensor()) // sin lectura válida no se toca la salida
        return;

    // velocidad = cuentas recorridas / tiempo real transcurrido
    uint32_t pos = as5600_posicion(); // cuentas acumuladas (4096 por vuelta)
    uint32_t dt = ahora - t_ant;      // [ms]
    if (dt == 0)
        return;
    float rpm = (float)(int32_t)(pos - pos_ant) * (60000.0f / AS5600_CUENTAS) / (float)dt;
    pos_ant = pos;
    t_ant = ahora;
    pv += RPM_FILTRO_ALFA * (rpm - pv); // filtro pasa-bajos de 1er orden de la medición

    if (fabsf(pv) > RPM_MAX_SEGURIDAD)  // protección por sobrevelocidad
        declarar_falla(PSTR("sobrevelocidad"));
    chequear_iman();

    float u = 0.0f; // salida a aplicar [%]
    switch (modo)
    {
    case CTRL_MANUAL:
        u = u_manual;                     // lazo abierto: lo que se pidió con 'm'
        break;
    case CTRL_PID:
        u = pid_calcular(&pid, sp, pv);   // lazo cerrado: usa pid.kp, pid.ki, pid.kd
        break;
    case CTRL_ENSAYO:
        u = ensayo_tick();                // u0 / u1 del comando 't'
        break;
    default: // PARADO / FALLA
        u = 0.0f;
        break;
    }
    if (modo == CTRL_FALLA) // por si la falla apareció durante este mismo tick
        u = 0.0f;
    u_actual = u;
    pwm_set_porcentaje(u); // % -> registro OCR1A del Timer1 (D9)

    if (log_on && (modo == CTRL_PID || modo == CTRL_MANUAL || modo == CTRL_ENSAYO))
        log_linea(ahora);
}

// LED de la placa: apagado = parado, fijo = motor habilitado, parpadeo = falla
static void led_estado(void)
{
    static uint32_t t_led = 0;
    if (modo == CTRL_FALLA)
    {
        if (vencio(&t_led, 250))
            PINB = (1 << LED_PIN); // escribir 1 en PINx invierte el pin
    }
    else if (modo == CTRL_PARADO)
        clear_bit(PORTB, LED_PIN);
    else
        set_bit(PORTB, LED_PIN);
}

// Se llama en cada vuelta del while(1) de main.cpp
void control_tarea(void)
{
    if (vencio(&t_pid_prox, TS_PID_MS)) // cada 20 ms: lazo de control
        tick_pid();
    tarea_sensor();                     // cada 2 ms: lectura del AS5600
    reportar_falla();
    led_estado();
}

//--------------API--------------//
// Arranque: PID con las ganancias de config.h, inicia el AS5600 y (opcional) entra en PID
void control_init(void)
{
    pid_init(&pid, KP_INICIAL, KI_INICIAL, KD_INICIAL, TS_PID_S, U_MIN, U_MAX); // ganancias iniciales
    pid_set_filtro_d(&pid, PID_TF_S);

    UART_PRINT("# PID de velocidad - Nano + AS5600 + MOSFET D9\r\n");
    uint8_t e = 1;
    for (uint8_t i = 0; i < 5 && e; i++) // hasta 5 intentos de encontrar el sensor
    {
        e = as5600_init();
        if (e)
            esperar_ms(10);
    }
    if (e == AS5600_ERR_IMAN)
        declarar_falla(PSTR("el AS5600 responde pero no detecta el iman"));
    else if (e)
        declarar_falla(PSTR("el AS5600 no responde por I2C (SDA=A4, SCL=A5, VCC, GND)"));
    else
    {
        UART_PRINT("# AS5600 OK\r\n");
        if (as5600_estado() & AS5600_ML)
            UART_PRINT("# AVISO: iman debil\r\n");
        if (as5600_estado() & AS5600_MH)
            UART_PRINT("# AVISO: iman muy fuerte\r\n");
    }
    pos_ant = as5600_posicion();          // punto de partida para medir velocidad
    t_ant = tiempo_ms();
    t_pid_prox = t_ant + TS_PID_MS;
    t_sensor_prox = t_ant + TS_SENSOR_MS;
    uart_set_espera(tarea_sensor);        // el sensor se sigue leyendo mientras se imprime
    reportar_falla();
    UART_PRINT("# ? = ayuda, e = diagnostico del AS5600\r\n");
#if ARRANQUE_EN_PID
    if (control_iniciar_pid()) // con sp = 0 la salida es 0 hasta girar el encoder
        UART_PRINT("# PID ON: girar el encoder para subir el setpoint\r\n");
#endif
}

// Comando 'x' y botón: para el motor y borra la falla
void control_parar(void)
{
    errores_sensor = 0;
    sin_iman = 0;
    falla_pend = 0;
    u_manual = 0.0f;  // el próximo 'm' arranca de cero
    u_actual = 0.0f;
    pwm_set(0);       // motor apagado
    entrar_modo(CTRL_PARADO);
}

// modo actual (lo usa el botón del encoder)
ctrl_modo_t control_get_modo(void)
{
    return modo;
}

// Comando 'g' y botón: PID ON
uint8_t control_iniciar_pid(void)
{
    if (modo == CTRL_FALLA) // con falla hay que rearmar primero ('x')
        return 0;
    pid_reset(&pid, sp, pv, u_actual); // arranca sin salto
    entrar_modo(CTRL_PID);
    return 1;
}

// Comando 's' y encoder: CAMBIA EL SETPOINT
uint8_t control_set_sp(float rpm)
{
    if (rpm < 0.0f || rpm > RPM_SP_MAX) // fuera de rango -> ERROR
        return 0;
    sp = rpm;
    return 1;
}

// setpoint actual [rpm]
float control_get_sp(void)
{
    return sp;
}

// Comando 'm <%>': CAMBIA u_manual y pasa a lazo abierto
uint8_t control_manual(float pct)
{
    if (modo == CTRL_FALLA || pct < U_MIN || pct > U_MAX)
        return 0;
    u_manual = pct;
    if (modo != CTRL_MANUAL)
        entrar_modo(CTRL_MANUAL);
    return 1;
}

// Comando 't <u0> <u1>': arranca el ensayo al escalón
uint8_t control_ensayo(float u0, float u1)
{
    if (modo == CTRL_FALLA || u0 < U_MIN || u1 > U_MAX || u1 < U_MIN || u0 > U_MAX || u0 == u1)
        return 0;
    ens.fase = ENS_ESPERA;
    ens.u0 = u0;
    ens.u1 = u1;
    ens.cnt = 0;
    ens.ventanas = 0;
    ens.pv_ref = 0.0f;
    ens.n = 0;
    entrar_modo(CTRL_ENSAYO);
    return 1;
}

// Comandos 'p', 'i', 'd' y 'a': CAMBIAN pid.kp, pid.ki, pid.kd
void control_set_ganancias(const ganancias_t *g)
{
    pid_set_ganancias(&pid, g->kp, g->ki, g->kd);
}

// copia las ganancias en uso (para modificar una sola)
void control_get_ganancias(ganancias_t *g)
{
    g->kp = pid.kp;
    g->ki = pid.ki;
    g->kd = pid.kd;
}

// Comando 'z <Ku> <Tu>': calcula ZN de lazo cerrado (no las aplica: "a k")
void control_calcular_zn_cerrado(float ku, float tu)
{
    sint_zn_cerrado(ku, tu, &sug_zc);
    sug_ok |= 0x04; // habilita "a k"
    UART_PRINT("# ZN cerrado: ");
    imprimir_gan(&sug_zc);
    UART_PRINT("# usar: a k\r\n");
}

// Comando 'a z|c|k': copia una sugerencia a pid.kp, pid.ki, pid.kd
uint8_t control_aplicar_sugerencia(char tipo)
{
    const ganancias_t *g = 0;
    if (tipo == 'z' && (sug_ok & 0x01))      // ZN curva de reacción (de 't')
        g = &sug_zn;
    else if (tipo == 'c' && (sug_ok & 0x02)) // CHR 0 % sobreimpulso (de 't')
        g = &sug_chr;
    else if (tipo == 'k' && (sug_ok & 0x04)) // ZN lazo cerrado (de 'z')
        g = &sug_zc;
    if (!g)                                  // no existe o no se calculó todavía
        return 0;
    control_set_ganancias(g);
    return 1;
}

// imprime un byte en hexadecimal: 0x36
static void uart_put_hex8(uint8_t v)
{
    const char *h = "0123456789ABCDEF";
    UART_PRINT("0x");
    uart_putc(h[v >> 4]);   // nibble alto
    uart_putc(h[v & 0x0F]); // nibble bajo
}

// Comando 'e'. Diagnóstico del sensor: busca dispositivos en el bus I2C y muestra el estado
// del imán. Solo con el motor quieto (el barrido deja de leer el sensor un rato).
// No cambia ninguna ganancia ni constante.
uint8_t control_diagnostico(void)
{
    if (u_actual > 0.0f || fabsf(pv) > 30.0f) // motor andando -> el comando responde ERROR
        return 0;

    // en reposo las dos líneas tienen que estar en 1 (pull-ups)
    uint8_t sda, scl;
    i2c_niveles(&sda, &scl);
    UART_PRINT("# lineas en reposo: SDA(A4)=");
    uart_putc(sda ? '1' : '0');
    UART_PRINT(" SCL(A5)=");
    uart_putc(scl ? '1' : '0');
    UART_PRINT("\r\n");
    if (!sda || !scl)
        UART_PRINT("# una linea esta en 0: corto a GND, sensor sin alimentacion o cable en otro pin\r\n");

    UART_PRINT("# I2C:");
    uint8_t hay = 0, hay_as5600 = 0;  // algún dispositivo / el AS5600 en particular
    for (uint8_t d = 0x08; d < 0x78; d++) // barre todas las direcciones válidas de 7 bits
        if (i2c_probar(d) == 0)            // respondió con ACK: lo imprime
        {
            uart_putc(' ');
            uart_put_hex8(d);
            hay = 1;
            if (d == AS5600_DIR)
                hay_as5600 = 1;
        }
    if (!hay)
    {
        UART_PRINT(" ningun dispositivo -> revisar SDA=A4, SCL=A5, VCC, GND y pull-ups\r\n");
        return 1;
    }
    UART_PRINT("\r\n");
    if (!hay_as5600)
    {
        UART_PRINT("# no hay nada en ");
        uart_put_hex8(AS5600_DIR);
        UART_PRINT(" (si es AS5600L, poner AS5600_DIR 0x40 en as5600.h)\r\n");
        return 1;
    }

    uint8_t agc;       // ganancia automática del AS5600
    uint16_t mag, ang; // magnitud del campo y ángulo crudo
    if (as5600_leer_diag(&agc, &mag, &ang))
    {
        UART_PRINT("# AS5600: responde a la direccion pero fallan las lecturas (ruido / pull-ups)\r\n");
        return 1;
    }
    uint8_t st = as5600_estado();
    UART_PRINT("# AS5600: MD=");
    uart_putc((st & AS5600_MD) ? '1' : '0');
    UART_PRINT(" ML=");
    uart_putc((st & AS5600_ML) ? '1' : '0');
    UART_PRINT(" MH=");
    uart_putc((st & AS5600_MH) ? '1' : '0');
    UART_PRINT(" AGC=");
    uart_put_i(agc);
    UART_PRINT(" MAG=");
    uart_put_i(mag);
    UART_PRINT(" ang=");
    uart_put_i(ang);
    UART_PRINT("\r\n");
    if (!(st & AS5600_MD))
        UART_PRINT("# sin iman: centrarlo sobre el chip a 0,5-3 mm (iman diametral)\r\n");
    else if (st & AS5600_ML)
        UART_PRINT("# iman debil: acercarlo\r\n");
    else if (st & AS5600_MH)
        UART_PRINT("# iman muy fuerte: alejarlo\r\n");
    else
        UART_PRINT("# iman OK. Girar el eje a mano y repetir 'e': ang tiene que cambiar\r\n");
    return 1;
}

// Comando 'l': prende / apaga el log CSV
uint8_t control_log_toggle(void)
{
    log_on = !log_on;
    return log_on;
}

// Comando '?' y después de 'p', 'i', 'd', 'a': imprime el estado y las ganancias en uso
void control_estado(void)
{
    UART_PRINT("# modo=");
    uart_put_i(modo); // 0 parado, 1 manual, 2 pid, 3 ensayo, 4 falla
    UART_PRINT("   sp=   ");
    uart_put_f(sp, 1);
    UART_PRINT("   rpm=");
    uart_put_f(pv, 1);
    UART_PRINT("    u=");
    uart_put_f(u_actual, 1);
    UART_PRINT("%\r\n# ");
    ganancias_t g;
    control_get_ganancias(&g);
    imprimir_gan(&g);
}
