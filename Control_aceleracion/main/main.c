#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"

// =====================================================
// PINES: ESP32-WROOM-32 clasico
// =====================================================

#define PIN_RPWM       25
#define PIN_LPWM       26
#define PIN_ENABLE     27

#define PIN_ENC_A      32
#define PIN_ENC_B      33

// =====================================================
// MOTOR Y ENCODER
// =====================================================

#define MOTOR_RPM_MAX       330.0f
#define CONTEOS_POR_REV     1320.0f

#define PWM_MAX             1023
#define PWM_LIMITE          500

#define PWM_FREQ            1000
#define PERIODO_MS          100
#define DT                  (PERIODO_MS / 1000.0f)

// Control PI
#define KP                  1.2f
#define KI                  0.8f
#define INTEGRAL_MAX        300.0f

// Limite duro de seguridad para cualquier referencia (RPM)
#define RPM_LIMITE_DURO     150.0f

// =====================================================
// PERFILES
// =====================================================

typedef enum {
    PERFIL_CONSTANTE = 1,
    PERFIL_RAMPA,
    PERFIL_TRAPEZOIDAL,
    PERFIL_SENOIDAL,
    PERFIL_ESCALONADO,
    PERFIL_EXPONENCIAL,
    PERFIL_GAUSSIANO,
    PERFIL_SIERRA,
    PERFIL_RUIDO,
    PERFIL_TOTAL            // siempre al final
} perfil_t;

static const char *NOMBRES_PERFIL[] = {
    "", "constante", "rampa", "trapezoidal", "senoidal",
    "escalonado", "exponencial", "gaussiano", "sierra", "ruido"
};

// =====================================================
// ESTADO COMPARTIDO (modificado por la tarea de comandos)
// =====================================================

static volatile perfil_t perfil_activo = PERFIL_GAUSSIANO;
static volatile bool     en_marcha     = false;   // arranca detenido
static volatile bool     reiniciar     = false;   // pide reiniciar t=0
static volatile float    rpm_prueba_max = 120.0f;
static volatile float    rpm_constante  = 60.0f;

static volatile TickType_t inicio_perfil = 0;
static volatile int ruido_ultimo_bloque = -1;

static const char *TAG = "CONTROL_MOTOR";
static pcnt_unit_handle_t encoder_unit;

// =====================================================
// UTILIDADES
// =====================================================

static float limitar(float x, float minimo, float maximo)
{
    if (x < minimo) return minimo;
    if (x > maximo) return maximo;
    return x;
}

static float tiempo_perfil(void)
{
    return (float)(xTaskGetTickCount() - inicio_perfil)
           / (float)configTICK_RATE_HZ;
}

// =====================================================
// GENERADOR DE LOS NUEVE PERFILES
// =====================================================

static float generar_referencia(perfil_t perfil, float t)
{
    const float vmax = rpm_prueba_max;

    switch (perfil) {

        case PERFIL_CONSTANTE:
            return rpm_constante;

        case PERFIL_RAMPA: {
            float pendiente = vmax / 10.0f;
            return limitar(pendiente * t, 0.0f, vmax);
        }

        case PERFIL_TRAPEZOIDAL: {
            if (t < 5.0f) {
                return vmax * t / 5.0f;
            } else if (t < 10.0f) {
                return vmax;
            } else if (t < 15.0f) {
                return vmax * (15.0f - t) / 5.0f;
            }
            return 0.0f;
        }

        case PERFIL_SENOIDAL: {
            const float base = 60.0f;
            const float amplitud = 40.0f;
            const float frecuencia = 0.1f;
            return base + amplitud * sinf(2.0f * (float)M_PI
                                          * frecuencia * t);
        }

        case PERFIL_ESCALONADO: {
            unsigned fase = ((unsigned)(t / 5.0f)) % 2;
            return fase == 0 ? 40.0f : 100.0f;
        }

        case PERFIL_EXPONENCIAL: {
            const float tau = 4.0f;
            return vmax * (1.0f - expf(-t / tau));
        }

        case PERFIL_GAUSSIANO: {
            float fase = fmodf(t, 15.0f);
            float x = (fase - 7.5f) / 2.0f;
            return vmax * expf(-0.5f * x * x);
        }

        case PERFIL_SIERRA: {
            float fase = fmodf(t, 10.0f);
            return vmax * fase / 10.0f;
        }

        case PERFIL_RUIDO: {
            static float valor = 0.0f;
            int bloque = (int)(t / 0.5f);

            if (bloque != ruido_ultimo_bloque) {
                ruido_ultimo_bloque = bloque;
                uint32_t aleatorio = esp_random();
                valor = 30.0f +
                    ((float)(aleatorio % 9001) / 100.0f);
            }

            return limitar(valor, 30.0f, vmax);
        }

        default:
            return 0.0f;
    }
}

// =====================================================
// PWM Y PUENTE H
// =====================================================

static void detener_motor(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    gpio_set_level(PIN_LPWM, 0);
}

static void establecer_pwm(uint32_t duty)
{
    if (duty > PWM_LIMITE) duty = PWM_LIMITE;

    gpio_set_level(PIN_LPWM, 0);

    ESP_ERROR_CHECK(
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty)
    );
    ESP_ERROR_CHECK(
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0)
    );
}

static void configurar_pwm(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t pwm_cfg = {
        .gpio_num = PIN_RPWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_cfg));

    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << PIN_LPWM) |
                        (1ULL << PIN_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));

    gpio_set_level(PIN_ENABLE, 0);
    gpio_set_level(PIN_LPWM, 0);
    detener_motor();
}

// =====================================================
// ENCODER: PCNT EN CUADRATURA
// =====================================================

static void configurar_encoder(void)
{
    pcnt_unit_config_t unit_cfg = {
        .low_limit = -30000,
        .high_limit = 30000
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &encoder_unit));

    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = 1000
    };
    ESP_ERROR_CHECK(
        pcnt_unit_set_glitch_filter(encoder_unit, &filter_cfg)
    );

    pcnt_chan_config_t cfg_a = {
        .edge_gpio_num = PIN_ENC_A,
        .level_gpio_num = PIN_ENC_B
    };
    pcnt_chan_config_t cfg_b = {
        .edge_gpio_num = PIN_ENC_B,
        .level_gpio_num = PIN_ENC_A
    };

    pcnt_channel_handle_t chan_a;
    pcnt_channel_handle_t chan_b;

    ESP_ERROR_CHECK(pcnt_new_channel(encoder_unit, &cfg_a, &chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(encoder_unit, &cfg_b, &chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        chan_a,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE
    ));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    ));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        chan_b,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE
    ));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    ));

    ESP_ERROR_CHECK(pcnt_unit_enable(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(encoder_unit));

    ESP_LOGI(TAG, "Encoder: %.0f conteos por revolucion",
             CONTEOS_POR_REV);
}

// =====================================================
// CONSOLA SERIAL: comandos en tiempo de ejecucion
// =====================================================

static void imprimir_ayuda(void)
{
    printf("\n--- COMANDOS ---\n");
    printf("stop            detiene el motor\n");
    printf("run             arranca el perfil desde t=0\n");
    printf("N  o  p N       selecciona perfil por numero (1-%d)\n",
           PERFIL_TOTAL - 1);
    printf("<nombre>        selecciona perfil por nombre\n");
    printf("vmax X          velocidad maxima de perfiles (RPM)\n");
    printf("cte X           RPM del perfil constante\n");
    printf("estado          muestra la configuracion\n");
    printf("Perfiles:");
    for (int i = 1; i < PERFIL_TOTAL; i++) {
        printf(" %d=%s", i, NOMBRES_PERFIL[i]);
    }
    printf("\n----------------\n");
}

static void imprimir_estado(void)
{
    printf("[estado] %s | perfil=%d (%s) | vmax=%.0f | cte=%.0f\n",
           en_marcha ? "EN MARCHA" : "DETENIDO",
           (int)perfil_activo, NOMBRES_PERFIL[perfil_activo],
           rpm_prueba_max, rpm_constante);
}

static void seleccionar_perfil(int n)
{
    if (n < 1 || n >= PERFIL_TOTAL) {
        printf("[error] perfil invalido: %d\n", n);
        return;
    }
    perfil_activo = (perfil_t)n;
    printf("[ok] perfil %d (%s)%s\n", n, NOMBRES_PERFIL[n],
           en_marcha ? " - reiniciando" : " - escribe 'run'");
    if (en_marcha) {
        reiniciar = true;   // cambio en caliente
    }
}

static void procesar_comando(const char *cmd)
{
    float x;
    int n;

    if (strcmp(cmd, "stop") == 0 || strcmp(cmd, "s") == 0) {
        en_marcha = false;
        printf("[ok] motor detenido\n");

    } else if (strcmp(cmd, "run") == 0 || strcmp(cmd, "r") == 0) {
        reiniciar = true;
        en_marcha = true;
        printf("[ok] arrancando perfil %d (%s)\n",
               (int)perfil_activo, NOMBRES_PERFIL[perfil_activo]);

    } else if (sscanf(cmd, "vmax %f", &x) == 1) {
        rpm_prueba_max = limitar(x, 10.0f, RPM_LIMITE_DURO);
        printf("[ok] vmax = %.0f RPM\n", rpm_prueba_max);

    } else if (sscanf(cmd, "cte %f", &x) == 1) {
        rpm_constante = limitar(x, 10.0f, RPM_LIMITE_DURO);
        printf("[ok] cte = %.0f RPM\n", rpm_constante);

    } else if (sscanf(cmd, "p %d", &n) == 1) {
        seleccionar_perfil(n);

    } else if (strcmp(cmd, "estado") == 0) {
        imprimir_estado();

    } else if (strcmp(cmd, "ayuda") == 0 || strcmp(cmd, "?") == 0
               || strcmp(cmd, "help") == 0) {
        imprimir_ayuda();

    } else if (isdigit((unsigned char)cmd[0])) {
        seleccionar_perfil(atoi(cmd));

    } else {
        // Buscar por nombre (coincide con el inicio: "gauss" -> gaussiano)
        size_t len = strlen(cmd);
        for (int i = 1; i < PERFIL_TOTAL; i++) {
            if (len >= 3 && strncmp(cmd, NOMBRES_PERFIL[i], len) == 0) {
                seleccionar_perfil(i);
                return;
            }
        }
        printf("[error] comando desconocido: '%s' (escribe 'ayuda')\n",
               cmd);
    }
}

static void tarea_consola(void *arg)
{
    char linea[48];
    size_t n = 0;

    for (;;) {
        uint8_t c;
        int r = uart_read_bytes(UART_NUM_0, &c, 1, pdMS_TO_TICKS(50));
        if (r <= 0) continue;

        if (c == '\r' || c == '\n') {
            if (n > 0) {
                linea[n] = '\0';
                procesar_comando(linea);
                n = 0;
            }
        } else if (n < sizeof(linea) - 1) {
            linea[n++] = (char)tolower(c);
        }
    }
}

// =====================================================
// PROGRAMA PRINCIPAL
// =====================================================

void app_main(void)
{
    configurar_pwm();
    configurar_encoder();

    // Driver UART0 solo para RECIBIR comandos. printf/log siguen
    // saliendo por la consola normal.
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0));
    xTaskCreate(tarea_consola, "consola", 4096, NULL, 3, NULL);

    ESP_LOGI(TAG, "Control PI de velocidad iniciado");
    ESP_LOGI(TAG, "PWM limitado a %u/%u",
             (unsigned)PWM_LIMITE, (unsigned)PWM_MAX);

    vTaskDelay(pdMS_TO_TICKS(1000));
    imprimir_ayuda();
    imprimir_estado();

    int conteo_anterior = 0;
    float integral = 0.0f;
    bool estaba_en_marcha = false;
    TickType_t marca_tiempo = xTaskGetTickCount();

    while (1) {
        vTaskDelayUntil(&marca_tiempo, pdMS_TO_TICKS(PERIODO_MS));

        // ---- Lectura del encoder (siempre, incluso detenido) ----
        int conteo_actual = 0;
        esp_err_t err = pcnt_unit_get_count(encoder_unit,
                                            &conteo_actual);
        if (err != ESP_OK) {
            detener_motor();
            gpio_set_level(PIN_ENABLE, 0);
            ESP_LOGE(TAG, "Fallo al leer encoder; motor detenido");
            return;
        }

        int delta = conteo_actual - conteo_anterior;
        conteo_anterior = conteo_actual;

        if (conteo_actual >= 25000 || conteo_actual <= -25000) {
            ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
            conteo_anterior = 0;
        }

        float rpm_reales = fabsf((float)delta) * 60.0f /
                           (CONTEOS_POR_REV * DT);

        // ---- Transiciones marcha/paro ----
        bool marcha = en_marcha;

        if (reiniciar) {
            reiniciar = false;
            inicio_perfil = xTaskGetTickCount();
            ruido_ultimo_bloque = -1;
            integral = 0.0f;
        }

        if (marcha && !estaba_en_marcha) {
            integral = 0.0f;
            inicio_perfil = xTaskGetTickCount();
            ruido_ultimo_bloque = -1;
            gpio_set_level(PIN_ENABLE, 1);
        } else if (!marcha && estaba_en_marcha) {
            detener_motor();
            gpio_set_level(PIN_ENABLE, 0);
            integral = 0.0f;
        }
        estaba_en_marcha = marcha;

        // ---- Calculo del control ----
        float t = 0.0f;
        float rpm_deseadas = 0.0f;
        float error = 0.0f;
        uint32_t duty = 0;

        if (marcha) {
            t = tiempo_perfil();
            rpm_deseadas = generar_referencia(perfil_activo, t);
            rpm_deseadas = limitar(rpm_deseadas, 0.0f, RPM_LIMITE_DURO);
            error = rpm_deseadas - rpm_reales;

            if (rpm_deseadas < 1.0f) {
                integral = 0.0f;
                duty = 0;
            } else {
                float pwm_base =
                    (rpm_deseadas / MOTOR_RPM_MAX) * PWM_MAX;

                integral += KI * error * DT;
                integral = limitar(integral,
                                   -INTEGRAL_MAX, INTEGRAL_MAX);

                float salida = pwm_base + KP * error + integral;
                salida = limitar(salida, 0.0f, (float)PWM_LIMITE);
                duty = (uint32_t)(salida + 0.5f);
            }
            establecer_pwm(duty);
        } else {
            error = -rpm_reales;
            // El motor ya esta detenido; se asegura PWM = 0.
            establecer_pwm(0);
        }

        float porcentaje_pwm = 100.0f * (float)duty / PWM_MAX;

        // ---- Teleplot ----
        printf(">referencia:%.2f\n", rpm_deseadas);
        printf(">real:%.2f\n", rpm_reales);
        printf(">error:%.2f\n", error);
        printf(">pwm:%.2f\n", porcentaje_pwm);
        printf(">tiempo:%.2f\n", t);
    }
}


/*#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"

// =====================================================
// PINES: ESP32-WROOM-32 clasico
// =====================================================

#define PIN_RPWM       25
#define PIN_LPWM       26
#define PIN_ENABLE     27

#define PIN_ENC_A      32
#define PIN_ENC_B      33

// =====================================================
// MOTOR Y ENCODER
// =====================================================

#define MOTOR_RPM_MAX       330.0f
#define CONTEOS_POR_REV     1320.0f

// Limite inicial conservador para las pruebas.
// 400/1023 equivale aproximadamente al 39 % del PWM.
#define PWM_MAX             1023
#define PWM_LIMITE          500

#define PWM_FREQ            1000
#define PERIODO_MS          100
#define DT                  (PERIODO_MS / 1000.0f)

// Control PI: ganancias iniciales, ajustar experimentalmente.
#define KP                  1.2f
#define KI                  0.8f
#define INTEGRAL_MAX        300.0f

// =====================================================
// SELECCION DEL PERFIL
// Cambia esta constante y vuelve a compilar.
// =====================================================

typedef enum {
    PERFIL_CONSTANTE = 1,
    PERFIL_RAMPA,
    PERFIL_TRAPEZOIDAL,
    PERFIL_SENOIDAL,
    PERFIL_ESCALONADO,
    PERFIL_EXPONENCIAL,
    PERFIL_GAUSSIANO,
    PERFIL_SIERRA,
    PERFIL_RUIDO
} perfil_t;

#define PERFIL_ACTIVO PERFIL_GAUSSIANO

// Limites de referencia para la primera prueba
#define RPM_PRUEBA_MAX      120.0f
#define RPM_CONSTANTE       60.0f

static const char *TAG = "CONTROL_MOTOR";
static pcnt_unit_handle_t encoder_unit;

// =====================================================
// UTILIDADES
// =====================================================

static float limitar(float x, float minimo, float maximo)
{
    if (x < minimo) return minimo;
    if (x > maximo) return maximo;
    return x;
}

// Tiempo desde el inicio del perfil, en segundos.
static float tiempo_perfil(void)
{
    static TickType_t inicio = 0;

    if (inicio == 0) {
        inicio = xTaskGetTickCount();
    }

    return (float)(xTaskGetTickCount() - inicio)
           / (float)configTICK_RATE_HZ;
}

// =====================================================
// GENERADOR DE LOS NUEVE PERFILES
// =====================================================

static float generar_referencia(perfil_t perfil, float t)
{
    const float vmax = RPM_PRUEBA_MAX;

    switch (perfil) {

        case PERFIL_CONSTANTE:
            return RPM_CONSTANTE;

        case PERFIL_RAMPA: {
            // 0 a 120 RPM en 10 segundos; despues mantiene.
            float pendiente = vmax / 10.0f;
            return limitar(pendiente * t, 0.0f, vmax);
        }

        case PERFIL_TRAPEZOIDAL: {
            // 0-5 s: acelera; 5-10 s: constante;
            // 10-15 s: desacelera; despues permanece en cero.
            if (t < 5.0f) {
                return vmax * t / 5.0f;
            } else if (t < 10.0f) {
                return vmax;
            } else if (t < 15.0f) {
                return vmax * (15.0f - t) / 5.0f;
            }
            return 0.0f;
        }

        case PERFIL_SENOIDAL: {
            // Oscila entre 20 y 100 RPM.
            const float base = 60.0f;
            const float amplitud = 40.0f;
            const float frecuencia = 0.1f;
            return base + amplitud * sinf(2.0f * (float)M_PI
                                          * frecuencia * t);
        }

        case PERFIL_ESCALONADO: {
            // Alterna entre 40 y 100 RPM cada 5 segundos.
            unsigned fase = ((unsigned)(t / 5.0f)) % 2;
            return fase == 0 ? 40.0f : 100.0f;
        }

        case PERFIL_EXPONENCIAL: {
            // Crecimiento suave de 0 hacia vmax.
            const float tau = 4.0f;
            return vmax * (1.0f - expf(-t / tau));
        }

        case PERFIL_GAUSSIANO: {
            // Pulso de velocidad que se repite cada 15 segundos.
            float fase = fmodf(t, 15.0f);
            float x = (fase - 7.5f) / 2.0f;
            return vmax * expf(-0.5f * x * x);
        }

        case PERFIL_SIERRA: {
            // Rampa repetitiva: 0 a vmax cada 10 segundos.
            float fase = fmodf(t, 10.0f);
            return vmax * fase / 10.0f;
        }

        case PERFIL_RUIDO: {
            // Nueva referencia aleatoria cada 500 ms.
            // Se mantiene constante entre actualizaciones.
            static int ultimo_bloque = -1;
            static float valor = 0.0f;
            int bloque = (int)(t / 0.5f);

            if (bloque != ultimo_bloque) {
                ultimo_bloque = bloque;
                uint32_t aleatorio = esp_random();
                valor = 30.0f +
                    ((float)(aleatorio % 9001) / 100.0f);
            }

            return limitar(valor, 30.0f, vmax);
        }

        default:
            return 0.0f;
    }
}

// =====================================================
// PWM Y PUENTE H
// =====================================================

static void detener_motor(void)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    gpio_set_level(PIN_LPWM, 0);
}

static void establecer_pwm(uint32_t duty)
{
    if (duty > PWM_LIMITE) duty = PWM_LIMITE;

    // Giro en un solo sentido.
    gpio_set_level(PIN_LPWM, 0);

    ESP_ERROR_CHECK(
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty)
    );
    ESP_ERROR_CHECK(
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0)
    );
}

static void configurar_pwm(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    ledc_channel_config_t pwm_cfg = {
        .gpio_num = PIN_RPWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_cfg));

    gpio_config_t gpio_cfg = {
        .pin_bit_mask = (1ULL << PIN_LPWM) |
                        (1ULL << PIN_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));

    gpio_set_level(PIN_ENABLE, 0);
    gpio_set_level(PIN_LPWM, 0);
    detener_motor();
}

// =====================================================
// ENCODER: PCNT EN CUADRATURA
// =====================================================

static void configurar_encoder(void)
{
    pcnt_unit_config_t unit_cfg = {
        .low_limit = -30000,
        .high_limit = 30000
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &encoder_unit));

    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = 1000
    };
    ESP_ERROR_CHECK(
        pcnt_unit_set_glitch_filter(encoder_unit, &filter_cfg)
    );

    pcnt_chan_config_t cfg_a = {
        .edge_gpio_num = PIN_ENC_A,
        .level_gpio_num = PIN_ENC_B
    };
    pcnt_chan_config_t cfg_b = {
        .edge_gpio_num = PIN_ENC_B,
        .level_gpio_num = PIN_ENC_A
    };

    pcnt_channel_handle_t chan_a;
    pcnt_channel_handle_t chan_b;

    ESP_ERROR_CHECK(pcnt_new_channel(encoder_unit, &cfg_a, &chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(encoder_unit, &cfg_b, &chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        chan_a,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE
    ));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    ));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        chan_b,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE
    ));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    ));

    ESP_ERROR_CHECK(pcnt_unit_enable(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(encoder_unit));

    ESP_LOGI(TAG, "Encoder: %.0f conteos por revolucion",
             CONTEOS_POR_REV);
}

// =====================================================
// PROGRAMA PRINCIPAL
// =====================================================

void app_main(void)
{
    configurar_pwm();
    configurar_encoder();

    ESP_LOGI(TAG, "Control PI de velocidad iniciado");
    ESP_LOGI(TAG, "Perfil activo: %d", (int)PERFIL_ACTIVO);
    ESP_LOGI(TAG, "Teleplot: referencia, real, error y pwm");
    ESP_LOGI(TAG, "PWM limitado inicialmente a %u/%u",
             (unsigned)PWM_LIMITE, (unsigned)PWM_MAX);

    // Mantener el driver deshabilitado durante el arranque.
    vTaskDelay(pdMS_TO_TICKS(1000));

    int conteo_anterior = 0;
    float integral = 0.0f;
    TickType_t marca_tiempo = xTaskGetTickCount();

    // Habilitar el puente solo después de inicializar todo.
    gpio_set_level(PIN_ENABLE, 1);

    while (1) {
        vTaskDelayUntil(
            &marca_tiempo,
            pdMS_TO_TICKS(PERIODO_MS)
        );

        int conteo_actual = 0;
        esp_err_t err = pcnt_unit_get_count(
            encoder_unit, &conteo_actual
        );

        if (err != ESP_OK) {
            detener_motor();
            gpio_set_level(PIN_ENABLE, 0);
            ESP_LOGE(TAG, "Fallo al leer encoder; motor detenido");
            return;
        }

        // Calcular delta incluso si el contador cruza el limite.
        int delta = conteo_actual - conteo_anterior;
        conteo_anterior = conteo_actual;

        // Evitar alcanzar los limites del contador PCNT.
        if (conteo_actual >= 25000 || conteo_actual <= -25000) {
            ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
            conteo_anterior = 0;
        }

        // Magnitud de velocidad; este programa gira en un sentido.
        float rpm_reales =
            fabsf((float)delta) * 60.0f /
            (CONTEOS_POR_REV * DT);

        float t = tiempo_perfil();
        float rpm_deseadas = generar_referencia(PERFIL_ACTIVO, t);
        rpm_deseadas = limitar(rpm_deseadas, 0.0f, RPM_PRUEBA_MAX);

        float error = rpm_deseadas - rpm_reales;
        float salida = 0.0f;
        uint32_t duty = 0;

        if (rpm_deseadas < 1.0f) {
            integral = 0.0f;
            duty = 0;
        } else {
            // Alimentacion anticipada mas PI.
            float pwm_base =
                (rpm_deseadas / MOTOR_RPM_MAX) * PWM_MAX;

            integral += KI * error * DT;
            integral = limitar(integral, -INTEGRAL_MAX, INTEGRAL_MAX);


            salida = pwm_base + KP * error + integral;
            salida = limitar(salida, 0.0f, (float)PWM_LIMITE);
            duty = (uint32_t)(salida + 0.5f);
        }

        establecer_pwm(duty);

        float porcentaje_pwm = 100.0f * (float)duty / PWM_MAX;

        // Teleplot: una variable por linea.
        printf(">referencia:%.2f\n", rpm_deseadas);
        printf(">real:%.2f\n", rpm_reales);
        printf(">error:%.2f\n", error);
        printf(">pwm:%.2f\n", porcentaje_pwm);
        printf(">tiempo:%.2f\n", t);

        // Registro de consola para diagnostico.
        ESP_LOGI(TAG,
                 "t=%.1f s | Ref=%.1f RPM | Real=%.1f RPM | "
                 "Error=%.1f | PWM=%.1f%%",
                 t, rpm_deseadas, rpm_reales,
                 error, porcentaje_pwm);
    }
}*/

