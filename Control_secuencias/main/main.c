// =====================================================
// CONTROL DE POSICION + PERFILES DE VELOCIDAD + SECUENCIAS
// Motor JGA25-370 + BTS7960 + ESP32 (ESP-IDF)
//
// - Cero angular por cruce de laser (homing con ISR)
// - Conteos por vuelta MEDIDOS en el homing
// - Modo POSICION: trayectoria suave + PD + freno al llegar
// - Modo VELOCIDAD: 9 perfiles (constante, rampa, ...) con PI
// - SECUENCIAS escritas por serial, por ejemplo:
//
//     60; ruido 3; rampa 5; escalon 2
//
//   = ir a 60 grados, ruido 3 s, rampa 5 s, escalon 2 s.
//
// Comandos (monitor serial o Teleplot): escribe "ayuda"
//
// Requiere en sdkconfig:
//   CONFIG_PCNT_CTRL_FUNC_IN_IRAM=y
//   CONFIG_PCNT_ISR_IRAM_SAFE=y
// =====================================================

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdarg.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// =====================================================
// 1. PINES
// =====================================================

// Laser KY-008
#define PIN_LASER       22
#define PIN_RECEPTOR    23

// Puente H BTS7960 / IBT-2
#define PIN_RPWM        25
#define PIN_LPWM        26
#define PIN_ENABLE      27

// Encoder incremental
#define PIN_ENC_A       32
#define PIN_ENC_B       33

// LED integrado
#define PIN_LED         2
#define LED_ACTIVO      1

// =====================================================
// 2. REFERENCIA OPTICA
// =====================================================

// 0 = LOW indica referencia, 1 = HIGH indica referencia
#define LASER_ACTIVE_LEVEL          0

#define LASER_MUESTRAS              5
#define LASER_INTERVALO_MS          2

// =====================================================
// 3. MOTOR Y ENCODER
// =====================================================

// Valor NOMINAL. El homing lo mide y lo reemplaza.
#define CONTEOS_POR_REV             1320.0f
#define MOTOR_RPM_MAX               330.0f

#define PCNT_LIM_ALTO               30000
#define PCNT_LIM_BAJO               (-30000)

// =====================================================
// 4. PWM Y TIEMPOS
// =====================================================

#define PWM_FREQ                    1000
#define PWM_MAX                     1023

// Limite de PWM por modo
#define PWM_LIMITE_POS              PWM_MAX
#define PWM_LIMITE_VEL              700

#define PERIODO_MS                  10
#define DT                          (PERIODO_MS / 1000.0f)

// Homing
#define HOMING_PWM                  320
#define HOMING_DIRECTION            1        // +1 RPWM, -1 LPWM
#define HOMING_RELEASE_TIMEOUT_MS   5000
#define HOMING_SEARCH_TIMEOUT_MS    20000
#define HOMING_MIN_COUNTS           ((int64_t)CONTEOS_POR_REV)
#define HOMING_MIN_SEGUNDO_CRUCE    100

// =====================================================
// 5. CONTROL DE POSICION (PD)
// =====================================================

#define KP_POS                      8.0f
#define KD_POS                      0.8f

#define ERROR_TOLERANCIA            2.0f
#define PWM_MINIMO                  260

#define VEL_MAX_DEG_S               250.0f
#define ACEL_DEG_S2                 500.0f

#define FILTRO_VEL                  0.4f
#define VEL_PARADO_DEG_S            15.0f

// 1 = frenar al llegar, 0 = motor libre
#define USAR_FRENO_ACTIVO           1

// Un paso de posicion termina cuando el error se mantiene
// dentro de tolerancia ESTABLE_CICLOS ciclos (200 ms)...
#define ESTABLE_CICLOS              20
// ...o cuando pasa este tiempo (para no quedarse atorado)
#define POS_TIMEOUT_S               15.0f

// 0 = camino mas corto hacia el angulo pedido
// 1 = siempre girar hacia adelante (sentido positivo)
#define POS_SOLO_ADELANTE           0

// =====================================================
// 6. CONTROL DE VELOCIDAD (PI)
// =====================================================

#define KP_VEL                      1.2f
#define KI_VEL                      0.8f
#define INTEGRAL_MAX                300.0f

// El PI actualiza su integral y mide RPM cada 100 ms
// (10 ciclos): con 10 ms por ciclo, 1 conteo = 4.5 RPM.
#define CICLOS_PI                   10

#define RPM_LIMITE_DURO             250.0f

// =====================================================
// 7. SERIAL Y TELEPLOT
// =====================================================

#define UART_CONSOLA        UART_NUM_0
#define UART_RX_BUF         256
#define UART_TX_BUF         2048
#define CMD_MAX             160

#define PRINT_CADA_N        5        // N * 10 ms

#define ESCALA_PASO         5
#define ESCALA_MAYOR        30

// =====================================================
// 8. MEDICION DE CORRIENTE (R_IS y L_IS del IBT-2)
// =====================================================

#define ADC_R_IS_CHANNEL        ADC_CHANNEL_6    // GPIO34
#define ADC_L_IS_CHANNEL        ADC_CHANNEL_7    // GPIO35

#define IS_ATTEN                ADC_ATTEN_DB_0
#define IS_ADC_FS_MV            1100

#define IS_RESISTENCIA_OHM      1000.0f
#define IS_KILIS                8500.0f
#define CORRIENTE_AJUSTE        1.0f

#define CORRIENTE_A_POR_V \
    ((IS_KILIS / IS_RESISTENCIA_OHM) * CORRIENTE_AJUSTE)

#define IS_RAFAGA_US            2500
#define IS_MAX_MUESTRAS         200
#define IS_UMBRAL_MV            10.0f
#define IS_PERIODO_MS           20
#define FILTRO_CORRIENTE        0.3f

#define USAR_PROTECCION_CORRIENTE   0
#define LIMITE_CORRIENTE_A          3.0f
#define LIMITE_TIEMPO_MS            300

#define GRAD_A_RAD          (3.14159265f / 180.0f)
#define PI_F                3.14159265f

static const char *TAG = "CONTROL";

// =====================================================
// 9. PERFILES DE VELOCIDAD
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

// Duracion "natural" de cada perfil (s). Si el escalado esta
// activo, el perfil se comprime/estira para que su forma
// completa ocurra dentro de la duracion pedida en el paso.
// 0 = no se escala (constante, ruido).
static const float PERIODO_NATURAL[] = {
    0.0f, 0.0f, 10.0f, 15.0f, 10.0f, 10.0f, 20.0f, 15.0f, 10.0f, 0.0f
};

// 1 = la forma completa del perfil cabe en la duracion del paso
// 0 = el perfil corre con sus tiempos originales y se corta
static volatile bool escalar_perfil = true;

static volatile float rpm_prueba_max = 120.0f;
static volatile float rpm_constante  = 60.0f;

static int   ruido_ultimo_bloque = -1;
static float ruido_valor = 0.0f;

static float limitar(float x, float minimo, float maximo)
{
    if (x < minimo) return minimo;
    if (x > maximo) return maximo;
    return x;
}

static float generar_referencia(perfil_t perfil, float t,
                                float vmax, float cte)
{
    switch (perfil) {

        case PERFIL_CONSTANTE:
            return cte;

        case PERFIL_RAMPA:
            return limitar((vmax / 10.0f) * t, 0.0f, vmax);

        case PERFIL_TRAPEZOIDAL:
            if (t < 5.0f)       return vmax * t / 5.0f;
            else if (t < 10.0f) return vmax;
            else if (t < 15.0f) return vmax * (15.0f - t) / 5.0f;
            return 0.0f;

        case PERFIL_SENOIDAL:
            return vmax * (0.55f + 0.45f * sinf(2.0f * PI_F * 0.1f * t));

        case PERFIL_ESCALONADO: {
            unsigned fase = ((unsigned)(t / 5.0f)) % 2;
            return fase == 0 ? 0.4f * vmax : vmax;
        }

        case PERFIL_EXPONENCIAL:
            return vmax * (1.0f - expf(-t / 4.0f));

        case PERFIL_GAUSSIANO: {
            float fase = fmodf(t, 15.0f);
            float x = (fase - 7.5f) / 2.0f;
            return vmax * expf(-0.5f * x * x);
        }

        case PERFIL_SIERRA:
            return vmax * fmodf(t, 10.0f) / 10.0f;

        case PERFIL_RUIDO: {
            int bloque = (int)(t / 0.5f);
            if (bloque != ruido_ultimo_bloque) {
                ruido_ultimo_bloque = bloque;
                uint32_t a = esp_random();
                ruido_valor = vmax * (0.25f + 0.75f *
                              ((float)(a % 10001) / 10000.0f));
            }
            return limitar(ruido_valor, 0.0f, vmax);
        }

        default:
            return 0.0f;
    }
}

// =====================================================
// 10. SECUENCIAS
// =====================================================

#define MAX_PASOS   16

typedef enum {
    PASO_POS,       // ir a un angulo (grados)
    PASO_PERFIL,    // aplicar un perfil durante X segundos
    PASO_ESPERA     // mantener la posicion X segundos
} tipo_paso_t;

typedef struct {
    tipo_paso_t tipo;
    perfil_t    perfil;   // solo PASO_PERFIL
    float       valor;    // angulo (PASO_POS)
    float       dur;      // segundos (PERFIL / ESPERA)
    float       param;    // vmax o rpm propios del paso (0 = global)
} paso_t;

// Secuencia cargada al arrancar (se ejecuta con "run")
static const paso_t SEC_DEFECTO[] = {
    { PASO_POS,    0,                60.0f, 0.0f, 0.0f },
    { PASO_PERFIL, PERFIL_RUIDO,     0.0f,  3.0f, 50.0f },
    { PASO_PERFIL, PERFIL_RAMPA,     0.0f,  5.0f, 100.0f },
    { PASO_PERFIL, PERFIL_ESCALONADO,0.0f,  2.0f, 160.0f },
};

// Secuencia activa (solo la toca el lazo de control)
static paso_t seq[MAX_PASOS];
static int    seq_n = 0;

// Secuencia nueva (la escribe la consola)
static paso_t        seq_stage[MAX_PASOS];
static volatile int  seq_stage_n = 0;

// Ordenes de la consola hacia el lazo de control
static volatile bool cmd_nueva = false;
static volatile bool cmd_run   = false;
static volatile bool cmd_stop  = false;

// =====================================================
// 11. ESTADO GLOBAL
// =====================================================

static pcnt_unit_handle_t encoder_unit;
static pcnt_channel_handle_t encoder_chan_a;
static pcnt_channel_handle_t encoder_chan_b;

static float conteos_por_rev = CONTEOS_POR_REV;

static int64_t posicion_conteos = 0;
static int total_prev = 0;
static volatile int32_t pcnt_desborde = 0;

// Captura del borde de referencia (ISR)
static volatile bool isr_armada      = false;
static volatile bool borde_capturado = false;
static volatile int  borde_total     = 0;
static volatile int  isr_base        = 0;
static volatile int  isr_minimo      = 0;

static TaskHandle_t tarea_escala_handle = NULL;

// Sensor de corriente
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle = NULL;
static float offset_is_mv = 0.0f;
static volatile float corriente_a = 0.0f;

// Estado del controlador
typedef enum { MODO_POS = 0, MODO_VEL = 1 } modo_t;

static volatile modo_t modo = MODO_POS;
static volatile bool   seq_activa = false;
static volatile int    paso_actual = -1;
static volatile float  angulo_actual = 0.0f;

static int64_t t_paso_us = 0;
static int     estable_ciclos = 0;

// Modo posicion
static float pos_obj = 0.0f;    // objetivo final (grados, absoluto)
static float pos_sp  = 0.0f;    // referencia de la trayectoria
static float pos_vel = 0.0f;    // velocidad de la trayectoria

// Modo velocidad
static perfil_t perfil_paso = PERFIL_CONSTANTE;
static float perfil_vmax = 0.0f;
static float perfil_cte  = 0.0f;
static float perfil_k    = 1.0f;     // factor de escala de tiempo
static float v_integral  = 0.0f;
static float v_rpm_med   = 0.0f;
static int   v_acum_cnt  = 0;
static int   v_ciclos    = 0;
static float v_acum_dt   = 0.0f;

// =====================================================
// 12. SALIDA SERIAL NO BLOQUEANTE (Teleplot)
//     Usa el buffer de TX del driver UART en vez de
//     printf (que espera byte a byte).
// =====================================================

static void tele(const char *fmt, ...)
{
    char buf[128];
    va_list ap;

    va_start(ap, fmt);
    int n = vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);

    if (n > 0) {
        if (n >= (int)sizeof(buf))
            n = sizeof(buf) - 1;
        uart_write_bytes(UART_CONSOLA, buf, n);
    }
}

// =====================================================
// 13. LED
// =====================================================

static void configurar_led(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_LED),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&cfg));
    ESP_ERROR_CHECK(gpio_set_level(PIN_LED, !LED_ACTIVO));
}

static void actualizar_led_laser(void)
{
    int nivel = gpio_get_level(PIN_RECEPTOR);
    bool detectado = (nivel == LASER_ACTIVE_LEVEL);

    gpio_set_level(PIN_LED, detectado ? LED_ACTIVO : !LED_ACTIVO);
}

// =====================================================
// 14. MOTOR
// =====================================================

static void detener_motor(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}

static void establecer_motor(int duty)
{
    if (duty > PWM_LIMITE_POS)
        duty = PWM_LIMITE_POS;

    if (duty < -PWM_LIMITE_POS)
        duty = -PWM_LIMITE_POS;

    uint32_t pwm_r = 0;
    uint32_t pwm_l = 0;

    if (duty > 0)
        pwm_r = (uint32_t)duty;
    else if (duty < 0)
        pwm_l = (uint32_t)(-duty);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, pwm_r));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, pwm_l));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}

// Freno activo: ambos lados del puente H en alto
static void frenar_motor(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, PWM_MAX));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, PWM_MAX));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
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

    ledc_channel_config_t pwm_r_cfg = {
        .gpio_num = PIN_RPWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_r_cfg));

    ledc_channel_config_t pwm_l_cfg = {
        .gpio_num = PIN_LPWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ESP_ERROR_CHECK(ledc_channel_config(&pwm_l_cfg));

    gpio_config_t enable_cfg = {
        .pin_bit_mask = (1ULL << PIN_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&enable_cfg));

    detener_motor();
    ESP_ERROR_CHECK(gpio_set_level(PIN_ENABLE, 0));
}

// =====================================================
// 15. LASER Y RECEPTOR
// =====================================================

static void configurar_laser_receptor(void)
{
    gpio_config_t laser_cfg = {
        .pin_bit_mask = (1ULL << PIN_LASER),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&laser_cfg));
    ESP_ERROR_CHECK(gpio_set_level(PIN_LASER, 0));

    gpio_config_t receptor_cfg = {
        .pin_bit_mask = (1ULL << PIN_RECEPTOR),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&receptor_cfg));

    ESP_LOGI(TAG, "Laser emisor: GPIO%d", PIN_LASER);
    ESP_LOGI(TAG, "Receptor digital: GPIO%d", PIN_RECEPTOR);

    ESP_ERROR_CHECK(gpio_set_level(PIN_LASER, 1));
}

static bool laser_activo(void)
{
    int activas = 0;

    for (int i = 0; i < LASER_MUESTRAS; i++) {
        actualizar_led_laser();

        if (gpio_get_level(PIN_RECEPTOR) == LASER_ACTIVE_LEVEL)
            activas++;

        vTaskDelay(pdMS_TO_TICKS(LASER_INTERVALO_MS));
    }

    return activas >= (LASER_MUESTRAS / 2 + 1);
}

static void tarea_monitor_laser(void *pvParameters)
{
    int ultimo_nivel = -1;

    while (1) {
        int nivel = gpio_get_level(PIN_RECEPTOR);

        actualizar_led_laser();

        if (nivel != ultimo_nivel) {
            ESP_LOGI(TAG, "Receptor=%d | Referencia=%s", nivel,
                     (nivel == LASER_ACTIVE_LEVEL) ? "ACTIVA" : "INACTIVA");
            ultimo_nivel = nivel;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// =====================================================
// 16. ENCODER PCNT (conteo total sin perdidas)
// =====================================================

static bool IRAM_ATTR pcnt_al_limite(
    pcnt_unit_handle_t unit,
    const pcnt_watch_event_data_t *edata,
    void *user_ctx)
{
    pcnt_desborde = pcnt_desborde + edata->watch_point_value;
    return false;
}

static inline int IRAM_ATTR leer_total(void)
{
    int32_t d1, d2;
    int cuenta = 0;

    do {
        d1 = pcnt_desborde;
        pcnt_unit_get_count(encoder_unit, &cuenta);
        d2 = pcnt_desborde;
    } while (d1 != d2);

    return (int)d1 + cuenta;
}

static void configurar_encoder(void)
{
    pcnt_unit_config_t unit_cfg = {
        .low_limit = PCNT_LIM_BAJO,
        .high_limit = PCNT_LIM_ALTO
    };
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_cfg, &encoder_unit));

    pcnt_glitch_filter_config_t filter_cfg = { .max_glitch_ns = 1000 };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(encoder_unit, &filter_cfg));

    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num = PIN_ENC_A,
        .level_gpio_num = PIN_ENC_B
    };
    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num = PIN_ENC_B,
        .level_gpio_num = PIN_ENC_A
    };

    ESP_ERROR_CHECK(pcnt_new_channel(encoder_unit, &chan_a_cfg, &encoder_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(encoder_unit, &chan_b_cfg, &encoder_chan_b));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        encoder_chan_a,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        encoder_chan_a,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        encoder_chan_b,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        encoder_chan_b,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder_unit, PCNT_LIM_ALTO));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(encoder_unit, PCNT_LIM_BAJO));

    pcnt_event_callbacks_t cbs = { .on_reach = pcnt_al_limite };
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(encoder_unit, &cbs, NULL));

    ESP_ERROR_CHECK(pcnt_unit_enable(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(encoder_unit));

    pcnt_desborde = 0;

    ESP_LOGI(TAG, "Encoder PCNT inicializado");
}

// =====================================================
// 17. ISR DE LA REFERENCIA
// =====================================================

static void IRAM_ATTR isr_referencia(void *arg)
{
    if (!isr_armada || borde_capturado)
        return;

    if (gpio_get_level(PIN_RECEPTOR) != LASER_ACTIVE_LEVEL)
        return;

    int total = leer_total();

    if (abs(total - isr_base) < isr_minimo)
        return;

    borde_total     = total;
    borde_capturado = true;
}

static void configurar_isr_referencia(void)
{
    gpio_int_type_t tipo = (LASER_ACTIVE_LEVEL == 1)
        ? GPIO_INTR_POSEDGE
        : GPIO_INTR_NEGEDGE;

    ESP_ERROR_CHECK(gpio_set_intr_type(PIN_RECEPTOR, tipo));

    esp_err_t r = gpio_install_isr_service(0);

    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(r);

    ESP_ERROR_CHECK(gpio_isr_handler_add(PIN_RECEPTOR, isr_referencia, NULL));
}

static bool esperar_borde(int timeout_ms)
{
    int64_t inicio = esp_timer_get_time();

    while (1) {
        int64_t transcurrido = (esp_timer_get_time() - inicio) / 1000;

        if (transcurrido >= timeout_ms)
            return false;

        if (borde_capturado) {
            esp_rom_delay_us(3000);

            if (gpio_get_level(PIN_RECEPTOR) == LASER_ACTIVE_LEVEL)
                return true;

            borde_capturado = false;   // era ruido
        }

        vTaskDelay(1);
    }
}

// =====================================================
// 18. HOMING: CERO OPTICO Y CONTEOS POR VUELTA
// =====================================================

static bool ejecutar_homing(void)
{
    ESP_LOGW(TAG, "Iniciando homing optico");

    detener_motor();
    ESP_ERROR_CHECK(gpio_set_level(PIN_ENABLE, 1));

    // FASE A: si comienza sobre la referencia, alejarse
    if (laser_activo()) {
        ESP_LOGI(TAG, "Referencia activa al inicio; liberando");

        int64_t inicio_liberacion = esp_timer_get_time();

        establecer_motor(-HOMING_DIRECTION * HOMING_PWM);

        while (laser_activo()) {
            int64_t transcurrido =
                (esp_timer_get_time() - inicio_liberacion) / 1000;

            if (transcurrido >= HOMING_RELEASE_TIMEOUT_MS) {
                detener_motor();
                gpio_set_level(PIN_ENABLE, 0);
                ESP_LOGE(TAG, "Tiempo agotado liberando referencia");
                return false;
            }

            vTaskDelay(pdMS_TO_TICKS(PERIODO_MS));
        }

        detener_motor();
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    // FASE B: primer cruce del haz
    isr_base        = leer_total();
    isr_minimo      = (int)HOMING_MIN_COUNTS;
    borde_capturado = false;
    isr_armada      = true;

    establecer_motor(HOMING_DIRECTION * HOMING_PWM);

    if (!esperar_borde(HOMING_SEARCH_TIMEOUT_MS)) {
        isr_armada = false;
        detener_motor();
        gpio_set_level(PIN_ENABLE, 0);
        ESP_LOGE(TAG, "Timeout buscando el primer cruce");
        return false;
    }

    int borde1 = borde_total;

    // FASE C: segundo cruce = una vuelta exacta
    isr_armada      = false;
    isr_base        = borde1;
    isr_minimo      = HOMING_MIN_SEGUNDO_CRUCE;
    borde_capturado = false;
    isr_armada      = true;

    if (!esperar_borde(HOMING_SEARCH_TIMEOUT_MS)) {
        isr_armada = false;
        detener_motor();
        gpio_set_level(PIN_ENABLE, 0);
        ESP_LOGE(TAG, "Timeout buscando el segundo cruce");
        return false;
    }

    int borde2 = borde_total;

    isr_armada = false;
    detener_motor();

    vTaskDelay(pdMS_TO_TICKS(300));

    int vuelta = abs(borde2 - borde1);

    conteos_por_rev = (float)vuelta;

    ESP_LOGI(TAG, "Conteos por vuelta medidos: %d (nominal %.0f)",
             vuelta, (float)CONTEOS_POR_REV);

    float relacion = conteos_por_rev / (float)CONTEOS_POR_REV;

    if (relacion < 0.8f || relacion > 1.2f) {
        ESP_LOGW(TAG,
                 "El valor medido difiere mas de 20%% del nominal. "
                 "Actualiza CONTEOS_POR_REV a %d", vuelta);
    }

    int total_final = leer_total();

    posicion_conteos = (int64_t)total_final - (int64_t)borde2;
    total_prev = total_final;

    ESP_LOGI(TAG, "Cero angular establecido en el cruce");
    ESP_LOGI(TAG, "Desplazamiento posterior: %.2f grados",
             (float)posicion_conteos * 360.0f / conteos_por_rev);
    ESP_LOGI(TAG, "Homing finalizado");

    return true;
}

// =====================================================
// 19. MEDICION DE CORRIENTE
// =====================================================

static int raw_a_mv(int raw)
{
    int mv = 0;

    if (adc_cali_handle != NULL &&
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv) == ESP_OK) {
        return mv;
    }

    return (raw * IS_ADC_FS_MV) / 4095;
}

static void configurar_adc_corriente(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = IS_ATTEN
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc_handle, ADC_R_IS_CHANNEL, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc_handle, ADC_L_IS_CHANNEL, &chan_cfg));

    adc_cali_line_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .atten = IS_ATTEN,
        .bitwidth = ADC_BITWIDTH_12
    };

    esp_err_t r = adc_cali_create_scheme_line_fitting(
        &cali_cfg, &adc_cali_handle);

    if (r != ESP_OK) {
        adc_cali_handle = NULL;
        ESP_LOGW(TAG, "ADC sin calibracion de fabrica; "
                      "se usara conversion lineal aproximada");
    }

    ESP_LOGI(TAG, "Sensor de corriente: R_IS=ADC1_CH%d, L_IS=ADC1_CH%d",
             (int)ADC_R_IS_CHANNEL, (int)ADC_L_IS_CHANNEL);
}

static void calibrar_offset_corriente(void)
{
    const int N = 200;
    float acum = 0.0f;
    int validas = 0;

    for (int i = 0; i < N; i++) {
        int raw_r = 0, raw_l = 0;

        if (adc_oneshot_read(adc_handle, ADC_R_IS_CHANNEL, &raw_r) == ESP_OK &&
            adc_oneshot_read(adc_handle, ADC_L_IS_CHANNEL, &raw_l) == ESP_OK) {
            acum += (float)(raw_a_mv(raw_r) + raw_a_mv(raw_l));
            validas++;
        }

        esp_rom_delay_us(500);
    }

    if (validas > 0)
        offset_is_mv = acum / (float)validas;

    ESP_LOGI(TAG, "Offset del sensor de corriente: %.1f mV", offset_is_mv);
}

static float medir_rafaga_mv(void)
{
    static float muestras[IS_MAX_MUESTRAS];
    int n = 0;

    int64_t inicio = esp_timer_get_time();

    while (n < IS_MAX_MUESTRAS &&
           (esp_timer_get_time() - inicio) < IS_RAFAGA_US) {

        int raw_r = 0, raw_l = 0;

        if (adc_oneshot_read(adc_handle, ADC_R_IS_CHANNEL, &raw_r) != ESP_OK ||
            adc_oneshot_read(adc_handle, ADC_L_IS_CHANNEL, &raw_l) != ESP_OK) {
            break;
        }

        muestras[n++] =
            (float)(raw_a_mv(raw_r) + raw_a_mv(raw_l)) - offset_is_mv;
    }

    if (n < 4)
        return 0.0f;

    float maximo = muestras[0];

    for (int i = 1; i < n; i++) {
        if (muestras[i] > maximo)
            maximo = muestras[i];
    }

    if (maximo < IS_UMBRAL_MV)
        return 0.0f;

    float limite = 0.5f * maximo;
    float acum = 0.0f;
    int activas = 0;

    for (int i = 0; i < n; i++) {
        if (muestras[i] >= limite) {
            acum += muestras[i];
            activas++;
        }
    }

    return (activas > 0) ? (acum / (float)activas) : 0.0f;
}

static void tarea_corriente(void *pvParameters)
{
    float filtrada = 0.0f;

    while (1) {
        float mv = medir_rafaga_mv();
        float amperes = (mv / 1000.0f) * CORRIENTE_A_POR_V;

        filtrada += FILTRO_CORRIENTE * (amperes - filtrada);
        corriente_a = filtrada;

        vTaskDelay(pdMS_TO_TICKS(IS_PERIODO_MS));
    }
}

// =====================================================
// 20. TRANSICIONES ENTRE MODOS Y PASOS (lazo de control)
// =====================================================

// Entra en modo posicion.
// destino = NAN  -> mantener la posicion donde se detendria
// destino = angulo (0..360) -> ir al angulo equivalente mas
//                              cercano (o hacia adelante)
// Se parte de la velocidad actual para que el cambio desde un
// perfil de velocidad frene suave en vez de dar un tiron.
static void entrar_pos(float angulo_real, float vel_degs, float destino)
{
    float pred = angulo_real;

    if (fabsf(vel_degs) > VEL_PARADO_DEG_S) {
        pred += copysignf(vel_degs * vel_degs /
                          (2.0f * ACEL_DEG_S2), vel_degs);
    }

    float obj = pred;

    if (!isnan(destino)) {
        float resto = fmodf(pred, 360.0f);

        if (resto < 0.0f)
            resto += 360.0f;

        float d = destino - resto;

#if POS_SOLO_ADELANTE
        if (d < -ERROR_TOLERANCIA)
            d += 360.0f;
#else
        if (d > 180.0f)
            d -= 360.0f;
        else if (d < -180.0f)
            d += 360.0f;
#endif
        obj = pred + d;
    }

    modo = MODO_POS;
    pos_obj = obj;
    pos_sp  = angulo_real;
    pos_vel = vel_degs;
    estable_ciclos = 0;
}

// Entra en modo velocidad con un perfil.
static void entrar_vel(perfil_t perfil, float param, float dur,
                       float vel_degs)
{
    modo = MODO_VEL;
    perfil_paso = perfil;
    perfil_vmax = (param > 0.0f) ? param : rpm_prueba_max;
    perfil_cte  = (param > 0.0f) ? param : rpm_constante;

    float T = PERIODO_NATURAL[perfil];
    perfil_k = (escalar_perfil && T > 0.0f && dur > 0.0f) ? (T / dur) : 1.0f;

    ruido_ultimo_bloque = -1;

    v_integral = 0.0f;
    v_rpm_med  = fabsf(vel_degs) / 6.0f;    // 6 deg/s = 1 RPM
    v_acum_cnt = 0;
    v_acum_dt  = 0.0f;
    v_ciclos   = 0;
}

static void iniciar_paso(int idx, float angulo_real, float vel_degs)
{
    const paso_t *p = &seq[idx];

    paso_actual = idx;
    t_paso_us = esp_timer_get_time();

    switch (p->tipo) {

        case PASO_POS:
            ESP_LOGI(TAG, "Paso %d/%d: ir a %.1f grados",
                     idx + 1, seq_n, p->valor);
            entrar_pos(angulo_real, vel_degs, p->valor);
            break;

        case PASO_ESPERA:
            ESP_LOGI(TAG, "Paso %d/%d: mantener posicion %.1f s",
                     idx + 1, seq_n, p->dur);
            if (modo != MODO_POS)
                entrar_pos(angulo_real, vel_degs, NAN);
            break;

        case PASO_PERFIL:
            ESP_LOGI(TAG, "Paso %d/%d: perfil %s durante %.1f s",
                     idx + 1, seq_n, NOMBRES_PERFIL[p->perfil], p->dur);
            entrar_vel(p->perfil, p->param, p->dur, vel_degs);
            break;
    }
}

// =====================================================
// 21. CONSOLA: PARSER DE SECUENCIAS
// =====================================================

static bool es_numero(const char *s, float *v)
{
    if (!(isdigit((unsigned char)s[0]) || s[0] == '.' ||
          s[0] == '-' || s[0] == '+'))
        return false;

    char *fin = NULL;
    float x = strtof(s, &fin);

    if (fin == s)
        return false;

    *v = x;      // lo que sobre ("s", "seg", "o") se ignora
    return true;
}

static int perfil_por_nombre(const char *nombre)
{
    size_t len = strlen(nombre);

    if (len < 3)
        return 0;

    for (int i = 1; i < PERFIL_TOTAL; i++) {
        if (strncmp(nombre, NOMBRES_PERFIL[i], len) == 0)
            return i;
    }

    return 0;
}

// Devuelve 1 = paso valido, 0 = segmento vacio, -1 = error
static int parsear_segmento(char *seg, paso_t *p)
{
    char *tok[8];
    int nt = 0;
    char *sp = NULL;

    for (char *t = strtok_r(seg, " \t", &sp);
         t != NULL && nt < 8;
         t = strtok_r(NULL, " \t", &sp)) {
        tok[nt++] = t;
    }

    if (nt == 0)
        return 0;

    // Numeros del segmento (se ignoran las palabras sueltas,
    // asi "ve a 60", "ruido blanco 3s" tambien funcionan)
    float num[3];
    int nn = 0;
    float v;

    bool primero_num = es_numero(tok[0], &v);

    if (primero_num)
        num[nn++] = v;

    // Velocidad maxima del paso: un numero seguido de "rpm"
    // ("50rpm" o "50 rpm"), o el numero que va despues de la duracion.
    bool has_rpm = false;
    float rpm_exp = 0.0f;

    for (int i = 1; i < nt && nn < 3; i++) {
        if (es_numero(tok[i], &v)) {
            bool es_rpm = (strstr(tok[i], "rpm") != NULL) ||
                          (i + 1 < nt && strncmp(tok[i + 1], "rpm", 3) == 0);
            if (es_rpm) {
                has_rpm = true;
                rpm_exp = v;
            } else {
                num[nn++] = v;
            }
        }
    }

    char nombre[16];
    size_t k = 0;

    while (k < sizeof(nombre) - 1 && isalpha((unsigned char)tok[0][k])) {
        nombre[k] = tok[0][k];
        k++;
    }
    nombre[k] = '\0';

    memset(p, 0, sizeof(*p));

    bool es_pos = (primero_num && nn == 1) ||
                  strcmp(nombre, "pos") == 0 ||
                  strcmp(nombre, "go") == 0 ||
                  strcmp(nombre, "ir") == 0 ||
                  strcmp(nombre, "ve") == 0 ||
                  strcmp(nombre, "ang") == 0 ||
                  strcmp(nombre, "angulo") == 0 ||
                  strcmp(nombre, "a") == 0;

    bool es_espera = strcmp(nombre, "espera") == 0 ||
                     strcmp(nombre, "esperar") == 0 ||
                     strcmp(nombre, "hold") == 0 ||
                     strcmp(nombre, "pausa") == 0;

    if (es_pos) {
        if (nn < 1) {
            printf("[error] falta el angulo\n");
            return -1;
        }
        if (num[0] < 0.0f || num[0] > 360.0f) {
            printf("[error] angulo fuera de rango (0-360): %.1f\n", num[0]);
            return -1;
        }
        p->tipo  = PASO_POS;
        p->valor = num[0];
        return 1;
    }

    if (es_espera) {
        if (nn < 1 || num[0] < 0.1f || num[0] > 600.0f) {
            printf("[error] tiempo de espera invalido (0.1-600 s)\n");
            return -1;
        }
        p->tipo = PASO_ESPERA;
        p->dur  = num[0];
        return 1;
    }

    // Perfil: "<nombre|numero> <segundos> [rpm]"
    int idx;
    float dur;
    float par = 0.0f;

    if (primero_num) {
        idx = (int)num[0];
        dur = num[1];
        par = (nn > 2) ? num[2] : 0.0f;
    } else {
        idx = perfil_por_nombre(nombre);
        if (idx == 0) {
            printf("[error] perfil o comando desconocido: '%s'\n", nombre);
            return -1;
        }
        if (nn < 1) {
            printf("[error] falta la duracion de '%s'\n", nombre);
            return -1;
        }
        dur = num[0];
        par = (nn > 1) ? num[1] : 0.0f;
    }

    if (idx < 1 || idx >= PERFIL_TOTAL) {
        printf("[error] perfil invalido: %d\n", idx);
        return -1;
    }
    if (dur < 0.1f || dur > 600.0f) {
        printf("[error] duracion invalida (0.1-600 s): %.1f\n", dur);
        return -1;
    }
    if (has_rpm)
        par = rpm_exp;

    if (par != 0.0f)
        par = limitar(par, 10.0f, RPM_LIMITE_DURO);

    p->tipo   = PASO_PERFIL;
    p->perfil = (perfil_t)idx;
    p->dur    = dur;
    p->param  = par;
    return 1;
}

static bool parsear_secuencia(const char *linea, paso_t *out, int *n_out)
{
    char buf[CMD_MAX];
    strncpy(buf, linea, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';

    int n = 0;
    char *sp = NULL;

    for (char *seg = strtok_r(buf, ";", &sp);
         seg != NULL;
         seg = strtok_r(NULL, ";", &sp)) {

        if (n >= MAX_PASOS) {
            printf("[error] maximo %d pasos\n", MAX_PASOS);
            return false;
        }

        int r = parsear_segmento(seg, &out[n]);

        if (r < 0) {
            printf("[error] paso %d no valido\n", n + 1);
            return false;
        }
        if (r > 0)
            n++;
    }

    if (n == 0)
        return false;

    *n_out = n;
    return true;
}

static void imprimir_pasos(const paso_t *s, int n)
{
    for (int i = 0; i < n; i++) {
        switch (s[i].tipo) {
            case PASO_POS:
                printf("  %d) ir a %.1f grados\n", i + 1, s[i].valor);
                break;
            case PASO_ESPERA:
                printf("  %d) mantener %.1f s\n", i + 1, s[i].dur);
                break;
            case PASO_PERFIL:
                printf("  %d) %s por %.1f s", i + 1,
                       NOMBRES_PERFIL[s[i].perfil], s[i].dur);
                if (s[i].param > 0.0f)
                    printf(" (%.0f RPM)", s[i].param);
                printf("\n");
                break;
        }
    }
}

static void imprimir_ayuda(void)
{
    printf("\n--- COMANDOS ---\n");
    printf("Secuencia (se ejecuta al enviarla, pasos separados por ';'):\n");
    printf("  60; ruido 3 50; rampa 5 100; escalon 2 160\n");
    printf("  <angulo>            ir a un angulo (0-360)\n");
    printf("  <perfil> <seg> [rpm]  perfil durante X segundos con su\n");
    printf("                        velocidad maxima, ej: ruido 3 50\n");
    printf("                        (tambien: ruido 3s 50rpm). Sin rpm usa vmax\n");
    printf("  espera <seg>        mantener la posicion\n");
    printf("Otros:\n");
    printf("  run        repite la ultima secuencia\n");
    printf("  stop       cancela y mantiene la posicion\n");
    printf("  lista      muestra la secuencia cargada\n");
    printf("  vmax X     velocidad maxima de perfiles (RPM)\n");
    printf("  cte X      RPM del perfil constante\n");
    printf("  escalar 0/1  forma completa del perfil dentro del tiempo\n");
    printf("  estado     muestra la configuracion\n");
    printf("  e          redibuja el transportador\n");
    printf("Perfiles:");
    for (int i = 1; i < PERFIL_TOTAL; i++)
        printf(" %d=%s", i, NOMBRES_PERFIL[i]);
    printf("\n----------------\n");
}

static void imprimir_estado(void)
{
    printf("[estado] modo=%s | secuencia=%s paso=%d/%d | ang=%.1f | "
           "vmax=%.0f cte=%.0f | escalar=%d | I=%.2f A\n",
           modo == MODO_POS ? "POSICION" : "VELOCIDAD",
           seq_activa ? "EN CURSO" : "detenida",
           paso_actual + 1, seq_n,
           (float)angulo_actual,
           (float)rpm_prueba_max, (float)rpm_constante,
           (int)escalar_perfil, (float)corriente_a);
}

static void procesar_comando(const char *cmd)
{
    static paso_t tmp[MAX_PASOS];
    int n = 0;
    float x;

    if (strcmp(cmd, "stop") == 0 || strcmp(cmd, "s") == 0) {
        cmd_stop = true;
        printf("[ok] deteniendo\n");

    } else if (strcmp(cmd, "run") == 0 || strcmp(cmd, "r") == 0) {
        cmd_run = true;

    } else if (strcmp(cmd, "estado") == 0) {
        imprimir_estado();

    } else if (strcmp(cmd, "lista") == 0) {
        printf("[secuencia cargada: %d pasos]\n", seq_n);
        imprimir_pasos(seq, seq_n);

    } else if (strcmp(cmd, "ayuda") == 0 || strcmp(cmd, "?") == 0 ||
               strcmp(cmd, "help") == 0) {
        imprimir_ayuda();

    } else if (strcmp(cmd, "e") == 0) {
        if (tarea_escala_handle != NULL)
            xTaskNotifyGive(tarea_escala_handle);

    } else if (sscanf(cmd, "vmax %f", &x) == 1) {
        rpm_prueba_max = limitar(x, 10.0f, RPM_LIMITE_DURO);
        printf("[ok] vmax = %.0f RPM\n", (float)rpm_prueba_max);

    } else if (sscanf(cmd, "cte %f", &x) == 1) {
        rpm_constante = limitar(x, 10.0f, RPM_LIMITE_DURO);
        printf("[ok] cte = %.0f RPM\n", (float)rpm_constante);

    } else if (sscanf(cmd, "escalar %f", &x) == 1) {
        escalar_perfil = (x != 0.0f);
        printf("[ok] escalar = %d\n", (int)escalar_perfil);

    } else if (parsear_secuencia(cmd, tmp, &n)) {
        memcpy(seq_stage, tmp, sizeof(paso_t) * n);
        seq_stage_n = n;
        printf("[ok] secuencia de %d pasos:\n", n);
        imprimir_pasos(tmp, n);
        cmd_nueva = true;           // el lazo la copia y la arranca
    }
}

static void tarea_serial(void *pvParameters)
{
    char linea[CMD_MAX];
    int n = 0;
    uint8_t byte;

    while (1) {
        int r = uart_read_bytes(UART_CONSOLA, &byte, 1, pdMS_TO_TICKS(50));

        if (r <= 0)
            continue;

        if (byte == '\n' || byte == '\r') {
            if (n > 0) {
                linea[n] = '\0';
                procesar_comando(linea);
                n = 0;
            }
        } else if (n < (int)sizeof(linea) - 1) {
            linea[n++] = (char)tolower(byte);
        } else {
            n = 0;   // linea demasiado larga, descartar
            printf("[error] linea demasiado larga\n");
        }
    }
}

// =====================================================
// 22. TRANSPORTADOR EN TELEPLOT
// =====================================================

static void tarea_escala(void *pvParameters)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        for (int a = 0; a <= 360; a += ESCALA_PASO) {
            tele(">escala:%.3f:%.3f|xy\n",
                 cosf(a * GRAD_A_RAD), sinf(a * GRAD_A_RAD));
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        for (int a = 0; a <= 360; a += ESCALA_MAYOR) {
            tele(">marcas:%.3f:%.3f|xy\n",
                 0.90f * cosf(a * GRAD_A_RAD),
                 0.90f * sinf(a * GRAD_A_RAD));
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        tele(">centro:0:0|xy\n");
    }
}

// =====================================================
// 23. PROGRAMA PRINCIPAL
// =====================================================

void app_main(void)
{
    configurar_pwm();
    configurar_laser_receptor();
    configurar_led();
    configurar_encoder();
    configurar_isr_referencia();   // despues del encoder
    configurar_adc_corriente();

    ESP_ERROR_CHECK(
        uart_driver_install(UART_CONSOLA, UART_RX_BUF * 2,
                            UART_TX_BUF, 0, NULL, 0)
    );

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "POSICION + PERFILES DE VELOCIDAD + SECUENCIAS");
    ESP_LOGI(TAG, "Motor JGA25-370 + BTS7960");
    ESP_LOGI(TAG, "======================================");

    BaseType_t tarea_ok = xTaskCreate(
        tarea_monitor_laser, "monitor_laser", 2048, NULL, 4, NULL);

    if (tarea_ok != pdPASS) {
        ESP_LOGE(TAG, "No se pudo crear monitor laser");
        detener_motor();
        gpio_set_level(PIN_ENABLE, 0);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Cero del sensor de corriente (motor parado)
    ESP_ERROR_CHECK(gpio_set_level(PIN_ENABLE, 1));
    vTaskDelay(pdMS_TO_TICKS(100));
    calibrar_offset_corriente();
    ESP_ERROR_CHECK(gpio_set_level(PIN_ENABLE, 0));

    if (!ejecutar_homing()) {
        detener_motor();
        gpio_set_level(PIN_ENABLE, 0);

        while (1) {
            ESP_LOGE(TAG, "Homing fallido; motor deshabilitado");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Secuencia por defecto (se ejecuta con "run")
    seq_n = (int)(sizeof(SEC_DEFECTO) / sizeof(SEC_DEFECTO[0]));
    memcpy(seq, SEC_DEFECTO, sizeof(SEC_DEFECTO));

    xTaskCreate(tarea_serial, "serial", 4096, NULL, 3, NULL);
    xTaskCreate(tarea_escala, "escala", 3072, NULL, 2, &tarea_escala_handle);
    xTaskNotifyGive(tarea_escala_handle);

    // Corriente en el nucleo 1 para no interferir con el lazo
    xTaskCreatePinnedToCore(tarea_corriente, "corriente", 3072,
                            NULL, 2, NULL, 1);

    vTaskDelay(pdMS_TO_TICKS(300));
    imprimir_ayuda();
    printf("[secuencia cargada por defecto: escribe 'run']\n");
    imprimir_pasos(seq, seq_n);

    ESP_LOGI(TAG, "Iniciando control");

    float vel_filtrada = 0.0f;
    int contador_print = 0;

#if USAR_PROTECCION_CORRIENTE
    int64_t t_sobrecorriente_us = 0;
#endif

    // Despues del homing, ir a 0 grados
    {
        float ang0 = ((float)posicion_conteos * 360.0f) / conteos_por_rev;
        entrar_pos(ang0, 0.0f, 0.0f);
    }

    ESP_ERROR_CHECK(gpio_set_level(PIN_ENABLE, 1));

    TickType_t ultimo_despertar = xTaskGetTickCount();
    int64_t t_prev = esp_timer_get_time();

    while (1) {
        // ---------------------------------------------
        // 0. Periodo real del ciclo
        // ---------------------------------------------

        int64_t ahora = esp_timer_get_time();
        float dt = (float)(ahora - t_prev) / 1000000.0f;
        t_prev = ahora;

        if (dt < 0.002f || dt > 0.05f)
            dt = DT;

        // ---------------------------------------------
        // 1. Encoder (sin borrar el contador)
        // ---------------------------------------------

        int total = leer_total();
        int delta_conteos = total - total_prev;
        total_prev = total;

        posicion_conteos += delta_conteos;

        float velocidad_angular =
            ((float)delta_conteos * 360.0f) / (conteos_por_rev * dt);

        vel_filtrada += FILTRO_VEL * (velocidad_angular - vel_filtrada);

        float angulo_real =
            ((float)posicion_conteos * 360.0f) / conteos_por_rev;

        angulo_actual = angulo_real;

        // ---------------------------------------------
        // 2. Ordenes de la consola
        // ---------------------------------------------

        if (cmd_stop) {
            cmd_stop = false;
            seq_activa = false;
            entrar_pos(angulo_real, vel_filtrada, NAN);
            ESP_LOGI(TAG, "Detenido; manteniendo posicion");
        }

        if (cmd_nueva) {
            cmd_nueva = false;
            int n = seq_stage_n;

            if (n > 0 && n <= MAX_PASOS) {
                memcpy(seq, seq_stage, sizeof(paso_t) * n);
                seq_n = n;
                cmd_run = true;
            }
        }

        if (cmd_run) {
            cmd_run = false;

            if (seq_n > 0) {
                seq_activa = true;
                ESP_LOGI(TAG, "Iniciando secuencia de %d pasos", seq_n);
                iniciar_paso(0, angulo_real, vel_filtrada);
            } else {
                ESP_LOGW(TAG, "No hay secuencia cargada");
            }
        }

        // ---------------------------------------------
        // 3. Avance de la secuencia
        // ---------------------------------------------

        bool llegado = (modo == MODO_POS) &&
                       fabsf(pos_obj - angulo_real) <= ERROR_TOLERANCIA &&
                       pos_vel == 0.0f;

        if (seq_activa) {
            const paso_t *p = &seq[paso_actual];
            float t_paso = (float)(ahora - t_paso_us) / 1000000.0f;
            bool fin = false;

            if (p->tipo == PASO_POS) {
                estable_ciclos = llegado ? (estable_ciclos + 1) : 0;

                if (estable_ciclos >= ESTABLE_CICLOS) {
                    fin = true;
                } else if (t_paso > POS_TIMEOUT_S) {
                    ESP_LOGW(TAG, "Paso %d: no llego en %.0f s; sigo",
                             paso_actual + 1, POS_TIMEOUT_S);
                    fin = true;
                }
            } else {
                fin = (t_paso >= p->dur);
            }

            if (fin) {
                int sig = paso_actual + 1;

                if (sig < seq_n) {
                    iniciar_paso(sig, angulo_real, vel_filtrada);
                } else {
                    seq_activa = false;

                    if (modo == MODO_VEL)
                        entrar_pos(angulo_real, vel_filtrada, NAN);

                    ESP_LOGI(TAG, "Secuencia terminada; manteniendo posicion");
                }
            }
        }

        float t_paso = (float)(ahora - t_paso_us) / 1000000.0f;
        if (t_paso < 0.0f)
            t_paso = 0.0f;

        // ---------------------------------------------
        // 4. Control segun el modo
        // ---------------------------------------------

        int  duty  = 0;
        bool freno = false;

        float error_pos = 0.0f;
        float error_rpm = 0.0f;
        float rpm_ref   = 0.0f;

        if (modo == MODO_POS) {

            // ----- Generador de trayectoria -----
            float dist = pos_obj - pos_sp;

            float v_obj = sqrtf(2.0f * ACEL_DEG_S2 * fabsf(dist));

            if (v_obj > VEL_MAX_DEG_S)
                v_obj = VEL_MAX_DEG_S;

            if (dist < 0.0f)
                v_obj = -v_obj;

            float dv     = v_obj - pos_vel;
            float dv_max = ACEL_DEG_S2 * dt;

            if (dv > dv_max)  dv = dv_max;
            if (dv < -dv_max) dv = -dv_max;

            pos_vel += dv;
            pos_sp  += pos_vel * dt;

            if (fabsf(pos_obj - pos_sp) < 0.2f) {
                pos_sp  = pos_obj;
                pos_vel = 0.0f;
            }

            // ----- PD -----
            error_pos      = pos_obj - angulo_real;
            float error_sp = pos_sp - angulo_real;

            rpm_ref = pos_vel / 6.0f;

            if (fabsf(error_pos) <= ERROR_TOLERANCIA && pos_vel == 0.0f) {
                duty  = 0;
                freno = true;
            } else {
                float salida = KP_POS * error_sp +
                               KD_POS * (pos_vel - vel_filtrada);

                if (salida > PWM_LIMITE_POS)  salida = PWM_LIMITE_POS;
                if (salida < -PWM_LIMITE_POS) salida = -PWM_LIMITE_POS;

                duty = (int)salida;

                // Empujon para vencer la friccion estatica
                if (fabsf(vel_filtrada) < VEL_PARADO_DEG_S &&
                    fabsf(error_pos) > ERROR_TOLERANCIA &&
                    fabsf(salida) < PWM_MINIMO) {

                    duty = (error_pos > 0.0f) ? PWM_MINIMO : -PWM_MINIMO;
                }
            }

        } else {

            // ----- Perfil de velocidad con PI -----
            float t_ef = t_paso * perfil_k;

            rpm_ref = generar_referencia(perfil_paso, t_ef,
                                         perfil_vmax, perfil_cte);
            rpm_ref = limitar(rpm_ref, 0.0f, RPM_LIMITE_DURO);

            // Medicion de RPM sobre ventana de CICLOS_PI ciclos
            v_acum_cnt += delta_conteos;
            v_acum_dt  += dt;
            v_ciclos++;

            if (v_ciclos >= CICLOS_PI) {
                v_rpm_med = fabsf((float)v_acum_cnt) * 60.0f /
                            (conteos_por_rev * v_acum_dt);

                if (rpm_ref < 1.0f) {
                    v_integral = 0.0f;
                } else {
                    v_integral += KI_VEL * (rpm_ref - v_rpm_med) * v_acum_dt;
                    v_integral = limitar(v_integral,
                                         -INTEGRAL_MAX, INTEGRAL_MAX);
                }

                v_acum_cnt = 0;
                v_acum_dt  = 0.0f;
                v_ciclos   = 0;
            }

            error_rpm = rpm_ref - v_rpm_med;

            if (rpm_ref < 1.0f) {
                duty = 0;
            } else {
                float pwm_base = (rpm_ref / MOTOR_RPM_MAX) * PWM_MAX;
                float salida = pwm_base + KP_VEL * error_rpm + v_integral;

                salida = limitar(salida, 0.0f, (float)PWM_LIMITE_VEL);
                duty = (int)(salida + 0.5f);
            }
        }

        // ---------------------------------------------
        // 5. Proteccion de corriente (opcional)
        // ---------------------------------------------

#if USAR_PROTECCION_CORRIENTE
        if (corriente_a > LIMITE_CORRIENTE_A) {
            if (t_sobrecorriente_us == 0) {
                t_sobrecorriente_us = ahora;
            } else if ((ahora - t_sobrecorriente_us) >
                       (int64_t)LIMITE_TIEMPO_MS * 1000) {
                detener_motor();
                gpio_set_level(PIN_ENABLE, 0);

                while (1) {
                    ESP_LOGE(TAG, "Sobrecorriente: %.2f A; "
                                  "motor deshabilitado", corriente_a);
                    vTaskDelay(pdMS_TO_TICKS(1000));
                }
            }
        } else {
            t_sobrecorriente_us = 0;
        }
#endif

        // ---------------------------------------------
        // 6. Aplicar PWM
        // ---------------------------------------------

#if USAR_FRENO_ACTIVO
        if (freno)
            frenar_motor();
        else
            establecer_motor(duty);
#else
        establecer_motor(duty);
#endif

        // ---------------------------------------------
        // 7. Telemetria para Teleplot
        // ---------------------------------------------

        if (++contador_print >= PRINT_CADA_N) {
            contador_print = 0;

            float ang_mod = fmodf(angulo_real, 360.0f);
            if (ang_mod < 0.0f)
                ang_mod += 360.0f;

            tele(">real_xy:%.3f:%.3f|xy\n",
                 cosf(angulo_real * GRAD_A_RAD),
                 sinf(angulo_real * GRAD_A_RAD));

            if (modo == MODO_POS) {
                tele(">ref_xy:%.3f:%.3f|xy\n",
                     cosf(pos_obj * GRAD_A_RAD),
                     sinf(pos_obj * GRAD_A_RAD));
                tele(">error_pos:%.1f\n", error_pos);
            } else {
                tele(">error_rpm:%.1f\n", error_rpm);
            }

            tele(">modo:%d\n", (int)modo);
            tele(">paso:%d\n", seq_activa ? paso_actual + 1 : 0);
            tele(">ang:%.1f\n", ang_mod);
            tele(">rpm_ref:%.1f\n", rpm_ref);
            tele(">rpm_real:%.1f\n",
                 modo == MODO_VEL ? v_rpm_med : vel_filtrada / 6.0f);
            tele(">pwm:%.1f\n", 100.0f * (float)duty / PWM_MAX);
            tele(">corriente:%.2f\n", corriente_a);
            tele(">t_paso:%.2f\n", seq_activa ? t_paso : 0.0f);
        }

        vTaskDelayUntil(&ultimo_despertar, pdMS_TO_TICKS(PERIODO_MS));
    }
}
