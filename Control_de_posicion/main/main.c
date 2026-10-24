// =====================================================
// CONTROL PD DE POSICION CON CERO OPTICO
// Motor JGA25-370 + BTS7960 + ESP32 (ESP-IDF)
//
// - Cero angular por cruce de laser (captura con ISR)
// - Conteos por vuelta MEDIDOS en el homing (dos cruces)
// - Conteo del encoder sin perdidas (sin clear en el lazo)
// - Objetivo escrito por serial (monitor o Teleplot)
// - Trayectoria suave + freno al llegar
// - Transportador en Teleplot (grafica XY)
// =====================================================

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// =====================================================
// 1. CONFIGURACION DE PINES
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
// 2. CONFIGURACION DE LA REFERENCIA OPTICA
// =====================================================

// 0 = LOW indica referencia
// 1 = HIGH indica referencia
#define LASER_ACTIVE_LEVEL          0

#define LASER_MUESTRAS              5
#define LASER_INTERVALO_MS          2

// =====================================================
// 3. MOTOR Y ENCODER
// =====================================================

// Valor NOMINAL. El homing lo mide y lo reemplaza
// (ver variable conteos_por_rev).
#define CONTEOS_POR_REV             1320.0f

#define ANGULO_MAX                  360.0f

// Limites del contador PCNT (16 bits). El desborde se
// acumula por software para no perder conteos.
#define PCNT_LIM_ALTO               30000
#define PCNT_LIM_BAJO               (-30000)

// =====================================================
// 4. PWM Y TIEMPOS
// =====================================================

#define PWM_FREQ                    1000
#define PWM_MAX                     1023

#define PERIODO_MS                  10
#define DT                          (PERIODO_MS / 1000.0f)

// Debe superar la friccion del motor (PWM_MINIMO = 260).
// Si aun no gira, subir a 350-450.
#define HOMING_PWM                  320

// +1: RPWM
// -1: LPWM
#define HOMING_DIRECTION            1

#define HOMING_RELEASE_TIMEOUT_MS   5000
#define HOMING_SEARCH_TIMEOUT_MS    20000

// Conteos minimos antes de aceptar el primer cruce
#define HOMING_MIN_COUNTS \
    ((int64_t)CONTEOS_POR_REV)

// Conteos minimos entre el primer y el segundo cruce
// (solo para ignorar rebotes del borde)
#define HOMING_MIN_SEGUNDO_CRUCE    100

// =====================================================
// 5. CONTROL PD
// =====================================================

#define KP_POS                      8.0f
#define KD_POS                      0.8f

#define ERROR_TOLERANCIA            2.0f
#define PWM_MINIMO                  260
#define PWM_LIMITE                  PWM_MAX

// --- Generador de trayectoria (evita que se pase) ---
#define VEL_MAX_DEG_S               250.0f   // velocidad maxima
#define ACEL_DEG_S2                 500.0f   // aceleracion y frenado

// Filtro de la velocidad medida (0..1, menor = mas suave)
#define FILTRO_VEL                  0.4f

// Por debajo de esta velocidad se considera "detenido"
// y se aplica el empujon para vencer la friccion.
#define VEL_PARADO_DEG_S            15.0f

// 1 = frenar el motor al llegar (ambos lados del puente en alto)
// 0 = dejar el motor libre (duty 0)
#define USAR_FRENO_ACTIVO           1

// =====================================================
// 6. SERIAL Y TELEPLOT
// =====================================================

#define UART_CONSOLA        UART_NUM_0
#define UART_RX_BUF         256
#define UART_TX_BUF         2048

// Se imprime cada N ciclos de control (N * 10 ms)
#define PRINT_CADA_N        5

// Escala del transportador (grados)
#define ESCALA_PASO         5
#define ESCALA_MAYOR        30

// 1 = tambien dibuja agujas en 3D (opcional)
#define USAR_AGUJA_3D       0

// =====================================================
// 6b. MEDICION DE CORRIENTE (pines R_IS y L_IS del IBT-2)
// =====================================================

// Conectar cada pin IS a su entrada ADC1 por una
// resistencia de 1 kohm en serie (proteccion).
#define ADC_R_IS_CHANNEL        ADC_CHANNEL_6    // GPIO34
#define ADC_L_IS_CHANNEL        ADC_CHANNEL_7    // GPIO35

// Atenuacion: con 0 dB el rango util es ~0.1 a 0.95 V
// (unos 8 A con el modulo tipico). Con corrientes bajas
// da mejor resolucion que 12 dB.
#define IS_ATTEN                ADC_ATTEN_DB_0
#define IS_ADC_FS_MV            1100   // solo si no hay calibracion de fabrica

// Resistencia a tierra de cada salida IS en el modulo
// (verificar en tu modulo; el IBT-2 tipico trae 1 kohm).
#define IS_RESISTENCIA_OHM      1000.0f

// Relacion de corriente del BTS7960: I_carga = 8500 * I_IS
// (valor tipico del fabricante; varia entre unidades).
#define IS_KILIS                8500.0f

// Factor de ajuste: corriente_real / corriente_leida,
// medido con un multimetro en serie con el motor.
#define CORRIENTE_AJUSTE        1.0f

// Amperes por volt en el pin IS (8.5 A/V con 1 kohm)
#define CORRIENTE_A_POR_V \
    ((IS_KILIS / IS_RESISTENCIA_OHM) * CORRIENTE_AJUSTE)

// El IS solo entrega senal mientras el lado alto del puente
// conduce, es decir, pulsa al ritmo del PWM (1 kHz). Se toma
// una rafaga que cubre al menos un periodo y se promedian
// solo las muestras de la fase activa.
#define IS_RAFAGA_US            2500
#define IS_MAX_MUESTRAS         200
#define IS_UMBRAL_MV            10.0f    // por debajo = 0 A
#define IS_PERIODO_MS           20
#define FILTRO_CORRIENTE        0.3f

// Proteccion opcional: apaga el motor si la corriente supera
// el limite durante cierto tiempo (por ejemplo, motor atorado).
#define USAR_PROTECCION_CORRIENTE   0
#define LIMITE_CORRIENTE_A          3.0f
#define LIMITE_TIEMPO_MS            300

#define GRAD_A_RAD          (3.14159265f / 180.0f)

static const char *TAG = "CONTROL_POSICION";

static pcnt_unit_handle_t encoder_unit;
static pcnt_channel_handle_t encoder_chan_a;
static pcnt_channel_handle_t encoder_chan_b;

// Conteos por vuelta reales (se miden en el homing)
static float conteos_por_rev = CONTEOS_POR_REV;

// Posicion angular respecto al cero optico (conteos)
static int64_t posicion_conteos = 0;

// Conteo total del encoder en la lectura anterior
static int total_prev = 0;

// Desborde acumulado del PCNT
static volatile int32_t pcnt_desborde = 0;

// Objetivo escrito por serial (grados)
static volatile float angulo_objetivo = 0.0f;

// Captura del borde de referencia (la llena la ISR)
static volatile bool isr_armada      = false;
static volatile bool borde_capturado = false;
static volatile int  borde_total     = 0;
static volatile int  isr_base        = 0;
static volatile int  isr_minimo      = 0;

static TaskHandle_t tarea_escala_handle = NULL;

// Sensor de corriente
static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle = NULL;
static float offset_is_mv = 0.0f;           // cero del sensor (R+L)
static volatile float corriente_a = 0.0f;   // corriente estimada (A)

// =====================================================
// 7. LED
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

    ESP_ERROR_CHECK(
        gpio_set_level(PIN_LED, !LED_ACTIVO)
    );
}

static void actualizar_led_laser(void)
{
    int nivel = gpio_get_level(PIN_RECEPTOR);

    bool detectado = (nivel == LASER_ACTIVE_LEVEL);

    gpio_set_level(
        PIN_LED,
        detectado ? LED_ACTIVO : !LED_ACTIVO
    );
}

// =====================================================
// 8. MOTOR
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
    if (duty > PWM_LIMITE)
        duty = PWM_LIMITE;

    if (duty < -PWM_LIMITE)
        duty = -PWM_LIMITE;

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

// Freno activo: ambos lados del puente H en alto.
static void frenar_motor(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, PWM_MAX));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, PWM_MAX));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}

// =====================================================
// 9. CONFIGURACION PWM Y ENABLE
// =====================================================

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
// 10. LASER Y RECEPTOR
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

    // Mantener apagado durante la configuracion
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

// Lectura filtrada del estado de referencia
static bool laser_activo(void)
{
    int activas = 0;

    for (int i = 0; i < LASER_MUESTRAS; i++) {
        actualizar_led_laser();

        if (gpio_get_level(PIN_RECEPTOR) ==
            LASER_ACTIVE_LEVEL) {
            activas++;
        }

        vTaskDelay(pdMS_TO_TICKS(LASER_INTERVALO_MS));
    }

    return activas >= (LASER_MUESTRAS / 2 + 1);
}

// =====================================================
// 11. MONITOR DEL RECEPTOR
// =====================================================

static void tarea_monitor_laser(void *pvParameters)
{
    int ultimo_nivel = -1;

    while (1) {
        int nivel = gpio_get_level(PIN_RECEPTOR);

        actualizar_led_laser();

        if (nivel != ultimo_nivel) {
            ESP_LOGI(
                TAG,
                "Receptor=%d | Referencia=%s",
                nivel,
                (nivel == LASER_ACTIVE_LEVEL)
                    ? "ACTIVA" : "INACTIVA"
            );

            ultimo_nivel = nivel;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// =====================================================
// 12. ENCODER PCNT (conteo total sin perdidas)
//     Requiere CONFIG_PCNT_CTRL_FUNC_IN_IRAM=y
//     y CONFIG_PCNT_ISR_IRAM_SAFE=y
// =====================================================

// Acumula el desborde cuando el contador llega a un limite
static bool IRAM_ATTR pcnt_al_limite(
    pcnt_unit_handle_t unit,
    const pcnt_watch_event_data_t *edata,
    void *user_ctx)
{
    pcnt_desborde = pcnt_desborde + edata->watch_point_value;
    return false;
}

// Conteo total = desborde acumulado + conteo del hardware.
// Se repite la lectura si ocurrio un desborde a la mitad.
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

    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = 1000
    };

    ESP_ERROR_CHECK(
        pcnt_unit_set_glitch_filter(encoder_unit, &filter_cfg)
    );

    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num = PIN_ENC_A,
        .level_gpio_num = PIN_ENC_B
    };

    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num = PIN_ENC_B,
        .level_gpio_num = PIN_ENC_A
    };

    ESP_ERROR_CHECK(
        pcnt_new_channel(encoder_unit, &chan_a_cfg, &encoder_chan_a)
    );

    ESP_ERROR_CHECK(
        pcnt_new_channel(encoder_unit, &chan_b_cfg, &encoder_chan_b)
    );

    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(
            encoder_chan_a,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE
        )
    );

    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(
            encoder_chan_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE
        )
    );

    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(
            encoder_chan_b,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE
        )
    );

    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(
            encoder_chan_b,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE
        )
    );

    // Puntos de vigilancia en los limites para acumular
    // el desborde por software
    ESP_ERROR_CHECK(
        pcnt_unit_add_watch_point(encoder_unit, PCNT_LIM_ALTO)
    );

    ESP_ERROR_CHECK(
        pcnt_unit_add_watch_point(encoder_unit, PCNT_LIM_BAJO)
    );

    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_al_limite
    };

    ESP_ERROR_CHECK(
        pcnt_unit_register_event_callbacks(encoder_unit, &cbs, NULL)
    );

    ESP_ERROR_CHECK(pcnt_unit_enable(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(encoder_unit));

    pcnt_desborde = 0;

    ESP_LOGI(TAG, "Encoder PCNT inicializado");
}

// =====================================================
// 13. ISR DE LA REFERENCIA
//     Guarda el conteo total exacto en el flanco.
// =====================================================

static void IRAM_ATTR isr_referencia(void *arg)
{
    if (!isr_armada || borde_capturado)
        return;

    if (gpio_get_level(PIN_RECEPTOR) != LASER_ACTIVE_LEVEL)
        return;

    int total = leer_total();

    // Ignorar cruces muy cercanos al punto de referencia
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

    ESP_ERROR_CHECK(
        gpio_isr_handler_add(PIN_RECEPTOR, isr_referencia, NULL)
    );
}

// Espera a que la ISR capture un borde valido
static bool esperar_borde(int timeout_ms)
{
    int64_t inicio = esp_timer_get_time();

    while (1) {
        int64_t transcurrido =
            (esp_timer_get_time() - inicio) / 1000;

        if (transcurrido >= timeout_ms)
            return false;

        if (borde_capturado) {
            // Confirmacion anti-ruido: el haz debe seguir activo
            esp_rom_delay_us(3000);

            if (gpio_get_level(PIN_RECEPTOR) == LASER_ACTIVE_LEVEL)
                return true;

            borde_capturado = false;   // era ruido
        }

        vTaskDelay(1);
    }
}

// =====================================================
// 14. HOMING: CERO OPTICO Y CONTEOS POR VUELTA
// =====================================================

static bool ejecutar_homing(void)
{
    ESP_LOGW(TAG, "Iniciando homing optico");

    detener_motor();
    ESP_ERROR_CHECK(gpio_set_level(PIN_ENABLE, 1));

    // ---------------------------------------------
    // FASE A: si comienza sobre la referencia,
    // alejarse hasta que el receptor cambie de estado
    // ---------------------------------------------

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

    // ---------------------------------------------
    // FASE B: primer cruce del haz (cero optico)
    // ---------------------------------------------

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

    // ---------------------------------------------
    // FASE C: segundo cruce. Los conteos entre los
    // dos cruces son exactamente una vuelta del eje.
    // ---------------------------------------------

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

    // Esperar a que termine el movimiento residual
    vTaskDelay(pdMS_TO_TICKS(300));

    int vuelta = abs(borde2 - borde1);

    conteos_por_rev = (float)vuelta;

    ESP_LOGI(TAG, "Conteos por vuelta medidos: %d (nominal %.0f)",
             vuelta, (float)CONTEOS_POR_REV);

    float relacion = conteos_por_rev / (float)CONTEOS_POR_REV;

    if (relacion < 0.8f || relacion > 1.2f) {
        ESP_LOGW(TAG,
                 "El valor medido difiere mas de 20%% del nominal. "
                 "Actualiza CONTEOS_POR_REV a %d",
                 vuelta);
    }

    // Posicion actual relativa al ultimo cruce (el cero)
    int total_final = leer_total();

    posicion_conteos = (int64_t)total_final - (int64_t)borde2;
    total_prev = total_final;

    ESP_LOGI(TAG, "Cero angular establecido en el cruce");
    ESP_LOGI(
        TAG,
        "Desplazamiento posterior: %.2f grados",
        (float)posicion_conteos * 360.0f / conteos_por_rev
    );
    ESP_LOGI(TAG, "Homing finalizado");

    return true;
}

// =====================================================
// 14b. MEDICION DE CORRIENTE
// =====================================================

static int raw_a_mv(int raw)
{
    int mv = 0;

    if (adc_cali_handle != NULL &&
        adc_cali_raw_to_voltage(adc_cali_handle, raw, &mv) == ESP_OK) {
        return mv;
    }

    // Sin calibracion de fabrica: conversion lineal aproximada
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

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(adc_handle, ADC_R_IS_CHANNEL, &chan_cfg)
    );

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(adc_handle, ADC_L_IS_CHANNEL, &chan_cfg)
    );

    // Calibracion de fabrica del ADC (si el chip la tiene)
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

// Mide el cero del sensor con el motor parado y habilitado
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

    ESP_LOGI(TAG, "Offset del sensor de corriente: %.1f mV",
             offset_is_mv);
}

// Rafaga de muestras. Devuelve el promedio (mV) de las
// muestras de la fase activa del PWM, sin offset.
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

        // Solo conduce un lado a la vez: la suma da la corriente
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

    // Promedio de las muestras de la fase activa
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
// 15. ENTRADA DEL ANGULO POR SERIAL
//     - Numero (ej. 90)  -> nuevo objetivo en grados
//     - "e"              -> vuelve a dibujar la escala
// =====================================================

static void procesar_linea(const char *linea)
{
    if (linea[0] == 'e' || linea[0] == 'E') {
        if (tarea_escala_handle != NULL)
            xTaskNotifyGive(tarea_escala_handle);
        return;
    }

    char *fin = NULL;
    float valor = strtof(linea, &fin);

    if (fin == linea) {
        ESP_LOGW(TAG, "Entrada invalida: '%s'", linea);
        return;
    }

    if (valor < 0.0f || valor > ANGULO_MAX) {
        ESP_LOGW(TAG, "Fuera de rango (0 a %.0f): %.1f",
                 (float)ANGULO_MAX, valor);
        return;
    }

    angulo_objetivo = valor;
    ESP_LOGI(TAG, "Nuevo objetivo: %.1f grados", valor);
}

static void tarea_serial(void *pvParameters)
{
    char linea[32];
    int n = 0;
    uint8_t byte;

    while (1) {
        int r = uart_read_bytes(UART_CONSOLA, &byte, 1,
                                pdMS_TO_TICKS(50));

        if (r <= 0)
            continue;

        if (byte == '\n' || byte == '\r') {
            if (n > 0) {
                linea[n] = '\0';
                procesar_linea(linea);
                n = 0;
            }
        } else if (n < (int)sizeof(linea) - 1) {
            linea[n++] = (char)byte;
        } else {
            n = 0;   // linea demasiado larga, descartar
        }
    }
}

// =====================================================
// 16. TRANSPORTADOR EN TELEPLOT
// =====================================================

static void tarea_escala(void *pvParameters)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Arco exterior (radio 1)
        for (int a = 0; a <= (int)ANGULO_MAX; a += ESCALA_PASO) {
            printf(">escala:%.3f:%.3f|xy\n",
                   cosf(a * GRAD_A_RAD), sinf(a * GRAD_A_RAD));
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        // Marcas largas (radio 0.90)
        for (int a = 0; a <= (int)ANGULO_MAX; a += ESCALA_MAYOR) {
            printf(">marcas:%.3f:%.3f|xy\n",
                   0.90f * cosf(a * GRAD_A_RAD),
                   0.90f * sinf(a * GRAD_A_RAD));
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        // Centro del transportador
        printf(">centro:0:0|xy\n");
    }
}

static void imprimir_telemetria(float angulo_real,
                                float angulo_deseado,
                                float error,
                                float velocidad,
                                int duty)
{
    // Cada printf es una linea completa para no mezclarse
    // con las lineas de otras tareas.

    printf(">real_xy:%.3f:%.3f|xy\n",
           cosf(angulo_real * GRAD_A_RAD),
           sinf(angulo_real * GRAD_A_RAD));

    printf(">ref_xy:%.3f:%.3f|xy\n",
           cosf(angulo_deseado * GRAD_A_RAD),
           sinf(angulo_deseado * GRAD_A_RAD));

    printf(">ref:%.1f\n",   angulo_deseado);
    printf(">real:%.1f\n",  angulo_real);
    printf(">error:%.1f\n", error);
    printf(">vel:%.1f\n",   velocidad);
    printf(">pwm:%d\n",     duty);

#if USAR_AGUJA_3D
    {
        const float L = 1.0f;

        printf(">3D|aguja_real:S:cube:P:%.3f:%.3f:0:RPY:0:0:%.1f:"
               "W:%.2f:H:0.03:D:0.03:C:red\n",
               (L / 2) * cosf(angulo_real * GRAD_A_RAD),
               (L / 2) * sinf(angulo_real * GRAD_A_RAD),
               angulo_real, L);

        printf(">3D|aguja_ref:S:cube:P:%.3f:%.3f:0:RPY:0:0:%.1f:"
               "W:%.2f:H:0.02:D:0.02:C:green\n",
               (L / 2) * cosf(angulo_deseado * GRAD_A_RAD),
               (L / 2) * sinf(angulo_deseado * GRAD_A_RAD),
               angulo_deseado, L);
    }
#endif
}

// =====================================================
// 17. PROGRAMA PRINCIPAL
// =====================================================

void app_main(void)
{
    configurar_pwm();
    configurar_laser_receptor();
    configurar_led();
    configurar_encoder();
    configurar_isr_referencia();   // despues del encoder
    configurar_adc_corriente();

    // Buffer de transmision: printf no bloquea el lazo
    // de control mientras se envian los datos.
    ESP_ERROR_CHECK(
        uart_driver_install(UART_CONSOLA, UART_RX_BUF * 2,
                            UART_TX_BUF, 0, NULL, 0)
    );

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "CONTROL PD DE POSICION");
    ESP_LOGI(TAG, "Motor JGA25-370 + BTS7960");
    ESP_LOGI(TAG, "Laser: GPIO22");
    ESP_LOGI(TAG, "Receptor: GPIO23");
    ESP_LOGI(TAG, "Encoder: GPIO32 y GPIO33");
    ESP_LOGI(TAG, "Objetivo: escribe el angulo por serial");
    ESP_LOGI(TAG, "======================================");

    BaseType_t tarea_ok = xTaskCreate(
        tarea_monitor_laser,
        "monitor_laser",
        2048,
        NULL,
        4,
        NULL
    );

    if (tarea_ok != pdPASS) {
        ESP_LOGE(TAG, "No se pudo crear monitor laser");
        detener_motor();
        gpio_set_level(PIN_ENABLE, 0);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Calibrar el cero del sensor de corriente (motor parado)
    ESP_ERROR_CHECK(gpio_set_level(PIN_ENABLE, 1));
    vTaskDelay(pdMS_TO_TICKS(100));
    calibrar_offset_corriente();
    ESP_ERROR_CHECK(gpio_set_level(PIN_ENABLE, 0));

    // No iniciar control de posicion si falla homing
    if (!ejecutar_homing()) {
        detener_motor();
        gpio_set_level(PIN_ENABLE, 0);

        // Repetir el error para verlo aunque se conecte
        // Teleplot despues del arranque
        while (1) {
            ESP_LOGE(TAG, "Homing fallido; motor deshabilitado");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Al terminar el homing, ir a 0 grados
    angulo_objetivo = 0.0f;

    xTaskCreate(tarea_serial, "serial", 3072, NULL, 3, NULL);

    xTaskCreate(tarea_escala, "escala", 3072, NULL, 2,
                &tarea_escala_handle);

    // Dibuja el transportador una vez al arrancar
    xTaskNotifyGive(tarea_escala_handle);

    // Medicion de corriente en el nucleo 1 para no
    // interferir con el lazo de control (nucleo 0)
    xTaskCreatePinnedToCore(tarea_corriente, "corriente", 3072,
                            NULL, 2, NULL, 1);

    ESP_LOGI(TAG, "Iniciando control PD");

    float velocidad_angular = 0.0f;
    float vel_filtrada = 0.0f;
    int contador_print = 0;

#if USAR_PROTECCION_CORRIENTE
    int64_t t_sobrecorriente_us = 0;
#endif

    // La referencia intermedia arranca en la posicion actual
    float sp_pos = ((float)posicion_conteos * 360.0f) /
                   conteos_por_rev;
    float sp_vel = 0.0f;

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
        // 1. Leer encoder SIN borrar el contador.
        //    Se calcula la diferencia con la lectura
        //    anterior, asi no se pierde ningun conteo.
        // ---------------------------------------------

        int total = leer_total();
        int delta_conteos = total - total_prev;
        total_prev = total;

        posicion_conteos += delta_conteos;

        velocidad_angular =
            ((float)delta_conteos * 360.0f) /
            (conteos_por_rev * dt);

        vel_filtrada += FILTRO_VEL *
                        (velocidad_angular - vel_filtrada);

        // ---------------------------------------------
        // 2. Posicion real respecto al cero optico
        // ---------------------------------------------

        float angulo_real =
            ((float)posicion_conteos * 360.0f) /
            conteos_por_rev;

        // ---------------------------------------------
        // 3. Objetivo final (escrito por serial)
        // ---------------------------------------------

        float objetivo = angulo_objetivo;

        // ---------------------------------------------
        // 4. Generador de trayectoria
        // ---------------------------------------------

        float dist = objetivo - sp_pos;

        float v_obj = sqrtf(2.0f * ACEL_DEG_S2 * fabsf(dist));

        if (v_obj > VEL_MAX_DEG_S)
            v_obj = VEL_MAX_DEG_S;

        if (dist < 0.0f)
            v_obj = -v_obj;

        float dv     = v_obj - sp_vel;
        float dv_max = ACEL_DEG_S2 * dt;

        if (dv > dv_max)
            dv = dv_max;

        if (dv < -dv_max)
            dv = -dv_max;

        sp_vel += dv;
        sp_pos += sp_vel * dt;

        // Cerca del final, fijar la referencia en el objetivo
        if (fabsf(objetivo - sp_pos) < 0.2f) {
            sp_pos = objetivo;
            sp_vel = 0.0f;
        }

        // ---------------------------------------------
        // 5. Error y control PD
        // ---------------------------------------------

        float error    = objetivo - angulo_real;
        float error_sp = sp_pos - angulo_real;

        int  duty  = 0;
        bool freno = false;

        if (fabsf(error) <= ERROR_TOLERANCIA && sp_vel == 0.0f) {
            // Llego: parar y mantener
            duty  = 0;
            freno = true;
        } else {
            float termino_p = KP_POS * error_sp;
            float termino_d = KD_POS * (sp_vel - vel_filtrada);

            float salida = termino_p + termino_d;

            if (salida > PWM_MAX)
                salida = PWM_MAX;

            if (salida < -PWM_MAX)
                salida = -PWM_MAX;

            duty = (int)salida;

            // Empujon para vencer la friccion estatica,
            // solo si el motor esta detenido y falta recorrido.
            if (fabsf(vel_filtrada) < VEL_PARADO_DEG_S &&
                fabsf(error) > ERROR_TOLERANCIA &&
                fabsf(salida) < PWM_MINIMO) {

                duty = (error > 0.0f)
                    ? PWM_MINIMO
                    : -PWM_MINIMO;
            }
        }

#if USAR_PROTECCION_CORRIENTE
        // Proteccion: corriente alta sostenida (motor atorado)
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

            imprimir_telemetria(angulo_real,
                                objetivo,
                                error,
                                vel_filtrada,
                                duty);

            printf(">sp:%.1f\n", sp_pos);
            printf(">corriente:%.2f\n", corriente_a);
        }

        // Periodo fijo de 10 ms, sin acumular retardos
        vTaskDelayUntil(&ultimo_despertar,
                        pdMS_TO_TICKS(PERIODO_MS));
    }
}




/*// =====================================================
// CONTROL PD DE POSICION CON CERO OPTICO
// Motor JGA25-370 + BTS7960 + ESP32 (ESP-IDF)
//
// - Cero angular por cruce de laser (captura con ISR)
// - Conteos por vuelta MEDIDOS en el homing (dos cruces)
// - Conteo del encoder sin perdidas (sin clear en el lazo)
// - Objetivo escrito por serial (monitor o Teleplot)
// - Trayectoria suave + freno al llegar
// - Transportador en Teleplot (grafica XY)
// =====================================================

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"
#include "driver/uart.h"

#include "esp_err.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

// =====================================================
// 1. CONFIGURACION DE PINES
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
// 2. CONFIGURACION DE LA REFERENCIA OPTICA
// =====================================================

// 0 = LOW indica referencia
// 1 = HIGH indica referencia
#define LASER_ACTIVE_LEVEL          0

#define LASER_MUESTRAS              5
#define LASER_INTERVALO_MS          2

// =====================================================
// 3. MOTOR Y ENCODER
// =====================================================

// Valor NOMINAL. El homing lo mide y lo reemplaza
// (ver variable conteos_por_rev).
#define CONTEOS_POR_REV             1320.0f

#define ANGULO_MAX                  360.0f

// Limites del contador PCNT (16 bits). El desborde se
// acumula por software para no perder conteos.
#define PCNT_LIM_ALTO               30000
#define PCNT_LIM_BAJO               (-30000)

// =====================================================
// 4. PWM Y TIEMPOS
// =====================================================

#define PWM_FREQ                    1000
#define PWM_MAX                     1023

#define PERIODO_MS                  10
#define DT                          (PERIODO_MS / 1000.0f)

// Debe superar la friccion del motor (PWM_MINIMO = 260).
// Si aun no gira, subir a 350-450.
#define HOMING_PWM                  320

// +1: RPWM
// -1: LPWM
#define HOMING_DIRECTION            1

#define HOMING_RELEASE_TIMEOUT_MS   5000
#define HOMING_SEARCH_TIMEOUT_MS    20000

// Conteos minimos antes de aceptar el primer cruce
#define HOMING_MIN_COUNTS \
    ((int64_t)CONTEOS_POR_REV)

// Conteos minimos entre el primer y el segundo cruce
// (solo para ignorar rebotes del borde)
#define HOMING_MIN_SEGUNDO_CRUCE    100

// =====================================================
// 5. CONTROL PD
// =====================================================

#define KP_POS                      8.0f
#define KD_POS                      0.8f

#define ERROR_TOLERANCIA            2.0f
#define PWM_MINIMO                  260
#define PWM_LIMITE                  PWM_MAX

// --- Generador de trayectoria (evita que se pase) ---
#define VEL_MAX_DEG_S               250.0f   // velocidad maxima
#define ACEL_DEG_S2                 500.0f   // aceleracion y frenado

// Filtro de la velocidad medida (0..1, menor = mas suave)
#define FILTRO_VEL                  0.4f

// Por debajo de esta velocidad se considera "detenido"
// y se aplica el empujon para vencer la friccion.
#define VEL_PARADO_DEG_S            15.0f

// 1 = frenar el motor al llegar (ambos lados del puente en alto)
// 0 = dejar el motor libre (duty 0)
#define USAR_FRENO_ACTIVO           1

// =====================================================
// 6. SERIAL Y TELEPLOT
// =====================================================

#define UART_CONSOLA        UART_NUM_0
#define UART_RX_BUF         256
#define UART_TX_BUF         2048

// Se imprime cada N ciclos de control (N * 10 ms)
#define PRINT_CADA_N        5

// Escala del transportador (grados)
#define ESCALA_PASO         5
#define ESCALA_MAYOR        30

// 1 = tambien dibuja agujas en 3D (opcional)
#define USAR_AGUJA_3D       0

#define GRAD_A_RAD          (3.14159265f / 180.0f)

static const char *TAG = "CONTROL_POSICION";

static pcnt_unit_handle_t encoder_unit;
static pcnt_channel_handle_t encoder_chan_a;
static pcnt_channel_handle_t encoder_chan_b;

// Conteos por vuelta reales (se miden en el homing)
static float conteos_por_rev = CONTEOS_POR_REV;

// Posicion angular respecto al cero optico (conteos)
static int64_t posicion_conteos = 0;

// Conteo total del encoder en la lectura anterior
static int total_prev = 0;

// Desborde acumulado del PCNT
static volatile int32_t pcnt_desborde = 0;

// Objetivo escrito por serial (grados)
static volatile float angulo_objetivo = 0.0f;

// Captura del borde de referencia (la llena la ISR)
static volatile bool isr_armada      = false;
static volatile bool borde_capturado = false;
static volatile int  borde_total     = 0;
static volatile int  isr_base        = 0;
static volatile int  isr_minimo      = 0;

static TaskHandle_t tarea_escala_handle = NULL;

// =====================================================
// 7. LED
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

    ESP_ERROR_CHECK(
        gpio_set_level(PIN_LED, !LED_ACTIVO)
    );
}

static void actualizar_led_laser(void)
{
    int nivel = gpio_get_level(PIN_RECEPTOR);

    bool detectado = (nivel == LASER_ACTIVE_LEVEL);

    gpio_set_level(
        PIN_LED,
        detectado ? LED_ACTIVO : !LED_ACTIVO
    );
}

// =====================================================
// 8. MOTOR
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
    if (duty > PWM_LIMITE)
        duty = PWM_LIMITE;

    if (duty < -PWM_LIMITE)
        duty = -PWM_LIMITE;

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

// Freno activo: ambos lados del puente H en alto.
static void frenar_motor(void)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, PWM_MAX));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0));

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, PWM_MAX));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1));
}

// =====================================================
// 9. CONFIGURACION PWM Y ENABLE
// =====================================================

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
// 10. LASER Y RECEPTOR
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

    // Mantener apagado durante la configuracion
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

// Lectura filtrada del estado de referencia
static bool laser_activo(void)
{
    int activas = 0;

    for (int i = 0; i < LASER_MUESTRAS; i++) {
        actualizar_led_laser();

        if (gpio_get_level(PIN_RECEPTOR) ==
            LASER_ACTIVE_LEVEL) {
            activas++;
        }

        vTaskDelay(pdMS_TO_TICKS(LASER_INTERVALO_MS));
    }

    return activas >= (LASER_MUESTRAS / 2 + 1);
}

// =====================================================
// 11. MONITOR DEL RECEPTOR
// =====================================================

static void tarea_monitor_laser(void *pvParameters)
{
    int ultimo_nivel = -1;

    while (1) {
        int nivel = gpio_get_level(PIN_RECEPTOR);

        actualizar_led_laser();

        if (nivel != ultimo_nivel) {
            ESP_LOGI(
                TAG,
                "Receptor=%d | Referencia=%s",
                nivel,
                (nivel == LASER_ACTIVE_LEVEL)
                    ? "ACTIVA" : "INACTIVA"
            );

            ultimo_nivel = nivel;
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// =====================================================
// 12. ENCODER PCNT (conteo total sin perdidas)
//     Requiere CONFIG_PCNT_CTRL_FUNC_IN_IRAM=y
//     y CONFIG_PCNT_ISR_IRAM_SAFE=y
// =====================================================

// Acumula el desborde cuando el contador llega a un limite
static bool IRAM_ATTR pcnt_al_limite(
    pcnt_unit_handle_t unit,
    const pcnt_watch_event_data_t *edata,
    void *user_ctx)
{
    pcnt_desborde = pcnt_desborde + edata->watch_point_value;
    return false;
}

// Conteo total = desborde acumulado + conteo del hardware.
// Se repite la lectura si ocurrio un desborde a la mitad.
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

    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = 1000
    };

    ESP_ERROR_CHECK(
        pcnt_unit_set_glitch_filter(encoder_unit, &filter_cfg)
    );

    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num = PIN_ENC_A,
        .level_gpio_num = PIN_ENC_B
    };

    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num = PIN_ENC_B,
        .level_gpio_num = PIN_ENC_A
    };

    ESP_ERROR_CHECK(
        pcnt_new_channel(encoder_unit, &chan_a_cfg, &encoder_chan_a)
    );

    ESP_ERROR_CHECK(
        pcnt_new_channel(encoder_unit, &chan_b_cfg, &encoder_chan_b)
    );

    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(
            encoder_chan_a,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE
        )
    );

    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(
            encoder_chan_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE
        )
    );

    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(
            encoder_chan_b,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE
        )
    );

    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(
            encoder_chan_b,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE
        )
    );

    // Puntos de vigilancia en los limites para acumular
    // el desborde por software
    ESP_ERROR_CHECK(
        pcnt_unit_add_watch_point(encoder_unit, PCNT_LIM_ALTO)
    );

    ESP_ERROR_CHECK(
        pcnt_unit_add_watch_point(encoder_unit, PCNT_LIM_BAJO)
    );

    pcnt_event_callbacks_t cbs = {
        .on_reach = pcnt_al_limite
    };

    ESP_ERROR_CHECK(
        pcnt_unit_register_event_callbacks(encoder_unit, &cbs, NULL)
    );

    ESP_ERROR_CHECK(pcnt_unit_enable(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(encoder_unit));

    pcnt_desborde = 0;

    ESP_LOGI(TAG, "Encoder PCNT inicializado");
}

// =====================================================
// 13. ISR DE LA REFERENCIA
//     Guarda el conteo total exacto en el flanco.
// =====================================================

static void IRAM_ATTR isr_referencia(void *arg)
{
    if (!isr_armada || borde_capturado)
        return;

    if (gpio_get_level(PIN_RECEPTOR) != LASER_ACTIVE_LEVEL)
        return;

    int total = leer_total();

    // Ignorar cruces muy cercanos al punto de referencia
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

    ESP_ERROR_CHECK(
        gpio_isr_handler_add(PIN_RECEPTOR, isr_referencia, NULL)
    );
}

// Espera a que la ISR capture un borde valido
static bool esperar_borde(int timeout_ms)
{
    int64_t inicio = esp_timer_get_time();

    while (1) {
        int64_t transcurrido =
            (esp_timer_get_time() - inicio) / 1000;

        if (transcurrido >= timeout_ms)
            return false;

        if (borde_capturado) {
            // Confirmacion anti-ruido: el haz debe seguir activo
            esp_rom_delay_us(3000);

            if (gpio_get_level(PIN_RECEPTOR) == LASER_ACTIVE_LEVEL)
                return true;

            borde_capturado = false;   // era ruido
        }

        vTaskDelay(1);
    }
}

// =====================================================
// 14. HOMING: CERO OPTICO Y CONTEOS POR VUELTA
// =====================================================

static bool ejecutar_homing(void)
{
    ESP_LOGW(TAG, "Iniciando homing optico");

    detener_motor();
    ESP_ERROR_CHECK(gpio_set_level(PIN_ENABLE, 1));

    // ---------------------------------------------
    // FASE A: si comienza sobre la referencia,
    // alejarse hasta que el receptor cambie de estado
    // ---------------------------------------------

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

    // ---------------------------------------------
    // FASE B: primer cruce del haz (cero optico)
    // ---------------------------------------------

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

    // ---------------------------------------------
    // FASE C: segundo cruce. Los conteos entre los
    // dos cruces son exactamente una vuelta del eje.
    // ---------------------------------------------

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

    // Esperar a que termine el movimiento residual
    vTaskDelay(pdMS_TO_TICKS(300));

    int vuelta = abs(borde2 - borde1);

    conteos_por_rev = (float)vuelta;

    ESP_LOGI(TAG, "Conteos por vuelta medidos: %d (nominal %.0f)",
             vuelta, (float)CONTEOS_POR_REV);

    float relacion = conteos_por_rev / (float)CONTEOS_POR_REV;

    if (relacion < 0.8f || relacion > 1.2f) {
        ESP_LOGW(TAG,
                 "El valor medido difiere mas de 20%% del nominal. "
                 "Actualiza CONTEOS_POR_REV a %d",
                 vuelta);
    }

    // Posicion actual relativa al ultimo cruce (el cero)
    int total_final = leer_total();

    posicion_conteos = (int64_t)total_final - (int64_t)borde2;
    total_prev = total_final;

    ESP_LOGI(TAG, "Cero angular establecido en el cruce");
    ESP_LOGI(
        TAG,
        "Desplazamiento posterior: %.2f grados",
        (float)posicion_conteos * 360.0f / conteos_por_rev
    );
    ESP_LOGI(TAG, "Homing finalizado");

    return true;
}

// =====================================================
// 15. ENTRADA DEL ANGULO POR SERIAL
//     - Numero (ej. 90)  -> nuevo objetivo en grados
//     - "e"              -> vuelve a dibujar la escala
// =====================================================

static void procesar_linea(const char *linea)
{
    if (linea[0] == 'e' || linea[0] == 'E') {
        if (tarea_escala_handle != NULL)
            xTaskNotifyGive(tarea_escala_handle);
        return;
    }

    char *fin = NULL;
    float valor = strtof(linea, &fin);

    if (fin == linea) {
        ESP_LOGW(TAG, "Entrada invalida: '%s'", linea);
        return;
    }

    if (valor < 0.0f || valor > ANGULO_MAX) {
        ESP_LOGW(TAG, "Fuera de rango (0 a %.0f): %.1f",
                 (float)ANGULO_MAX, valor);
        return;
    }

    angulo_objetivo = valor;
    ESP_LOGI(TAG, "Nuevo objetivo: %.1f grados", valor);
}

static void tarea_serial(void *pvParameters)
{
    char linea[32];
    int n = 0;
    uint8_t byte;

    while (1) {
        int r = uart_read_bytes(UART_CONSOLA, &byte, 1,
                                pdMS_TO_TICKS(50));

        if (r <= 0)
            continue;

        if (byte == '\n' || byte == '\r') {
            if (n > 0) {
                linea[n] = '\0';
                procesar_linea(linea);
                n = 0;
            }
        } else if (n < (int)sizeof(linea) - 1) {
            linea[n++] = (char)byte;
        } else {
            n = 0;   // linea demasiado larga, descartar
        }
    }
}

// =====================================================
// 16. TRANSPORTADOR EN TELEPLOT
// =====================================================

static void tarea_escala(void *pvParameters)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        // Arco exterior (radio 1)
        for (int a = 0; a <= (int)ANGULO_MAX; a += ESCALA_PASO) {
            printf(">escala:%.3f:%.3f|xy\n",
                   cosf(a * GRAD_A_RAD), sinf(a * GRAD_A_RAD));
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        // Marcas largas (radio 0.90)
        for (int a = 0; a <= (int)ANGULO_MAX; a += ESCALA_MAYOR) {
            printf(">marcas:%.3f:%.3f|xy\n",
                   0.90f * cosf(a * GRAD_A_RAD),
                   0.90f * sinf(a * GRAD_A_RAD));
            vTaskDelay(pdMS_TO_TICKS(5));
        }

        // Centro del transportador
        printf(">centro:0:0|xy\n");
    }
}

static void imprimir_telemetria(float angulo_real,
                                float angulo_deseado,
                                float error,
                                float velocidad,
                                int duty)
{
    // Cada printf es una linea completa para no mezclarse
    // con las lineas de otras tareas.

    printf(">real_xy:%.3f:%.3f|xy\n",
           cosf(angulo_real * GRAD_A_RAD),
           sinf(angulo_real * GRAD_A_RAD));

    printf(">ref_xy:%.3f:%.3f|xy\n",
           cosf(angulo_deseado * GRAD_A_RAD),
           sinf(angulo_deseado * GRAD_A_RAD));

    printf(">ref:%.1f\n",   angulo_deseado);
    printf(">real:%.1f\n",  angulo_real);
    printf(">error:%.1f\n", error);
    printf(">vel:%.1f\n",   velocidad);
    printf(">pwm:%d\n",     duty);

#if USAR_AGUJA_3D
    {
        const float L = 1.0f;

        printf(">3D|aguja_real:S:cube:P:%.3f:%.3f:0:RPY:0:0:%.1f:"
               "W:%.2f:H:0.03:D:0.03:C:red\n",
               (L / 2) * cosf(angulo_real * GRAD_A_RAD),
               (L / 2) * sinf(angulo_real * GRAD_A_RAD),
               angulo_real, L);

        printf(">3D|aguja_ref:S:cube:P:%.3f:%.3f:0:RPY:0:0:%.1f:"
               "W:%.2f:H:0.02:D:0.02:C:green\n",
               (L / 2) * cosf(angulo_deseado * GRAD_A_RAD),
               (L / 2) * sinf(angulo_deseado * GRAD_A_RAD),
               angulo_deseado, L);
    }
#endif
}

// =====================================================
// 17. PROGRAMA PRINCIPAL
// =====================================================

void app_main(void)
{
    configurar_pwm();
    configurar_laser_receptor();
    configurar_led();
    configurar_encoder();
    configurar_isr_referencia();   // despues del encoder

    // Buffer de transmision: printf no bloquea el lazo
    // de control mientras se envian los datos.
    ESP_ERROR_CHECK(
        uart_driver_install(UART_CONSOLA, UART_RX_BUF * 2,
                            UART_TX_BUF, 0, NULL, 0)
    );

    ESP_LOGI(TAG, "======================================");
    ESP_LOGI(TAG, "CONTROL PD DE POSICION");
    ESP_LOGI(TAG, "Motor JGA25-370 + BTS7960");
    ESP_LOGI(TAG, "Laser: GPIO22");
    ESP_LOGI(TAG, "Receptor: GPIO23");
    ESP_LOGI(TAG, "Encoder: GPIO32 y GPIO33");
    ESP_LOGI(TAG, "Objetivo: escribe el angulo por serial");
    ESP_LOGI(TAG, "======================================");

    BaseType_t tarea_ok = xTaskCreate(
        tarea_monitor_laser,
        "monitor_laser",
        2048,
        NULL,
        4,
        NULL
    );

    if (tarea_ok != pdPASS) {
        ESP_LOGE(TAG, "No se pudo crear monitor laser");
        detener_motor();
        gpio_set_level(PIN_ENABLE, 0);
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(1000));

    // No iniciar control de posicion si falla homing
    if (!ejecutar_homing()) {
        detener_motor();
        gpio_set_level(PIN_ENABLE, 0);

        // Repetir el error para verlo aunque se conecte
        // Teleplot despues del arranque
        while (1) {
            ESP_LOGE(TAG, "Homing fallido; motor deshabilitado");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }

    // Al terminar el homing, ir a 0 grados
    angulo_objetivo = 0.0f;

    xTaskCreate(tarea_serial, "serial", 3072, NULL, 3, NULL);

    xTaskCreate(tarea_escala, "escala", 3072, NULL, 2,
                &tarea_escala_handle);

    // Dibuja el transportador una vez al arrancar
    xTaskNotifyGive(tarea_escala_handle);

    ESP_LOGI(TAG, "Iniciando control PD");

    float velocidad_angular = 0.0f;
    float vel_filtrada = 0.0f;
    int contador_print = 0;

    // La referencia intermedia arranca en la posicion actual
    float sp_pos = ((float)posicion_conteos * 360.0f) /
                   conteos_por_rev;
    float sp_vel = 0.0f;

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
        // 1. Leer encoder SIN borrar el contador.
        //    Se calcula la diferencia con la lectura
        //    anterior, asi no se pierde ningun conteo.
        // ---------------------------------------------

        int total = leer_total();
        int delta_conteos = total - total_prev;
        total_prev = total;

        posicion_conteos += delta_conteos;

        velocidad_angular =
            ((float)delta_conteos * 360.0f) /
            (conteos_por_rev * dt);

        vel_filtrada += FILTRO_VEL *
                        (velocidad_angular - vel_filtrada);

        // ---------------------------------------------
        // 2. Posicion real respecto al cero optico
        // ---------------------------------------------

        float angulo_real =
            ((float)posicion_conteos * 360.0f) /
            conteos_por_rev;

        // ---------------------------------------------
        // 3. Objetivo final (escrito por serial)
        // ---------------------------------------------

        float objetivo = angulo_objetivo;

        // ---------------------------------------------
        // 4. Generador de trayectoria
        // ---------------------------------------------

        float dist = objetivo - sp_pos;

        float v_obj = sqrtf(2.0f * ACEL_DEG_S2 * fabsf(dist));

        if (v_obj > VEL_MAX_DEG_S)
            v_obj = VEL_MAX_DEG_S;

        if (dist < 0.0f)
            v_obj = -v_obj;

        float dv     = v_obj - sp_vel;
        float dv_max = ACEL_DEG_S2 * dt;

        if (dv > dv_max)
            dv = dv_max;

        if (dv < -dv_max)
            dv = -dv_max;

        sp_vel += dv;
        sp_pos += sp_vel * dt;

        // Cerca del final, fijar la referencia en el objetivo
        if (fabsf(objetivo - sp_pos) < 0.2f) {
            sp_pos = objetivo;
            sp_vel = 0.0f;
        }

        // ---------------------------------------------
        // 5. Error y control PD
        // ---------------------------------------------

        float error    = objetivo - angulo_real;
        float error_sp = sp_pos - angulo_real;

        int  duty  = 0;
        bool freno = false;

        if (fabsf(error) <= ERROR_TOLERANCIA && sp_vel == 0.0f) {
            // Llego: parar y mantener
            duty  = 0;
            freno = true;
        } else {
            float termino_p = KP_POS * error_sp;
            float termino_d = KD_POS * (sp_vel - vel_filtrada);

            float salida = termino_p + termino_d;

            if (salida > PWM_MAX)
                salida = PWM_MAX;

            if (salida < -PWM_MAX)
                salida = -PWM_MAX;

            duty = (int)salida;

            // Empujon para vencer la friccion estatica,
            // solo si el motor esta detenido y falta recorrido.
            if (fabsf(vel_filtrada) < VEL_PARADO_DEG_S &&
                fabsf(error) > ERROR_TOLERANCIA &&
                fabsf(salida) < PWM_MINIMO) {

                duty = (error > 0.0f)
                    ? PWM_MINIMO
                    : -PWM_MINIMO;
            }
        }

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

            imprimir_telemetria(angulo_real,
                                objetivo,
                                error,
                                vel_filtrada,
                                duty);

            printf(">sp:%.1f\n", sp_pos);
        }

        // Periodo fijo de 10 ms, sin acumular retardos
        vTaskDelayUntil(&ultimo_despertar,
                        pdMS_TO_TICKS(PERIODO_MS));
    }
}
*/