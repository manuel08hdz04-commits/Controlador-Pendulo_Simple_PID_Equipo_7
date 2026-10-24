
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/pulse_cnt.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"

// =====================================================
// PINES
// =====================================================

// Potenciometro 10k conectado a GPIO34
#define ADC_POT_CHANNEL ADC_CHANNEL_6

// Puente H IBT-2 / BTS7960
#define PIN_RPWM   25
#define PIN_LPWM   26
#define PIN_ENABLE 27

// Encoder Hall en cuadratura
#define PIN_ENC_A 32
#define PIN_ENC_B 33

// =====================================================
// CONFIGURACION DEL MOTOR
// =====================================================

#define MOTOR_RPM_MAX 330.0f

// Valor teorico inicial:
// 11 pulsos/vuelta del motor x 30:1 x 4
#define CONTEOS_POR_REV 1320

// =====================================================
// PWM
// =====================================================

#define PWM_FREQ 1000
#define PWM_MAX 1023

// Periodo de actualizacion del control
#define PERIODO_MS 100

// =====================================================
// CONTROL PI
// =====================================================

// Ganancias iniciales: requieren ajuste experimental
#define KP 1.2f
#define KI 0.8f

#define INTEGRAL_MAX 500.0f

static const char *TAG = "CONTROL_MOTOR";

static adc_oneshot_unit_handle_t adc_handle;
static pcnt_unit_handle_t encoder_unit;

// =====================================================
// CONFIGURAR ADC
// =====================================================

static void configurar_adc(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };

    ESP_ERROR_CHECK(
        adc_oneshot_new_unit(&init_cfg, &adc_handle)
    );

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_12
    };

    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(
            adc_handle,
            ADC_POT_CHANNEL,
            &chan_cfg
        )
    );
}

// =====================================================
// CONFIGURAR PWM Y PUENTE H
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

    // LPWM y ENABLE como salidas
    gpio_config_t gpio_cfg = {
        .pin_bit_mask =
            (1ULL << PIN_LPWM) |
            (1ULL << PIN_ENABLE),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    ESP_ERROR_CHECK(gpio_config(&gpio_cfg));

    // Solo giro en un sentido
    ESP_ERROR_CHECK(gpio_set_level(PIN_LPWM, 0));

    // Motor deshabilitado durante inicializacion
    ESP_ERROR_CHECK(gpio_set_level(PIN_ENABLE, 0));
}

static void establecer_pwm(uint32_t duty)
{
    if (duty > PWM_MAX) {
        duty = PWM_MAX;
    }

    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_0,
            duty
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            LEDC_CHANNEL_0
        )
    );
}

// =====================================================
// CONFIGURAR ENCODER CON PCNT
// =====================================================

static void configurar_encoder(void)
{
    pcnt_unit_config_t unit_cfg = {
        .low_limit = -30000,
        .high_limit = 30000
    };

    ESP_ERROR_CHECK(
        pcnt_new_unit(&unit_cfg, &encoder_unit)
    );

    pcnt_glitch_filter_config_t filter_cfg = {
        .max_glitch_ns = 1000
    };

    ESP_ERROR_CHECK(
        pcnt_unit_set_glitch_filter(
            encoder_unit,
            &filter_cfg
        )
    );

    pcnt_chan_config_t chan_a_cfg = {
        .edge_gpio_num = PIN_ENC_A,
        .level_gpio_num = PIN_ENC_B
    };

    pcnt_chan_config_t chan_b_cfg = {
        .edge_gpio_num = PIN_ENC_B,
        .level_gpio_num = PIN_ENC_A
    };

    pcnt_channel_handle_t chan_a;
    pcnt_channel_handle_t chan_b;

    ESP_ERROR_CHECK(
        pcnt_new_channel(
            encoder_unit,
            &chan_a_cfg,
            &chan_a
        )
    );

    ESP_ERROR_CHECK(
        pcnt_new_channel(
            encoder_unit,
            &chan_b_cfg,
            &chan_b
        )
    );

    // Canal A: contar ambos flancos
    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(
            chan_a,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE
        )
    );

    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(
            chan_a,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE
        )
    );

    // Canal B: acciones complementarias
    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(
            chan_b,
            PCNT_CHANNEL_EDGE_ACTION_DECREASE,
            PCNT_CHANNEL_EDGE_ACTION_INCREASE
        )
    );

    ESP_ERROR_CHECK(
        pcnt_channel_set_level_action(
            chan_b,
            PCNT_CHANNEL_LEVEL_ACTION_KEEP,
            PCNT_CHANNEL_LEVEL_ACTION_INVERSE
        )
    );

    ESP_ERROR_CHECK(pcnt_unit_enable(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(encoder_unit));
    ESP_ERROR_CHECK(pcnt_unit_start(encoder_unit));

    ESP_LOGI(
        TAG,
        "Encoder configurado: %d conteos/vuelta",
        CONTEOS_POR_REV
    );
}

// =====================================================
// PROGRAMA PRINCIPAL
// =====================================================

void app_main(void)
{
    configurar_pwm();
    configurar_adc();
    configurar_encoder();

    ESP_LOGI(TAG, "Control de velocidad JGA25-370");
    ESP_LOGI(TAG, "Motor nominal: 12 V / 330 RPM");
    ESP_LOGI(TAG, "Control PI activado");
    ESP_LOGI(TAG, "Porcentaje | RPM deseadas | RPM reales | PWM");

    // Tiempo para verificar el sistema antes de habilitar
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_ERROR_CHECK(gpio_set_level(PIN_ENABLE, 1));

    int lectura_adc = 0;
    int conteo_anterior = 0;
    int conteo_actual = 0;

    float integral = 0.0f;
    uint32_t duty = 0;

    const float dt = PERIODO_MS / 1000.0f;

    while (1) {
        // ---------------------------------------------
        // 1. Leer potenciometro
        // ---------------------------------------------

        esp_err_t resultado = adc_oneshot_read(
            adc_handle,
            ADC_POT_CHANNEL,
            &lectura_adc
        );

        if (resultado != ESP_OK) {
            establecer_pwm(0);
            integral = 0.0f;

            ESP_LOGE(TAG, "Error leyendo ADC");

            vTaskDelay(pdMS_TO_TICKS(PERIODO_MS));
            continue;
        }

        // ---------------------------------------------
        // 2. Leer encoder
        // ---------------------------------------------

        ESP_ERROR_CHECK(
            pcnt_unit_get_count(
                encoder_unit,
                &conteo_actual
            )
        );

        int delta = conteo_actual - conteo_anterior;

        // Evitar desbordamiento del contador
        if (conteo_actual >= 20000 ||
            conteo_actual <= -20000) {

            ESP_ERROR_CHECK(
                pcnt_unit_clear_count(encoder_unit)
            );

            conteo_anterior = 0;
            conteo_actual = 0;

        } else {
            conteo_anterior = conteo_actual;
        }

        // ---------------------------------------------
        // 3. Calcular RPM reales
        // ---------------------------------------------

        float rpm_reales =
            fabsf((float)delta) * 60.0f /
            ((float)CONTEOS_POR_REV * dt);

        // ---------------------------------------------
        // 4. Referencia de 0 a 100 %, pasos de 1 %
        // ---------------------------------------------

        int porcentaje =
            (lectura_adc * 100 + 2047) / 4095;

        if (porcentaje < 0) porcentaje = 0;
        if (porcentaje > 100) porcentaje = 100;

        float rpm_deseadas = porcentaje * 3.3f;

        // ---------------------------------------------
        // 5. Control PI
        // ---------------------------------------------

        if (porcentaje == 0) {
            duty = 0;
            integral = 0.0f;

        } else {
            float error = rpm_deseadas - rpm_reales;

            // Estimacion inicial de PWM
            float pwm_base =
                (rpm_deseadas / MOTOR_RPM_MAX) * PWM_MAX;

            // Accion integral
            integral += KI * error * dt;

            if (integral > INTEGRAL_MAX)
                integral = INTEGRAL_MAX;

            if (integral < -INTEGRAL_MAX)
                integral = -INTEGRAL_MAX;

            float salida =
                pwm_base + KP * error + integral;

            if (salida > PWM_MAX)
                salida = PWM_MAX;

            if (salida < 0.0f)
                salida = 0.0f;

            duty = (uint32_t)(salida + 0.5f);
        }

        // ---------------------------------------------
        // 6. Aplicar PWM
        // ---------------------------------------------

        establecer_pwm(duty);

        float porcentaje_pwm =
            ((float)duty * 100.0f) / PWM_MAX;

        // ---------------------------------------------
        // 7. Monitor serie
        // ---------------------------------------------

        ESP_LOGI(
            TAG,
            "Ref: %3d%% | Deseada: %6.1f RPM | "
            "Real: %6.1f RPM | PWM: %5.1f%% (%lu/%d)",
            porcentaje,
            rpm_deseadas,
            rpm_reales,
            porcentaje_pwm,
            (unsigned long)duty,
            PWM_MAX
        );

        vTaskDelay(pdMS_TO_TICKS(PERIODO_MS));
    }
}
