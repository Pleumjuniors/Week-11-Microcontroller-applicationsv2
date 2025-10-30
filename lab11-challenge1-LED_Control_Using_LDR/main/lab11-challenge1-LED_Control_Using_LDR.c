// main.c
// LED Brightness Control with LDR (ESP32 + ESP-IDF)
//
// Function:
// - Read LDR (ADC1_CH7 = GPIO35)
// - Control LED brightness (GPIO18) with PWM (LEDC)
// - Dark -> LED dim
// - Bright -> LED bright

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/ledc.h"
#include "esp_log.h"

#define LDR_CHANNEL     ADC1_CHANNEL_7   // GPIO35
#define DEFAULT_VREF    1100
#define NO_OF_SAMPLES   64

#define LED_GPIO        GPIO_NUM_18
#define LEDC_MODE       LEDC_HIGH_SPEED_MODE
#define LEDC_CHANNEL    LEDC_CHANNEL_0
#define LEDC_TIMER      LEDC_TIMER_0
#define LEDC_RES        LEDC_TIMER_10_BIT   // ความละเอียด 10-bit (0–1023)
#define LEDC_FREQ_HZ    5000                // 5kHz PWM

static const char *TAG = "LDR_LED_PWM";
static esp_adc_cal_characteristics_t *adc_chars;

static void init_adc(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(LDR_CHANNEL, ADC_ATTEN_DB_11);
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(
        ADC_UNIT_1, ADC_ATTEN_DB_11, ADC_WIDTH_BIT_12,
        DEFAULT_VREF, adc_chars);
}

static void init_led_pwm(void) {
    // ตั้งค่า timer
    ledc_timer_config_t timer_conf = {
        .speed_mode       = LEDC_MODE,
        .duty_resolution  = LEDC_RES,
        .timer_num        = LEDC_TIMER,
        .freq_hz          = LEDC_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&timer_conf);

    // ตั้งค่า channel
    ledc_channel_config_t ch_conf = {
        .gpio_num       = LED_GPIO,
        .speed_mode     = LEDC_MODE,
        .channel        = LEDC_CHANNEL,
        .timer_sel      = LEDC_TIMER,
        .duty           = 0,
        .hpoint         = 0
    };
    ledc_channel_config(&ch_conf);
}

void app_main(void) {
    init_adc();
    init_led_pwm();

    ESP_LOGI(TAG, "Start LDR-based LED brightness control");

    while (1) {
        // อ่านค่า ADC หลายครั้งแล้วเฉลี่ย
        uint32_t adc_sum = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_sum += adc1_get_raw((adc1_channel_t)LDR_CHANNEL);
        }
        uint32_t adc_raw = adc_sum / NO_OF_SAMPLES;

        // คำนวณ % แสง
        float light_pct = (adc_raw / 4095.0f) * 100.0f;

        // Map จาก ADC (0–4095) -> PWM duty (0–1023)
        uint32_t duty = (adc_raw * ((1 << LEDC_RES) - 1)) / 4095;

        // ตั้งค่า duty ให้ LEDC
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);

        ESP_LOGI(TAG, "ADC:%4d  Light:%.1f%%  LED duty:%d/1023",
                 adc_raw, light_pct, duty);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
    