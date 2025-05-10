#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define BUTTON_GPIO 17
#define LED_GPIO    2

#define PCNT_UNIT   PCNT_UNIT_0

static portMUX_TYPE pulse_mux = portMUX_INITIALIZER_UNLOCKED;

volatile int32_t total_pulses = 0;


void pcnt_setup(pcnt_unit_t unit, gpio_num_t pin) {
  pcnt_config_t config = {
      .pulse_gpio_num = pin,
      .ctrl_gpio_num = PCNT_PIN_NOT_USED,
      .channel = PCNT_CHANNEL_0,
      .unit = unit,
      .pos_mode = PCNT_COUNT_INC,
      .neg_mode = PCNT_COUNT_DIS,
      .lctrl_mode = PCNT_MODE_KEEP,
      .hctrl_mode = PCNT_MODE_KEEP,
      .counter_h_lim = 32767,
      .counter_l_lim = -32768,
  };

  pcnt_unit_config(&config);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

void pcnt_accumulator_task(void *arg) {
  int16_t pulse_count = 0;

  while (1) {
      pcnt_get_counter_value(PCNT_UNIT, &pulse_count);

      ESP_LOGI("PCNT", "Pulse delta: %d | Total: %d", pulse_count, total_pulses);
      
      portENTER_CRITICAL(&pulse_mux);
      total_pulses += pulse_count;
      portEXIT_CRITICAL(&pulse_mux);

      pcnt_counter_clear(PCNT_UNIT);

      vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void goal_checker_task(void *arg) {
  while (1) {
    portENTER_CRITICAL(&pulse_mux);
    int32_t current_pulse_total = total_pulses;
    portEXIT_CRITICAL(&pulse_mux);

    if (current_pulse_total > 5) {
      gpio_set_level(LED_GPIO, 1);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void appMain(void) {
    ESP_LOGI("BOOT", "appMain is running");

    pcnt_setup(PCNT_UNIT, BUTTON_GPIO);
    gpio_set_pull_mode(BUTTON_GPIO, GPIO_PULLDOWN_ONLY);

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_GPIO, 0);

    xTaskCreate(pcnt_accumulator_task, "pcnt_accumulator", 2048, NULL, 5, NULL);
    xTaskCreate(goal_checker_task, "goal_checker", 4096, NULL, 5, NULL);

    // Keep the app running forever so uros_task doesn't return
    // Needed because this code is compiled inside the micros_ROS workspace
    while (true) {
      vTaskDelay(portMAX_DELAY);
    }
}
