#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"
#include "esp_log.h"

#define LEFT_A_GPIO   17
#define LEFT_B_GPIO   16
#define LEFT_LED_GPIO 33

#define LEFT_UNIT   PCNT_UNIT_0

static portMUX_TYPE pulse_mux = portMUX_INITIALIZER_UNLOCKED;

volatile int32_t left_ticks_total = 0;


void pcnt_setup_quadrature(pcnt_unit_t unit, gpio_num_t pin_a, gpio_num_t pin_b) {
  pcnt_config_t config = {
    .pulse_gpio_num = pin_a,        // A signal (tick pulse)
    .ctrl_gpio_num = pin_b,         // B signal (direction)
    .channel = PCNT_CHANNEL_0,
    .unit = unit,
    .pos_mode = PCNT_COUNT_INC,     // Count up if B is high
    .neg_mode = PCNT_COUNT_DEC,     // Count down if B is low
    .lctrl_mode = PCNT_MODE_REVERSE,
    .hctrl_mode = PCNT_MODE_KEEP,
    .counter_h_lim = 32767,
    .counter_l_lim = -32768,
  };

  pcnt_unit_config(&config);
  pcnt_counter_pause(unit);
  pcnt_counter_clear(unit);
  pcnt_counter_resume(unit);
}

// Use PCNT to accumulate ticks from the encoders
void encoder_task(void *arg) {
  int16_t left_now = 0;

  while (1) {
      pcnt_get_counter_value(LEFT_UNIT, &left_now);
      
      portENTER_CRITICAL(&pulse_mux);
      left_ticks_total += left_now;
      portEXIT_CRITICAL(&pulse_mux);

      pcnt_counter_clear(LEFT_UNIT);

      ESP_LOGI("ENCODER", "L: %d", left_ticks_total);

      vTaskDelay(pdMS_TO_TICKS(100));
  }
}

// Provide visual feedback when an encoder's ticks count has changed
void new_ticks_led_task(void *arg) {
  int16_t previous_left_ticks_total = 0;

  while (1) {
    portENTER_CRITICAL(&pulse_mux);
    int32_t current_left_ticks_total = left_ticks_total;
    portEXIT_CRITICAL(&pulse_mux);

    if (current_left_ticks_total != previous_left_ticks_total) {
      gpio_set_level(LEFT_LED_GPIO, 1);
      previous_left_ticks_total = current_left_ticks_total; // Only need to update previous total_ticks if current total_ticks has changed
    } else {
      gpio_set_level(LEFT_LED_GPIO, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void appMain(void) {
    gpio_set_direction(LEFT_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LEFT_LED_GPIO, 0);

    // Testing with both channels on a single encoder
    pcnt_setup_quadrature(LEFT_UNIT, LEFT_A_GPIO, LEFT_B_GPIO);

    xTaskCreate(encoder_task, "encoder", 2048, NULL, 5, NULL);
    xTaskCreate(new_ticks_led_task, "new_ticks_led", 4096, NULL, 5, NULL);

    // Keep the app running forever so uros_task doesn't return
    // Needed because this code is compiled inside the micros_ROS workspace
    while (true) {
      vTaskDelay(portMAX_DELAY);
    }
}
