#include "microros_config.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/pcnt.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>

#include "slambot_interfaces/msg/detail/encoder_ticks__struct.h"
#include "slambot_interfaces/msg/detail/encoder_ticks__functions.h"

#define LEFT_A_GPIO     17
#define LEFT_B_GPIO     16
#define LEFT_LED_GPIO   33
#define RIGHT_A_GPIO    18
#define RIGHT_B_GPIO    19
#define RIGHT_LED_GPIO  21

#define LEFT_UNIT       PCNT_UNIT_0
#define RIGHT_UNIT      PCNT_UNIT_1

static portMUX_TYPE pulse_mux = portMUX_INITIALIZER_UNLOCKED;

volatile int32_t left_ticks_total = 0;
volatile int32_t right_ticks_total = 0;

// Globals
rcl_publisher_t publisher;
rclc_executor_t executor;
rcl_timer_t timer;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;

#define TAG "ENCODER_MICROROS"

slambot_interfaces__msg__EncoderTicks msg;


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
  int16_t right_now = 0;

  while (1) {
      pcnt_get_counter_value(LEFT_UNIT, &left_now);
      pcnt_get_counter_value(RIGHT_UNIT, &right_now);
      
      portENTER_CRITICAL(&pulse_mux);
      left_ticks_total += left_now;
      right_ticks_total += right_now;
      portEXIT_CRITICAL(&pulse_mux);

      pcnt_counter_clear(LEFT_UNIT);
      pcnt_counter_clear(RIGHT_UNIT);

      vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void encoder_publisher_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  portENTER_CRITICAL(&pulse_mux);
  msg.left_encoder = left_ticks_total;
  msg.right_encoder = right_ticks_total;
  portEXIT_CRITICAL(&pulse_mux);

  ESP_LOGI(TAG, "EncoderTicks size: %d", sizeof(slambot_interfaces__msg__EncoderTicks));

  rcl_publish(&publisher, &msg, NULL);
}

// Provide visual feedback when an encoder's ticks count has changed
void new_ticks_led_task(void *arg) {
  int32_t previous_left_ticks_total = 0;
  int32_t previous_right_ticks_total = 0;

  while (1) {
    portENTER_CRITICAL(&pulse_mux);
    int32_t current_left_ticks_total = left_ticks_total;
    int32_t current_right_ticks_total = right_ticks_total;
    portEXIT_CRITICAL(&pulse_mux);

    if (current_left_ticks_total != previous_left_ticks_total) {
      gpio_set_level(LEFT_LED_GPIO, 1);
      previous_left_ticks_total = current_left_ticks_total; // Only need to update previous total_ticks if current total_ticks has changed
    } else {
      gpio_set_level(LEFT_LED_GPIO, 0);
    }

    if (current_right_ticks_total != previous_right_ticks_total) {
      gpio_set_level(RIGHT_LED_GPIO, 1);
      previous_right_ticks_total = current_right_ticks_total;
    } else {
      gpio_set_level(RIGHT_LED_GPIO, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void run_self_test() {
  gpio_set_level(LEFT_LED_GPIO, 1);
  gpio_set_level(RIGHT_LED_GPIO, 1);

  vTaskDelay(pdMS_TO_TICKS(1000));

  gpio_set_level(LEFT_LED_GPIO, 0);
  gpio_set_level(RIGHT_LED_GPIO, 0);

}

void appMain(void) {
    allocator = rcl_get_default_allocator();
    slambot_interfaces__msg__EncoderTicks__init(&msg);
  
    gpio_set_direction(LEFT_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(RIGHT_LED_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(LEFT_LED_GPIO, 0);
    gpio_set_level(RIGHT_LED_GPIO, 0);

    // Set up PCNT for both encoders
    pcnt_setup_quadrature(LEFT_UNIT, LEFT_A_GPIO, LEFT_B_GPIO);
    pcnt_setup_quadrature(RIGHT_UNIT, RIGHT_A_GPIO, RIGHT_B_GPIO);

    // Run self test of encoder LEDs
    run_self_test();

    // Start tasks
    xTaskCreate(encoder_task, "encoder", 2048, NULL, 5, NULL);
    xTaskCreate(new_ticks_led_task, "new_ticks_led", 2048, NULL, 5, NULL);

    // micro-ROS support init
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "encoder_publisher_node", "", &support);

    // Publisher init
    rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(slambot_interfaces, msg, EncoderTicks),
      "/encoder_ticks"
    );

    // Timer init: 200ms interval
    rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(200),
      encoder_publisher_callback
    );

    // Executor init
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    // Spin forever
    while (true) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
      usleep(100000);  // 100 ms
    }
}
