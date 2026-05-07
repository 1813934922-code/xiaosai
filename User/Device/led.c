#include "led.h"

#include "gpio.h"
#include "log.h"
#include "status.h"
#include "stdbool.h"

void driver_LED(LED *led) {
  if (led->which == 1) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, 1 ^ ((bool)(led->on) ^ (bool)(led->High_level_is_on)));
  }
  if (led->which == 2) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_2, 1 ^ ((bool)(led->on) ^ (bool)(led->High_level_is_on)));
  }
  if (led->which == 3) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 1 ^ ((bool)(led->on) ^ (bool)(led->High_level_is_on)));
  }
  return;
}

void init_LED(LED *led, uint8_t which, uint8_t High_level_is_on) {
  led->High_level_is_on = High_level_is_on;
  led->on = 0;
  led->which = which;
  return;
}