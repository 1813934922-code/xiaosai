#include "buzzer.h"
#include "main.h"
#include "gpio.h"
#include "stdbool.h"

void driver_BUZZER(BUZZER *buzzer) {
  if (buzzer->which == 1) {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1 ^ ((bool)(buzzer->on) ^ (bool)(buzzer->High_level_is_on)));
  }

  return;
}

void init_BUZZER(BUZZER *buzzer, uint8_t which, uint8_t High_level_is_on) {
  buzzer->High_level_is_on = High_level_is_on;
  buzzer->on = 0;
  buzzer->which = which;

  return;
}