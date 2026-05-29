#include "button.h"
#include "main.h"
#include "gw_anagloge.h"
#include "gy901.h"
#include "led.h"
#include "log.h"
#include "math_tool.h"
#include "status.h"
#include "task.h"
#include "buzzer.h"
#include "i2c.h"
#include "timer_it.h"

extern PID balance_pid;
extern float base_angle;

// 运行阶段

void server_button(BUTTON *button, BUTTON_STATION station) {
  if (button->which == 1) {
    if (station == BUTTON_DOWN) {
      mode = (mode + 1) % 5;
      status.device.buzzer1.on = 1;
      switch (mode) {
        case stop:
          status.device.led1.on = 0;
          status.device.led2.on = 0;
          status.device.led3.on = 0;
          break;
        case first:
          status.device.led1.on = 1;
          status.device.led2.on = 0;
          status.device.led3.on = 0;
          break;
        case second:
          status.device.led1.on = 0;
          status.device.led2.on = 1;
          status.device.led3.on = 0;
          break;
        case third:
          status.device.led1.on = 0;
          status.device.led2.on = 0;
          status.device.led3.on = 1;
          break;
        case fourth:
          status.device.led1.on = 1;
          status.device.led2.on = 1;
          status.device.led3.on = 1;

          //set_gyr_angle_reference(&hi2c1);
          break;
      }
    } else if (station == BUTTON_UP) {
      status.device.buzzer1.on = 0;
    } else if (station == BUTTON_LONG) {
      if (status.sensor.gw_analogue.sta == 0) {
        status.device.led3.on = 1;
      } else {
        status.device.led3.on = 1;
      }
      correct_gw_analogue(&status.sensor.gw_analogue);
      driver_LED(&status.device.led1);
    }
  } else if (button->which == 2) {
    if (station == BUTTON_DOWN) {
    } else if (station == BUTTON_UP) {
      status.device.buzzer1.on = 0;
    } else if (station == BUTTON_LONG) {
      start_task();
    }
  }

  return;
}

void driver_button(BUTTON *button) {
  if (button->which == 1) {
    button->now = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);
  } else if (button->which == 2) {
    button->now = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);
  }

  if (1 ^ (button->now ^ button->Press_is_high_level)) {  // 按键长按检测
    if (button->long_press_cnt > 0) {
      button->long_press_cnt--;
    } else if (button->long_press_cnt == 0) {
      server_button(button, BUTTON_LONG);
      button->long_press_cnt = -1;
    }
  } else {
    button->long_press_cnt = LONG_PRESS_CNT;
  }

  if (button->now != button->last) {  // 按键短按检测
    if (button->Press_is_high_level == 1) {
      if (button->now == 1) {
        server_button(button, BUTTON_DOWN);
        // 按键按下
        if (button->long_press_cnt - 1 >= 0) {
          button->long_press_cnt--;
        } else {
          server_button(button, BUTTON_LONG);
        }
      } else {
        server_button(button, BUTTON_UP);
        // 按键释放
        button->long_press_cnt = LONG_PRESS_CNT;
      }
    } else {
      if (button->now == 0) {
        server_button(button, BUTTON_DOWN);
        // 按键按下
      } else {
        server_button(button, BUTTON_UP);
        // 按键释放
        button->long_press_cnt = LONG_PRESS_CNT;
      }
    }
    button->last = button->now;
  }
}

void init_button(BUTTON *button, uint8_t which, uint8_t Press_is_high_level) {
  button->which = which;
  button->Press_is_high_level = Press_is_high_level;
  button->last = Press_is_high_level ? 0 : 1;
  button->now = Press_is_high_level ? 0 : 1;
  button->long_press_cnt = LONG_PRESS_CNT;
  return;
}
