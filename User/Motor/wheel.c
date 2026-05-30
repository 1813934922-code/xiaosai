#include "wheel.h"

#include "log.h"
#include "math_tool.h"

float get_wheel_speed(WHEEL *wheel) {
  int16_t encoder_ticks = 0;
  if (wheel->which == 1) {
    encoder_ticks = TIM1->CNT - 30000;
    TIM1->CNT = 30000;
  } else if (wheel->which == 2) {
    encoder_ticks = TIM2->CNT - 30000;
    TIM2->CNT = 30000;
  }
  encoder_ticks = encoder_ticks * wheel->dir;

  // <--- 新增：累加每次计算的脉冲数，记录总位移
  wheel->total_ticks += encoder_ticks;

  return (float)encoder_ticks * 60.0f * PID_HZ / MOTOR_REDUCTION;
}

void set_wheel_dir(WHEEL *wheel, int16_t trust) {
  if (wheel->which == 1) {
    if (trust * wheel->dir > 0) {
      HAL_GPIO_WritePin(M1_D1_GPIO_Port, M1_D1_Pin, 1);
      HAL_GPIO_WritePin(M1_D2_GPIO_Port, M1_D2_Pin, 0);
    } else {
      HAL_GPIO_WritePin(M1_D1_GPIO_Port, M1_D1_Pin, 0);
      HAL_GPIO_WritePin(M1_D2_GPIO_Port, M1_D2_Pin, 1);
    }
  } else if (wheel->which == 2) {
    if (trust * wheel->dir < 0) {
      HAL_GPIO_WritePin(M2_D1_GPIO_Port, M2_D1_Pin, 1);
      HAL_GPIO_WritePin(M2_D2_GPIO_Port, M2_D2_Pin, 0);
    } else {
      HAL_GPIO_WritePin(M2_D1_GPIO_Port, M2_D1_Pin, 0);
      HAL_GPIO_WritePin(M2_D2_GPIO_Port, M2_D2_Pin, 1);
    }
  }
}

void driver_wheel(WHEEL *wheel) {
  int16_t trust = wheel->trust;
  trust = CONFINE(trust, -TRUST_CONFINE, TRUST_CONFINE);

  if (wheel->which == 1) {
    set_wheel_dir(wheel, trust);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ABS(trust));
  } else if (wheel->which == 2) {
    set_wheel_dir(wheel, trust);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ABS(trust));
  }
}

void init_wheel(WHEEL *wheel, uint8_t which, int8_t dir) {
  wheel->which = which;
  wheel->trust = 0;
  wheel->cur_speed = 0;
  wheel->tar_speed = 0;
  wheel->dir = dir;
  wheel->total_ticks = 0; // <--- 新增：初始化累计脉冲为 0
  wheel->wheel_pid = init_pid(1, 1, 1, 5, 5000);

  if (wheel->which == 1) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  } else if (wheel->which == 2) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  }

  if (wheel->which == 1) {
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
    TIM1->CNT = 30000;
  } else if (wheel->which == 2) {
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    TIM2->CNT = 30000;
  }
}