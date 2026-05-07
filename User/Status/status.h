// @63 @551

#ifndef __STATUS_H
#define __STATUS_H

#include "gw_anagloge.h"
#include "led.h"
#include "main.h"
#include "pid.h"
#include "task.h"
#include "wheel.h"
#include "button.h"
#include "buzzer.h"
#include "task.h"
#include "gy901.h"

#define MOTION_BASE_SPEED 2000

typedef enum MOTION_STATION {
  STOP,
  KEEP_ANGLE,
  FIND_LINE,
} MOTION_STATION;  // 当前运行模式

typedef struct SENSOR {
  GW_ANALOGUE gw_analogue;  // 感为模拟传感器
  GYR gyr;
} SENSOR;

typedef struct DEVICE {
  LED led1;
  LED led2;
  LED led3;
  BUTTON button1;
  BUTTON button2;
  BUZZER buzzer1;
} DEVICE;

typedef struct MOTOR {
  WHEEL wheel[2];  // 轮子结构体
} MOTOR;

typedef struct STATE {
  int8_t T;       // 系统周期单位ms
  uint64_t time;  // 系统时间单位ms

} STATE;

typedef struct STATUS {
  TASK task;  // 任务结构体
  STATE state;       // 系统状态
  SENSOR sensor;     // 传感器数据
  MOTOR motor;       // 电机数据
  DEVICE device;     // 挂载设备
} STATUS;

extern STATUS status;

void init_status(STATUS *status, uint8_t T);
void driver_status(STATUS *status);

#endif
