//
// Created by 1234455 on 2026/4/30.
//

#ifndef XIAOSAI_TIMER_IT_H
#define XIAOSAI_TIMER_IT_H
#include <stdint.h>

void start_task();
void beep();
void stop_motor();

#define GYRO_TURN_ARC 0
#define GYRO_TURN_PIVOT 1
#define GYRO_TURN_SINGLE 2

// 定义枚举类型
typedef enum {
    GYRO_TEST_IDLE = 0,
    GYRO_TEST_START = 1,    // 自动抓取初始角度的过渡状态
    GYRO_TEST_HOLDING = 2,  // 持续保持（锁死）状态
    GYRO_TEST_DONE = 3      // 手动停止
} GYRO_TEST_STATE;

// ！！！这 4 个外部声明非常重要，让 button.c 能够访问到 timer_it.c 中的测试变量 ！！！
extern GYRO_TEST_STATE gyro_test_state;
extern float gyro_test_tar;
extern uint8_t gyro_test_mode;
extern float gyro_start_angle;

#endif //XIAOSAI_TIMER_IT_H