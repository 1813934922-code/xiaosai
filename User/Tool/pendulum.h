#ifndef __PENDULUM_H
#define __PENDULUM_H

#include "pid.h"
#include <stdint.h>

// 倒立摆运行状态机
typedef enum {
    PEND_STOP = 0,    // 停止不动 (释放电机控制权)
    PEND_SWING_UP,    // 开环甩鞭起摆阶段
    PEND_BALANCING    // 闭环平衡与寻迹阶段
} PEND_STATE;

typedef struct {
    PID upright_pid;
    PID speed_pid;
    PID turn_pid;

    float mech_zero;      // 机械中值 (硬件校准后一般填0)
    float target_speed;   // 目标速度（A点去B点时赋予正值）

    PEND_STATE state;     // 当前系统运行状态

    float out_upright;
    float out_speed;
    float out_turn;
} PENDULUM_CTRL;

extern PENDULUM_CTRL pendulum_ctrl;

void init_pendulum(void);
void update_pendulum_control(void);

#endif