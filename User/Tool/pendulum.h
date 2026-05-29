#ifndef __PENDULUM_H
#define __PENDULUM_H

#include "pid.h"

typedef struct {
    PID upright_pid;
    PID speed_pid;
    PID turn_pid;

    float mech_zero;      // 机械中值
    float target_speed;   // 目标速度（A点去B点时赋予正值即可）

    float out_upright;
    float out_speed;
    float out_turn;

    uint8_t is_balancing; // 系统平衡总开关
} PENDULUM_CTRL;

extern PENDULUM_CTRL pendulum_ctrl;

void init_pendulum(void);
void update_pendulum_control(void);

#endif