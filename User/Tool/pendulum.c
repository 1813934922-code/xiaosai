//
// Created by 1234455 on 2026/5/29.
//

#include "pendulum.h"

#include "pendulum.h"
#include "status.h"
#include "math_tool.h"

// 建立倒立摆专属状态结构体
PENDULUM_CTRL pendulum_ctrl;

void init_pendulum(void) {
    // 1. 初始化三个环的 PID (参数仅供参考，需上车实调)
    // 直立环: PD控制 (极速响应，不需要积分)
    pendulum_ctrl.upright_pid = init_pid(200.0f, 0.0f, 5.0f, 5.0f, 5000);
    // 速度环: PI控制 (正反馈机制，缓慢拉回原点)
    pendulum_ctrl.speed_pid   = init_pid(15.0f, 0.5f, 0.0f, 5.0f, 2000);
    // 寻迹环: PD控制 (普通循迹)
    pendulum_ctrl.turn_pid    = init_pid(10.0f, 0.0f, 2.0f, 5.0f, 1000);

    pendulum_ctrl.mech_zero = 0.0f; // 理想的物理垂直零偏角
    pendulum_ctrl.target_speed = 0.0f; // 默认目标为原地悬停
    pendulum_ctrl.is_balancing = 0; // 默认不开启平衡
}

void update_pendulum_control(void) {
    if (!pendulum_ctrl.is_balancing) {
        return; // 未开启平衡时，交回原有的业务逻辑控制
    }

    // ================= 1. 直立环 (PD) =================
    // 获取 Y轴 角度和角速度
    float current_angle = get_gyr_value(&status.sensor.gyr, gyr_y_pitch) - pendulum_ctrl.mech_zero;
    float current_gyro  = get_gyr_value(&status.sensor.gyr, gyr_w_y);

    // 标准直立环计算：Kp * 角度误差 + Kd * 陀螺仪角速度 (用真实角速度代替误差的微分项，更平滑)
    pendulum_ctrl.out_upright = pendulum_ctrl.upright_pid.kp * current_angle +
                                pendulum_ctrl.upright_pid.kd * current_gyro;

    // ================= 2. 速度环 (PI) =================
    // 计算车体平均速度 (基于 wheel.c 中已更新的 cur_speed)
    float avg_speed = (status.motor.wheel[0].cur_speed + status.motor.wheel[1].cur_speed) / 2.0f;
    float speed_error = pendulum_ctrl.target_speed - avg_speed;

    // 直接复用你写好的 PID 计算器计算速度环补偿
    pendulum_ctrl.out_speed = compute_pid(&pendulum_ctrl.speed_pid, speed_error);

    // ================= 3. 寻迹转向环 (PD) =================
    // 使用 gw_analogue 已经算好的线性偏差 diff
    pendulum_ctrl.out_turn = compute_pid(&pendulum_ctrl.turn_pid, status.sensor.gw_analogue.diff);

    // ================= 4. 三环输出融合 =================
    // 最终 PWM = 直立环 + 速度环 + 转向环 (极性取决于电机接线，这里假设向前方倾倒是Y角变大)
    int16_t final_L = (int16_t)(pendulum_ctrl.out_upright - pendulum_ctrl.out_speed + pendulum_ctrl.out_turn);
    int16_t final_R = (int16_t)(pendulum_ctrl.out_upright - pendulum_ctrl.out_speed - pendulum_ctrl.out_turn);

    // ！！！关键点：直接覆盖 trust (PWM占空比)，绕过底层原来的轮速闭环 ！！！
    status.motor.wheel[0].trust = final_L;
    status.motor.wheel[1].trust = final_R;

    // 立即执行电机驱动
    driver_wheel(&status.motor.wheel[0]);
    driver_wheel(&status.motor.wheel[1]);
}