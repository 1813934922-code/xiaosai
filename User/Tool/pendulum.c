//
// Created by 1234455 on 2026/5/29.
//

#include "pendulum.h"
#include "status.h"
#include "math_tool.h"

// 建立倒立摆专属状态结构体
PENDULUM_CTRL pendulum_ctrl;

void init_pendulum(void) {
    // 1. 初始化三个环的 PID (参数需上车实调)
    // 直立环: PD控制 (极速响应，不需要积分)
    pendulum_ctrl.upright_pid = init_pid(105.025f, 0.0f, 5.025f, 5.0f, 5000);
    // 速度环: PI控制 (正反馈机制，缓慢拉回原点，不需要微分)
    pendulum_ctrl.speed_pid   = init_pid(3.5f, 0.0375f, 0.0f, 5.0f, 10000);
    // 寻迹环: PD控制 (普通循迹)
    pendulum_ctrl.turn_pid    = init_pid(10.0f, 0.0f, 2.0f, 5.0f, 1000);

    pendulum_ctrl.mech_zero = -1.3f; // 硬件校准置零后，这里填0.0f即可
    pendulum_ctrl.target_speed = 0.0f;
    pendulum_ctrl.state = PEND_STOP; // 默认停机
}

void update_pendulum_control(void) {
    if (pendulum_ctrl.state == PEND_STOP) {
        return; // 停机状态，交回原有的业务逻辑控制
    }

    // 获取 X轴 角度和角速度
    float current_angle = get_gyr_value(&status.sensor.gyr, gyr_x_roll) - pendulum_ctrl.mech_zero;
    float current_gyro  = get_gyr_value(&status.sensor.gyr, gyr_w_x);

    // ================= 状态1：暴力起摆 (带 PWM 动态预测拦截) =================
    if (pendulum_ctrl.state == PEND_SWING_UP) {
        static uint8_t swing_step = 0;
        static uint16_t swing_timer = 0;
        static uint16_t total_timer = 0;

        // 【调参点 1：物理角度底线】放宽限制，只要进了 ±15 度，就允许进行 PWM 预测评估
        float catch_angle = 15.0f;

        // 【调参点 2：PWM 预测阈值】评估此时如果接管，需要的推力代价
        // 设为 200 意味着：只要 PID 觉得花不到 20% 的满载力气就能稳住，立刻接管！
        float catch_pwm = 100.0f;

        // 【调参点 3：起摆爆发力】
        int16_t pull_thrust = -600;          // 阶段0：往后退的拉力
        int16_t dash_thrust = 680;           // 阶段1：往前冲的爆发力

        // 🌟 1. 动态预测：算出如果此刻闭环，PID 瞬间要求输出多大推力
        float predict_out = pendulum_ctrl.upright_pid.kp * current_angle +
                            pendulum_ctrl.upright_pid.kd * current_gyro;

        // 🌟 2. 双重拦截条件：既要在合理角度内，又要预测代价极小！
        if (current_angle > -catch_angle && current_angle < catch_angle &&
            predict_out > -catch_pwm && predict_out < catch_pwm) {

            pendulum_ctrl.speed_pid.integral = 0;
            pendulum_ctrl.upright_pid.integral = 0;
            pendulum_ctrl.state = PEND_BALANCING;

            // 复位起摆变量
            swing_step = 0; swing_timer = 0; total_timer = 0;
            return; // 💥 立刻退出并移交控制权！
        }

        // [没触发拦截时的单次甩鞭动作]
        if (swing_step == 0) {
            status.motor.wheel[0].trust = pull_thrust;
            status.motor.wheel[1].trust = pull_thrust;

            swing_timer++;
            // 往后退的蓄力时间：30 * 5ms = 150ms
            if (swing_timer > 23) {
                swing_step = 1;
                swing_timer = 0;
            }
        }
        else if (swing_step == 1) {
            status.motor.wheel[0].trust = dash_thrust;
            status.motor.wheel[1].trust = dash_thrust;
        }

        // 超时重试机制：2.5 秒没成功就重头来
        total_timer++;
        if (total_timer > 500) {
            swing_step = 0; swing_timer = 0; total_timer = 0;
        }

        driver_wheel(&status.motor.wheel[0]);
        driver_wheel(&status.motor.wheel[1]);
        return;
    }

    // ================= 状态2：闭环平衡与寻迹 (保留你原本的代码) =================
    if (pendulum_ctrl.state == PEND_BALANCING) {

        // 1. 直立环 (PD)
        pendulum_ctrl.out_upright = pendulum_ctrl.upright_pid.kp * current_angle +
                                    pendulum_ctrl.upright_pid.kd * current_gyro;

        // 2. 速度环 (PI)
        float avg_speed = (status.motor.wheel[0].cur_speed + status.motor.wheel[1].cur_speed) / 2.0f;
        float speed_error = pendulum_ctrl.target_speed - avg_speed;

        pendulum_ctrl.out_speed = compute_pid(&pendulum_ctrl.speed_pid, speed_error);

        // 3. 寻迹转向环 (PD)
        pendulum_ctrl.out_turn = 0;

        // 4. 三环输出融合
        int16_t final_L = (int16_t)(-1*(pendulum_ctrl.out_upright + pendulum_ctrl.out_speed + pendulum_ctrl.out_turn));
        int16_t final_R = (int16_t)(-1*(pendulum_ctrl.out_upright + pendulum_ctrl.out_speed - pendulum_ctrl.out_turn));

        // 覆盖 trust (PWM占空比)
        status.motor.wheel[0].trust = final_L;
        status.motor.wheel[1].trust = final_R;

        // 执行电机驱动
        driver_wheel(&status.motor.wheel[0]);
        driver_wheel(&status.motor.wheel[1]);
    }
}