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
    pendulum_ctrl.upright_pid = init_pid(105.0f, 0.0f, 5.0f, 5.0f, 5000);
    // 速度环: PI控制 (正反馈机制，缓慢拉回原点，不需要微分)
    pendulum_ctrl.speed_pid   = init_pid(3.5f, 0.0375f, 0.0f, 5.0f, 10000);
    // 寻迹环: PD控制 (普通循迹)
    pendulum_ctrl.turn_pid    = init_pid(10.0f, 0.0f, 2.0f, 5.0f, 1000);

    pendulum_ctrl.mech_zero = -0.035f; // 硬件校准置零后，这里填0.0f即可
    pendulum_ctrl.target_speed = 0.0f;
    pendulum_ctrl.state = PEND_STOP; // 默认停机
}

void update_pendulum_control(void) {
    if (pendulum_ctrl.state == PEND_STOP) {
        return; // 停机状态，交回原有的业务逻辑控制
    }

    // 获取 X轴 角度和角速度 (注意：已根据实际测量的X轴修改)
    float current_angle = get_gyr_value(&status.sensor.gyr, gyr_x_roll) - pendulum_ctrl.mech_zero;
    float current_gyro  = get_gyr_value(&status.sensor.gyr, gyr_w_x);

    // ================= 状态1：能量共振起摆 (荡秋千 + 无限重试) =================
    if (pendulum_ctrl.state == PEND_SWING_UP) {
        static uint8_t swing_step = 0;       // 0:向后推, 1:向前冲, 2:循环结束等待接管
        static uint8_t swing_count = 0;      // 已经完成的完整循环次数 (0, 1, 2)
        static uint16_t swing_timer = 0;     // 当前动作计时器
        static int16_t current_thrust = 600; // 初始最大推力
        static uint16_t total_timer = 0;     // 总超时保护计时器

        // 【调参点】：每次推拉的持续时间 (例如 40 * 5ms = 200ms)
        const uint16_t PHASE_TIME = 100;
        float catch_angle = 6.0f;            // 闭环接管阈值 (±6度)

        if (swing_step < 2) {
            if (swing_step == 0) {
                status.motor.wheel[0].trust = -current_thrust;
                status.motor.wheel[1].trust = -current_thrust;
            } else {
                status.motor.wheel[0].trust = current_thrust;
                status.motor.wheel[1].trust = current_thrust;
            }

            swing_timer++;
            if (swing_timer >= PHASE_TIME) {
                swing_timer = 0;
                if (swing_step == 0) {
                    swing_step = 1;
                } else {
                    swing_count++;
                    if (swing_count >= 1) {
                        swing_step = 2;
                    } else {
                        current_thrust = (int16_t)(current_thrust * 1.0f); // 衰减推力
                        swing_step = 0;
                    }
                }
            }
        }
        else if (swing_step == 2) {
            status.motor.wheel[0].trust = current_thrust;
            status.motor.wheel[1].trust = current_thrust;

            // 成功接管条件
            if (current_angle > -catch_angle && current_angle < catch_angle) {
                pendulum_ctrl.speed_pid.integral = 0;
                pendulum_ctrl.upright_pid.integral = 0;
                pendulum_ctrl.state = PEND_BALANCING;

                // 复位静态变量
                swing_step = 0; swing_count = 0; swing_timer = 0; current_thrust = 800; total_timer = 0;
            }
        }

        // 永不言弃机制：挣扎超过 4 秒 (800*5ms) 还没接住，直接重新开始新一轮甩鞭！
        total_timer++;
        if (total_timer > 800) {
            swing_step = 0; swing_count = 0; swing_timer = 0; current_thrust = 800; total_timer = 0;
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
        //pendulum_ctrl.out_upright = 0;
        // 2. 速度环 (PI)
        float avg_speed = (status.motor.wheel[0].cur_speed + status.motor.wheel[1].cur_speed) / 2.0f;
        float speed_error = pendulum_ctrl.target_speed - avg_speed;

        pendulum_ctrl.out_speed = compute_pid(&pendulum_ctrl.speed_pid, speed_error);
        //pendulum_ctrl.out_speed = 0;
        // 3. 寻迹转向环 (PD)
        //pendulum_ctrl.out_turn = compute_pid(&pendulum_ctrl.turn_pid, status.sensor.gw_analogue.diff);
        pendulum_ctrl.out_turn = 0;
        // 4. 三环输出融合
        // 极性注意：必须确保向前方倾倒时(车要往前加速)，final的算力结果是让车轮向前的PWM
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