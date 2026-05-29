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
    pendulum_ctrl.upright_pid = init_pid(500.0f, 0.0f, 5.0f, 5.0f, 5000);
    // 速度环: PI控制 (正反馈机制，缓慢拉回原点，不需要微分)
    pendulum_ctrl.speed_pid   = init_pid(15.0f, 0.5f, 0.0f, 5.0f, 2000);
    // 寻迹环: PD控制 (普通循迹)
    pendulum_ctrl.turn_pid    = init_pid(10.0f, 0.0f, 2.0f, 5.0f, 1000);

    pendulum_ctrl.mech_zero = 0.0f; // 硬件校准置零后，这里填0.0f即可
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

    // ================= 状态1：开环两段式甩鞭起摆 =================
    if (pendulum_ctrl.state == PEND_SWING_UP) {
        static uint8_t swing_step = 0;
        static uint16_t swing_timer = 0;

        float catch_angle = 6.0f; // 闭环接管阈值 (±6度)

        // [阶段1]：反向蓄力，把靠在支架上的摆杆“拖”下来
        if (swing_step == 0) {
            // 假设：摆杆在后侧(角度为负)，这里给负推力让车后退
            status.motor.wheel[0].trust = -350;
            status.motor.wheel[1].trust = -350;

            swing_timer++;

            // 触发条件：摆杆脱离支架往下掉(比如到达-8度)，或超时强行进入下一段
            if (current_angle > -8.0f || swing_timer > 200) { // 200*5ms = 1秒
                swing_step = 1;
                swing_timer = 0;
            }
        }
        // [阶段2]：正向爆发，迎头痛击甩起摆杆
        else if (swing_step == 1) {
            // 满功率向前猛冲 (需根据轮胎抓地力调整，防打滑)
            status.motor.wheel[0].trust = 500;
            status.motor.wheel[1].trust = 500;

            swing_timer++;

            // 成功接管条件：角度甩进了 ±6 度以内
            if (current_angle > -catch_angle && current_angle < catch_angle) {
                pendulum_ctrl.speed_pid.integral = 0; // 清空速度环积分，防疯跑
                pendulum_ctrl.state = PEND_BALANCING; // 成功！移交PID闭环控制权
                swing_step = 0;
                swing_timer = 0;
            }
            // // 失败保护：如果在冲刺阶段0.8秒都没甩上来，说明力量不够，强行停机防撞
            // else if (swing_timer > 160) {
            //     stop_motor();
            //     pendulum_ctrl.state = PEND_STOP;
            //     swing_step = 0;
            //     swing_timer = 0;
            // }
        }

        // 起摆阶段直接下发驱动并退出，不执行下方闭环算法
        driver_wheel(&status.motor.wheel[0]);
        driver_wheel(&status.motor.wheel[1]);
        return;
    }

    // ================= 状态2：闭环平衡与寻迹 =================
    if (pendulum_ctrl.state == PEND_BALANCING) {

        // 1. 直立环 (PD)
        pendulum_ctrl.out_upright = pendulum_ctrl.upright_pid.kp * current_angle +
                                    pendulum_ctrl.upright_pid.kd * current_gyro;

        // 2. 速度环 (PI)
        float avg_speed = (status.motor.wheel[0].cur_speed + status.motor.wheel[1].cur_speed) / 2.0f;
        float speed_error = pendulum_ctrl.target_speed - avg_speed;

        //pendulum_ctrl.out_speed = compute_pid(&pendulum_ctrl.speed_pid, speed_error);
        pendulum_ctrl.out_speed = 0;
        // 3. 寻迹转向环 (PD)
        //pendulum_ctrl.out_turn = compute_pid(&pendulum_ctrl.turn_pid, status.sensor.gw_analogue.diff);
        pendulum_ctrl.out_turn = 0;
        // 4. 三环输出融合
        // 极性注意：必须确保向前方倾倒时(车要往前加速)，final的算力结果是让车轮向前的PWM
        int16_t final_L = (int16_t)(-1*(pendulum_ctrl.out_upright - pendulum_ctrl.out_speed + pendulum_ctrl.out_turn));
        int16_t final_R = (int16_t)(-1*(pendulum_ctrl.out_upright - pendulum_ctrl.out_speed - pendulum_ctrl.out_turn));

        // 覆盖 trust (PWM占空比)
        status.motor.wheel[0].trust = final_L;
        status.motor.wheel[1].trust = final_R;

        // 执行电机驱动
        driver_wheel(&status.motor.wheel[0]);
        driver_wheel(&status.motor.wheel[1]);
    }
}