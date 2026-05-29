#include "timer_it.h"

#include <math.h>
#include <stdlib.h>

#include "button.h"
#include "led.h"
#include "log.h"
#include "status.h"
#include "task.h"
#include "tim.h"
#include "usart.h"
#include "wheel.h"
#include "buzzer.h"
#include "i2c.h"
#include "pid.h"
#include "math_tool.h"
#include "pendulum.h"

MODE mode = stop;




typedef enum {
  STANDBY = 0,   // 待机，电机停止
  COUNTDOWN,     // 倒计时，1秒后开始任务
  RUNNING,       // 任务运行中
  FINISHED,      // 任务完成，闪烁提示
} RUN_STATE;

RUN_STATE run_state = STANDBY;              // 当前运行阶段

uint8_t third_cross_cnt = 0;                  // 任务三：已检测到的路口计数
uint8_t cross_cnt = 0;                      // 任务一：已检测到的路口计数
uint8_t turn_dir = 0;                       // 转弯方向 0=左转 1=右转
uint16_t cross_delay_cnt = 0;               // 路口冲过延时计数(单位10ms)
uint16_t countdown_cnt = 0;                 // 倒计时计数(单位10ms)
uint8_t blink_phase = 0;                    // 闪烁阶段计数
uint8_t blink_timer = 0;                    // 闪烁定时计数(单位10ms)
uint8_t second_turn_cnt = 0;                // 任务二：去程已转弯路口计数
uint8_t second_return_cnt = 0;              // 任务二：返程已转弯路口计数
float gyro_start_angle = 0;                 // 陀螺仪转向起始角度
float gyro_tar_angle = 0;                   // 陀螺仪转向目标角度差(正=左转,负=右转)
uint8_t gyro_turn_done = 0;                 // 陀螺仪转向完成标志
PID gyro_turn_pid;                          // 角度环PID
uint8_t is_first = 1;
uint8_t black_line_cnt = 0;
uint8_t gyro_turn_phase = 0;
uint8_t black_line_shield_cnt = 0;
uint16_t reverse_park_cnt = 0;
uint8_t first_flag=1;

#define CROSS_DELAY_MS 10          // 冲过路口延时(ms)
#define CROSS_STOP_MS 10          // 检测到路口后停车等待时间(ms)
#define GYRO_TURN_ARC 0
#define GYRO_TURN_PIVOT 1
#define GYRO_TURN_SINGLE 2
#define GYRO_BASE_SPEED 35          // 陀螺仪转向时基速(RPM)
#define COUNTDOWN_MS 100           // 发车倒计时(ms)
#define BLINK_ON_MS 100             // 闪烁亮灯时长(ms)
#define BLINK_OFF_MS 100            // 闪烁灭灯时长(ms)
#define BLINK_TIMES 3               // 到达闪烁次数
#define SECOND_GO_CORNERS 3         // 任务二去程路口数(A/D/C/B)
#define SECOND_RETURN_CORNERS 3     // 任务二返程路口数(C/D/A)
#define GYRO_TURN_THRESHOLD 5.0f    // 陀螺仪转向允许偏差(度)
#define GYRO_DIR 1                  // 陀螺仪方向: 1=左转角度增大, -1=左转角度减小(方向反了改这里)
#define CROSS_SHIELD_TIMES 200       // 转弯后屏蔽路口检测次数(单位10ms)
#define GYRO_SHIELD_TIMES 2000        // 陀螺仪转向后屏蔽路口检测次数(单位10ms, 500ms)
#define GYRO_TURN_MAX_DIFF 25.0f     // 角度PID最大轮速差(RPM)
#define BLACK_LINE_DETECT_TIMES 1    // 陀螺仪转向时黑线连续检测次数阈值
#define BLACK_LINE_MASK_LTURN 0x3C   // 左转时检测右通道5,6
#define BLACK_LINE_MASK_RTURN 0x3C   // 右转时检测左通道3,4
#define BLACK_LINE_SHIELD_TIMES 10   // 转弯开始后屏蔽黑线检测次数(单位10ms, 200ms)
#define BLACK_LINE_INIT_SHIELD_TIMES 90 // 初始转弯屏蔽黑线检测次数(单位10ms, 400ms)
#define GYRO_TURN_CONST_SPEED 20.0f
#define REVERSE_PARK_TIMES 50









void stop_motor() {
  status.motor.wheel[0].tar_speed = 0;
  status.motor.wheel[1].tar_speed = 0;
}

void follow(float speed) {
  get_gw_analogue_analogue_diff(&status.sensor.gw_analogue);
  float motor_diff = compute_pid(&status.sensor.gw_analogue.gw_analogue_pid,
                                 status.sensor.gw_analogue.diff);
  status.motor.wheel[0].tar_speed = speed + motor_diff;
  status.motor.wheel[1].tar_speed = speed - motor_diff;
}

void update_wheel_speed_control() {
  status.motor.wheel[0].cur_speed = (int16_t)get_wheel_speed(&status.motor.wheel[0]);
  status.motor.wheel[1].cur_speed = (int16_t)get_wheel_speed(&status.motor.wheel[1]);

  status.motor.wheel[0].trust = (int16_t)compute_pid(&status.motor.wheel[0].wheel_pid,
      (float)(status.motor.wheel[0].tar_speed - status.motor.wheel[0].cur_speed));
  status.motor.wheel[1].trust = (int16_t)compute_pid(&status.motor.wheel[1].wheel_pid,
      (float)(status.motor.wheel[1].tar_speed - status.motor.wheel[1].cur_speed));
  driver_wheel(&status.motor.wheel[0]);
  driver_wheel(&status.motor.wheel[1]);
}

void beep() {
  status.device.buzzer1.on = 1;
}

void beep_off() {
  status.device.buzzer1.on = 0;
}

void all_led_on() {
  status.device.led1.on = 1;
  status.device.led2.on = 1;
  status.device.led3.on = 1;
}

void all_led_off() {
  status.device.led1.on = 0;
  status.device.led2.on = 0;
  status.device.led3.on = 0;
}

void start_blink() {
  blink_phase = 0;
  blink_timer = BLINK_ON_MS / 10;
  all_led_on();
  beep();
}

uint8_t update_blink() {
  if (blink_timer > 0) {
    blink_timer--;
    if (blink_timer == 0) {
      blink_phase++;
      if (blink_phase >= BLINK_TIMES * 2) {
        all_led_off();
        beep_off();
        return 1;
      }
      if (blink_phase % 2 == 0) {
        all_led_on();
        beep();
        blink_timer = BLINK_ON_MS / 10;
      } else {
        all_led_off();
        beep_off();
        blink_timer = BLINK_OFF_MS / 10;
      }
    }
  }
  return 0;
}

// void angle_handler(float tar_angle_diff, uint8_t turn_mode) {
//   float cur_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
//
//   // 1. 计算绝对目标角度
//   float target_angle = gyro_start_angle + (tar_angle_diff * GYRO_DIR);
//   while (target_angle > 180.0f) target_angle -= 360.0f;
//   while (target_angle < -180.0f) target_angle += 360.0f;
//
//   // 2. 计算最短路径误差
//   float error = target_angle - cur_angle;
//   while (error > 180.0f) error -= 360.0f;
//   while (error < -180.0f) error += 360.0f;
//
//   // 3. 计算 PID
//   float diff_speed = compute_pid(&gyro_turn_pid, error);
//   diff_speed = CONFINE(diff_speed, -GYRO_TURN_MAX_DIFF, GYRO_TURN_MAX_DIFF);
//
//   // 4. 死区补偿 (防止差一点点角度时电机卡住不动)
//   float min_turn_speed = 15.0f;
//   if (diff_speed > 0 && diff_speed < min_turn_speed) {
//     diff_speed = min_turn_speed;
//   } else if (diff_speed < 0 && diff_speed > -min_turn_speed) {
//     diff_speed = -min_turn_speed;
//   }
//
//   // 5. 速度分配
//   switch (turn_mode) {
//   case GYRO_TURN_PIVOT: // 原地打转
//     status.motor.wheel[0].tar_speed = -diff_speed;
//     status.motor.wheel[1].tar_speed = diff_speed;
//     break;
//   case GYRO_TURN_SINGLE: // 单轮转弯
//     if (error > 0) {
//       status.motor.wheel[0].tar_speed = -diff_speed * 0.40f;
//       status.motor.wheel[1].tar_speed = diff_speed;
//     } else {
//       status.motor.wheel[0].tar_speed = -diff_speed;
//       status.motor.wheel[1].tar_speed = diff_speed * 0.40f;
//     }
//     break;
//   default: // GYRO_TURN_ARC (圆弧弯)
//     status.motor.wheel[0].tar_speed = GYRO_BASE_SPEED - diff_speed;
//     status.motor.wheel[1].tar_speed = GYRO_BASE_SPEED + diff_speed;
//     break;
//   }
// }

typedef enum
{
  forward = 0,
  first_stop,
}First_Statu;

First_Statu first_statu = forward;

void first_mode_handler() {
  // 确保平衡控制被彻底关闭，交出底层电机控制权
  pendulum_ctrl.state = PEND_STOP;

  // 终点检测：全白防抖逻辑
  static uint8_t all_white_cnt = 0;

  // 如果8个探头都没压到黑线 (digital_8bit == 0x00)
  if (status.sensor.gw_analogue.digital_8bit == 0x00) {
    all_white_cnt++;
  } else {
    all_white_cnt = 0; // 一旦碰到一点黑线，立刻清零
  }

  // 连续3次(15ms)读到全白，确认车头已驶出B点
  if (all_white_cnt >= 1) {
    first_statu = first_stop;
  }

  switch (first_statu) {
  case forward:
    follow(40); // 基础速度设为 40 RPM
    break;
  case first_stop:
    stop_motor();         // 普通车，直接切断动力刹车
    run_state = FINISHED;
    all_white_cnt = 0;    // 状态机重置，为下次运行做准备
    break;
  }
}

void second_mode_handler() {
  // 首次进入任务时，如果处于停机状态，则触发甩鞭起摆
  if (pendulum_ctrl.state == PEND_STOP) {
    pendulum_ctrl.target_speed = 0.0f;     // 目标：原地站立
    pendulum_ctrl.state = PEND_SWING_UP;   // 触发起摆状态机！
  }

}
void start_task() {
  if (mode == stop) return;
  run_state = COUNTDOWN;
  countdown_cnt = COUNTDOWN_MS / 10;
  beep();
  all_led_on();
}

void task_handler() {
  switch (run_state) {
    case STANDBY:
      stop_motor();
      break;

    case COUNTDOWN:
      stop_motor();
      if (countdown_cnt > 0) {
        countdown_cnt--;
        if (countdown_cnt == 0) {
          beep_off();
          status.device.led2.on = 0;
          status.device.led3.on = 0;
          run_state = RUNNING;
          cross_cnt = 0;

          if (status.sensor.gw_analogue.digital_8bit & 0x80) {
            turn_dir = 1;
          } else {
            turn_dir = 0;
          }
          gyro_start_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
          gyro_turn_pid.integral = 0;
          gyro_turn_pid.is_first = 1;
          gyro_turn_done = 0;
          black_line_cnt = 0;
          gyro_turn_phase = 0;
          black_line_shield_cnt = BLACK_LINE_INIT_SHIELD_TIMES;

          first_statu = forward;
          third_cross_cnt = 0;
          second_turn_cnt = 0;
          second_return_cnt = 0;
          status.sensor.gw_analogue.diff = 0;
          status.motor.wheel[0].wheel_pid.integral = 0;
          status.motor.wheel[1].wheel_pid.integral = 0;
          log_uprintf(&huart1, "[FIRST] COUNTDOWN done, dir=%d (L=%d R=%d)\n",
                      turn_dir,
                      (status.sensor.gw_analogue.digital_8bit & 0x80) ? 1 : 0,
                      (status.sensor.gw_analogue.digital_8bit & 0x01) ? 1 : 0);
        }
      }
      break;

    case RUNNING:
      // if (is_first)
      // {
      //   status.sensor.gw_analogue.diff = 0;
      //   status.motor.wheel[0].tar_speed = 0;
      //   status.motor.wheel[1].tar_speed = 0;
      //   status.motor.wheel[0].wheel_pid.integral = 0;
      //   status.motor.wheel[1].wheel_pid.integral = 0;
      // }
      // is_first = 0;
      switch (mode) {
        case first:
          first_mode_handler();
          break;
        case second:
          second_mode_handler();
          break;
        case third:
          //set_pid(&status.sensor.gw_analogue.gw_analogue_pid, 0.4,0,0.25);
          // third_mode_handler();
          break;
        case fourth:
          break;
        default:
          run_state = STANDBY;
          break;
      }
      break;

    case FINISHED:
      stop_motor();
      is_first = 1;
      if (update_blink()) {
        run_state = STANDBY;
      }
      break;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  status.state.time += status.state.T;
  update_task_clock(&status.task, status.state.time);
  if (htim == &htim7) {
    if (status.state.time == 50)
      status.device.led3.on = 1;
    else if (status.state.time == 100)
      status.device.led1.on = 0;
    else if (status.state.time == 150)
      status.device.led3.on = 1;
    else if (status.state.time == 200)
      status.device.led3.on = 0;
    else if (status.state.time == 250)
      status.device.led3.on = 1;
    else if (status.state.time == 300)
      status.device.led3.on = 0;
    driver_LED(&status.device.led3);
    //status.motor.wheel[0].trust = 500;
    //status.motor.wheel[1].trust = 500;
    //更新状态
    //satus.motor.wheel[0].trust = 400;
    //status.motor.wheel[1].trust = 400;
    //status.motor.wheel[0].tar_speed = 50;
    //status.motor.wheel[1].tar_speed = 50;
    //update_wheel_speed_control();
    if (status.state.time % 5 == 0)
    {
      get_gyr_raw_data(&hi2c1, &status.sensor.gyr); // 陀螺仪从10ms挪到5ms，对直立环至关重要
      driver_gw_analogue(&status.sensor.gw_analogue); // 读取黑线位置

      // 2. 更新编码器速度 (计算出 cur_speed)
      status.motor.wheel[0].cur_speed = (int16_t)get_wheel_speed(&status.motor.wheel[0]);
      status.motor.wheel[1].cur_speed = (int16_t)get_wheel_speed(&status.motor.wheel[1]);

      // 3. 执行核心控制算法
      if (pendulum_ctrl.state != 0) {
        // 倒立摆模式：执行三环融合，内部会直接刷新 trust 并调用 driver_wheel
        update_pendulum_control();
      } else {
        // 常规智能车模式：执行普通的轮速闭环
        status.motor.wheel[0].trust = (int16_t)compute_pid(&status.motor.wheel[0].wheel_pid,
            (float)(status.motor.wheel[0].tar_speed - status.motor.wheel[0].cur_speed));
        status.motor.wheel[1].trust = (int16_t)compute_pid(&status.motor.wheel[1].wheel_pid,
            (float)(status.motor.wheel[1].tar_speed - status.motor.wheel[1].cur_speed));
        driver_wheel(&status.motor.wheel[0]);
        driver_wheel(&status.motor.wheel[1]);
      }

    }
    if (status.state.time % 10 == 0) {
      driver_status(&status);

      get_gw_analogue_analogue_diff(&status.sensor.gw_analogue);
      //follow(90);
      task_handler();
      //gyro_turn_test();
    }
  }
}
