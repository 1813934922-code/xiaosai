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

MODE mode = stop;

typedef enum {
  FIRST_FOLLOW_LINE = 0,    // 巡线直行
  FIRST_STOP,               // 检测到路口，停车等待
  FIRST_TURNING,            // 陀螺仪转向
  FIRST_PAUSE,              // 转向完成短暂停顿
  FIRST_ARRIVED,
  FIRST_INIT_TURN,// 到达终点，停车
} FIRST_STATE;

typedef enum {
  SECOND_INIT_TURN = 0,              // 发车初始转弯
  SECOND_FOLLOW_LINE,                // 去程巡线直行
  SECOND_CROSSING_IGNORE,            // 忽略T字/十字路口，冲过继续直行
  SECOND_STOP,                       // 检测到直角路口，停车等待
  SECOND_TURNING,                    // 去程直角转弯
  SECOND_UTURN,                      // B点陀螺仪掉头180度
  SECOND_RETURN_FOLLOW,              // 返程巡线直行
  SECOND_RETURN_CROSSING_IGNORE,     // 返程忽略T字/十字路口，冲过继续直行
  SECOND_RETURN_STOP,                // 返程检测到直角路口，停车等待
  SECOND_RETURN_TURNING,             // 返程直角转弯
  SECOND_RETURN_GYRO_TURN,           // A点陀螺仪转90度后停车
  SECOND_ARRIVED,                    // 到达终点，停车
} SECOND_STATE;

typedef enum {
  THIRD_INIT_TURN = 0,               // 发车初始转弯
  THIRD_FOLLOW_LINE,                 // 巡线直行
  THIRD_CROSSING_IGNORE,             // 忽略T字/十字路口，冲过继续直行
  THIRD_STOP,                        // 检测到直角路口，停车等待
  THIRD_TURNING,                     // 直角转弯
  THIRD_ARRIVED,                     // 到达终点，停车
} THIRD_STATE;

typedef enum {
  STANDBY = 0,   // 待机，电机停止
  COUNTDOWN,     // 倒计时，1秒后开始任务
  RUNNING,       // 任务运行中
  FINISHED,      // 任务完成，闪烁提示
} RUN_STATE;

RUN_STATE run_state = STANDBY;              // 当前运行阶段
FIRST_STATE first_state = FIRST_INIT_TURN; // 任务一当前状态
SECOND_STATE second_state = SECOND_INIT_TURN; // 任务二当前状态
THIRD_STATE third_state = THIRD_INIT_TURN;   // 任务三当前状态
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

typedef struct {
  float base_speed;
  uint8_t cross_shield_times;
  uint16_t cross_delay_ms;
  uint16_t cross_stop_ms;
  uint8_t corners ;
} third_params_t;

third_params_t third_params = {
  .base_speed = 90.0f,
  .cross_shield_times = 200,
  .cross_delay_ms = 10,
  .cross_stop_ms = 10,
  .corners = 4,
};


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
  status.motor.wheel[0].cur_speed = get_wheel_speed(&status.motor.wheel[0]);
  status.motor.wheel[1].cur_speed = get_wheel_speed(&status.motor.wheel[1]);

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

void angle_handler(float tar_angle_diff, uint8_t turn_mode) {
  float cur_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);

  // 1. 计算绝对目标角度
  float target_angle = gyro_start_angle + (tar_angle_diff * GYRO_DIR);
  while (target_angle > 180.0f) target_angle -= 360.0f;
  while (target_angle < -180.0f) target_angle += 360.0f;

  // 2. 计算最短路径误差
  float error = target_angle - cur_angle;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;

  // 3. 计算 PID
  float diff_speed = compute_pid(&gyro_turn_pid, error);
  diff_speed = CONFINE(diff_speed, -GYRO_TURN_MAX_DIFF, GYRO_TURN_MAX_DIFF);

  // 4. 死区补偿 (防止差一点点角度时电机卡住不动)
  float min_turn_speed = 15.0f;
  if (diff_speed > 0 && diff_speed < min_turn_speed) {
    diff_speed = min_turn_speed;
  } else if (diff_speed < 0 && diff_speed > -min_turn_speed) {
    diff_speed = -min_turn_speed;
  }

  // 5. 速度分配
  switch (turn_mode) {
  case GYRO_TURN_PIVOT: // 原地打转
    status.motor.wheel[0].tar_speed = -diff_speed;
    status.motor.wheel[1].tar_speed = diff_speed;
    break;
  case GYRO_TURN_SINGLE: // 单轮转弯
    if (error > 0) {
      status.motor.wheel[0].tar_speed = -diff_speed * 0.40f;
      status.motor.wheel[1].tar_speed = diff_speed;
    } else {
      status.motor.wheel[0].tar_speed = -diff_speed;
      status.motor.wheel[1].tar_speed = diff_speed * 0.40f;
    }
    break;
  default: // GYRO_TURN_ARC (圆弧弯)
    status.motor.wheel[0].tar_speed = GYRO_BASE_SPEED - diff_speed;
    status.motor.wheel[1].tar_speed = GYRO_BASE_SPEED + diff_speed;
    break;
  }
}

void first_mode_handler() {
  Road road = status.sensor.gw_analogue.cross.cross;
  uint8_t digital = status.sensor.gw_analogue.digital_8bit;
  //uint8_t center_black = (digital & 0x18) != 0;

  switch (first_state) {
    case FIRST_INIT_TURN:
      //turn_dir = 0;
      gyro_start_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
      gyro_turn_pid.integral = 0;
      gyro_turn_pid.is_first = 1;
      gyro_turn_done = 0;
      black_line_cnt = 0;
      gyro_turn_phase = 0;
      first_flag = 1;
      black_line_shield_cnt = BLACK_LINE_INIT_SHIELD_TIMES;
      first_state = FIRST_TURNING;
      log_uprintf(&huart1, "[FIRST] INIT_TURN dir=%d (L=%d R=%d)\n",
                  turn_dir,
                  (status.sensor.gw_analogue.digital_8bit & 0x80) ? 1 : 0,
                  (status.sensor.gw_analogue.digital_8bit & 0x01) ? 1 : 0);
      //cross_cnt++;
      break;

    case FIRST_FOLLOW_LINE:
      follow(BASE_SPEED);
      if (road == LeftRoad ) {
        cross_cnt++;
        turn_dir = 0;
        if (cross_cnt >= 4) {
          stop_motor();
          reverse_park_cnt = REVERSE_PARK_TIMES;
          first_state = FIRST_ARRIVED;
          start_blink();
          log_uprintf(&huart1, "[FIRST] ARRIVED cross=%d\n", cross_cnt);
        } else {
          stop_motor();
          cross_delay_cnt = CROSS_STOP_MS / 10;
          beep();
          first_state = FIRST_STOP;
          log_uprintf(&huart1, "[FIRST] LEFT cross=%d stop=%dms\n", cross_cnt, CROSS_STOP_MS);
        }
      } else if (road == RightRoad ) {
        cross_cnt++;
        turn_dir = 1;
        if (cross_cnt >= 4) {
          stop_motor();
          reverse_park_cnt = REVERSE_PARK_TIMES;
          first_state = FIRST_ARRIVED;
          start_blink();
          log_uprintf(&huart1, "[FIRST] ARRIVED cross=%d\n", cross_cnt);
        } else {
          stop_motor();
          cross_delay_cnt = CROSS_STOP_MS / 10;
          beep();
          first_state = FIRST_STOP;
          log_uprintf(&huart1, "[FIRST] RIGHT cross=%d stop=%dms\n", cross_cnt, CROSS_STOP_MS);
        }
      }
      break;

    case FIRST_STOP:
      if (cross_delay_cnt > 0) {
        cross_delay_cnt--;
      } else {
        gyro_start_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
        gyro_turn_pid.integral = 0;
        gyro_turn_pid.is_first = 1;
        gyro_turn_done = 0;
        black_line_cnt = 0;
        gyro_turn_phase = 0;
        black_line_shield_cnt = BLACK_LINE_SHIELD_TIMES;
        beep_off();
        first_state = FIRST_TURNING;
        log_uprintf(&huart1, "[FIRST] TURNING dir=%d angle=%.0f\n", turn_dir, turn_dir == 0 ? 45.0f : -45.0f);
      }
      break;

    case FIRST_TURNING:
      if (black_line_shield_cnt > 0) {
        black_line_shield_cnt--;
        black_line_cnt = 0;
      } else if (status.sensor.gw_analogue.digital_8bit & (turn_dir == 0 ? BLACK_LINE_MASK_LTURN : BLACK_LINE_MASK_RTURN)) {
        black_line_cnt++;
        if (black_line_cnt >= BLACK_LINE_DETECT_TIMES) {
          black_line_cnt = 0;
          gyro_turn_pid.integral = 0;
          stop_motor();
          first_state = FIRST_FOLLOW_LINE;
          status.sensor.gw_analogue.cross.cross = Straight;
          status.sensor.gw_analogue.cross.cross_delay = GYRO_SHIELD_TIMES;
          first_flag = 0;
          log_uprintf(&huart1, "[FIRST] Black line detected, switch to FOLLOW_LINE\n");
          break;
        }
      } else {
        black_line_cnt = 0;
      }
      if (gyro_turn_phase == 0) {
        if (first_flag)
        {
          angle_handler(turn_dir == 0 ? 80.0f : -80.0f, GYRO_TURN_ARC);
        }else
        {
          angle_handler(turn_dir == 0 ? 90.0f : -90.0f, GYRO_TURN_ARC);
        }

        if (gyro_turn_done) {
          gyro_turn_done = 0;
          gyro_turn_phase = 1;
          log_uprintf(&huart1, "[FIRST] Angle reached, constant speed searching line\n");
        }
      } else {
        if (turn_dir == 0) {
          status.motor.wheel[0].tar_speed = GYRO_BASE_SPEED - GYRO_TURN_CONST_SPEED;
          status.motor.wheel[1].tar_speed = GYRO_BASE_SPEED + GYRO_TURN_CONST_SPEED;
        } else {
          status.motor.wheel[0].tar_speed = GYRO_BASE_SPEED + GYRO_TURN_CONST_SPEED;
          status.motor.wheel[1].tar_speed = GYRO_BASE_SPEED - GYRO_TURN_CONST_SPEED;
        }
      }
      break;

    case FIRST_PAUSE:
      first_state = FIRST_FOLLOW_LINE;
      status.sensor.gw_analogue.cross.cross = Straight;
      status.sensor.gw_analogue.cross.cross_delay = GYRO_SHIELD_TIMES;
      break;

    case FIRST_ARRIVED:
      if (reverse_park_cnt > 0) {
        reverse_park_cnt--;
        status.motor.wheel[0].tar_speed = -BASE_SPEED / 2;
        status.motor.wheel[1].tar_speed = -BASE_SPEED / 2;
      } else {
        stop_motor();
        run_state = FINISHED;
      }
      break;
  }
}

void second_mode_handler() {
  Road road = status.sensor.gw_analogue.cross.cross;

  switch (second_state) {
    case SECOND_INIT_TURN:

      gyro_start_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
      gyro_turn_pid.integral = 0;
      gyro_turn_pid.is_first = 1;
      gyro_turn_done = 0;
      black_line_cnt = 0;
      gyro_turn_phase = 0;
      first_flag = 1;
      black_line_shield_cnt = BLACK_LINE_INIT_SHIELD_TIMES;
      second_state = SECOND_TURNING;
      //log_uprintf(&huart1, "[SECOND] INIT_TURN dir=%d (L=%d R=%d)\n",
                  //turn_dir,
                  //(status.sensor.gw_analogue.digital_8bit & 0x80) ? 1 : 0,
                  //(status.sensor.gw_analogue.digital_8bit & 0x01) ? 1 : 0);
      break;

    case SECOND_FOLLOW_LINE:
      follow(BASE_SPEED);
      if (road == TLRoad || road == TRRoad || road == CrossRoad || road == TBRoad) {
        beep();
        second_state = SECOND_CROSSING_IGNORE;
        cross_delay_cnt = CROSS_DELAY_MS / 10;
        log_uprintf(&huart1, "[SECOND] T/Cross road, ignore\n");
      } else if (road == LeftRoad) {
        second_turn_cnt++;
        turn_dir = 0;
        if (second_turn_cnt >= SECOND_GO_CORNERS) {
          gyro_start_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
          gyro_turn_pid.integral = 0;
          gyro_turn_pid.is_first = 1;
          gyro_turn_done = 0;
          black_line_cnt = 0;
          gyro_turn_phase = 0;
          black_line_shield_cnt = BLACK_LINE_SHIELD_TIMES;
          stop_motor();
          second_state = SECOND_UTURN;
          beep();
          //log_uprintf(&huart1, "[SECOND] LEFT cross=%d -> UTURN\n", second_turn_cnt);
        } else {
          stop_motor();
          cross_delay_cnt = CROSS_STOP_MS / 10;
          beep();
          second_state = SECOND_STOP;
          //log_uprintf(&huart1, "[SECOND] LEFT cross=%d stop=%dms\n", second_turn_cnt, CROSS_STOP_MS);
        }
      } else if (road == RightRoad) {
        second_turn_cnt++;
        turn_dir = 1;
        if (second_turn_cnt >= SECOND_GO_CORNERS) {
          gyro_start_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
          gyro_turn_pid.integral = 0;
          gyro_turn_pid.is_first = 1;
          gyro_turn_done = 0;
          black_line_cnt = 0;
          gyro_turn_phase = 0;
          black_line_shield_cnt = BLACK_LINE_SHIELD_TIMES;
          stop_motor();
          second_state = SECOND_UTURN;
          beep();
          ///log_uprintf(&huart1, "[SECOND] RIGHT cross=%d -> UTURN\n", second_turn_cnt);
        } else {
          stop_motor();
          cross_delay_cnt = CROSS_STOP_MS / 10;
          beep();
          second_state = SECOND_STOP;
          //log_uprintf(&huart1, "[SECOND] RIGHT cross=%d stop=%dms\n", second_turn_cnt, CROSS_STOP_MS);
        }
      }
      break;

    case SECOND_CROSSING_IGNORE:
      follow(BASE_SPEED);


      beep_off();
      second_state = SECOND_FOLLOW_LINE;
      status.sensor.gw_analogue.cross.cross = Straight;
      //status.sensor.gw_analogue.cross.cross_delay = CROSS_SHIELD_TIMES;


      break;

    case SECOND_STOP:
      if (cross_delay_cnt > 0) {
        cross_delay_cnt--;
      } else {
        gyro_start_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
        gyro_turn_pid.integral = 0;
        gyro_turn_pid.is_first = 1;
        gyro_turn_done = 0;
        black_line_cnt = 0;
        gyro_turn_phase = 0;
        black_line_shield_cnt = BLACK_LINE_SHIELD_TIMES;
        beep_off();
        second_state = SECOND_TURNING;
        log_uprintf(&huart1, "[SECOND] TURNING dir=%d angle=%.0f\n", turn_dir, turn_dir == 0 ? 90.0f : -90.0f);
      }
      break;

    case SECOND_TURNING:
      if (black_line_shield_cnt > 0) {
        black_line_shield_cnt--;
        black_line_cnt = 0;
      } else if (status.sensor.gw_analogue.digital_8bit & (turn_dir == 0 ? BLACK_LINE_MASK_LTURN : BLACK_LINE_MASK_RTURN)) {
        black_line_cnt++;
        if (black_line_cnt >= BLACK_LINE_DETECT_TIMES) {
          black_line_cnt = 0;
          gyro_turn_pid.integral = 0;
          stop_motor();
          second_state = SECOND_FOLLOW_LINE;
          status.sensor.gw_analogue.cross.cross = Straight;
          first_flag = 0;
          status.sensor.gw_analogue.cross.cross_delay = GYRO_SHIELD_TIMES;
          log_uprintf(&huart1, "[SECOND] Black line detected, switch to FOLLOW_LINE\n");
          break;
        }
      } else {
        black_line_cnt = 0;
      }
      if (gyro_turn_phase == 0) {
        if (first_flag)
        {
          angle_handler(turn_dir == 0 ? 80.0f : -80.0f, GYRO_TURN_ARC);
        }else
        {
          angle_handler(turn_dir == 0 ? 90.0f : -90.0f, GYRO_TURN_ARC);
        }

        if (gyro_turn_done) {
          gyro_turn_done = 0;
          gyro_turn_phase = 1;
          log_uprintf(&huart1, "[SECOND] Angle reached, constant speed searching line\n");
        }
      } else {
        if (turn_dir == 0) {
          status.motor.wheel[0].tar_speed = GYRO_BASE_SPEED - GYRO_TURN_CONST_SPEED;
          status.motor.wheel[1].tar_speed = GYRO_BASE_SPEED + GYRO_TURN_CONST_SPEED;
        } else {
          status.motor.wheel[0].tar_speed = GYRO_BASE_SPEED + GYRO_TURN_CONST_SPEED;
          status.motor.wheel[1].tar_speed = GYRO_BASE_SPEED - GYRO_TURN_CONST_SPEED;
        }
      }
      break;

    case SECOND_UTURN: {
      // 1. 获取当前陀螺仪角度，并计算相对于转向起点的偏差角度
      float cur_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
      float diff = cur_angle - gyro_start_angle;

      // 角度规范化到 -180 到 180 之间
      while (diff > 180.0f) diff -= 360.0f;
      while (diff < -180.0f) diff += 360.0f;

      // 取绝对值，得到实际已经转过的度数
      if (diff < 0) diff = -diff;

      // 2. 捕捉转过90度的点，满足条件后开启黑线检测
      if (diff >= 90.0f) {
        // 使用原有的左转掉头掩码检测黑线
        if (status.sensor.gw_analogue.digital_8bit & BLACK_LINE_MASK_LTURN) {
          black_line_cnt++;
          if (black_line_cnt >= BLACK_LINE_DETECT_TIMES) {
            // 确认检测到黑线，清除状态并切换到直线模式
            black_line_cnt = 0;
            gyro_turn_pid.integral = 0;
            stop_motor();

            second_state = SECOND_RETURN_FOLLOW;
            status.sensor.gw_analogue.cross.cross = Straight;
            status.sensor.gw_analogue.cross.cross_delay = GYRO_SHIELD_TIMES;

            log_uprintf(&huart1, "[SECOND] UTURN black line detected after 90deg, switch to RETURN_FOLLOW\n");
            break; // 成功切线，跳出当前 case
          }
        } else {
          black_line_cnt = 0;
        }
      }

      // 3. 始终使用陀螺仪控制目标转 180 度 (原地打转)
      // angle_handler 会在内部自动处理 PID 计算和死区限制
      angle_handler(180.0f, GYRO_TURN_PIVOT);

      // 4. 清除到位标志位 (防止 PID 锁死在 180 度时不断触发 done)
      if (gyro_turn_done) {
        gyro_turn_done = 0;
      }

      break;
    }

    case SECOND_RETURN_FOLLOW:
      follow(BASE_SPEED);
      if (road == TLRoad || road == TRRoad || road == CrossRoad || road == TBRoad) {
        beep();
        second_state = SECOND_RETURN_CROSSING_IGNORE;
        cross_delay_cnt = CROSS_DELAY_MS / 10;
        log_uprintf(&huart1, "[SECOND] RETURN T/Cross road, ignore\n");
      } else if (road == LeftRoad) {
        second_return_cnt++;
        turn_dir = 0;
        if (second_return_cnt >= SECOND_RETURN_CORNERS) {
          stop_motor();
          reverse_park_cnt = REVERSE_PARK_TIMES;
          second_state = SECOND_ARRIVED;
          start_blink();
          log_uprintf(&huart1, "[SECOND] RETURN LEFT cross=%d -> ARRIVED\n", second_return_cnt);
        } else {
          stop_motor();
          cross_delay_cnt = CROSS_STOP_MS / 10;
          beep();
          second_state = SECOND_RETURN_STOP;
          log_uprintf(&huart1, "[SECOND] RETURN LEFT cross=%d stop=%dms\n", second_return_cnt, CROSS_STOP_MS);
        }
      } else if (road == RightRoad) {
        second_return_cnt++;
        turn_dir = 1;
        if (second_return_cnt >= SECOND_RETURN_CORNERS) {
          stop_motor();
          reverse_park_cnt = REVERSE_PARK_TIMES;
          second_state = SECOND_ARRIVED;
          start_blink();
          log_uprintf(&huart1, "[SECOND] RETURN RIGHT cross=%d -> ARRIVED\n", second_return_cnt);
        } else {
          stop_motor();
          cross_delay_cnt = CROSS_STOP_MS / 10;
          beep();
          second_state = SECOND_RETURN_STOP;
          log_uprintf(&huart1, "[SECOND] RETURN RIGHT cross=%d stop=%dms\n", second_return_cnt, CROSS_STOP_MS);
        }
      }
      break;

    case SECOND_RETURN_CROSSING_IGNORE:
      follow(BASE_SPEED);


      beep_off();
      second_state = SECOND_RETURN_FOLLOW;
      status.sensor.gw_analogue.cross.cross = Straight;
        //status.sensor.gw_analogue.cross.cross_delay = CROSS_SHIELD_TIMES;


      break;

    case SECOND_RETURN_STOP:
      if (cross_delay_cnt > 0) {
        cross_delay_cnt--;
      } else {
        gyro_start_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
        gyro_turn_pid.integral = 0;
        gyro_turn_pid.is_first = 1;
        gyro_turn_done = 0;
        black_line_cnt = 0;
        gyro_turn_phase = 0;
        black_line_shield_cnt = BLACK_LINE_SHIELD_TIMES;
        beep_off();
        second_state = SECOND_RETURN_TURNING;
        log_uprintf(&huart1, "[SECOND] RETURN_TURNING dir=%d angle=%.0f\n", turn_dir, turn_dir == 0 ? 70.0f : -70.0f);
      }
      break;

    case SECOND_RETURN_TURNING:
      if (black_line_shield_cnt > 0) {
        black_line_shield_cnt--;
        black_line_cnt = 0;
      } else if (status.sensor.gw_analogue.digital_8bit & (turn_dir == 0 ? BLACK_LINE_MASK_LTURN : BLACK_LINE_MASK_RTURN)) {
        black_line_cnt++;
        if (black_line_cnt >= BLACK_LINE_DETECT_TIMES) {
          black_line_cnt = 0;
          gyro_turn_pid.integral = 0;
          stop_motor();
          second_state = SECOND_RETURN_FOLLOW;
          status.sensor.gw_analogue.cross.cross = Straight;
          status.sensor.gw_analogue.cross.cross_delay = GYRO_SHIELD_TIMES;
          log_uprintf(&huart1, "[SECOND] Black line detected during RETURN_TURN, switch to RETURN_FOLLOW\n");
          break;
        }
      } else {
        black_line_cnt = 0;
      }
      if (gyro_turn_phase == 0) {
        angle_handler(turn_dir == 0 ? 90.0f : -90.0f, GYRO_TURN_ARC);
        if (gyro_turn_done) {
          gyro_turn_done = 0;
          gyro_turn_phase = 1;
          log_uprintf(&huart1, "[SECOND] RETURN_TURN angle reached, constant speed searching line\n");
        }
      } else {
        if (turn_dir == 0) {
          status.motor.wheel[0].tar_speed = GYRO_BASE_SPEED - GYRO_TURN_CONST_SPEED;
          status.motor.wheel[1].tar_speed = GYRO_BASE_SPEED + GYRO_TURN_CONST_SPEED;
        } else {
          status.motor.wheel[0].tar_speed = GYRO_BASE_SPEED + GYRO_TURN_CONST_SPEED;
          status.motor.wheel[1].tar_speed = GYRO_BASE_SPEED - GYRO_TURN_CONST_SPEED;
        }
      }
      break;

    case SECOND_RETURN_GYRO_TURN:
      if (black_line_shield_cnt > 0) {
        black_line_shield_cnt--;
        black_line_cnt = 0;
      } else if (status.sensor.gw_analogue.digital_8bit & (turn_dir == 0 ? BLACK_LINE_MASK_LTURN : BLACK_LINE_MASK_RTURN)) {
        black_line_cnt++;
        if (black_line_cnt >= BLACK_LINE_DETECT_TIMES) {
          black_line_cnt = 0;
          gyro_turn_pid.integral = 0;
          stop_motor();
          reverse_park_cnt = REVERSE_PARK_TIMES;
          second_state = SECOND_ARRIVED;
          start_blink();
          log_uprintf(&huart1, "[SECOND] RETURN GYRO_TURN black line detected, ARRIVED\n");
          break;
        }
      } else {
        black_line_cnt = 0;
      }
      if (gyro_turn_phase == 0) {
        angle_handler(turn_dir == 0 ? 90.0f : -90.0f, GYRO_TURN_ARC);
        if (gyro_turn_done) {
          gyro_turn_done = 0;
          gyro_turn_phase = 1;
          log_uprintf(&huart1, "[SECOND] RETURN GYRO_TURN angle reached, constant speed searching line\n");
        }
      } else {
        if (turn_dir == 0) {
          status.motor.wheel[0].tar_speed = GYRO_BASE_SPEED - GYRO_TURN_CONST_SPEED;
          status.motor.wheel[1].tar_speed = GYRO_BASE_SPEED + GYRO_TURN_CONST_SPEED;
        } else {
          status.motor.wheel[0].tar_speed = GYRO_BASE_SPEED + GYRO_TURN_CONST_SPEED;
          status.motor.wheel[1].tar_speed = GYRO_BASE_SPEED - GYRO_TURN_CONST_SPEED;
        }
      }
      break;

    case SECOND_ARRIVED:
      if (reverse_park_cnt > 0) {
        reverse_park_cnt--;
        status.motor.wheel[0].tar_speed = -BASE_SPEED / 2;
        status.motor.wheel[1].tar_speed = -BASE_SPEED / 2;
      } else {
        stop_motor();
        run_state = FINISHED;
      }
      break;
  }
}

uint8_t first_dir;
void third_mode_handler() {
  Road road = status.sensor.gw_analogue.cross.cross;

  switch (third_state) {
    case THIRD_INIT_TURN:
      gyro_start_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
      gyro_turn_pid.integral = 0;
      gyro_turn_pid.is_first = 1;
      gyro_turn_done = 0;
      black_line_cnt = 0;
      gyro_turn_phase = 0;
      first_flag = 1;
      first_dir = turn_dir;
      black_line_shield_cnt = BLACK_LINE_INIT_SHIELD_TIMES;
      third_state = THIRD_TURNING;
      // log_uprintf(&huart1, "[THIRD] INIT_TURN dir=%d (L=%d R=%d)\n",
      //             turn_dir,
      //             (status.sensor.gw_analogue.digital_8bit & 0x80) ? 1 : 0,
      //             (status.sensor.gw_analogue.digital_8bit & 0x01) ? 1 : 0);
      break;

    case THIRD_FOLLOW_LINE:

      follow(third_params.base_speed);

      if (road == TLRoad || road == TRRoad || road == CrossRoad || road == TBRoad) {
        beep();
        third_state = THIRD_CROSSING_IGNORE;
        cross_delay_cnt = third_params.cross_delay_ms / 10;
        // log_uprintf(&huart1, "[THIRD] T/Cross road, ignore\n");
      } else if (road == LeftRoad) {
        third_cross_cnt++;
        turn_dir = 0;
        if (third_cross_cnt >= 4) {
          stop_motor();
          reverse_park_cnt = REVERSE_PARK_TIMES;
          third_state = THIRD_ARRIVED;
          start_blink();
          // log_uprintf(&huart1, "[THIRD] ARRIVED cross=%d\n", third_cross_cnt);
        } else {
          stop_motor();
          cross_delay_cnt = third_params.cross_stop_ms / 10;
          beep();
          third_state = THIRD_STOP;
          //log_uprintf(&huart1, "[THIRD] LEFT cross=%d stop=%dms\n", third_cross_cnt, third_params.cross_stop_ms);
        }
      } else if (road == RightRoad) {
        third_cross_cnt++;
        turn_dir = 1;
        if (third_cross_cnt >= 4) {
          stop_motor();
          reverse_park_cnt = REVERSE_PARK_TIMES;
          third_state = THIRD_ARRIVED;
          start_blink();
          //log_uprintf(&huart1, "[THIRD] ARRIVED cross=%d\n", third_cross_cnt);
        } else {
          stop_motor();
          cross_delay_cnt = third_params.cross_stop_ms / 10;
          beep();
          third_state = THIRD_STOP;
          //log_uprintf(&huart1, "[THIRD] RIGHT cross=%d stop=%dms\n", third_cross_cnt, third_params.cross_stop_ms);
        }
      }
      break;

    case THIRD_CROSSING_IGNORE:
      follow(third_params.base_speed);
      beep_off();
      third_state = THIRD_FOLLOW_LINE;
      status.sensor.gw_analogue.cross.cross = Straight;
          //status.sensor.gw_analogue.cross.cross_delay = third_params.cross_shield_times;


      break;

    case THIRD_STOP:
      if (cross_delay_cnt > 0) {
        cross_delay_cnt--;
      } else {
        gyro_start_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
        gyro_turn_pid.integral = 0;
        gyro_turn_pid.is_first = 1;
        gyro_turn_done = 0;
        black_line_cnt = 0;
        gyro_turn_phase = 0;
        black_line_shield_cnt = BLACK_LINE_SHIELD_TIMES;
        beep_off();
        third_state = THIRD_TURNING;
        log_uprintf(&huart1, "[THIRD] TURNING dir=%d angle=%.0f\n", turn_dir, turn_dir == 0 ? 70.0f : -70.0f);
      }
      break;

    case THIRD_TURNING:
      if (black_line_shield_cnt > 0) {
        black_line_shield_cnt--;
        black_line_cnt = 0;
      } else if (status.sensor.gw_analogue.digital_8bit & (turn_dir == 0 ? BLACK_LINE_MASK_LTURN : BLACK_LINE_MASK_RTURN)) {
        black_line_cnt++;
        if (black_line_cnt >= BLACK_LINE_DETECT_TIMES) {
          black_line_cnt = 0;
          gyro_turn_pid.integral = 0;
          stop_motor();
          third_state = THIRD_FOLLOW_LINE;
          status.sensor.gw_analogue.cross.cross = Straight;
          status.sensor.gw_analogue.cross.cross_delay = GYRO_SHIELD_TIMES;
          log_uprintf(&huart1, "[THIRD] Black line detected, switch to FOLLOW_LINE\n");
          break;
        }
      } else {
        black_line_cnt = 0;
      }
      if (gyro_turn_phase == 0) {
        if (first_flag)
        {
          angle_handler(turn_dir == 0 ? 80.0f : -80.0f, GYRO_TURN_ARC);
        }else
        {
          angle_handler(turn_dir == 0 ? 90.0f : -90.0f, GYRO_TURN_ARC);
        }

        if (gyro_turn_done) {
          gyro_turn_done = 0;
          gyro_turn_phase = 1;
          log_uprintf(&huart1, "[THIRD] Angle reached, constant speed searching line\n");
        }
      } else {
        if (turn_dir == 0) {
          status.motor.wheel[0].tar_speed = GYRO_BASE_SPEED - GYRO_TURN_CONST_SPEED;
          status.motor.wheel[1].tar_speed = GYRO_BASE_SPEED + GYRO_TURN_CONST_SPEED;
        } else {
          status.motor.wheel[0].tar_speed = GYRO_BASE_SPEED + GYRO_TURN_CONST_SPEED;
          status.motor.wheel[1].tar_speed = GYRO_BASE_SPEED - GYRO_TURN_CONST_SPEED;
        }
      }
      break;

    case THIRD_ARRIVED:
      if (reverse_park_cnt > 0) {
        reverse_park_cnt--;
        status.motor.wheel[0].tar_speed = -third_params.base_speed / 2;
        status.motor.wheel[1].tar_speed = -third_params.base_speed / 2;
      } else {
        stop_motor();
        run_state = FINISHED;
      }
      break;
  }
}

#if 1

// ==========================================
// 陀螺仪锁死测试程序 (变量已在 timer_it.h 中声明)
// ==========================================

GYRO_TEST_STATE gyro_test_state = GYRO_TEST_IDLE; // 初始化为静止
float gyro_test_tar = -90.0f;                     // 测试目标角度
uint8_t gyro_test_mode = 0;         // 默认使用原地打转模式测试

void gyro_turn_test() {
  switch (gyro_test_state) {
  case GYRO_TEST_IDLE:
    stop_motor();
    break;

  case GYRO_TEST_START:
    // 自动抓取起步角度，并清空 PID 历史
    gyro_start_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
    gyro_turn_pid.integral = 0;
    gyro_turn_pid.is_first = 1;

    gyro_test_state = GYRO_TEST_HOLDING;
    log_uprintf(&huart1, "TEST START! Base: %.1f, Tar: %.1f\n", gyro_start_angle, gyro_test_tar);
    break;

  case GYRO_TEST_HOLDING: {
      // 持续死锁目标角度
      angle_handler(gyro_test_tar, gyro_test_mode);

      // 计算纯粹用于打印显示的数据
      float cur_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
      float target_angle = gyro_start_angle + (gyro_test_tar * GYRO_DIR);
      while (target_angle > 180.0f) target_angle -= 360.0f;
      while (target_angle < -180.0f) target_angle += 360.0f;

      float error = target_angle - cur_angle;
      while (error > 180.0f) error -= 360.0f;
      while (error < -180.0f) error += 360.0f;

      log_uprintf(&huart1, "Cur: %5.1f | Err: %5.1f | L_Spd: %5d | R_Spd: %5d\n",
                  cur_angle, error,
                  status.motor.wheel[0].tar_speed, status.motor.wheel[1].tar_speed);
      break;
  }

  case GYRO_TEST_DONE:
    stop_motor();
    break;
  }
}
#endif


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

          first_state = FIRST_TURNING;
          second_state = SECOND_INIT_TURN;
          third_state = THIRD_INIT_TURN;
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
      if (is_first)
      {
        status.sensor.gw_analogue.diff = 0;
        status.motor.wheel[0].tar_speed = 0;
        status.motor.wheel[1].tar_speed = 0;
        status.motor.wheel[0].wheel_pid.integral = 0;
        status.motor.wheel[1].wheel_pid.integral = 0;
      }
      is_first = 0;
      switch (mode) {
        case first:
          first_mode_handler();
          break;
        case second:
          second_mode_handler();
          break;
        case third:
          //set_pid(&status.sensor.gw_analogue.gw_analogue_pid, 0.4,0,0.25);
          third_mode_handler();
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
    //status.motor.wheel[0].tar_speed = 90;
    //status.motor.wheel[1].tar_speed = 90;
    update_wheel_speed_control();
    if (status.state.time % 5 == 0)
    {
      driver_gw_analogue(&status.sensor.gw_analogue);

      //driver_wheel(&status.motor.wheel[0]);
      //driver_wheel(&status.motor.wheel[1]);
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
