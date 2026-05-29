/**
 ******************************************************************************
 * @file    timer_it.c
 * @brief   定时器中断处理与任务调度核心实现
 *
 * @details 本文件是巡线小车的核心任务逻辑文件，包含：
 *          - 三种任务模式的状态机实现（first/second/third mode handler，当前已注释）
 *          - 陀螺仪转向PID控制（angle_handler函数实现角度闭环控制）
 *          - 巡线PID控制（follow函数实现差速巡线）
 *          - 轮子速度PID控制（update_wheel_speed_control实现底层电机闭环）
 *          - 路口识别和转弯逻辑（黑线检测、路口类型判断、转向时机）
 *          - 任务调度状态机（STANDBY → COUNTDOWN → RUNNING → FINISHED）
 *          - 陀螺仪锁死测试程序（用于调试PID参数）
 *          - TIM7定时器中断回调（10ms周期调度所有任务）
 *
 * @note    本文件基于STM32 HAL库，定时器中断周期为10ms。
 *          任务状态每10ms更新一次，轮子速度控制每5ms更新一次。
 ******************************************************************************
 */

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

/**
 * @brief 当前选择的任务模式
 * @details mode变量定义在task.h中，可选值：
 *          - stop:   停止状态
 *          - first:  任务一（基础巡线+路口转向）
 *          - second: 任务二（往返巡线+U-turn掉头）
 *          - third:  任务三（高速巡线+可配置参数）
 *          - fourth: 任务四（预留）
 */
MODE mode = stop;

/* ========================================================================
 * 任务一状态机枚举
 * ========================================================================
 * @details 状态流转：
 *          FIRST_INIT_TURN → FIRST_FOLLOW_LINE → FIRST_STOP → FIRST_TURNING
 *                                              → FIRST_PAUSE
 *                                              → FIRST_ARRIVED
 *
 *          任务一是基础巡线任务：从起点出发，依次经过4个路口并转向，
 *          到达终点后倒车入库停车。
 */
typedef enum {
  FIRST_FOLLOW_LINE = 0,    /**< 巡线直行状态：沿黑线前进，检测路口类型 */
  FIRST_STOP,               /**< 检测到路口，停车等待状态：等待CROSS_STOP_MS后开始转向 */
  FIRST_TURNING,            /**< 陀螺仪转向状态：根据turn_dir执行90度转向 */
  FIRST_PAUSE,              /**< 转向完成短暂停顿：切换回巡线状态 */
  FIRST_ARRIVED,            /**< 到达终点状态：倒车入库后停车 */
  FIRST_INIT_TURN,          /**< 发车初始转弯状态：起步时先转弯进入赛道 */
} FIRST_STATE;

/* ========================================================================
 * 任务二状态机枚举（往返巡线任务）
 * ========================================================================
 * @details 状态流转：
 *          SECOND_INIT_TURN → SECOND_FOLLOW_LINE → (SECOND_CROSSING_IGNORE)
 *                                                  → SECOND_STOP → SECOND_TURNING
 *                                                  → SECOND_UTURN（B点掉头180度）
 *                                                  → SECOND_RETURN_FOLLOW（返程巡线）
 *                                                  → (SECOND_RETURN_CROSSING_IGNORE)
 *                                                  → SECOND_RETURN_STOP → SECOND_RETURN_TURNING
 *                                                  → SECOND_RETURN_GYRO_TURN
 *                                                  → SECOND_ARRIVED
 *
 *          任务二是往返巡线任务：A→D→C→B（去程），B点U-turn掉头，
 *          然后B→C→D→A（返程），最后A点转向后倒车入库。
 *          去程和返程途中遇到T字/十字路口时冲过不停车（ignore状态）。
 */
typedef enum {
  SECOND_INIT_TURN = 0,              /**< 发车初始转弯：起步时转弯进入赛道 */
  SECOND_FOLLOW_LINE,                /**< 去程巡线直行：沿黑线前进，检测路口 */
  SECOND_CROSSING_IGNORE,            /**< 忽略T字/十字路口：冲过路口继续直行（短暂状态） */
  SECOND_STOP,                       /**< 检测到直角路口，停车等待：等待CROSS_STOP_MS后转向 */
  SECOND_TURNING,                    /**< 去程直角转弯：陀螺仪控制转向90度 */
  SECOND_UTURN,                      /**< B点陀螺仪掉头180度：原地打转直到检测到黑线 */
  SECOND_RETURN_FOLLOW,              /**< 返程巡线直行：掉头后沿黑线返回 */
  SECOND_RETURN_CROSSING_IGNORE,     /**< 返程忽略T字/十字路口：冲过路口继续直行 */
  SECOND_RETURN_STOP,                /**< 返程检测到直角路口，停车等待 */
  SECOND_RETURN_TURNING,             /**< 返程直角转弯：陀螺仪控制转向 */
  SECOND_RETURN_GYRO_TURN,           /**< A点陀螺仪转90度后停车：最后一个转弯 */
  SECOND_ARRIVED,                    /**< 到达终点，停车：倒车入库 */
} SECOND_STATE;

/* ========================================================================
 * 任务三状态机枚举（高速巡线任务）
 * ========================================================================
 * @details 状态流转：
 *          THIRD_INIT_TURN → THIRD_FOLLOW_LINE → (THIRD_CROSSING_IGNORE)
 *                                               → THIRD_STOP → THIRD_TURNING
 *                                               → THIRD_ARRIVED
 *
 *          任务三是高速巡线任务：使用third_params可配置参数（基速、路口数等），
 *          依次经过4个路口并转向，到达终点后倒车入库。
 *          与任务一的区别：速度更高、参数可配置、T字/十字路口冲过不停。
 */
typedef enum {
  THIRD_INIT_TURN = 0,               /**< 发车初始转弯：起步时转弯进入赛道 */
  THIRD_FOLLOW_LINE,                 /**< 巡线直行：使用third_params.base_speed高速巡线 */
  THIRD_CROSSING_IGNORE,             /**< 忽略T字/十字路口：冲过路口继续直行 */
  THIRD_STOP,                        /**< 检测到直角路口，停车等待 */
  THIRD_TURNING,                     /**< 直角转弯：陀螺仪控制转向90度 */
  THIRD_ARRIVED,                     /**< 到达终点，停车：倒车入库 */
} THIRD_STATE;

/* ========================================================================
 * 运行阶段状态机枚举（总任务调度状态）
 * ========================================================================
 * @details 状态流转：
 *          STANDBY → COUNTDOWN → RUNNING → FINISHED → STANDBY
 *
 *          - STANDBY:   待机状态，电机停止，等待start_task()触发
 *          - COUNTDOWN: 倒计时状态，1秒倒计时，蜂鸣器报警+全LED亮起
 *          - RUNNING:   任务运行中，根据mode选择执行对应任务状态机
 *          - FINISHED:  任务完成，LED闪烁3次提示，闪烁结束后回到STANDBY
 */
typedef enum {
  STANDBY = 0,   /**< 待机状态：电机停止，等待用户触发任务 */
  COUNTDOWN,     /**< 倒计时状态：1秒后开始任务，蜂鸣器报警提示 */
  RUNNING,       /**< 任务运行中：执行对应模式的状态机逻辑 */
  FINISHED,      /**< 任务完成状态：LED闪烁3次，蜂鸣器提示 */
} RUN_STATE;

/* ========================================================================
 * 全局状态变量
 * ======================================================================== */

/** @brief 当前运行阶段（STANDBY/COUNTDOWN/RUNNING/FINISHED） */
RUN_STATE run_state = STANDBY;

/** @brief 任务一当前状态（FIRST_STATE枚举值） */
FIRST_STATE first_state = FIRST_INIT_TURN;

/** @brief 任务二当前状态（SECOND_STATE枚举值） */
SECOND_STATE second_state = SECOND_INIT_TURN;

/** @brief 任务三当前状态（THIRD_STATE枚举值） */
THIRD_STATE third_state = THIRD_INIT_TURN;

/** @brief 任务三：已检测到的直角路口计数（到达4个后结束） */
uint8_t third_cross_cnt = 0;

/** @brief 任务一：已检测到的路口计数（到达4个后结束） */
uint8_t cross_cnt = 0;

/** @brief 转弯方向标志：0=左转，1=右转（由路口检测时确定） */
uint8_t turn_dir = 0;

/** @brief 路口冲过延时计数器（单位10ms）：用于CROSSING_IGNORE状态冲过路口的时长控制 */
uint16_t cross_delay_cnt = 0;

/** @brief 倒计时计数器（单位10ms）：COUNTDOWN状态下的倒计时 */
uint16_t countdown_cnt = 0;

/** @brief LED闪烁阶段计数（BLINK_TIMES*2个阶段：亮→灭→亮→灭...） */
uint8_t blink_phase = 0;

/** @brief 闪烁定时计数器（单位10ms）：控制LED亮/灭的持续时间 */
uint8_t blink_timer = 0;

/** @brief 任务二：去程已转弯路口计数（到达SECOND_GO_CORNERS后U-turn） */
uint8_t second_turn_cnt = 0;

/** @brief 任务二：返程已转弯路口计数（到达SECOND_RETURN_CORNERS后到达终点） */
uint8_t second_return_cnt = 0;

/** @brief 陀螺仪转向起始角度：每次转向前抓取当前yaw轴角度作为基准 */
float gyro_start_angle = 0;

/** @brief 陀螺仪转向目标角度差：正数=左转角度增大，负数=右转角度减小 */
float gyro_tar_angle = 0;

/** @brief 陀螺仪转向完成标志：PID计算误差小于阈值时置1 */
uint8_t gyro_turn_done = 0;

/** @brief 角度环PID控制器实例：用于陀螺仪转向时的角度闭环控制 */
PID gyro_turn_pid;

/** @brief 首次转弯标志：初始转弯和后续转弯使用不同角度参数 */
uint8_t is_first = 1;

/** @brief 黑线连续检测计数：转向时检测到黑线的连续次数，达到阈值后切换回巡线 */
uint8_t black_line_cnt = 0;

/** @brief 陀螺仪转向阶段：0=PID角度控制阶段，1=恒速寻线阶段 */
uint8_t gyro_turn_phase = 0;

/** @brief 黑线检测屏蔽计数：转弯开始后屏蔽路口检测的次数（单位10ms） */
uint8_t black_line_shield_cnt = 0;

/** @brief 倒车入库计数器：到达终点后倒车行驶的时长控制（单位10ms） */
uint16_t reverse_park_cnt = 0;

/** @brief 首次标志：用于区分第一次转弯和后续转弯的角度参数 */
uint8_t first_flag=1;

/* ========================================================================
 * 宏定义常量
 * ======================================================================== */

/** @brief 冲过路口延时(ms)：T字/十字路口冲过时保持直行的时间 */
#define CROSS_DELAY_MS 10

/** @brief 检测到路口后停车等待时间(ms)：停车后等待再开始转向 */
#define CROSS_STOP_MS 10

/** @brief 陀螺仪转向模式宏定义（与timer_it.h中一致） */
#define GYRO_TURN_ARC 0
#define GYRO_TURN_PIVOT 1
#define GYRO_TURN_SINGLE 2

/** @brief 陀螺仪转向时基速(RPM)：圆弧弯模式下双轮的基础速度 */
#define GYRO_BASE_SPEED 35

/** @brief 发车倒计时(ms)：从start_task()到正式开始的等待时间 */
#define COUNTDOWN_MS 100

/** @brief 闪烁亮灯时长(ms)：FINISHED状态下LED每次亮起的时间 */
#define BLINK_ON_MS 100

/** @brief 闪烁灭灯时长(ms)：FINISHED状态下LED每次熄灭的时间 */
#define BLINK_OFF_MS 100

/** @brief 到达终点闪烁次数：LED和蜂鸣器闪烁提示的次数 */
#define BLINK_TIMES 3

/** @brief 任务二去程路口数(A→D→C→B)：经过3个路口后到达B点进行U-turn */
#define SECOND_GO_CORNERS 3

/** @brief 任务二返程路口数(B→C→D→A)：经过3个路口后到达A点结束 */
#define SECOND_RETURN_CORNERS 3

/** @brief 陀螺仪转向允许偏差(度)：误差小于此值时认为转向到位 */
#define GYRO_TURN_THRESHOLD 5.0f

/** @brief 陀螺仪方向设置：1=左转角度增大，-1=左转角度减小（方向反了改这里） */
#define GYRO_DIR 1

/** @brief 转弯后屏蔽路口检测次数(单位10ms)：防止刚转完弯误检测到路口 */
#define CROSS_SHIELD_TIMES 200

/** @brief 陀螺仪转向后屏蔽路口检测次数(单位10ms, 约2000ms)：转向完成后屏蔽较长时间 */
#define GYRO_SHIELD_TIMES 2000

/** @brief 角度PID最大轮速差(RPM)：限制PID输出的最大差速，防止转向过猛 */
#define GYRO_TURN_MAX_DIFF 25.0f

/** @brief 陀螺仪转向时黑线连续检测次数阈值：连续检测到N次黑线才确认找到巡线 */
#define BLACK_LINE_DETECT_TIMES 1

/** @brief 左转时检测右通道掩码：左转时检测右侧传感器（通道5,6）是否压到黑线 */
#define BLACK_LINE_MASK_LTURN 0x3C

/** @brief 右转时检测左通道掩码：右转时检测左侧传感器（通道3,4）是否压到黑线 */
#define BLACK_LINE_MASK_RTURN 0x3C

/** @brief 转弯开始后屏蔽黑线检测次数(单位10ms, 约100ms)：防止转弯初期误检测 */
#define BLACK_LINE_SHIELD_TIMES 10

/** @brief 初始转弯屏蔽黑线检测次数(单位10ms, 约900ms)：起步转弯屏蔽更长时间 */
#define BLACK_LINE_INIT_SHIELD_TIMES 90

/** @brief 陀螺仪转向恒速寻线阶段的速度差值(RPM)：PID到位后以固定差速继续寻线 */
#define GYRO_TURN_CONST_SPEED 20.0f

/** @brief 倒车入库时长(单位10ms)：到达终点后倒车行驶的时长 */
#define REVERSE_PARK_TIMES 50

/**
 * @brief 任务三可配置参数结构体
 * @details 通过修改此结构体可以调整任务三的巡线速度、路口屏蔽时间等参数，
 *          而不影响任务一和任务二的运行。
 */
typedef struct {
  float base_speed;            /**< 巡线基速(RPM)：任务三的巡线速度 */
  uint8_t cross_shield_times;  /**< 路口检测屏蔽次数：转弯后屏蔽路口检测的时长 */
  uint16_t cross_delay_ms;     /**< 冲过路口延时(ms)：T字/十字路口冲过的时长 */
  uint16_t cross_stop_ms;      /**< 路口停车等待(ms)：直角路口停车等待的时长 */
  uint8_t corners;             /**< 路口总数：到达此数量后任务结束 */
} third_params_t;

/** @brief 任务三参数实例：基速90RPM，4个路口 */
third_params_t third_params = {
  .base_speed = 90.0f,
  .cross_shield_times = 200,
  .cross_delay_ms = 10,
  .cross_stop_ms = 10,
  .corners = 4,
};

/* ========================================================================
 * 基础控制函数
 * ======================================================================== */

/**
 * @brief  停止电机
 * @details 将左右轮目标速度设为0，由update_wheel_speed_control()读取并执行。
 *          在路口停车、任务完成、状态切换等场景下调用。
 */
void stop_motor() {
  status.motor.wheel[0].tar_speed = 0;
  status.motor.wheel[1].tar_speed = 0;
}

/**
 * @brief  巡线PID控制函数（差速巡线）
 *
 * @param[in] speed 巡线基速(RPM)，左右轮的基础目标速度
 *
 * @details 巡线原理：
 *          1. 调用get_gw_analogue_analogue_diff()获取传感器偏差值（diff）
 *             diff > 0 表示偏右，需要向左修正
 *             diff < 0 表示偏左，需要向右修正
 *          2. 通过gw_analogue_pid计算修正量motor_diff
 *          3. 左轮目标速度 = speed + motor_diff
 *             右轮目标速度 = speed - motor_diff
 *          4. 当小车偏右时，左轮加速、右轮减速，小车向左修正；反之亦然
 *
 * @note    此函数每10ms调用一次，实现高频巡线修正。
 *          速度控制由update_wheel_speed_control()在下一周期执行。
 */
void follow(float speed) {
  get_gw_analogue_analogue_diff(&status.sensor.gw_analogue);
  float motor_diff = compute_pid(&status.sensor.gw_analogue.gw_analogue_pid,
                                 status.sensor.gw_analogue.diff);
  status.motor.wheel[0].tar_speed = speed + motor_diff;
  status.motor.wheel[1].tar_speed = speed - motor_diff;
}

/**
 * @brief  轮子速度PID控制函数（底层电机闭环）
 *
 * @details 控制流程（双闭环控制的外环）：
 *          1. 读取左右轮当前实际速度（编码器反馈）
 *          2. 计算速度误差 = tar_speed - cur_speed
 *          3. 通过wheel_pid计算电机输出占空比（trust）
 *          4. 调用driver_wheel()将trust写入PWM寄存器
 *
 * @note    此函数每5ms调用一次（10ms中断中每2次执行一次），
 *          实现比巡线更高频率的速度闭环控制。
 *          这是双闭环控制结构：内环是轮子速度PID，外环是巡线/角度PID。
 */
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

/** @brief  开启蜂鸣器 */
void beep() {
  status.device.buzzer1.on = 1;
}

/** @brief  关闭蜂鸣器 */
void beep_off() {
  status.device.buzzer1.on = 0;
}

/** @brief  开启所有LED（LED1+LED2+LED3） */
void all_led_on() {
  status.device.led1.on = 1;
  status.device.led2.on = 1;
  status.device.led3.on = 1;
}

/** @brief  关闭所有LED */
void all_led_off() {
  status.device.led1.on = 0;
  status.device.led2.on = 0;
  status.device.led3.on = 0;
}

/**
 * @brief  启动LED闪烁和蜂鸣器提示（到达终点时调用）
 * @details 初始化闪烁状态：
 *          - blink_phase = 0
 *          - blink_timer = BLINK_ON_MS / 10（亮灯时长）
 *          - 全LED亮起
 *          - 蜂鸣器响起
 *
 * @note    闪烁由update_blink()在每10ms中更新状态
 */
void start_blink() {
  blink_phase = 0;
  blink_timer = BLINK_ON_MS / 10;
  all_led_on();
  beep();
}

/**
 * @brief  更新LED闪烁状态（每10ms调用一次）
 * @return 1表示闪烁完成，0表示闪烁仍在进行中
 *
 * @details 闪烁状态机：
 *          - 每个阶段持续BLINK_ON_MS或BLINK_OFF_MS
 *          - 总共BLINK_TIMES*2个阶段（3次亮+3次灭=6个阶段）
 *          - 阶段0: 亮（0→1）, 阶段1: 灭（1→2）, 阶段2: 亮（2→3）...
 *          - 偶数阶段：亮灯+蜂鸣器
 *          - 奇数阶段：灭灯+关蜂鸣器
 *          - 完成后关闭所有LED和蜂鸣器，返回1
 */
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

/* ========================================================================
 * 核心PID控制函数：陀螺仪转向角度闭环控制
 * ========================================================================
 *
 * @details 角度控制原理（最短路径PID控制）：
 *          本函数实现基于陀螺仪的角度闭环控制，用于各种转向场景：
 *          - 直角转弯（90度）
 *          - 圆弧弯（80度）
 *          - U-turn掉头（180度）
 *
 *          控制流程：
 *          1. 获取当前yaw轴角度（-180 ~ +180度）
 *          2. 计算绝对目标角度 = gyro_start_angle + tar_angle_diff * GYRO_DIR
 *          3. 规范化目标角度到-180~180范围
 *          4. 计算最短路径误差（考虑跨0度线的情况）
 *          5. 通过gyro_turn_pid计算差速输出
 *          6. 限制最大差速（防止转向过猛）
 *          7. 死区补偿（防止小角度时电机卡住）
 *          8. 根据turn_mode分配左右轮速度
 *
 * @param[in] tar_angle_diff 目标角度差（正=左转，负=右转）
 * @param[in] turn_mode      转向模式（GYRO_TURN_ARC / PIVOT / SINGLE）
 *
 * @note    三种转向模式的速度分配：
 *          - GYRO_TURN_ARC（圆弧弯）：双轮保持基速，通过差速实现弧线
 *            wheel[0] = BASE_SPEED - diff_speed
 *            wheel[1] = BASE_SPEED + diff_speed
 *
 *          - GYRO_TURN_PIVOT（原地打转）：两轮反向旋转
 *            wheel[0] = -diff_speed
 *            wheel[1] = diff_speed
 *
 *          - GYRO_TURN_SINGLE（单轮转弯）：内侧轮减速40%，外侧轮全速
 *            error > 0（左转）：wheel[0] = -diff*0.4, wheel[1] = diff
 *            error < 0（右转）：wheel[0] = -diff, wheel[1] = diff*0.4
 */
void angle_handler(float tar_angle_diff, uint8_t turn_mode) {
  float cur_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);

  // 1. 计算绝对目标角度
  //    目标角度 = 起始角度 + 目标差值 * 方向系数
  //    例如：起始90度，需要左转90度 → 目标180度
  //          起始90度，需要右转90度 → 目标0度（或180度，取决于GYRO_DIR）
  float target_angle = gyro_start_angle + (tar_angle_diff * GYRO_DIR);
  // 规范化到-180~180范围（处理跨0度线的情况）
  while (target_angle > 180.0f) target_angle -= 360.0f;
  while (target_angle < -180.0f) target_angle += 360.0f;

  // 2. 计算最短路径误差
  //    例如：当前170度，目标-170度
  //    直接计算：-170 - 170 = -340度（绕远路）
  //    最短路径：-340 + 360 = 20度（反向绕）
  float error = target_angle - cur_angle;
  while (error > 180.0f) error -= 360.0f;
  while (error < -180.0f) error += 360.0f;

  // 3. 计算PID差速输出
  //    PID根据角度误差计算左右轮的速度差
  //    error > 0 需要左转 → diff_speed > 0 → 左轮减速、右轮加速
  //    error < 0 需要右转 → diff_speed < 0 → 左轮加速、右轮减速
  float diff_speed = compute_pid(&gyro_turn_pid, error);
  // 限制最大差速，防止转向过猛导致翻车或打滑
  diff_speed = CONFINE(diff_speed, -GYRO_TURN_MAX_DIFF, GYRO_TURN_MAX_DIFF);

  // 4. 死区补偿
  //    当差速很小时，电机可能无法克服摩擦力而卡住
  //    设置最小转向速度15RPM，确保电机能转动
  float min_turn_speed = 15.0f;
  if (diff_speed > 0 && diff_speed < min_turn_speed) {
    diff_speed = min_turn_speed;
  } else if (diff_speed < 0 && diff_speed > -min_turn_speed) {
    diff_speed = -min_turn_speed;
  }

  // 5. 速度分配（根据转向模式）
  switch (turn_mode) {
  case GYRO_TURN_PIVOT: // 原地打转：两轮反向旋转
    status.motor.wheel[0].tar_speed = -diff_speed;
    status.motor.wheel[1].tar_speed = diff_speed;
    break;
  case GYRO_TURN_SINGLE: // 单轮转弯：内侧轮减速40%，外侧轮全速
    if (error > 0) { // 左转：左轮减速，右轮全速
      status.motor.wheel[0].tar_speed = -diff_speed * 0.40f;
      status.motor.wheel[1].tar_speed = diff_speed;
    } else {         // 右转：右轮减速，左轮全速
      status.motor.wheel[0].tar_speed = -diff_speed;
      status.motor.wheel[1].tar_speed = diff_speed * 0.40f;
    }
    break;
  default: // GYRO_TURN_ARC (圆弧弯)：双轮基速 + 差速修正
    status.motor.wheel[0].tar_speed = GYRO_BASE_SPEED - diff_speed;
    status.motor.wheel[1].tar_speed = GYRO_BASE_SPEED + diff_speed;
    break;
  }
}

/* ========================================================================
 * 以下三个任务模式状态机已注释，保留作为参考
 * ======================================================================== */

// void first_mode_handler() {
//   Road road = status.sensor.gw_analogue.cross.cross;
//   uint8_t digital = status.sensor.gw_analogue.digital_8bit;
//
//   switch (first_state) {
//     case FIRST_INIT_TURN:
//       // 初始转弯：抓取起始角度，清空PID，设置黑线屏蔽
//       // 状态切换：FIRST_INIT_TURN → FIRST_TURNING
//       ...
//
//     case FIRST_FOLLOW_LINE:
//       // 巡线直行：调用follow(BASE_SPEED)进行差速巡线
//       // 检测到LeftRoad/RightRoad时：
//       //   - 路口计数cross_cnt++
//       //   - 如果cross_cnt >= 4 → 到达终点 → FIRST_ARRIVED
//       //   - 否则 → 停车等待 → FIRST_STOP
//       ...
//
//     case FIRST_STOP:
//       // 停车等待：倒计时cross_delay_cnt
//       // 倒计时结束后 → FIRST_TURNING（开始转向）
//       ...
//
//     case FIRST_TURNING:
//       // 陀螺仪转向：分两个阶段
//       //   phase 0: angle_handler() PID角度控制，直到gyro_turn_done
//       //   phase 1: 恒速差速寻线，直到检测到黑线 → FIRST_FOLLOW_LINE
//       // 黑线屏蔽：black_line_shield_cnt期间不检测黑线
//       ...
//
//     case FIRST_ARRIVED:
//       // 到达终点：倒车入库reverse_park_cnt时长 → 停车 → FINISHED
//       ...
//   }
// }

// void second_mode_handler() {
//   Road road = status.sensor.gw_analogue.cross.cross;
//
//   switch (second_state) {
//     case SECOND_INIT_TURN:
//       // 初始转弯：同first模式
//       ...
//
//     case SECOND_FOLLOW_LINE:
//       // 去程巡线：
//       //   - T字/十字路口 → SECOND_CROSSING_IGNORE（冲过不停）
//       //   - 直角路口 → SECOND_STOP（停车等待）→ SECOND_TURNING
//       //   - 路口计数second_turn_cnt，到达3个 → SECOND_UTURN（B点掉头）
//       ...
//
//     case SECOND_CROSSING_IGNORE:
//       // 冲过T字/十字路口：短暂直行后回到FOLLOW_LINE
//       ...
//
//     case SECOND_UTURN:
//       // B点U-turn掉头180度：
//       //   - 使用GYRO_TURN_PIVOT模式原地打转
//       //   - 转过90度后开始检测黑线
//       //   - 检测到黑线 → SECOND_RETURN_FOLLOW（返程巡线）
//       ...
//
//     case SECOND_RETURN_FOLLOW:
//       // 返程巡线：同去程逻辑
//       //   - T字/十字路口 → RETURN_CROSSING_IGNORE
//       //   - 直角路口 → RETURN_STOP → RETURN_TURNING
//       //   - 路口计数second_return_cnt，到达3个 → SECOND_ARRIVED
//       ...
//
//     case SECOND_ARRIVED:
//       // 到达终点：倒车入库 → FINISHED
//       ...
//   }
// }

// void third_mode_handler() {
//   Road road = status.sensor.gw_analogue.cross.cross;
//
//   switch (third_state) {
//     // 同first模式逻辑，但使用third_params可配置参数
//     // 速度更高（90RPM vs BASE_SPEED）
//     // T字/十字路口冲过不停（有CROSSING_IGNORE状态）
//     ...
//   }
// }

#if 1

/* ========================================================================
 * 陀螺仪锁死测试程序
 * ========================================================================
 * @details 用于调试陀螺仪PID控制参数，验证角度闭环稳定性。
 *          通过按钮触发测试，可以观察PID在不同目标角度下的响应。
 *
 *          状态流转：
 *          GYRO_TEST_IDLE → GYRO_TEST_START → GYRO_TEST_HOLDING → GYRO_TEST_DONE
 *
 *          - IDLE:    电机停止，等待启动
 *          - START:   抓取当前角度作为基准，清空PID积分，切换到HOLDING
 *          - HOLDING: 持续调用angle_handler()锁死目标角度，打印调试信息
 *          - DONE:    电机停止
 *
 * @note    变量已在timer_it.h中声明为extern，供button.c访问
 */

/** @brief 陀螺仪测试状态机当前状态（初始化为IDLE静止） */
GYRO_TEST_STATE gyro_test_state = GYRO_TEST_IDLE;

/** @brief 测试目标角度（默认-90度，即右转90度） */
float gyro_test_tar = -90.0f;

/** @brief 测试转向模式（0=GYRO_TURN_ARC圆弧弯模式） */
uint8_t gyro_test_mode = 0;

/**
 * @brief  陀螺仪转向测试函数（每10ms调用一次）
 * @details 根据gyro_test_state执行不同逻辑：
 *          - IDLE:    停止电机
 *          - START:   初始化（抓取起始角度、清空PID）→ 切换到HOLDING
 *          - HOLDING: 持续角度闭环控制，打印调试信息（当前角度、误差、左右轮速度）
 *          - DONE:    停止电机
 */
void gyro_turn_test() {
  switch (gyro_test_state) {
  case GYRO_TEST_IDLE:
    stop_motor();
    break;

  case GYRO_TEST_START:
    // 自动抓取起步角度，并清空PID历史
    gyro_start_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
    gyro_turn_pid.integral = 0;
    gyro_turn_pid.is_first = 1;

    gyro_test_state = GYRO_TEST_HOLDING;
    log_uprintf(&huart1, "TEST START! Base: %.1f, Tar: %.1f\n", gyro_start_angle, gyro_test_tar);
    break;

  case GYRO_TEST_HOLDING: {
      // 持续死锁目标角度：调用angle_handler进行PID闭环控制
      angle_handler(gyro_test_tar, gyro_test_mode);

      // 计算纯粹用于打印显示的数据（不影响控制逻辑）
      float cur_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
      float target_angle = gyro_start_angle + (gyro_test_tar * GYRO_DIR);
      while (target_angle > 180.0f) target_angle -= 360.0f;
      while (target_angle < -180.0f) target_angle += 360.0f;

      float error = target_angle - cur_angle;
      while (error > 180.0f) error -= 360.0f;
      while (error < -180.0f) error += 360.0f;

      // 打印调试信息：当前角度、误差、左右轮目标速度
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


/* ========================================================================
 * 任务调度核心函数
 * ======================================================================== */

/**
 * @brief  启动任务（触发1秒倒计时）
 * @details 被调用后：
 *          1. 检查mode是否为stop，是则直接返回（不启动）
 *          2. 设置run_state = COUNTDOWN
 *          3. 初始化countdown_cnt = COUNTDOWN_MS / 10 = 10（1秒）
 *          4. 蜂鸣器报警，全LED亮起
 *
 * @note    通常在按钮中断中调用此函数
 */
void start_task() {
  if (mode == stop) return;
  run_state = COUNTDOWN;
  countdown_cnt = COUNTDOWN_MS / 10;
  beep();
  all_led_on();
}

/**
 * @brief  任务调度主函数（每10ms调用一次）
 * @details 根据run_state执行不同逻辑：
 *
 *          STANDBY:
 *            - 停止电机，等待start_task()触发
 *
 *          COUNTDOWN:
 *            - 停止电机（等待期间不移动）
 *            - 倒计时countdown_cnt，每秒10次计数
 *            - 倒计时结束时：
 *              a. 关闭蜂鸣器和LED2/LED3
 *              b. 切换run_state = RUNNING
 *              c. 重置所有状态变量（cross_cnt、陀螺仪参数、PID积分等）
 *              d. 根据传感器8位数据判断初始转弯方向（最左/最右传感器压线）
 *              e. 初始化三个任务状态机为初始转弯状态
 *
 *          RUNNING:
 *            - 根据mode选择执行对应任务状态机（first/second/third）
 *            - 当前所有任务状态机都已注释，此分支为空操作
 *
 *          FINISHED:
 *            - 停止电机
 *            - 调用update_blink()更新闪烁状态
 *            - 闪烁完成后切换回STANDBY
 */
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

          // 根据传感器判断初始转弯方向
          // 0x80表示最左侧传感器（通道8）检测到黑线 → 需要右转
          // 否则 → 需要左转
          if (status.sensor.gw_analogue.digital_8bit & 0x80) {
            turn_dir = 1;  // 右转
          } else {
            turn_dir = 0;  // 左转
          }

          // 初始化陀螺仪参数
          gyro_start_angle = get_gyr_value(&status.sensor.gyr, gyr_z_yaw);
          gyro_turn_pid.integral = 0;
          gyro_turn_pid.is_first = 1;
          gyro_turn_done = 0;
          black_line_cnt = 0;
          gyro_turn_phase = 0;
          black_line_shield_cnt = BLACK_LINE_INIT_SHIELD_TIMES;

          // 初始化三个任务状态机
          first_state = FIRST_TURNING;
          second_state = SECOND_INIT_TURN;
          third_state = THIRD_INIT_TURN;
          third_cross_cnt = 0;
          second_turn_cnt = 0;
          second_return_cnt = 0;

          // 重置巡线PID相关
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
      switch (mode) {
        case first:
          // first_mode_handler();
          break;
        case second:
          // second_mode_handler();
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

/* ========================================================================
 * TIM7定时器中断回调函数（10ms周期）
 * ========================================================================
 * @details 中断频率：10ms（由htim7配置决定）
 *          本回调是所有任务的调度入口，按不同频率执行各种操作：
 *
 *          每次中断（10ms）：
 *            - 更新系统时间（status.state.time += 10ms）
 *            - 更新任务时钟
 *            - LED指示灯控制（50ms/100ms/150ms/200ms/250ms/300ms时序）
 *            - 轮子速度PID控制（update_wheel_speed_control）
 *
 *          每50ms（time % 5 == 0）：
 *            - 巡线传感器数据采集（driver_gw_analogue）
 *
 *          每100ms（time % 10 == 0）：
 *            - 设备状态更新（driver_status：按钮、LED、蜂鸣器、陀螺仪）
 *            - 巡线偏差计算（get_gw_analogue_analogue_diff）
 *            - 任务状态机更新（task_handler）
 *
 * @note    status.state.time以ms为单位，每10ms增加一次。
 *          通过取模运算实现不同频率的任务调度。
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  // 更新系统时间（每次中断增加一个周期）
  status.state.time += status.state.T;
  // 更新任务时钟
  update_task_clock(&status.task, status.state.time);

  if (htim == &htim7) {
    // LED指示灯时序控制（300ms循环）
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

    // 轮子速度PID控制（每次中断执行，10ms周期）
    update_wheel_speed_control();

    // 巡线传感器数据采集（每50ms执行）
    if (status.state.time % 5 == 0)
    {
      driver_gw_analogue(&status.sensor.gw_analogue);
    }

    // 任务调度主循环（每100ms执行）
    if (status.state.time % 10 == 0) {
      // 更新设备状态（按钮、LED、蜂鸣器、陀螺仪）
      driver_status(&status);
      // 计算巡线偏差
      get_gw_analogue_analogue_diff(&status.sensor.gw_analogue);
      // 执行任务状态机
      task_handler();
      // 陀螺仪测试（已注释，调试时取消注释）
      //gyro_turn_test();
    }
  }
}
