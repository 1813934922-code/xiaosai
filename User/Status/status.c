/**
 ******************************************************************************
 * @file    status.c
 * @brief   系统状态树初始化和驱动实现
 *
 * @details 本文件实现了全局状态树status的初始化和运行时驱动：
 *          - 全局状态树实例定义（STATUS status）
 *          - 子模块初始化函数（motor、device、sensor、state）
 *          - 状态树总初始化函数（init_status）
 *          - 设备驱动更新函数（driver_status）
 *
 * @note    状态树架构说明：
 *          status是全局唯一的状态树根节点，包含所有系统数据。
 *          初始化时配置所有子模块参数，运行时通过driver_status统一更新。
 *          其他模块通过extern status访问，避免全局变量分散。
 ******************************************************************************
 */

#include "status.h"

#include "log.h"
#include "math_tool.h"
#include "status.h"
#include "wheel.h"
#include "uart_it.h"
#include "gy901.h"
#include "gw_anagloge.h"
#include "buzzer.h"
#include "i2c.h"

/** @brief 全局状态树实例（唯一全局变量，其他模块通过extern访问） */
STATUS status;

/**
 * @defgroup PID控制器实例
 * @brief 全局PID控制器定义（部分未使用，预留）
 * @{
 */

/** @brief 平衡PID（平衡车模式使用，当前未使用） */
PID balance_pid;

/** @brief 速度PID（平衡车模式使用，当前未使用） */
PID speed_pid;

/** @brief 偏航PID（平衡车模式使用，当前未使用） */
PID yaw_pid;

/** @brief 寻线PID（平衡车模式使用，当前未使用） */
PID find_line_pid;

/** @} */

/** @brief 陀螺仪转向PID（在timer_it.c中定义，此处为外部引用） */
extern PID gyro_turn_pid;

/* ========================================================================
 * 子模块初始化函数
 * ======================================================================== */

/**
 * @brief  初始化电机模块
 * @details 配置左右轮的基本参数：
 *          - 左轮(wheel[0]): PWM通道1，方向系数1（正转前进）
 *          - 右轮(wheel[1]): PWM通道2，方向系数-1（正转后退，物理安装方向相反）
 *
 * @note    方向系数-1是因为右轮电机物理安装方向与左轮相反，
 *          需要反向PWM才能保持前进方向一致。
 */
void init_motor() {
  init_wheel(&status.motor.wheel[0], 1, 1);   // 左轮：通道1，方向正
  init_wheel(&status.motor.wheel[1], 2, -1);  // 右轮：通道2，方向反

  return;
}

/**
 * @brief  初始化外设设备
 * @details 初始化所有外设设备的参数：
 *          - button1: 按钮1，通道1，无上拉
 *          - button2: 按钮2，通道2，无上拉
 *          - led1/2/3: 3个LED，分别对应通道1/2/3，默认高电平点亮
 *          - buzzer1: 蜂鸣器，通道1，默认高电平有效
 */
void init_device() {
  init_button(&status.device.button1, 1, 0);
  init_button(&status.device.button2, 2, 0);
  init_LED(&status.device.led1, 1, 1);
  init_LED(&status.device.led2, 2, 1);
  init_LED(&status.device.led3, 3, 1);
  init_BUZZER(&status.device.buzzer1,1,1);
  return;
}

/**
 * @brief  初始化传感器模块
 * @details 初始化所有传感器：
 *          - gw_analogue: 巡线传感器（8通道模拟/数字）
 *          - gyr: 陀螺仪（GY-901模块）
 *
 * @param[in] status 状态树指针
 */
void init_sensor(STATUS *status) {
  init_gw_analogue(&status->sensor.gw_analogue);
  init_gyr(&status->sensor.gyr);
}

/**
 * @brief  初始化系统状态
 * @details 设置系统时间参数：
 *          - T: 系统周期（通常10ms，由定时器中断周期决定）
 *          - time: 系统运行时间，从0开始
 *
 * @param[in] status 状态树指针
 * @param[in] T      系统周期（单位ms）
 */
void init_state(STATUS *status, uint8_t T) {
  status->state.T = T;
  status->state.time = 0;

  return;
}

/* ========================================================================
 * 状态树总初始化函数
 * ======================================================================== */

/**
 * @brief  初始化全局状态树
 *
 * @param[in] status 状态树指针
 * @param[in] T      系统周期（单位ms），通常设为10
 *
 * @details 初始化顺序（严格按依赖关系）：
 *          1. init_state():
 *             设置系统周期T=10ms，运行时间time=0
 *
 *          2. init_sensor():
 *             初始化巡线传感器（配置ADC/DMA/GPIO）
 *             初始化陀螺仪（配置I2C通信）
 *
 *          3. init_motor():
 *             初始化左轮（PWM通道1，方向系数1）
 *             初始化右轮（PWM通道2，方向系数-1）
 *
 *          4. init_device():
 *             初始化按钮（GPIO输入配置）
 *             初始化LED（GPIO输出配置）
 *             初始化蜂鸣器（PWM/TIM输出配置）
 *
 *          5. init_uart_idle_it():
 *             启动串口DMA空闲接收（USART1和USART3）
 *             USART3用于接收上位机下发的浮点参数
 *
 *          6. gyro_turn_pid = init_pid():
 *             初始化陀螺仪转向PID控制器
 *             参数：Kp=0.5, Ki=0.0, Kd=10.0, 输出限制±10.0, 积分限制±200.0
 *             注意：Kd较大说明主要依赖微分项实现快速角度响应
 *
 *          7. set_gyr_6axis_mode():
 *             设置陀螺仪为6轴模式（加速度计+陀螺仪）
 *             通过I2C配置GY-901模块
 *
 * @note    此函数应在main()中尽早调用，在启动定时器中断之前完成所有初始化。
 */
void init_status(STATUS *status, uint8_t T) {
  // 1. 初始化系统状态（周期、时间）
  init_state(status, T);

  // 2. 初始化传感器（巡线、陀螺仪）
  init_sensor(status);

  // 3. 初始化电机（左右轮）
  init_motor();

  // 4. 初始化外设（LED、按钮、蜂鸣器）
  init_device();

  // 5. 初始化串口DMA空闲接收
  init_uart_idle_it();

  // 6. 初始化陀螺仪转向PID
  //    参数说明：
  //    - Kp=0.5:    比例系数，角度误差的直接响应
  //    - Ki=0.0:    积分系数，当前未使用积分项
  //    - Kd=10.0:   微分系数，主要控制项，提供阻尼防止超调
  //    - 输出限制=±10.0: PID输出限制
  //    - 积分限制=±200.0: 积分项限制（防止积分饱和）
  gyro_turn_pid = init_pid(0.5f, 0.0f, 10.0f, 10.0f, 200.0f);

  // 7. 设置陀螺仪6轴模式
  set_gyr_6axis_mode(&hi2c1);

  return;
}

/* ========================================================================
 * 设备驱动更新函数
 * ======================================================================== */

/**
 * @brief  驱动所有设备更新状态（每100ms调用一次）
 *
 * @param[in] status 状态树指针
 *
 * @details 此函数负责将所有设备的"期望状态"转化为"实际硬件动作"：
 *          1. update_task_clock():
 *             更新任务时钟，基于系统运行时间计算任务状态
 *
 *          2. driver_button() × 2:
 *             读取按钮GPIO状态，更新按钮结构体中的状态标志
 *             检测按钮按下/释放事件
 *
 *          3. driver_LED() × 3:
 *             根据led.on标志控制LED引脚高低电平
 *             on=1时点亮，on=0时熄灭
 *
 *          4. driver_BUZZER():
 *             根据buzzer.on标志控制蜂鸣器引脚
 *             on=1时响起，on=0时静音
 *
 *          5. get_gyr_raw_data():
 *             通过I2C读取陀螺仪原始数据（6轴：3轴加速度+3轴角速度）
 *             数据存入status.sensor.gyr中
 *
 * @note    已注释的驱动函数：
 *          - driver_wheel(): 轮子驱动已移至update_wheel_speed_control()
 *          - driver_gw_analogue(): 巡线传感器驱动已移至TIM中断中（每50ms）
 *          - driver_gyr(): 陀螺仪驱动使用get_gyr_raw_data()替代
 *
 *          这种设计将不同频率的驱动分散到定时器中断的不同时机：
 *          - 每10ms:  轮子速度控制
 *          - 每50ms:  巡线传感器采集
 *          - 每100ms: 设备状态更新（本函数）
 */
void driver_status(STATUS *status) {
    // 更新任务时钟
    update_task_clock(&status->task, status->state.time);

    // 读取按钮状态
    driver_button(&status->device.button1);
    driver_button(&status->device.button2);

    // 更新LED状态
    driver_LED(&status->device.led1);
    driver_LED(&status->device.led2);
    driver_LED(&status->device.led3);

    // 轮子驱动已移至update_wheel_speed_control()
    // driver_wheel(&status->motor.wheel[0]);
    // driver_wheel(&status->motor.wheel[1]);

    // 巡线传感器驱动已移至TIM中断（每50ms）
    //driver_gw_analogue(&status->sensor.gw_analogue);

    // 更新蜂鸣器状态
    driver_BUZZER(&status->device.buzzer1);

    // 读取陀螺仪原始数据（I2C通信）
    get_gyr_raw_data(&hi2c1,&status->sensor.gyr);

    // 陀螺仪驱动（已使用get_gyr_raw_data替代）
    //driver_gyr(&status->sensor.gyr);
}
