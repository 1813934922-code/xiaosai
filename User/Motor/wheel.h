/**
 ******************************************************************************
 * @file    wheel.h
 * @brief   直流电机/轮子控制模块头文件
 * @details 本模块用于控制巡线小车的直流电机，支持：
 *          - PWM调速控制（通过定时器输出PWM信号）
 *          - 电机方向控制（通过GPIO引脚设置正反转）
 *          - 编码器速度反馈（通过定时器编码器模式读取脉冲）
 *          - PID闭环速度控制
 *
 *          硬件配置：
 *          - 电机数量：默认支持2个电机（可扩展至4个）
 *          - 编码器：500线增量式编码器，4倍频后为2000 PPR
 *          - 减速比：20:1
 *          - PWM定时器：TIM3（通道1-电机1，通道2-电机2）
 *          - 编码器定时器：TIM1-电机1，TIM2-电机2
 ******************************************************************************
 */

#ifndef __WHEEL_H
#define __WHEEL_H

#include "main.h"
#include "pid.h"
#include "tim.h"

/** @defgroup Wheel_Macros 宏定义
 * @{
 */

/** @brief  编码器线数（每转脉冲数）
 *  @note   增量式编码器的物理线数，一圈产生500个A相脉冲
 */
#define ENCODER_LINES 500

/** @brief  减速箱减速比
 *  @note   电机轴转20圈，输出轴转1圈
 */
#define GEAR_RATIO 20

/** @brief  编码器每转脉冲数（PPR - Pulses Per Revolution）
 *  @note   采用4倍频技术（A相上升沿、下降沿、B相上升沿、下降沿），
 *         实际分辨率为线数的4倍：500 × 4 = 2000 PPR
 */
#define ENCODER_PPR (ENCODER_LINES * 4)

/** @brief  电机总减速后的编码器脉冲数
 *  @note   输出轴每转一圈，编码器计数 = PPR × 减速比 = 2000 × 20 = 40000
 *         用于将编码器脉冲转换为实际转速
 */
#define MOTOR_REDUCTION (ENCODER_PPR * GEAR_RATIO)

/** @brief  PID控制频率（Hz）
 *  @note   PID控制回路每秒执行1000次，即每1ms执行一次
 *         用于速度闭环控制
 */
#define PID_HZ 1000.0f

/** @brief  PWM占空比限制值
 *  @note   推力(trust)的有效范围：[-1000, +1000]
 *         对应PWM占空比的百分比放大值
 */
#define TRUST_CONFINE 1000

/** @brief  基础速度设定值
 *  @note   小车的默认运行速度，单位：RPM（转/分钟）
 */
#define BASE_SPEED 90

/**
 * @}
 */

/** @defgroup Wheel_Structure 结构体定义
 * @{
 */

/**
 * @brief  轮子（直流电机）控制结构体
 * @note   本结构体挂载于 status->motor 状态树中，默认最多挂载4个轮子
 *         每个WHEEL实例代表一个直流电机的完整控制状态
 * @par    使用示例：
 * @code
 *         // 更新轮子当前速度（在status_update()中调用）：
 *         status->motor.wheel[0].cur_speed = get_wheel_speed(&status->motor.wheel[0]);
 *
 *         // 设置轮子目标速度：
 *         status->motor.wheel[0].tar_speed = 500;  // 目标速度500 RPM
 * @endcode
 */
typedef struct WHEEL {
  uint8_t which;       /**< 轮子编号：1~4，用于识别对应电机硬件（PWM通道、编码器等） */
  int16_t trust;       /**< PWM推力值（占空比控制值），范围：[-TRUST_CONFINE, +TRUST_CONFINE]，
                            正负号表示方向，绝对值表示PWM占空比大小 */
  int16_t cur_speed;   /**< 当前实际速度，单位：RPM（转/分钟），由编码器反馈计算得出 */
  int16_t tar_speed;   /**< 目标速度，单位：RPM（转/分钟），由上层运动控制算法设定 */
  int8_t dir;          /**< 方向修正系数：1 或 -1
                            用于校准电机安装方向，1表示正转与定义方向一致，
                            -1表示需要反向补偿 */
  PID wheel_pid;       /**< 速度闭环PID控制器实例，用于根据目标速度和实际速度计算PWM输出 */
} WHEEL;

/**
 * @}
 */

/** @defgroup Wheel_Functions 函数声明
 * @{
 */

/**
 * @brief  获取轮子当前实际转速
 * @param  wheel: 轮子结构体指针
 * @retval 当前转速，单位：RPM（转/分钟）
 * @note   计算公式：
 *         RPM = (encoder_ticks × 60 × PID_HZ) / MOTOR_REDUCTION
 *         其中 encoder_ticks 为两次采样间的编码器脉冲差值
 * @par    使用方法：
 *         - 放置在 status_update() 函数中定期调用
 *         - 只能定期调用，因为依赖定时器的采样周期
 *         - 返回编码器定时器的当前CNT值与上次CNT值的差值
 * @par    硬件细节：
 *         - 编码器定时器CNT基准值设为30000（防止溢出回零问题）
 *         - 正转时CNT递增，反转时CNT递减
 * @code
 *         // 在status_update()中调用：
 *         status->motor.wheel[0].cur_speed = get_wheel_speed(&status->motor.wheel[0]);
 * @endcode
 */
int16_t get_wheel_speed(WHEEL *wheel);

/**
 * @brief  驱动轮子（设置PWM占空比和方向）
 * @param  wheel: 轮子结构体指针
 * @retval 无
 * @note   本函数执行以下操作：
 *         1. 限制trust值在[-TRUST_CONFINE, +TRUST_CONFINE]范围内
 *         2. 根据trust值正负设置电机方向引脚（GPIO）
 *         3. 设置PWM占空比（绝对值）控制电机转速
 * @par    使用方法：
 *         - 放置在 status_driver() 函数中，每个控制周期调用
 *         - 调用前需先计算好 wheel->trust 值（通常由PID控制器输出）
 * @code
 *         // 在status_driver()中调用：
 *         driver_wheel(&status->motor.wheel[0]);
 * @endcode
 */
void driver_wheel(WHEEL *wheel);

/**
 * @brief  初始化轮子控制结构体和硬件
 * @param  wheel: 轮子结构体指针
 * @param  which: 轮子编号，默认1~4，用于识别电机对应的硬件资源
 *                @arg 1: 使用TIM3通道1(PWM)、TIM1(编码器)、M1方向引脚
 *                @arg 2: 使用TIM3通道2(PWM)、TIM2(编码器)、M2方向引脚
 * @param  dir:   方向修正系数，1 或 -1
 *               @arg  1: 电机正转方向与定义方向一致
 *               @arg -1: 电机正转方向与定义方向相反，需要反向补偿
 * @retval 无
 * @note   本函数执行以下初始化操作：
 *         1. 初始化WHEEL结构体各成员变量
 *         2. 初始化PID控制器参数
 *         3. 启动PWM输出（TIM3对应通道）
 *         4. 启动编码器模式（TIM1/TIM2）
 *         5. 设置编码器CNT基准值为30000
 * @par    使用方法：
 *         - 放置在 init_motor() 函数中调用
 * @code
 *         // 在init_motor()中调用：
 *         init_wheel(&status->motor.wheel[0], 1, 1);   // 初始化1号轮，方向正向
 *         init_wheel(&status->motor.wheel[1], 2, -1);  // 初始化2号轮，方向反向
 * @endcode
 */
void init_wheel(WHEEL *wheel, uint8_t which, int8_t dir);

/**
 * @}
 */

#endif
