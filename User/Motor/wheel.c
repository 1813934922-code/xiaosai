/**
 ******************************************************************************
 * @file    wheel.c
 * @brief   直流电机/轮子 PWM 驱动实现文件
 * @details 本文件实现了直流电机的完整控制逻辑，包括：
 *          - 编码器速度读取：通过STM32定时器编码器模式获取电机转速
 *          - 方向控制：通过GPIO引脚设置H桥驱动芯片的方向
 *          - PWM输出控制：通过STM32定时器PWM通道调节电机功率
 *          - 电机初始化：配置PWM、编码器、PID控制器等
 *
 *          硬件映射关系：
 *          +---------+------------+------------+------------------+
 *          | 电机编号 | PWM定时器  | 编码器定时器| 方向控制GPIO     |
 *          +---------+------------+------------+------------------+
 *          |   1号   | TIM3_CH1   |   TIM1     | M1_D1, M1_D2     |
 *          |   2号   | TIM3_CH2   |   TIM2     | M2_D1, M2_D2     |
 *          +---------+------------+------------+------------------+
 *
 *          速度计算公式：
 *          RPM = (encoder_ticks × 60 × PID_HZ) / MOTOR_REDUCTION
 *          其中：
 *          - encoder_ticks: 采样周期内编码器脉冲变化量
 *          - 60: 将秒转换为分钟
 *          - PID_HZ: PID控制频率(1000Hz)
 *          - MOTOR_REDUCTION: 编码器每转总脉冲数(40000)
 ******************************************************************************
 */

#include "wheel.h"

#include "log.h"
#include "math_tool.h"

/**
 * @brief  获取轮子当前实际转速
 * @param  wheel: 轮子结构体指针
 * @retval 当前转速，单位：RPM（转/分钟）
 * @note   速度计算原理：
 *         1. 读取编码器定时器的CNT值变化量
 *         2. 基准值设为30000，防止16位计数器溢出回零造成计算错误
 *            （16位定时器最大值为65535，30000有足够的正反向裕量）
 *         3. 应用方向修正系数wheel->dir进行校准
 *         4. 转换为RPM单位
 *
 *         溢出保护机制：
 *         - 每次采样后将CNT重置为30000
 *         - 正向转动：CNT从30000递增
 *         - 反向转动：CNT从30000递减
 *         - 这样即使反转到0或正转到65535，都有足够的缓冲区
 *
 *         计算公式推导：
 *         RPM = (ticks × 采样频率 × 60) / 每转总脉冲数
 *             = (ticks × PID_HZ × 60) / MOTOR_REDUCTION
 * @par    示例：
 *         如果采样间隔内编码器计数变化100个脉冲：
 *         RPM = (100 × 1000 × 60) / 40000 = 150 RPM
 */
int16_t get_wheel_speed(WHEEL *wheel) {
  int16_t encoder_ticks = 0;  // 编码器脉冲变化量

  // 根据电机编号读取对应的编码器定时器CNT值
  if (wheel->which == 1) {
    encoder_ticks = TIM1->CNT - 30000;  // 计算与基准值的差值
    TIM1->CNT = 30000;                   // 重置CNT为基准值，防止溢出
  } else if (wheel->which == 2) {
    encoder_ticks = TIM2->CNT - 30000;
    TIM2->CNT = 30000;
  }

  // 应用方向修正系数：校正电机安装方向导致的正反转差异
  encoder_ticks = encoder_ticks * wheel->dir;

  // 将编码器脉冲转换为RPM（转/分钟）
  // 公式：RPM = (脉冲数 × 60秒 × PID频率) / 总减速脉冲数
  return (int16_t)((float)encoder_ticks * 60.0f * PID_HZ / MOTOR_REDUCTION);
}

/**
 * @brief  设置轮子转动方向
 * @param  wheel: 轮子结构体指针
 * @param  trust: PWM推力值，正负号决定方向
 * @retval 无
 * @note   方向控制原理：
 *         直流电机通过H桥驱动芯片控制方向，需要设置两个方向引脚：
 *         - D1=1, D2=0: 正转
 *         - D1=0, D2=1: 反转
 *
 *         不同电机的方向逻辑：
 *         - 1号电机：trust*dir > 0 时正转
 *         - 2号电机：trust*dir < 0 时正转（由于安装位置镜像，逻辑相反）
 *
 *         wheel->dir的作用：
 *         用于校正电机实际安装方向与定义方向是否一致
 *         - dir=1: 不需要校正
 *         - dir=-1: 需要反转
 */
void set_wheel_dir(WHEEL *wheel, int16_t trust) {
  if (wheel->which == 1) {
    // 1号电机方向控制
    // 当推力与方向系数乘积为正时，设置为正转
    if (trust * wheel->dir > 0) {
      HAL_GPIO_WritePin(M1_D1_GPIO_Port, M1_D1_Pin, 1);  // D1置高
      HAL_GPIO_WritePin(M1_D2_GPIO_Port, M1_D2_Pin, 0);  // D2置低
    } else {
      HAL_GPIO_WritePin(M1_D1_GPIO_Port, M1_D1_Pin, 0);  // D1置低
      HAL_GPIO_WritePin(M1_D2_GPIO_Port, M1_D2_Pin, 1);  // D2置高
    }
  } else if (wheel->which == 2) {
    // 2号电机方向控制（由于安装镜像，方向逻辑与1号相反）
    if (trust * wheel->dir < 0) {
      HAL_GPIO_WritePin(M2_D1_GPIO_Port, M2_D1_Pin, 1);
      HAL_GPIO_WritePin(M2_D2_GPIO_Port, M2_D2_Pin, 0);
    } else {
      HAL_GPIO_WritePin(M2_D1_GPIO_Port, M2_D1_Pin, 0);
      HAL_GPIO_WritePin(M2_D2_GPIO_Port, M2_D2_Pin, 1);
    }
  }
}

/**
 * @brief  驱动轮子（设置PWM占空比和方向）
 * @param  wheel: 轮子结构体指针
 * @retval 无
 * @note   本函数是电机控制的核心执行函数，每个控制周期调用一次
 *         执行流程：
 *         1. 安全限制：将trust值限制在[-TRUST_CONFINE, +TRUST_CONFINE]范围内
 *            防止PWM值溢出导致硬件异常
 *         2. 方向设置：根据trust的正负调用set_wheel_dir()设置电机转向
 *         3. PWM输出：取trust的绝对值作为PWM比较值，控制电机转速
 *
 *         PWM工作原理：
 *         - TIM3定时器产生固定频率的PWM波形
 *         - 通过设置CCR（Capture Compare Register）寄存器改变占空比
 *         - CCR值越大，占空比越高，电机转速越快
 *         - __HAL_TIM_SET_COMPARE是HAL库提供的快捷宏，用于设置CCR值
 *
 *         调用位置：status_driver()函数中
 * @par    示例：
 * @code
 *         // 在PID控制循环中：
 *         wheel->trust = PID_Calculate(&wheel->wheel_pid, wheel->tar_speed, wheel->cur_speed);
 *         driver_wheel(wheel);  // 应用PID计算结果
 * @endcode
 */
void driver_wheel(WHEEL *wheel) {
  // 获取PID控制器输出的推力值
  int16_t trust = wheel->trust;

  // 安全限制：确保推力值在有效范围内，防止PWM溢出
  // CONFINE宏将值限制在[-1000, +1000]之间
  trust = CONFINE(trust, -TRUST_CONFINE, TRUST_CONFINE);

  // 根据电机编号设置对应的PWM通道和方向引脚
  if (wheel->which == 1) {
    set_wheel_dir(wheel, trust);  // 设置电机方向
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, ABS(trust));  // 设置TIM3通道1的PWM占空比
  } else if (wheel->which == 2) {
    set_wheel_dir(wheel, trust);  // 设置电机方向
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, ABS(trust));  // 设置TIM3通道2的PWM占空比
  }
}

/**
 * @brief  初始化轮子控制结构体和硬件资源
 * @param  wheel: 轮子结构体指针
 * @param  which: 轮子编号（1~4），用于选择对应的硬件资源
 * @param  dir:   方向修正系数（1或-1）
 * @retval 无
 * @note   初始化流程：
 *         1. 结构体初始化：
 *            - which: 保存电机编号
 *            - trust: 初始推力设为0（电机静止）
 *            - cur_speed: 初始速度设为0
 *            - tar_speed: 目标速度设为0
 *            - dir: 保存方向修正系数
 *
 *         2. PID控制器初始化：
 *            - init_pid(1, 1, 1, 1, 5000)
 *            - 参数依次为：Kp, Ki, Kd, 输出下限, 输出上限
 *            - 初始PID参数均为1，后续可通过调试优化
 *            - 输出限制在[1, 5000]范围内
 *
 *         3. PWM初始化：
 *            - 设置初始占空比为0（电机不转）
 *            - 启动TIM3对应通道的PWM输出
 *
 *         4. 编码器初始化：
 *            - 启动TIM1/TIM2的编码器模式
 *            - 编码器模式会自动计数A/B相脉冲的上升沿和下降沿
 *            - 设置CNT初始值为30000（溢出保护基准）
 *
 *         调用位置：init_motor()函数中
 * @par    硬件资源配置表：
 *         +---------+------------+------------+------------------+
 *         | which=1 | TIM3_CH1   | TIM1       | M1_D1, M1_D2     |
 *         | which=2 | TIM3_CH2   | TIM2       | M2_D1, M2_D2     |
 *         +---------+------------+------------+------------------+
 * @par    示例：
 * @code
 *         // 在系统初始化时：
 *         init_wheel(&status.motor.wheel[0], 1, 1);   // 1号轮，正向
 *         init_wheel(&status.motor.wheel[1], 2, -1);  // 2号轮，反向校正
 * @endcode
 */
void init_wheel(WHEEL *wheel, uint8_t which, int8_t dir) {
  // 初始化结构体成员变量
  wheel->which = which;        // 保存电机编号
  wheel->trust = 0;            // 初始推力为0
  wheel->cur_speed = 0;        // 初始速度为0
  wheel->tar_speed = 0;        // 初始目标速度为0
  wheel->dir = dir;            // 保存方向修正系数

  // 初始化PID控制器
  // 参数：Kp=1, Ki=1, Kd=1, 输出最小值=1, 输出最大值=5000
  wheel->wheel_pid = init_pid(1, 1, 1, 1, 5000);

  // 根据电机编号初始化对应的PWM通道
  if (wheel->which == 1) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);  // 设置TIM3通道1初始占空比为0
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);         // 启动TIM3通道1的PWM输出
  } else if (wheel->which == 2) {
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);  // 设置TIM3通道2初始占空比为0
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);         // 启动TIM3通道2的PWM输出
  }

  // 根据电机编号初始化对应的编码器
  if (wheel->which == 1) {
    HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);  // 启动TIM1编码器模式（双通道）
    TIM1->CNT = 30000;                                // 设置编码器计数器基准值
  } else if (wheel->which == 2) {
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);  // 启动TIM2编码器模式（双通道）
    TIM2->CNT = 30000;                                // 设置编码器计数器基准值
  }
}
