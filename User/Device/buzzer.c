/**
 * @file buzzer.c
 * @brief 蜂鸣器驱动模块实现文件
 * @details 本文件实现了巡线小车的蜂鸣器驱动功能，包括：
 *          - 蜂鸣器GPIO控制输出
 *          - 蜂鸣器参数初始化
 * @note 基于STM32 HAL库实现，通过GPIO引脚电平控制蜂鸣器发声
 */

#include "buzzer.h"
#include "main.h"
#include "gpio.h"
#include "stdbool.h"

/**
 * @brief 蜂鸣器驱动函数
 * @details 根据蜂鸣器结构体中的on状态和High_level_is_on配置，计算正确的GPIO输出电平，驱动蜂鸣器发声或停止
 * @param buzzer 蜂鸣器结构体指针，指向要驱动的蜂鸣器实例
 * 
 * 电平计算原理：
 * 使用异或运算实现电平转换：1^(on^High_level_is_on)
 * - 当High_level_is_on=1, on=1时：1^(1^1)=1，输出高电平，蜂鸣器响
 * - 当High_level_is_on=1, on=0时：1^(0^1)=0，输出低电平，蜂鸣器停
 * - 当High_level_is_on=0, on=1时：1^(1^0)=0，输出低电平，蜂鸣器响
 * - 当High_level_is_on=0, on=0时：1^(0^0)=1，输出高电平，蜂鸣器停
 * 
 * 此计算方法可以自动适配高电平有效和低电平有效两种不同的硬件电路设计
 * 
 * @note 当前仅支持蜂鸣器编号1，对应GPIO引脚PC1
 * @note 此函数应该在status_driver()中被周期性调用，以实时响应状态变化
 */
void driver_BUZZER(BUZZER *buzzer) {
  if (buzzer->which == 1) {  // 蜂鸣器1，连接到PC1引脚
    // 通过异或运算自动适配高/低电平有效的硬件设计
    // 将逻辑状态(on)转换为物理电平输出
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1 ^ ((bool)(buzzer->on) ^ (bool)(buzzer->High_level_is_on)));
  }

  return;
}

/**
 * @brief 蜂鸣器初始化函数
 * @details 初始化蜂鸣器结构体的各个成员变量，配置蜂鸣器的工作参数
 * @param buzzer 蜂鸣器结构体指针，指向要初始化的蜂鸣器实例
 * @param which 蜂鸣器编号（当前仅支持1），用于在driver_BUZZER()中区分不同蜂鸣器
 * @param High_level_is_on 电平有效极性配置
 *        - 1：高电平驱动蜂鸣器发声（适用于NPN三极管驱动电路）
 *        - 0：低电平驱动蜂鸣器发声（适用于PNP三极管驱动电路）
 * @note 需要在系统初始化阶段调用（通常放在init_device()函数中）
 * @note 初始化后蜂鸣器默认处于关闭状态（on=0），确保系统启动时不会意外发声
 * 
 * 初始化步骤：
 * 1. 设置电平有效极性（High_level_is_on）
 * 2. 设置初始状态为关闭（on=0）
 * 3. 设置蜂鸣器编号（which）
 * 
 * 使用示例：
 * @code
 * BUZZER buzzer1;
 * // 初始化蜂鸣器1，高电平有效（常见于NPN三极管驱动电路）
 * init_BUZZER(&buzzer1, 1, 1);
 * @endcode
 */
void init_BUZZER(BUZZER *buzzer, uint8_t which, uint8_t High_level_is_on) {
  buzzer->High_level_is_on = High_level_is_on;  // 设置电平有效极性
  buzzer->on = 0;  // 初始化状态为关闭，确保系统启动时蜂鸣器不发声
  buzzer->which = which;  // 设置蜂鸣器编号

  return;
}
