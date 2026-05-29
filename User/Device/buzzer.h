/**
 * @file buzzer.h
 * @brief 蜂鸣器驱动模块头文件
 * @details 本文件定义了巡线小车的蜂鸣器驱动接口，包括：
 *          - 蜂鸣器结构体定义（BUZZER）
 *          - 蜂鸣器驱动函数声明（driver_BUZZER）
 *          - 蜂鸣器初始化函数声明（init_BUZZER）
 * @note 基于STM32 HAL库实现，通过GPIO控制蜂鸣器发声
 * 
 * 使用说明：
 * 1. 通过init_BUZZER()初始化蜂鸣器，设置编号和有效电平
 * 2. 通过设置buzzer.on = 1或0来控制蜂鸣器响/停
 * 3. 在status_driver()中调用driver_BUZZER()刷新蜂鸣器状态
 * 
 * 典型应用场景：
 * - 按键操作提示音（短按/长按）
 * - 系统状态报警提示
 * - 任务完成提示音
 * - 错误警告音
 */

#ifndef __BUZZER_H
#define __BUZZER_H

#include "main.h"

/**
 * @brief 蜂鸣器控制结构体
 * @details 封装了蜂鸣器的控制参数和状态信息
 * @note 此结构体实例通常挂载于status.device中统一管理
 * 
 * 结构体成员说明：
 * - which: 蜂鸣器编号，用于区分多个蜂鸣器（当前仅支持编号1）
 * - High_level_is_on: 电平有效性配置，决定高电平还是低电平驱动蜂鸣器发声
 * - on: 蜂鸣器状态控制标志，1表示开启（响），0表示关闭（停）
 * 
 * 使用示例：
 * @code
 * BUZZER myBuzzer;
 * init_BUZZER(&myBuzzer, 1, 1);  // 初始化蜂鸣器1，高电平有效
 * myBuzzer.on = 1;               // 开启蜂鸣器
 * driver_BUZZER(&myBuzzer);      // 驱动蜂鸣器发声
 * myBuzzer.on = 0;               // 关闭蜂鸣器
 * driver_BUZZER(&myBuzzer);      // 驱动蜂鸣器停止
 * @endcode
 */
typedef struct BUZZER {
  uint8_t which;             /**< 蜂鸣器编号（1表示第一个蜂鸣器），用于在driver_BUZZER()中区分不同蜂鸣器 */
  uint8_t High_level_is_on;  /**< 电平有效极性配置：1表示高电平驱动蜂鸣器发声，0表示低电平驱动蜂鸣器发声 */
  uint8_t on;                /**< 蜂鸣器开关状态标志：1=开启（蜂鸣器响），0=关闭（蜂鸣器停）。通过修改此值控制蜂鸣器状态 */
} BUZZER;

/**
 * @brief 蜂鸣器驱动函数声明
 * @details 根据BUZZER结构体中的on状态，控制对应GPIO引脚输出高低电平，驱动蜂鸣器发声或停止
 * @param buzzer 蜂鸣器结构体指针，指向要驱动的蜂鸣器实例
 * @note 此函数应该放在status_driver()函数中调用，以实时响应状态变化
 * 
 * 使用示例：
 * @code
 * status.device.buzzer1.on = 1;   // 设置蜂鸣器状态为开启
 * driver_BUZZER(&status.device.buzzer1);  // 驱动蜂鸣器发声
 * @endcode
 */
void driver_BUZZER(BUZZER *buzzer);

/**
 * @brief 蜂鸣器初始化函数声明
 * @details 初始化蜂鸣器结构体的各个成员变量，配置蜂鸣器的工作参数
 * @param buzzer 蜂鸣器结构体指针，指向要初始化的蜂鸣器实例
 * @param which 蜂鸣器编号（当前仅支持1），用于在driver_BUZZER()中区分不同蜂鸣器
 * @param High_level_is_on 电平有效极性配置
 *        - 1：高电平驱动蜂鸣器发声（适用于NPN三极管驱动电路）
 *        - 0：低电平驱动蜂鸣器发声（适用于PNP三极管驱动电路）
 * @note 需要在系统初始化阶段调用（通常放在init_device()函数中）
 * @note 初始化后蜂鸣器默认处于关闭状态（on=0）
 * 
 * 使用示例：
 * @code
 * // 初始化蜂鸣器1，高电平有效（常见于NPN三极管驱动）
 * init_BUZZER(&status.device.buzzer1, 1, 1);
 * 
 * // 初始化蜂鸣器1，低电平有效（常见于PNP三极管驱动）
 * init_BUZZER(&status.device.buzzer1, 1, 0);
 * @endcode
 */
void init_BUZZER(BUZZER *buzzer, uint8_t which, uint8_t High_level_is_on);

#endif  // __BUZZER_H
