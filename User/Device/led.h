/**
 * @file led.h
 * @brief LED驱动模块头文件
 * @details 本文件定义了巡线小车的LED驱动接口，包括：
 *          - LED结构体定义（LED）
 *          - LED驱动函数声明（driver_LED）
 *          - LED初始化函数声明（init_LED）
 * @note 基于STM32 HAL库实现，通过GPIO控制LED亮灭
 * 
 * 使用说明：
 * 1. 通过init_LED()初始化LED，设置编号和有效电平
 * 2. 通过设置led.on = 1或0来控制LED亮/灭
 * 3. 在status_driver()中调用driver_LED()刷新LED状态
 * 
 * 典型应用场景：
 * - 运行模式指示（通过不同LED组合显示当前模式）
 * - 系统状态指示（正常工作、错误、校准等）
 * - 传感器校准状态指示
 * - 任务执行状态指示
 */

#ifndef __LED_H
#define __LED_H

#include "main.h"

/**
 * @brief LED控制结构体
 * @details 封装了LED的控制参数和状态信息
 * @note 此结构体实例通常挂载于status.device中统一管理，最多支持3个LED
 * 
 * 结构体成员说明：
 * - which: LED编号（1、2或3），用于区分多个LED灯
 * - High_level_is_on: 电平有效性配置，决定高电平还是低电平点亮LED
 * - on: LED状态控制标志，1表示点亮，0表示熄灭
 * 
 * 使用示例：
 * @code
 * LED myLed;
 * init_LED(&myLed, 1, 1);  // 初始化LED1，高电平点亮
 * myLed.on = 1;            // 点亮LED
 * driver_LED(&myLed);      // 驱动LED亮起
 * myLed.on = 0;            // 熄灭LED
 * driver_LED(&myLed);      // 驱动LED熄灭
 * @endcode
 */
typedef struct LED {
  uint8_t which;             /**< LED编号（1、2或3），用于在driver_LED()中区分不同LED */
  uint8_t High_level_is_on;  /**< 电平有效极性配置：1表示高电平点亮LED，0表示低电平点亮LED */
  uint8_t on;                /**< LED开关状态标志：1=点亮，0=熄灭。通过修改此值控制LED状态 */
} LED;

/**
 * @brief LED驱动函数声明
 * @details 根据LED结构体中的on状态，控制对应GPIO引脚输出高低电平，驱动LED点亮或熄灭
 * @param led LED结构体指针，指向要驱动的LED实例
 * @note 此函数应该放在status_driver()函数中调用，以实时响应状态变化
 * 
 * GPIO引脚映射：
 * - LED1: GPIOC, GPIO_PIN_0 (PC0)
 * - LED2: GPIOC, GPIO_PIN_2 (PC2)
 * - LED3: GPIOC, GPIO_PIN_13 (PC13)
 * 
 * 使用示例：
 * @code
 * status.device.led1.on = 1;   // 设置LED1状态为点亮
 * driver_LED(&status.device.led1);  // 驱动LED1亮起
 * @endcode
 */
void driver_LED(LED *led);

/**
 * @brief LED初始化函数声明
 * @details 初始化LED结构体的各个成员变量，配置LED的工作参数
 * @param led LED结构体指针，指向要初始化的LED实例
 * @param which LED编号（1、2或3），用于在driver_LED()中区分不同LED
 * @param High_level_is_on 电平有效极性配置
 *        - 1：高电平点亮LED（适用于LED阳极接GPIO，阴极接GND的电路）
 *        - 0：低电平点亮LED（适用于LED阳极接VCC，阴极接GPIO的电路）
 * @note 需要在系统初始化阶段调用（通常放在init_device()函数中）
 * @note 初始化后LED默认处于熄灭状态（on=0）
 * 
 * 使用示例：
 * @code
 * // 初始化LED1，高电平有效（LED阳极接GPIO）
 * init_LED(&status.device.led1, 1, 1);
 * 
 * // 初始化LED2，低电平有效（LED阴极接GPIO）
 * init_LED(&status.device.led2, 2, 0);
 * 
 * // 初始化LED3，高电平有效
 * init_LED(&status.device.led3, 3, 1);
 * @endcode
 */
void init_LED(LED *led, uint8_t which, uint8_t High_level_is_on);

#endif  // __LED_H
