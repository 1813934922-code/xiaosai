#ifndef __BUTTON_H_
#define __BUTTON_H_

#include "main.h"

// 长按判断时间阈值（单位：毫秒）
// 当按键持续按下超过此时间时，判定为长按事件
#define LONG_PRESS_CNT 100

/**
 * @brief 按键结构体
 * @note 挂载于 status.device
 * @note 用于驱动和管理按键状态
 * 
 * 使用说明：
 * - 每个按键需要初始化一个BUTTON结构体实例
 * - 通过driver_button()函数在定时中断中轮询按键状态
 * - 当检测到按键事件时，会调用server_button()回调函数
 */
typedef struct BUTTON {
  uint8_t last;                 // 按键上一次扫描的状态（0或1）
  uint8_t now;                  // 按键当前扫描的状态（0或1）
  uint8_t Press_is_high_level;  // 按键按下时的电平极性：1表示高电平为按下，0表示低电平为按下
  uint8_t which;                // 按键编号，用于区分不同的按键（如1, 2, 3...）
  int16_t long_press_cnt;       // 长按计数器，用于判断按键是否达到长按时间
} BUTTON;

/**
 * @brief 按键事件类型枚举
 * 
 * 定义了三种按键事件：
 * - BUTTON_UP: 按键从按下状态释放（抬起）
 * - BUTTON_DOWN: 按键从释放状态变为按下
 * - BUTTON_LONG: 按键持续按下超过长按阈值时间
 */
typedef enum BUTTON_STATION {
  BUTTON_UP,    // 按键抬起事件（短按释放）
  BUTTON_DOWN,  // 按键按下事件（短按按下）
  BUTTON_LONG,  // 按键长按事件（持续按下超过LONG_PRESS_CNT毫秒）
} BUTTON_STATION;

/**
 * @brief 运行模式枚举
 * 
 * 定义了系统的五种运行模式：
 * - stop: 停止模式
 * - first: 任务一模式
 * - second: 任务二模式
 * - third: 任务三模式
 * - fourth: 任务四模式
 */
typedef enum
{
  stop=0,    // 停止模式
  first,     // 任务一
  second,    // 任务二
  third,     // 任务三
  fourth     // 任务四
}MODE;

/**
 * @brief 按键事件回调函数
 * @param button 触发事件的按键指针
 * @param station 按键事件类型（BUTTON_UP/BUTTON_DOWN/BUTTON_LONG）
 * @note 此函数在driver_button()中被自动调用
 * @warning 请勿在其他地方手动调用此函数
 * @note 使用时需要根据button->which判断是哪个按键，根据station判断是什么事件
 * 
 * 使用示例：
 * @code
 * void server_button(BUTTON *button, BUTTON_STATION station) {
 *   if (button->which == 1) {                              // 判断按键编号
 *     if (station == BUTTON_DOWN) {                        // 判断按键按下事件
 *       status.motor.wheel[0].tar_speed = -status.motor.wheel[0].tar_speed;  // 反向轮子速度
 *     }
 *   }
 * }
 * @endcode
 */
void server_button(BUTTON *button, BUTTON_STATION station);

/**
 * @brief 按键驱动函数（状态机扫描）
 * @param button 要扫描的按键指针
 * @note 需要放置在1ms定时中断中调用
 * @note 此函数会自动检测按键的短按、长按和释放事件，并调用server_button()回调
 * 
 * 工作原理：
 * 1. 读取当前按键电平状态
 * 2. 与上一次状态比较，检测状态变化
 * 3. 根据长按计数器判断是否为长按
 * 4. 触发相应事件并调用回调函数
 */
void driver_button(BUTTON *button);

/**
 * @brief 按键初始化函数
 * @param button 要初始化的按键指针
 * @param which 按键编号（1, 2, 3...），用于在回调函数中区分不同按键
 * @param Press_is_high_level 按键按下时的电平：1=高电平表示按下，0=低电平表示按下
 * @note 需要在系统初始化时调用（放在init_device()中）
 * 
 * 使用示例：
 * @code
 * // 初始化按键1，低电平有效（按下为低电平）
 * init_button(&status.device.button1, 1, 0);
 * 
 * // 初始化按键2，高电平有效（按下为高电平）
 * init_button(&status.device.button2, 2, 1);
 * @endcode
 */
void init_button(BUTTON *button, uint8_t which, uint8_t Press_is_high_level);

extern MODE mode;  // 当前运行模式，可在server_button中修改

#endif
