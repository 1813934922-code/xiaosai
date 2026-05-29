/**
 * @file button.c
 * @brief 按键驱动模块实现文件
 * @details 本文件实现了巡线小车的按键驱动功能，包括：
 *          - 按键状态扫描与消抖
 *          - 短按/长按事件检测
 *          - 按键事件回调处理（模式切换、传感器校准、任务启动等）
 * @note 基于STM32 HAL库实现
 * @warning 需要放置在1ms定时中断中调用driver_button()以保证扫描精度
 */

#include "button.h"
#include "main.h"
#include "gw_anagloge.h"
#include "gy901.h"
#include "led.h"
#include "log.h"
#include "math_tool.h"
#include "status.h"
#include "task.h"
#include "buzzer.h"
#include "timer_it.h"

extern PID balance_pid;      /**< 平衡PID控制器，用于小车姿态平衡控制 */
extern float base_angle;     /**< 平衡基准角度，用于小车直立控制 */

/**
 * @brief 按键事件回调处理函数
 * @details 根据按键编号和事件类型执行相应的业务逻辑：
 *          - 按键1短按：循环切换运行模式（stop->first->second->third->fourth）
 *          - 按键1长按：校准光流/模拟量传感器
 *          - 按键2短按：启动陀螺仪测试流程
 *          - 按键2长按：启动小车任务
 * @param button 触发事件的按键结构体指针
 * @param station 按键事件类型（BUTTON_UP/BUTTON_DOWN/BUTTON_LONG）
 * @note 此函数由driver_button()在检测到按键事件时自动调用
 * @warning 请勿在其他地方手动调用此函数
 * 
 * 按键1功能说明：
 * - 短按切换5种模式，通过LED显示当前模式状态
 * - 模式切换时蜂鸣器短响提示
 * - 长按用于校准传感器，确保传感器数据准确性
 * 
 * 按键2功能说明：
 * - 短按启动陀螺仪自检，用于验证陀螺仪工作状态
 * - 长按启动小车正式任务，进入巡线运行状态
 */
void server_button(BUTTON *button, BUTTON_STATION station) {
  if (button->which == 1) {  // 处理按键1的事件
    if (station == BUTTON_DOWN) {  // 按键1短按事件：切换运行模式
      mode = (mode + 1) % 5;  // 模式循环递增：0->1->2->3->4->0，实现5种模式的循环切换
      status.device.buzzer1.on = 1;  // 开启蜂鸣器，提供按键操作声音反馈
      switch (mode) {  // 根据当前模式设置对应LED指示灯状态
        case stop:  // 停止模式：所有LED熄灭
          status.device.led1.on = 0;
          status.device.led2.on = 0;
          status.device.led3.on = 0;
          break;
        case first:  // 模式一：仅LED1亮起
          status.device.led1.on = 1;
          status.device.led2.on = 0;
          status.device.led3.on = 0;
          break;
        case second:  // 模式二：仅LED2亮起
          status.device.led1.on = 0;
          status.device.led2.on = 1;
          status.device.led3.on = 0;
          break;
        case third:  // 模式三：仅LED3亮起
          status.device.led1.on = 0;
          status.device.led2.on = 0;
          status.device.led3.on = 1;
          break;
        case fourth:  // 模式四：所有LED同时亮起
          status.device.led1.on = 1;
          status.device.led2.on = 1;
          status.device.led3.on = 1;
          break;
      }
    } else if (station == BUTTON_UP) {  // 按键1释放事件：关闭蜂鸣器
      status.device.buzzer1.on = 0;  // 关闭蜂鸣器，停止按键提示音
    } else if (station == BUTTON_LONG) {  // 按键1长按事件：传感器校准
      // 无论传感器当前状态如何，都亮起LED3作为校准指示
      if (status.sensor.gw_analogue.sta == 0) {
        status.device.led3.on = 1;
      } else {
        status.device.led3.on = 1;
      }
      correct_gw_analogue(&status.sensor.gw_analogue);  // 执行光流/模拟量传感器校准
      driver_LED(&status.device.led1);  // 刷新LED状态，显示校准结果
    }
  } else if (button->which == 2) {  // 处理按键2的事件
    if (station == BUTTON_DOWN) {  // 按键2短按事件：启动陀螺仪测试
      // 仅在陀螺仪处于空闲或已完成状态时才允许启动测试，防止重复启动
      if (gyro_test_state == GYRO_TEST_IDLE || gyro_test_state == GYRO_TEST_DONE) {
        gyro_test_state = GYRO_TEST_START;  // 设置陀螺仪测试状态为启动
        beep();  // 蜂鸣器短响，提示测试已开始
      }
    } else if (station == BUTTON_UP) {  // 按键2释放事件：关闭蜂鸣器
      status.device.buzzer1.on = 0;  // 关闭蜂鸣器
    } else if (station == BUTTON_LONG) {  // 按键2长按事件：启动任务
      start_task();  // 启动小车巡线任务，进入正式运行状态
    }
  }

  return;
}

/**
 * @brief 按键状态驱动扫描函数（状态机实现）
 * @details 采用状态机方式扫描按键状态，实现以下功能：
 *          1. 读取当前GPIO电平状态
 *          2. 长按检测：通过递减计数器判断是否达到长按阈值
 *          3. 短按检测：通过边沿检测识别按键按下和释放事件
 *          4. 自动调用server_button()回调函数处理事件
 * @param button 要扫描的按键结构体指针
 * @note 需要放置在1ms定时中断中调用，以保证长按判断的准确性
 * 
 * 工作原理：
 * 1. 长按检测：
 *    - 使用表达式 1^(now^Press_is_high_level) 判断当前是否处于按下状态
 *    - 按下状态时long_press_cnt递减，减到0时触发长按事件
 *    - 触发后设置为-1防止重复触发，释放时重置为LONG_PRESS_CNT
 * 
 * 2. 短按检测：
 *    - 通过比较now和last状态检测边沿变化
 *    - 根据Press_is_high_level区分高/低电平有效按键
 *    - 高电平有效：上升沿=按下，下降沿=释放
 *    - 低电平有效：下降沿=按下，上升沿=释放
 * 
 * @warning 必须保证调用周期稳定（建议1ms），否则长按时间判断会不准确
 */
void driver_button(BUTTON *button) {
  // 步骤1：读取当前按键GPIO状态
  if (button->which == 1) {
    button->now = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_11);  // 按键1连接到PB11引脚
  } else if (button->which == 2) {
    button->now = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);   // 按键2连接到PD2引脚
  }

  // 步骤2：长按检测逻辑
  // 表达式解析：1^(now^Press_is_high_level)
  // - 当Press_is_high_level=1, now=1时：1^(1^1)=1，表示按下
  // - 当Press_is_high_level=1, now=0时：1^(0^1)=0，表示释放
  // - 当Press_is_high_level=0, now=0时：1^(0^0)=1，表示按下
  // - 当Press_is_high_level=0, now=1时：1^(1^0)=0，表示释放
  // 结论：表达式结果为1表示按键处于按下状态，0表示释放状态
  if (1 ^ (button->now ^ button->Press_is_high_level)) {  // 按键当前处于按下状态，执行长按检测
    if (button->long_press_cnt > 0) {
      button->long_press_cnt--;  // 按下持续中，递减长按计数器
    } else if (button->long_press_cnt == 0) {
      server_button(button, BUTTON_LONG);  // 计数器减到0，达到长按阈值，触发长按事件
      button->long_press_cnt = -1;  // 设置为-1，防止长按事件重复触发（只在第一次达到阈值时触发）
    }
    // 当long_press_cnt为-1时，不执行任何操作，避免长按重复触发
  } else {  // 按键当前处于释放状态
    button->long_press_cnt = LONG_PRESS_CNT;  // 释放按键，重置长按计数器为初始值
  }

  // 步骤3：短按检测逻辑（边沿检测）
  if (button->now != button->last) {  // 检测到按键状态发生变化（边沿触发）
    if (button->Press_is_high_level == 1) {  // 高电平有效按键（按下为高电平）
      if (button->now == 1) {  // 从0变为1，上升沿，按键按下
        server_button(button, BUTTON_DOWN);  // 触发按键按下事件
        // 处理按下瞬间的长按计数器状态
        if (button->long_press_cnt - 1 >= 0) {
          button->long_press_cnt--;  // 递减计数器，继续长按计时
        } else {
          server_button(button, BUTTON_LONG);  // 如果计数器已异常，立即触发长按
        }
      } else {  // 从1变为0，下降沿，按键释放
        server_button(button, BUTTON_UP);  // 触发按键释放事件
        button->long_press_cnt = LONG_PRESS_CNT;  // 重置长按计数器
      }
    } else {  // 低电平有效按键（按下为低电平）
      if (button->now == 0) {  // 从1变为0，下降沿，按键按下
        server_button(button, BUTTON_DOWN);  // 触发按键按下事件
      } else {  // 从0变为1，上升沿，按键释放
        server_button(button, BUTTON_UP);  // 触发按键释放事件
        button->long_press_cnt = LONG_PRESS_CNT;  // 重置长按计数器
      }
    }
    button->last = button->now;  // 更新上一次状态，为下次边沿检测做准备
  }
}

/**
 * @brief 按键初始化函数
 * @details 初始化按键结构体的各个成员变量，设置按键的初始状态
 * @param button 要初始化的按键结构体指针
 * @param which 按键编号（1或2），用于在回调函数中区分不同按键
 * @param Press_is_high_level 按键按下时的电平极性
 *          - 1：高电平表示按键按下（高电平有效）
 *          - 0：低电平表示按键按下（低电平有效）
 * @note 需要在系统初始化阶段调用（通常放在init_device()函数中）
 * 
 * 初始化逻辑：
 * - 设置初始状态为释放状态：
 *   - 如果高电平有效，初始状态设为0（未按下）
 *   - 如果低电平有效，初始状态设为1（未按下）
 * - 长按计数器设为LONG_PRESS_CNT，等待首次按键按下
 * 
 * 使用示例：
 * @code
 * BUTTON button1, button2;
 * // 初始化按键1，低电平有效（常见于外部下拉电阻设计）
 * init_button(&button1, 1, 0);
 * // 初始化按键2，高电平有效（常见于外部上拉电阻设计）
 * init_button(&button2, 2, 1);
 * @endcode
 */
void init_button(BUTTON *button, uint8_t which, uint8_t Press_is_high_level) {
  button->which = which;  // 设置按键编号，用于区分不同按键
  button->Press_is_high_level = Press_is_high_level;  // 设置按键按下时的电平极性
  
  // 初始化按键状态为释放状态（与按下电平相反）
  // 如果高电平有效，初始状态为0（未按下）
  // 如果低电平有效，初始状态为1（未按下）
  button->last = Press_is_high_level ? 0 : 1;
  button->now = Press_is_high_level ? 0 : 1;
  
  button->long_press_cnt = LONG_PRESS_CNT;  // 初始化长按计数器为阈值，准备检测长按
  return;
}
