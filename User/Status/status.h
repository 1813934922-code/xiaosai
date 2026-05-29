/**
 ******************************************************************************
 * @file    status.h
 * @brief   系统状态树头文件
 *
 * @details 本文件定义了巡线小车的全局状态树架构，是整个系统的核心数据结构。
 *          STATUS结构体采用树状组织方式，包含以下子模块：
 *          - sensor: 传感器数据（巡线传感器、陀螺仪）
 *          - motor:  电机控制数据（左右轮速度、PID参数）
 *          - device: 外设设备数据（LED、按钮、蜂鸣器）
 *          - task:   任务管理数据
 *          - state:  系统运行状态（时间、周期）
 *
 * @note    状态树架构的优势：
 *          - 所有系统数据集中在一个全局变量status中，便于管理
 *          - 各模块通过status访问彼此数据，避免全局变量满天飞
 *          - 初始化时一次性配置所有模块，运行时统一驱动
 *          - 中断回调中只需访问status即可获取/更新所有状态
 ******************************************************************************
 */

#ifndef __STATUS_H
#define __STATUS_H

#include "gw_anagloge.h"
#include "led.h"
#include "main.h"
#include "pid.h"
#include "task.h"
#include "wheel.h"
#include "button.h"
#include "buzzer.h"
#include "task.h"
#include "gy901.h"

/** @brief 运动控制基础速度（用于平衡车模式，当前巡线模式未使用） */
#define MOTION_BASE_SPEED 2000

/**
 * @brief 运动状态枚举（用于平衡车模式）
 *
 * @details 状态流转说明：
 *          STOP → KEEP_ANGLE → FIND_LINE → STOP
 *          - STOP:        停止状态，电机不输出
 *          - KEEP_ANGLE:  保持平衡状态，通过PID维持车身角度
 *          - FIND_LINE:   寻线状态，在保持平衡的同时巡线
 *
 * @note    当前系统使用巡线模式（非平衡车），此枚举未使用
 */
typedef enum MOTION_STATION {
  STOP,        /**< 停止状态：电机不输出 */
  KEEP_ANGLE,  /**< 保持平衡状态：PID维持车身角度 */
  FIND_LINE,   /**< 寻线状态：平衡+巡线 */
} MOTION_STATION;

/**
 * @brief 传感器数据结构体
 * @details 包含所有传感器相关数据：
 *          - gw_analogue: 巡线传感器（8通道模拟/数字传感器）
 *          - gyr:         陀螺仪（GY-901模块，提供yaw/pitch/roll角度）
 *
 * @note    巡线传感器数据由driver_gw_analogue()每50ms更新
 *          陀螺仪数据由driver_gyr()每100ms更新
 */
typedef struct SENSOR {
  GW_ANALOGUE gw_analogue;  /**< 巡线模拟传感器（含8通道数据、偏差值、路口识别） */
  GYR gyr;                  /**< 陀螺仪传感器（提供yaw轴角度用于转向控制） */
} SENSOR;

/**
 * @brief 外设设备数据结构体
 * @details 包含所有外设设备：
 *          - led1/2/3:    3个LED指示灯
 *          - button1/2:   2个按钮（任务触发、测试等）
 *          - buzzer1:     蜂鸣器（提示音）
 */
typedef struct DEVICE {
  LED led1;           /**< LED1：状态指示 */
  LED led2;           /**< LED2：状态指示 */
  LED led3;           /**< LED3：状态指示（闪烁时序控制） */
  BUTTON button1;     /**< 按钮1：任务触发 */
  BUTTON button2;     /**< 按钮2：测试/调试 */
  BUZZER buzzer1;     /**< 蜂鸣器1：提示音（路口检测、倒计时等） */
} DEVICE;

/**
 * @brief 电机控制数据结构体
 * @details 包含左右两个轮子的控制数据：
 *          - wheel[0]:  左轮（编码器、PID参数、目标速度、当前速度、输出占空比）
 *          - wheel[1]:  右轮（同上）
 *
 * @note    轮子数据由update_wheel_speed_control()每10ms更新
 */
typedef struct MOTOR {
  WHEEL wheel[2];  /**< 左右轮结构体数组（索引0=左轮，1=右轮） */
} MOTOR;

/**
 * @brief 系统状态数据结构体
 * @details 包含系统运行的核心状态：
 *          - T:    系统周期（单位ms），定时器中断周期，固定为10ms
 *          - time: 系统运行时间（单位ms），每次中断递增T
 *
 * @note    time以ms为单位，从0开始递增。通过取模运算可实现不同频率的任务调度：
 *          - time % 5 == 0:  每50ms执行（巡线传感器采集）
 *          - time % 10 == 0: 每100ms执行（任务状态机更新）
 */
typedef struct STATE {
  int8_t T;       /**< 系统周期（单位ms）：定时器中断周期，固定为10 */
  uint64_t time;  /**< 系统运行时间（单位ms）：每次中断递增T */
} STATE;

/**
 * @brief 全局状态树根节点
 *
 * @details 这是整个系统的核心数据结构，采用树状组织：
 *          STATUS
 *          ├── task      (任务管理：任务时钟、任务ID等)
 *          ├── state     (系统状态：时间T、运行时间time)
 *          ├── sensor    (传感器：巡线gw_analogue、陀螺仪gyr)
 *          ├── motor     (电机：左右轮wheel[2])
 *          └── device    (外设：LED×3、BUTTON×2、BUZZER×1)
 *
 * @note    全局唯一实例status定义在status.c中
 *          其他模块通过extern声明访问
 */
typedef struct STATUS {
  TASK task;       /**< 任务结构体（任务时钟管理） */
  STATE state;     /**< 系统状态（时间、周期） */
  SENSOR sensor;   /**< 传感器数据（巡线、陀螺仪） */
  MOTOR motor;     /**< 电机数据（左右轮） */
  DEVICE device;   /**< 挂载设备（LED、按钮、蜂鸣器） */
} STATUS;

/** @brief 全局状态树实例（extern声明，定义在status.c中） */
extern STATUS status;

/**
 * @brief  初始化全局状态树
 *
 * @param[in] status 状态树指针
 * @param[in] T      系统周期（单位ms），通常为10
 *
 * @details 初始化顺序：
 *          1. init_state():    设置系统周期T=10ms，time=0
 *          2. init_sensor():   初始化巡线传感器和陀螺仪
 *          3. init_motor():    初始化左右轮（PID参数、方向等）
 *          4. init_device():   初始化LED、按钮、蜂鸣器
 *          5. init_uart_idle_it(): 启动串口DMA空闲接收
 *          6. 初始化陀螺仪转向PID参数
 *          7. 设置陀螺仪6轴模式
 */
void init_status(STATUS *status, uint8_t T);

/**
 * @brief  驱动所有设备更新状态
 *
 * @param[in] status 状态树指针
 *
 * @details 每100ms调用一次，更新所有外设状态：
 *          - 更新任务时钟
 *          - 读取按钮状态（button1/2）
 *          - 更新LED状态（led1/2/3）
 *          - 更新蜂鸣器状态（buzzer1）
 *          - 读取陀螺仪原始数据
 *
 * @note    此函数是状态树的"驱动"入口，负责将所有设备的
 *          期望状态（如led.on=1）转化为实际硬件动作。
 */
void driver_status(STATUS *status);

#endif
