/**
 ******************************************************************************
 * @file    lq_step.h
 * @brief   雷赛步进电机 UART 控制模块头文件
 * @details 本模块通过 UART 串口通信控制雷赛(Leadshine)步进电机驱动器，
 *          支持绝对角度控制、相对角度控制、速度控制和电流控制四种模式。
 *          通信协议帧格式：
 *          | 帧头(0x7B) | 地址(0x01) | 命令字 | 方向 | 模式(0x20) | 数据高字节 |
 *          数据低字节 | 速度高字节 | 速度低字节 | BCC校验 | 帧尾(0x7D) |
 ******************************************************************************
 */

#ifndef __LQ_STEP_H
#define __LQ_STEP_H

#include "main.h"
#include "math.h"
#include "usart.h"

/**
 * @brief  控制步进电机转到绝对角度
 * @param  huart: UART 句柄指针，用于串口通信
 * @param  angle: 目标绝对角度，单位：度 (°)，范围：0~360°
 * @param  speed: 转动速度，单位：rad/s（内部会乘以10转换为协议格式）
 * @retval 无
 * @note   电机将以指定速度转动到指定的绝对角度位置
 */
void trun_lq_step_abslute_angle(UART_HandleTypeDef *huart, float angle, float speed);

/**
 * @brief  控制步进电机转动相对角度
 * @param  huart: UART 句柄指针，用于串口通信
 * @param  angle: 相对转动角度，单位：度 (°)
 * @param  dir:   转动方向
 *               @arg 0x00: 逆时针 (CCW)
 *               @arg 0x01: 顺时针 (CW)
 * @param  speed: 转动速度，单位：rad/s（内部会乘以10转换为协议格式）
 * @retval 无
 * @note   电机将以指定方向转动指定的相对角度
 */
void trun_lq_step_angle(UART_HandleTypeDef *huart, float angle, uint8_t dir, float speed);

/**
 * @brief  控制步进电机以指定速度连续转动
 * @param  huart: UART 句柄指针，用于串口通信
 * @param  speed: 转动速度，单位：rad/s（内部会乘以10转换为协议格式）
 * @param  dir:   转动方向
 *               @arg 0x00: 逆时针 (CCW)
 *               @arg 0x01: 顺时针 (CW)
 * @retval 无
 * @note   电机将持续以指定速度转动，直到收到停止指令或新的控制指令
 */
void trun_lq_step_speed(UART_HandleTypeDef *huart, float speed, uint8_t dir);

/**
 * @brief  控制步进电机以指定电流输出
 * @param  huart:   UART 句柄指针，用于串口通信
 * @param  current: 输出电流值，单位：毫安 (mA)
 * @param  dir:     电流方向
 *                 @arg 0x00: 逆时针方向 (CCW)
 *                 @arg 0x01: 顺时针方向 (CW)
 * @retval 无
 * @note   电流模式用于需要精确控制输出力矩的场景
 */
void trun_lq_step_current(UART_HandleTypeDef *huart, uint16_t current, uint8_t dir);

#endif
