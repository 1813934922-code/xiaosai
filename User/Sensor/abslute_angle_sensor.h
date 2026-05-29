/**
 ******************************************************************************
 * @file    abslute_angle_sensor.h
 * @brief   布瑞特绝对值编码器UART驱动头文件
 * @details 本文件定义了布瑞特(BRIGHT)绝对值编码器的UART通信驱动接口。
 *          该编码器通过UART串口使用Modbus RTU协议进行通信，提供12位分辨率
 *          (4096个位置)的绝对角度测量，测量范围0-360度。
 *          支持主动模式(编码器主动发送)和被动模式(主机查询)两种工作方式。
 * @note    通信参数：UART2，Modbus RTU协议，CRC16校验
 *          编码器地址：0x01，精度：12位(4096计数)
 ******************************************************************************
 */

// @551
// 布瑞特绝对值编码器的驱动程序

#ifndef ABSLUTE_ANGLE_SENSOR_H
#define ABSLUTE_ANGLE_SENSOR_H

/**
 * @addtogroup Sensor_Drivers
 * @{
 */

/**
 * @addtogroup Absolute_Angle_Encoder
 * @brief 布瑞特绝对值编码器UART驱动
 * @{
 */

/**
 * @brief 编码器使用的UART句柄
 * @details 布瑞特绝对值编码器通过UART2与MCU通信
 */
#define ABS_ANGLE_UART huart2

#include "main.h"
#include "math_tool.h"
#include "usart.h"

/**
 * @brief 编码器精度位数
 * @details 12位分辨率，即编码器一圈有2^12=4096个计数位置
 *          角度分辨率 = 360°/4096 ≈ 0.0879°/count
 */
#define ACCURACY 12

/**
 * @brief 编码器最大计数值
 * @details 2^ACCURACY = 2^12 = 4096
 *          编码器旋转一圈对应的计数值，用于角度换算：
 *          角度 = (计数值 / MAX_CNT) * 360°
 */
#define MAX_CNT POW(2, ACCURACY)

/**
 * @brief 获取编码器当前角度值
 * 
 * @return float 当前角度值，单位：度(°)，范围0-360
 * @details 返回编码器实时测量的绝对角度值
 *          该值由UART中断接收回调函数更新
 * @note 该函数仅返回全局变量，无阻塞操作，可安全在任何上下文调用
 */
float get_abslute_angle_value();

/**
 * @brief 获取编码器设定的目标角度值
 * 
 * @return float 设定的目标角度值，单位：度(°)，范围0-360
 * @details 返回通过write_abslute_angle_reg()写入编码器的设定角度
 *          该值由UART中断接收的写寄存器应答帧更新
 * @note 该函数仅返回全局变量，无阻塞操作
 */
float get_set_angle_value();

/**
 * @brief 编码器主驱动函数
 * 
 * @details 该函数应在UART接收中断回调函数中调用，处理接收到的数据帧：
 *          1. 解析Modbus RTU应答帧
 *          2. 根据功能码(0x03读/0x06写)更新对应的角度值
 *          3. 重新发起下一次UART接收
 * @note 该函数必须放在对应的UART中断处理函数中
 *       每次调用会重新注册HAL_UART_Receive_IT接收中断
 */
void driver_abslute_angle();

/** @} */
/** @} */

#endif  /* ABSLUTE_ANGLE_SENSOR_H */
