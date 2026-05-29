/**
 ******************************************************************************
 * @file    uart_it.h
 * @brief   UART中断与DMA空闲接收头文件
 *
 * @details 本文件定义了串口通信相关的全局缓冲区和数据结构：
 *          - 4个串口（USART1/2/3/4）的DMA接收缓冲区声明
 *          - 全局目标数据结构体（用于接收上位机下发的浮点参数）
 *          - 串口空闲中断初始化接口
 *
 * @note    串口协议帧格式（USART2）：
 *          [FRAME_HEAD 0xAA] [8字节数据(2个float)] [FRAME_TAIL 0x55]
 *          总帧长10字节：1(头) + 8(两个浮点数) + 1(尾)
 ******************************************************************************
 */

#ifndef __UART_IT_H
#define __UART_IT_H

#include <stdint.h>

/**
 * @defgroup UART接收缓冲区
 * @brief 4个串口各自的DMA接收缓冲区（255字节）
 * @{
 */

/** @brief USART1 DMA接收缓冲区 */
extern uint8_t uart1_buf[255];

/** @brief USART2 DMA接收缓冲区 */
extern uint8_t uart2_buf[255];

/** @brief USART3 DMA接收缓冲区 */
extern uint8_t uart3_buf[255];

/** @brief USART4 DMA接收缓冲区 */
extern uint8_t uart4_buf[255];

/** @} */

/**
 * @brief 全局目标数据结构体
 * @details 用于存储从串口接收到的上位机下发的目标参数。
 *          目前包含两个浮点数l1和l2，可通过串口协议动态修改。
 *          典型应用：巡线PID参数调整、目标速度设置等。
 */
typedef struct {
    float l1;  /**< 目标参数1（如巡线P参数或左轮目标速度） */
    float l2;  /**< 目标参数2（如巡线D参数或右轮目标速度） */
} GlobalTargetData_t;

/** @brief 全局目标数据实例 */
extern GlobalTargetData_t target_data;

/**
 * @brief  初始化串口空闲中断接收
 * @details 为USART1和USART3启动DMA空闲中断接收：
 *          - USART1: 预留，当前未启用
 *          - USART3: 用于接收上位机下发的浮点参数（格式："float1,float2"）
 *
 * @note    DMA空闲中断接收机制：
 *          当串口收到数据后，DMA自动将数据搬运到缓冲区。
 *          当串口空闲（无新数据）超过1个字节时间时，触发RxEvent回调。
 *          在回调中可以获取接收到的数据长度，进行协议解析。
 */
void init_uart_idle_it(void);

#endif
