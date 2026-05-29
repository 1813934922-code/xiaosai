/**
 ******************************************************************************
 * @file    uart_it.c
 * @brief   UART DMA空闲中断接收实现
 *
 * @details 本文件实现了基于HAL库的串口DMA空闲中断接收机制：
 *          - 4个串口（USART1/2/3/4）的DMA接收缓冲区定义
 *          - DMA空闲中断初始化（HAL_UARTEx_ReceiveToIdle_DMA）
 *          - RxEvent回调函数（HAL_UARTEx_RxEventCallback）处理接收到的数据
 *          - USART3：接收上位机下发的浮点参数（格式："float1,float2"）
 *          - USART2：预留二进制帧协议解析（FRAME_HEAD 0xAA / FRAME_TAIL 0x55）
 *
 * @note    DMA空闲中断接收原理：
 *          传统串口接收需要逐个字节中断，效率低。
 *          DMA空闲中断机制：DMA自动搬运接收数据到缓冲区，当串口空闲
 *          （无新数据超过1个字节时间）时触发回调，一次性处理整包数据。
 *          优势：无需预先知道数据长度，自动分包，CPU占用低。
 ******************************************************************************
 */

#include "dma.h"
#include "log.h"
#include "usart.h"
#include "uart_it.h"
#include "pid.h"
#include <stdio.h>
#include "status.h"
#include "string.h"

/** @brief DMA接收缓冲区大小（255字节） */
#define BUFFER_SIZE 255

/** @brief USART1 DMA接收缓冲区实例 */
uint8_t uart1_buf[BUFFER_SIZE] = {0};

/** @brief USART2 DMA接收缓冲区实例 */
uint8_t uart2_buf[BUFFER_SIZE] = {0};

/** @brief USART3 DMA接收缓冲区实例 */
uint8_t uart3_buf[BUFFER_SIZE] = {0};

/** @brief USART4 DMA接收缓冲区实例 */
uint8_t uart4_buf[BUFFER_SIZE] = {0};

/** @brief USART1 DMA句柄外部声明 */
extern DMA_HandleTypeDef hdma_usart1_rx;

/** @brief USART2 DMA句柄外部声明 */
extern DMA_HandleTypeDef hdma_usart2_rx;

/** @brief USART3 DMA句柄外部声明 */
extern DMA_HandleTypeDef hdma_usart3_rx;

/** @brief USART4 DMA句柄外部声明 */
extern DMA_HandleTypeDef hdma_uart4_rx;

/** @brief 全局目标数据实例（存储从串口接收的参数） */
GlobalTargetData_t target_data = {0.0f, 0.0f};

/* ========================================================================
 * USART2二进制帧协议解析相关定义（预留功能）
 * ========================================================================
 *
 * @details 协议帧格式：
 *          [FRAME_HEAD] [float l1 (4字节)] [float l2 (4字节)] [FRAME_TAIL]
 *          [0xAA]       [4字节]             [4字节]             [0x55]
 *
 *          状态机流转：
 *          STATE_WAIT_HEAD → STATE_RECV_DATA → STATE_WAIT_TAIL → (完成/重新等待头)
 *
 *          - STATE_WAIT_HEAD:  等待帧头0xAA
 *          - STATE_RECV_DATA:  接收8字节数据（两个浮点数）
 *          - STATE_WAIT_TAIL:  等待帧尾0x55
 */

/** @brief 协议帧头标识符 */
#define FRAME_HEAD   0xAA

/** @brief 协议帧尾标识符 */
#define FRAME_TAIL   0x55

/** @brief 协议帧总长度：1(头) + 8(两个浮点数) + 1(尾) = 10字节 */
#define FRAME_LEN    10

/**
 * @brief 帧解析状态机枚举
 *
 * @details 状态流转说明：
 *          - STATE_WAIT_HEAD:  初始状态，逐字节检查是否为帧头0xAA
 *          - STATE_RECV_DATA:  收到帧头后，接收接下来8字节数据
 *          - STATE_WAIT_TAIL:  数据接收完成后，检查下一个字节是否为帧尾0x55
 *                              如果是，解析完成；如果不是，回到STATE_WAIT_HEAD重新同步
 */
typedef enum {
    STATE_WAIT_HEAD,     /**< 等待帧头状态：逐字节匹配0xAA */
    STATE_RECV_DATA,     /**< 接收数据状态：接收8字节有效载荷 */
    STATE_WAIT_TAIL      /**< 等待帧尾状态：匹配0x55确认帧完整 */
} FrameState_t;

/** @brief 帧解析状态机当前状态 */
FrameState_t rx_state = STATE_WAIT_HEAD;

/** @brief 帧接收缓冲区（10字节：1头+8数据+1尾） */
uint8_t      rx_buffer[FRAME_LEN];

/** @brief 当前接收字节索引（在rx_buffer中的位置） */
uint8_t      rx_index = 0;

/**
 * @brief  串口数据解析函数（每收到一个字节调用一次）
 * @note   当前未使用，预留用于USART2二进制帧协议解析
 */

/* ========================================================================
 * DMA空闲中断接收控制函数
 * ======================================================================== */

/**
 * @brief  启动单个串口的DMA空闲中断接收
 *
 * @param[in] huart 串口句柄指针
 * @param[in] buf   DMA接收缓冲区指针
 *
 * @details 调用HAL_UARTEx_ReceiveToIdle_DMA启动DMA接收：
 *          - DMA自动将串口RX数据搬运到buf
 *          - 当串口空闲（无新数据超过1个字节时间）时触发RxEvent回调
 *          - 回调参数中包含实际接收到的数据长度（Size）
 *
 * @note    每次RxEvent回调后需要重新调用此函数启动下一次DMA接收
 */
void start_uart_idle_it(UART_HandleTypeDef *huart, uint8_t *buf) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart,buf,BUFFER_SIZE);
}

/**
 * @brief  初始化所有串口的DMA空闲中断接收
 *
 * @details 当前启用的串口：
 *          - USART1: 未启用（预留）
 *          - USART2: 未启用（预留二进制帧协议）
 *          - USART3: 已启用，接收上位机浮点参数
 *          - USART4: 未启用（预留）
 *
 * @note    可以在后续需要时取消注释启用其他串口
 */
void init_uart_idle_it() {
  start_uart_idle_it(&huart1, uart1_buf);
  //start_uart_idle_it(&huart2, uart2_buf);
  start_uart_idle_it(&huart3, uart3_buf);
  //start_uart_idle_it(&huart4, uart4_buf);
}

/* ========================================================================
 * HAL库DMA空闲中断回调函数
 * ========================================================================
 *
 * @details 当DMA接收完成或串口空闲时，HAL库自动调用此回调。
 *          回调中需要：
 *          1. 判断是哪个串口触发
 *          2. 获取接收到的数据长度（Size参数）
 *          3. 解析数据
 *          4. 清空缓冲区
 *          5. 重新启动DMA接收（重要！否则下次无法接收）
 */

/**
 * @brief  HAL库DMA空闲中断回调函数
 *
 * @param[in] huart 触发回调的串口句柄
 * @param[in] Size  接收到的数据字节数
 *
 * @details 当前实现的串口处理：
 *          - USART1: 预留，无处理逻辑
 *          - USART3: 接收ASCII格式浮点参数，使用sscanf解析
 *                    数据格式："float1,float2"（如"3.14,2.71"）
 *                    解析结果存入target_data.l1和target_data.l2
 *
 * @note    每次处理完成后必须调用start_uart_idle_it重新启动DMA接收，
 *          否则下次不会触发回调。
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        // USART1处理逻辑（预留）
    }

    if (huart->Instance == USART3) {
        // 1. 保证字符串结束（防止越界）
        //    DMA可能写满整个缓冲区，需要手动添加字符串结束符
        if (Size < BUFFER_SIZE)
            uart3_buf[Size] = '\0';
        else
            uart3_buf[BUFFER_SIZE - 1] = '\0';

        // 2. 解析浮点参数
        //    使用sscanf从ASCII字符串中提取两个逗号分隔的浮点数
        //    例如：收到"3.14,2.71\n" → target_data.l1=3.14, target_data.l2=2.71
        sscanf((char*)uart3_buf, "%f,%f", &target_data.l1, &target_data.l2);

        // 3. 清空USART3缓冲区并重新开启DMA空闲接收
        //    必须重新开启，否则下次接收不会触发回调
        memset(uart3_buf, 0, BUFFER_SIZE);
        start_uart_idle_it(&huart3, uart3_buf);
    }
}
