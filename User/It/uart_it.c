#include "dma.h"
#include "log.h"
#include "usart.h"
#include "uart_it.h"
#include "pid.h"
#include <stdio.h>
#include "status.h"
#include "string.h"

#define BUFFER_SIZE 255

uint8_t uart1_buf[BUFFER_SIZE] = {0};
uint8_t uart2_buf[BUFFER_SIZE] = {0};
uint8_t uart3_buf[BUFFER_SIZE] = {0};
uint8_t uart4_buf[BUFFER_SIZE] = {0};

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_uart4_rx;

// --- 实例化全局变量 ---
GlobalTargetData_t target_data = {0.0f, 0.0f};

// --- 串口2解析状态机所需常量与变量 ---
#define FRAME_HEAD   0xAA
#define FRAME_TAIL   0x55
#define FRAME_LEN    10   // 总帧长：1(头) + 8(两个浮点数) + 1(尾)

typedef enum {
    STATE_WAIT_HEAD,
    STATE_RECV_DATA,
    STATE_WAIT_TAIL
} FrameState_t;

FrameState_t rx_state = STATE_WAIT_HEAD;
uint8_t      rx_buffer[FRAME_LEN];
uint8_t      rx_index = 0;

/**
 * @brief  串口数据解析函数，每收到一个字节调用一次
 */

void start_uart_idle_it(UART_HandleTypeDef *huart, uint8_t *buf) {
    HAL_UARTEx_ReceiveToIdle_DMA(huart,buf,BUFFER_SIZE);
}

void init_uart_idle_it() {
  start_uart_idle_it(&huart1, uart1_buf);
  //start_uart_idle_it(&huart2, uart2_buf);
  start_uart_idle_it(&huart3, uart3_buf);
  //start_uart_idle_it(&huart4, uart4_buf);
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if (huart->Instance == USART1) {
        // ... 你的 USART1 逻辑 ...
    }

    if (huart->Instance == USART3) {
        // 1. 保证字符串结束
        if (Size < BUFFER_SIZE)
            uart3_buf[Size] = '\0';
        else
            uart3_buf[BUFFER_SIZE - 1] = '\0';

        // 2. 正确解析浮点数
        sscanf((char*)uart3_buf, "%f,%f", &target_data.l1, &target_data.l2);

        // 3. 清空 USART3 缓冲区并重新开启 DMA 空闲接收
        memset(uart3_buf, 0, BUFFER_SIZE);
        start_uart_idle_it(&huart3, uart3_buf);
    }
}