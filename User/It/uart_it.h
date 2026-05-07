#ifndef __UART_IT_H
#define __UART_IT_H

#include <stdint.h>

extern uint8_t uart1_buf[255];
extern uint8_t uart2_buf[255];
extern uint8_t uart3_buf[255];
extern uint8_t uart4_buf[255];

// --- 全局变量声明 ---
typedef struct {
    float l1;
    float l2;
} GlobalTargetData_t;

extern GlobalTargetData_t target_data;

void init_uart_idle_it();

#endif