/**
 * @file log.c
 * @brief 嵌入式日志系统实现
 * 
 * 实现了基于UART的格式化日志输出，核心特性：
 * - 双缓冲机制：使用两个静态缓冲区交替使用，避免DMA发送时数据被覆盖
 * - 可变参数：支持printf风格的格式化输出
 * - 缓冲区大小：256字节，足够容纳大多数日志消息
 * 
 * 双缓冲工作原理：
 * 1. 定义两个静态缓冲区buf[0]和buf[1]
 * 2. 每次调用时通过abbuf标志切换使用哪个缓冲区
 * 3. 在buf[abbuf]中格式化字符串
 * 4. 通过HAL_UART_Transmit同步发送
 * 
 * 注意：当前实现使用HAL_UART_Transmit（阻塞式），
 * 若改为HAL_UART_Transmit_DMA可实现真正的非阻塞日志。
 */

#include "log.h"

#include "stdarg.h"
#include "stdio.h"
#include "usart.h"

/** @brief 日志格式化缓冲区长度（单个缓冲区大小） */
#define LOG_FORMAT_BUF_LENGTH 256

/** @brief 目标平台标识 */
#define STM32

#ifdef STM32

/**
 * @brief 可变参数格式化日志输出
 * 
 * 函数流程：
 * 1. 切换双缓冲索引（abbuf = !abbuf）
 * 2. 使用va_list解析可变参数
 * 3. 调用vsnprintf格式化到当前缓冲区
 * 4. 通过UART发送格式化后的字符串
 * 
 * 双缓冲设计说明：
 * - static变量确保缓冲区在函数调用间保持分配
 * - 两个缓冲区交替使用，防止DMA发送时缓冲区被新数据覆盖
 * - 当前使用阻塞式HAL_UART_Transmit，双缓冲主要用于未来DMA扩展
 * 
 * @param huart  UART句柄指针
 * @param format 格式化字符串（printf风格）
 * @param ...    可变参数列表
 * 
 * @note vsnprintf限制输出长度不超过LOG_FORMAT_BUF_LENGTH-1，防止缓冲区溢出
 * @note 使用HAL_UART_Transmit阻塞发送，超时时间100ms
 */
void log_uprintf(UART_HandleTypeDef *huart, const char *format, ...) {
  static unsigned char abbuf = 0;                          // 双缓冲切换索引
  static char buf[2][LOG_FORMAT_BUF_LENGTH];               // 双缓冲区

  abbuf = abbuf ? 0 : 1;                                   // 切换缓冲区（0->1, 1->0）

  va_list args;
  va_start(args, format);
  // 使用vsnprintf安全格式化，防止缓冲区溢出
  unsigned int len =
      vsnprintf(buf[abbuf], LOG_FORMAT_BUF_LENGTH - 1, format, args);
  va_end(args);

  // 通过UART发送日志数据（阻塞方式，超时100ms）
  HAL_UART_Transmit(huart, (uint8_t *)buf[abbuf], len, 100);
}

#endif
