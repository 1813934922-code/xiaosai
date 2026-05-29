/**
 * @file log.h
 * @brief 嵌入式日志系统头文件
 * 
 * 基于UART的轻量级日志系统，支持：
 * - 多级日志输出：ERROR、WARN、INFO、DEBUG、TRACE
 * - 时间戳格式化：自动附加系统时间（分:秒:节拍）
 * - 源码位置追踪：DEBUG/TRACE级别自动附加函数名和行号
 * - 编译期开关：通过LOG_ENABLE宏控制日志是否编译
 * - 双缓冲机制：log.c中实现，支持DMA发送不阻塞
 * 
 * 使用方式：
 * @code
 * // 基础日志输出
 * ERROR("Motor stalled!");
 * WARN("Temperature high: %d", temp);
 * INFO("System started");
 * 
 * // 带源码位置的调试日志
 * DEBUG("Value = %d", x);
 * 
 * // 变量追踪
 * TRACE(sensor_value, "%d");
 * @endcode
 * 
 * 日志级别说明：
 * - ERROR (E)：严重错误，系统可能无法正常运行
 * - WARN (W)：警告信息，系统可继续但需要注意
 * - INFO (I)：一般信息，系统正常运行状态
 * - DEBUG (D)：调试信息，附带函数名和行号
 * - TRACE (T)：变量追踪，用于跟踪特定变量值变化
 * 
 * @addtogroup LogSystem
 * @{
 */

#ifndef __LOG_H__
#define __LOG_H__

#include "usart.h"

/** @brief 日志输出使用的UART句柄 */
#define LOG_UART &huart1

/**
 * @brief 格式化日志输出函数
 * 
 * 内部使用vsprintf格式化字符串，通过UART发送。
 * 支持标准printf格式说明符。
 * 
 * @param huart  UART句柄指针
 * @param format 格式化字符串
 * @param ...    可变参数
 */
void log_uprintf(UART_HandleTypeDef *huart, const char *format, ...);

/**
 * @defgroup LogMacros 日志宏定义
 * @{
 */

/**
 * @brief 基础日志输出宏（不附加换行）
 * @param fmt 格式化字符串
 * @param ... 可变参数
 */
#define PRINTF(fmt, ...) log_uprintf(LOG_UART, fmt, ##__VA_ARGS__);

/**
 * @brief 带换行的日志输出宏
 * @param fmt 格式化字符串
 * @param ... 可变参数
 */
#define PRINTLN(fmt, ...) PRINTF(fmt "\r\n", ##__VA_ARGS__)

/** @brief 日志时间格式字符串模板：MM:SS:tick */
#define LOG_TIME_FMT_TYPE "%02u:%02u:%02u"

/**
 * @brief 时间格式参数展开宏
 * 
 * 将status.times转换为(分钟, 秒, tick)三个参数。
 * 假设STATUS_FREQ为系统tick频率（Hz）。
 * 
 * @param t 系统节拍计数
 */
#define LOG_TIME_FMT(t) \
  ((t / STATUS_FREQ) / 60), ((t / STATUS_FREQ) % 60), (t % STATUS_FREQ)

/**
 * @brief 事件日志宏（带时间戳）
 * 
 * 输出格式：[级别] MM:SS:tick 消息内容
 * 
 * @param level 日志级别标识（"E"/"W"/"I"等）
 * @param fmt   格式化字符串
 * @param ...   可变参数
 */
#define LOG_EVENT(level, fmt, ...)                                         \
  PRINTLN(level " " LOG_TIME_FMT_TYPE " " fmt, LOG_TIME_FMT(status.times), \
          ##__VA_ARGS__)

/**
 * @brief 带源码位置的事件日志宏
 * 
 * 在LOG_EVENT基础上附加函数名和行号。
 * 输出格式：[级别] MM:SS:tick 函数名:行号 消息内容
 * 
 * @param level 日志级别标识
 * @param fmt   格式化字符串
 * @param ...   可变参数
 */
#define LOG_SPAN(level, fmt, ...) \
  LOG_EVENT(level, "%s:%u " fmt, __func__, __LINE__, ##__VA_ARGS__)

/** @brief 错误日志宏（E级别） */
#define ERROR(fmt, ...) LOG_EVENT("E", fmt, ##__VA_ARGS__)

/** @brief 警告日志宏（W级别） */
#define WARN(fmt, ...) LOG_EVENT("W", fmt, ##__VA_ARGS__)

/** @brief 信息日志宏（I级别） */
#define INFO(fmt, ...) LOG_EVENT("I", fmt, ##__VA_ARGS__)

/** @brief 调试日志宏（D级别，带函数名和行号） */
#define DEBUG(fmt, ...) LOG_SPAN("D", fmt, ##__VA_ARGS__)

/**
 * @brief 变量追踪宏（T级别）
 * 
 * 自动将变量名作为标签输出。
 * 使用示例：TRACE(speed, "%d") -> 输出 "T 00:01:23 func:42 speed=100"
 * 
 * @param var 变量名
 * @param fmt 格式化字符串
 */
#define TRACE(var, fmt) LOG_SPAN("T", #var "=" fmt, var)

/** @brief 带源码位置的错误日志宏 */
#define THROW_ERROR(fmt, ...) LOG_SPAN("E", fmt, ##__VA_ARGS__)

/** @brief 带源码位置的警告日志宏 */
#define THROW_WARN(fmt, ...) LOG_SPAN("W", fmt, ##__VA_ARGS__)

/**
 * @defgroup LogControl 日志控制
 * @{
 */

#ifdef DEV
/** @brief 开发模式下启用日志 */
#define LOG_ENABLE
#endif  // DEV

#ifndef LOG_ENABLE
/**
 * @brief 日志禁用时，将所有日志宏展开为空操作
 * 
 * 编译期优化：当日志被禁用时，所有日志调用不会生成任何代码。
 */
#undef LOG_EVENT
#define LOG_EVENT(level, fmt, ...)
#endif  // !LOG_ENABLE

/** @} */  // end of LogControl
/** @} */  // end of LogMacros
/** @} */  // end of LogSystem

#endif  // !__LOG_H__
