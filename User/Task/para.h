#ifndef __PARA_H_
#define __PARA_H_

#include "para.h"
#include "stdbool.h"
#include "stdint.h"

/**
 * @defgroup ParameterType 泛型参数类型系统
 * @brief 定义数据类型枚举，用于运行时类型识别和泛型值萃取
 * 
 * 该模块为任务调度器提供类型擦除和泛型支持，使得同一个任务控制器
 * 可以监控任意类型的变量（int、float、uint8等）。
 * 
 * 核心机制：
 * - TYPE枚举：定义支持的所有数据类型
 * - get_cur_val()：类型萃取，将任意类型转换为float
 * - get_4byte_val()：将任意类型的值转换为uint32_t二进制表示
 * - set_4byte_val()：将uint32_t二进制值写回目标类型变量
 * 
 * @note 枚举值从1开始，0为未知类型，便于错误检测
 * @{
 */

/**
 * @enum TYPE
 * @brief 数据类型枚举，用于运行时类型识别
 * 
 * 覆盖C语言常用的整数和浮点类型。
 * 使用场景：任务控制器中的cur_addr指针类型擦除后，
 * 通过type字段在运行时确定正确的取值方式。
 * 
 * @note 64位类型在get_4byte_val/set_4byte_val中会被截断/扩展处理
 */
typedef enum TYPE {
  FLOAT = 1,   /**< @brief 32位浮点数 (float) */
  DOUBLE,      /**< @brief 64位浮点数 (double) */
  INT,         /**< @brief 平台相关整数 (int) */
  UINT8,       /**< @brief 8位无符号整数 */
  INT8,        /**< @brief 8位有符号整数 */
  UINT16,      /**< @brief 16位无符号整数 */
  INT16,       /**< @brief 16位有符号整数 */
  UINT32,      /**< @brief 32位无符号整数 */
  INT32,       /**< @brief 32位有符号整数 */
  UINT64,      /**< @brief 64位无符号整数 */
  INT64,       /**< @brief 64位有符号整数 */
  UNKNOWN,     /**< @brief 未知类型，用于错误检测 */
} TYPE;

/** @} */  // end of ParameterType

/**
 * @defgroup TypeConversion 类型转换工具
 * @{
 */

/**
 * @brief 泛型值萃取：将任意类型的内存值转换为float
 * 
 * 根据TYPE类型，对cur_addr指向的内存进行正确的类型转换，
 * 提取数值并转为float返回。用于任务控制器中统一比较接口。
 * 
 * @param cur_addr 数据变量地址（void*泛型指针）
 * @param type     数据类型标识
 * @return float   转换后的浮点值
 * @retval 0.0f    cur_addr为NULL或type为UNKNOWN
 * 
 * @note 64位整数和double转float时可能有精度损失
 * @note 此函数不会修改原始内存值
 * 
 * 使用示例：
 * @code
 * uint16_t sensor_value = 1024;
 * float val = get_cur_val(&sensor_value, UINT16);  // val = 1024.0f
 * @endcode
 */
float get_cur_val(void* cur_addr, TYPE type);

/**
 * @brief 将任意类型的值转换为32位二进制表示
 * 
 * 根据TYPE类型，将addr指向的值转换为uint32_t的二进制位模式。
 * 主要用于序列化、EEPROM存储或跨平台数据交换。
 * 
 * @param addr 源变量地址
 * @param type 数据类型标识
 * @return uint32_t 转换后的32位值
 * @retval 0        addr为NULL或type为UNKNOWN
 * 
 * @note 对于大于32位的类型（double、uint64、int64），只保留低32位
 * @note 对于小于32位的类型（uint8、int8等），会进行零扩展或符号扩展
 * @note float类型直接返回其IEEE 754二进制表示
 * 
 * 使用示例：
 * @code
 * float f = 3.14f;
 * uint32_t bits = get_4byte_val(&f, FLOAT);  // bits = 0x4048F5C3
 * @endcode
 */
uint32_t get_4byte_val(void* addr, TYPE type);

/**
 * @brief 将32位二进制值写入目标类型变量
 * 
 * get_4byte_val()的逆操作，将uint32_t值按目标类型写入内存。
 * 用于从EEPROM反序列化数据或恢复状态。
 * 
 * @param addr  目标变量地址
 * @param type  目标数据类型
 * @param value 要写入的32位值
 * @return true  写入成功
 * @return false addr为NULL或type为UNKNOWN
 * 
 * @note 对于float类型，使用memcpy确保位模式正确传递
 * @note 对于64位类型，只写入低32位，高32位不变
 * @note 对于小于32位的类型，会自动截断
 * 
 * 使用示例：
 * @code
 * float f;
 * set_4byte_val(&f, FLOAT, 0x4048F5C3);  // f = 3.14f
 * @endcode
 */
bool set_4byte_val(void* addr, TYPE type, uint32_t value);

/** @} */  // end of TypeConversion

#endif
