/**
 * @file para.c
 * @brief 泛型参数类型转换实现
 * 
 * 实现了TYPE枚举对应的类型转换函数。
 * 核心功能：
 * - get_cur_val()：运行时类型萃取，统一转为float用于比较
 * - get_4byte_val()：类型到uint32_t的二进制转换（序列化）
 * - set_4byte_val()：uint32_t到类型的二进制转换（反序列化）
 * 
 * 这些函数使用switch-case根据TYPE枚举值进行类型分发，
 * 是C语言中实现泛型的典型方式。
 */

#include "para.h"

#include "stdbool.h"
#include "stdint.h"
#include "string.h"

/**
 * @brief 将任意类型的值转换为32位二进制表示
 * 
 * 转换策略：
 * - 32位类型（int/int32/uint32/float）：直接按位转换
 * - 小于32位类型：零扩展或符号扩展
 * - 大于32位类型（double/64位）：截断取低32位
 * - float特殊处理：使用*(uint32_t*)直接获取IEEE 754位模式
 * 
 * @return uint32_t 转换后的值，addr为NULL或未知类型时返回0
 */
uint32_t get_4byte_val(void* addr, TYPE type) {
  if (addr == NULL) {
    return 0;  // 空指针保护
  }
  switch (type) {
    case INT:
    case INT32:
    case UINT32:
      return *(uint32_t*)addr;  // 32位整数直接转换
    case FLOAT:
      return *(uint32_t*)addr;  // 直接获取float的IEEE 754位模式
    case DOUBLE:
      return (uint32_t)(*(double*)addr);  // double截断为32位
    case UINT8:
      return *(uint8_t*)addr;  // 8位无符号，零扩展
    case INT8:
      return (uint32_t)(*(int8_t*)addr);  // 8位有符号，符号扩展
    case UINT16:
      return *(uint16_t*)addr;  // 16位无符号，零扩展
    case INT16:
      return (uint32_t)(*(int16_t*)addr);  // 16位有符号，符号扩展
    case UINT64:
      return (uint32_t)(*(uint64_t*)addr);  // 64位无符号，截断低32位
    case INT64:
      return (uint32_t)(*(int64_t*)addr);  // 64位有符号，截断低32位
    default:
      return 0;  // 未知类型返回0
  }
}

/**
 * @brief 将32位值按目标类型写入内存
 * 
 * 转换策略：
 * - float类型：使用memcpy传递位模式（避免类型双关警告）
 * - 整数类型：直接类型转换赋值
 * - 64位类型：使用memcpy写入低32位
 * - 小于32位类型：自动截断
 * 
 * @return true  写入成功
 * @return false addr为NULL或未知类型
 */
bool set_4byte_val(void* addr, TYPE type, uint32_t value) {
  if (addr == NULL) {
    return false;  // 空指针保护
  }

  switch (type) {
    case INT:
    case INT32:
      *(int32_t*)addr = (int32_t)value;
      break;
    case UINT32:
      *(uint32_t*)addr = value;
      break;
    case FLOAT:
      // 使用memcpy传递float的IEEE 754位模式
      // 避免类型双关(strict aliasing)违规
      memcpy(addr, &value, sizeof(float));
      break;
    case UINT8:
      *(uint8_t*)addr = (uint8_t)value;  // 截断为8位
      break;
    case INT8:
      *(int8_t*)addr = (int8_t)value;    // 截断为8位
      break;
    case UINT16:
      *(uint16_t*)addr = (uint16_t)value;  // 截断为16位
      break;
    case INT16:
      *(int16_t*)addr = (int16_t)value;    // 截断为16位
      break;
    case DOUBLE:
    case UINT64:
    case INT64:
      // 64位类型只写入低32位，高32位保持不变
      memcpy(addr, &value, sizeof(uint32_t));
      break;
    default:
      return false;  // 未知类型
  }
  return true;
}

/**
 * @brief 泛型值萃取：将任意类型的值转换为float
 * 
 * 通过指针类型转换和隐式类型转换，将不同类型的值统一转为float。
 * 用于任务控制器中judge函数的统一接口。
 * 
 * @return float 转换后的值，cur_addr为NULL或未知类型时返回0.0f
 */
float get_cur_val(void* cur_addr, TYPE type) {
  if (cur_addr == NULL) {
    return 0.0f;  // 空指针保护
  }
  switch (type) {
    case INT:
      return *(int*)cur_addr;
    case FLOAT:
      return *(float*)cur_addr;
    case DOUBLE:
      return *(double*)cur_addr;
    case UINT8:
      return *(uint8_t*)cur_addr;
    case UINT16:
      return *(uint16_t*)cur_addr;
    case UINT32:
      return *(uint32_t*)cur_addr;
    case UINT64:
      return *(uint64_t*)cur_addr;
    case INT8:
      return *(int8_t*)cur_addr;
    case INT16:
      return *(int16_t*)cur_addr;
    case INT32:
      return *(int32_t*)cur_addr;
    case INT64:
      return *(int64_t*)cur_addr;
    default:
      return 0.0f;  // 未知类型返回0
  }
}
