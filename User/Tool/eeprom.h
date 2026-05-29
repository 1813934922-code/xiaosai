/**
 * @file eeprom.h
 * @brief AT24C02 EEPROM I2C读写驱动头文件
 * 
 * 基于STM32 HAL库的I2C EEPROM读写驱动，支持：
 * - 页写入：自动处理跨页边界
 * - DMA传输：使用DMA提高传输效率
 * - 设备就绪检测：等待EEPROM内部写操作完成
 * 
 * EEPROM规格：
 * - 型号：AT24C02（2Kbit = 256字节）
 * - I2C地址：0xA0（写）/ 0xA1（读）
 * - 页大小：8字节（AT24C02实际页大小为8字节，本代码定义128字节可能适用于其他型号）
 * - 地址宽度：8位或16位（通过ADDR_SIZE_8BIT/ADDR_SIZE_16BIT宏选择）
 * 
 * @addtogroup EEPROM
 * @{
 */

#ifndef __EEPROM_H__
#define __EEPROM_H__

/** @brief EEPROM的I2C设备地址（7位地址左移1位后的写地址） */
#define EEPROM_ADDRESS 0xA0

/** @brief 每页字节数，用于跨页写入处理 */
#define PAGE_SIZE 128

/** @brief 总页数 */
#define PAGE_NUMB 16

/** @brief 启用16位地址模式（适用于大容量EEPROM） */
#define ADDR_SIZE_16BIT

#ifdef ADDR_SIZE_8BIT
/** @brief 8位地址模式对应的HAL宏 */
#define EEPROM_MEMADD_SIZE I2C_MEMADD_SIZE_8BIT
#endif

#ifdef ADDR_SIZE_16BIT
/** @brief 16位地址模式对应的HAL宏 */
#define EEPROM_MEMADD_SIZE I2C_MEMADD_SIZE_16BIT
#endif

#include "main.h"

/**
 * @defgroup EEPROM_API EEPROM读写API
 * @{
 */

/**
 * @brief 向EEPROM写入数据
 * 
 * 支持跨页写入，自动处理页边界。
 * 使用DMA传输，非阻塞。
 * 
 * @param tar_addr 目标内存地址（EEPROM内部地址）
 * @param data     要写入的数据缓冲区指针
 * @param size     要写入的字节数
 * 
 * @note 写入前会等待EEPROM就绪，确保上次写操作已完成
 * @note 跨页时会分多次DMA写入，每页独立等待就绪
 * @warning 超出EEPROM容量时会打印警告，但不会阻止写入操作
 * 
 * 使用示例：
 * @code
 * uint8_t data[] = {0x01, 0x02, 0x03};
 * EEPROM_WriteByte(0x00, data, sizeof(data));
 * @endcode
 */
void EEPROM_WriteByte(uint16_t tar_addr, uint8_t *data, uint16_t size);

/**
 * @brief 从EEPROM读取数据
 * 
 * 使用DMA传输，非阻塞。
 * 读取操作无需分页处理。
 * 
 * @param tar_addr 目标内存地址（EEPROM内部地址）
 * @param data     存储读取数据的缓冲区指针
 * @param size     要读取的字节数
 * 
 * @note 读取前会等待EEPROM就绪
 * 
 * 使用示例：
 * @code
 * uint8_t buffer[10];
 * EEPROM_ReadByte(0x00, buffer, sizeof(buffer));
 * @endcode
 */
void EEPROM_ReadByte(uint16_t tar_addr, uint8_t *data, uint16_t size);

/** @} */  // end of EEPROM_API
/** @} */  // end of EEPROM

#endif
