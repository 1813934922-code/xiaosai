/**
 * @file eeprom.c
 * @brief AT24C02 EEPROM I2C读写驱动实现
 * 
 * 实现了基于STM32 HAL库I2C的EEPROM读写功能：
 * - 设备就绪检测（轮询I2C ACK）
 * - 分页写入（自动处理跨页边界）
 * - DMA传输（非阻塞，提高效率）
 * 
 * EEPROM写操作特性：
 * AT24Cxx系列EEPROM在接收到写命令后，需要内部编程时间（typical 5ms），
 * 在此期间不会响应I2C ACK。本驱动通过HAL_I2C_IsDeviceReady()检测ACK状态，
 * 确保在EEPROM空闲时才发起新的传输。
 * 
 * 跨页写入逻辑：
 * EEPROM页写入时，若写入跨越页边界，数据会回卷到页首覆盖已有数据。
 * 本驱动检测跨页情况，将一次写入拆分为多次页对齐的DMA传输。
 */

#include "eeprom.h"

#include "i2c.h"
#include "log.h"

/** @brief EEPROM页写入缓冲区（当前未使用，保留供扩展） */
uint8_t eeprom_buff[PAGE_SIZE] = {0};

/**
 * @brief 检测AT24C02是否空闲
 * 
 * 通过HAL_I2C_IsDeviceReady()发送I2C START+地址，
 * 检测EEPROM是否返回ACK。若返回ACK说明内部写操作已完成。
 * 
 * @retval 1 EEPROM空闲，可以发起新操作
 * @retval 0 EEPROM忙碌，正在执行内部写操作
 */
uint8_t EEPROM_IsReady(void) {
  HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(&hi2c1, EEPROM_ADDRESS, 1, 10);
  return (status == HAL_OK) ? 1 : 0;
}

/**
 * @brief 阻塞等待直到EEPROM空闲
 * 
 * 轮询调用EEPROM_IsReady()，直到返回1。
 * 阻塞时间取决于EEPROM内部写操作完成时间（typical 5ms）。
 */
void EEPROM_WaitUntilReady(void) {
  while (!EEPROM_IsReady()) {
    continue;
  }
}

/**
 * @brief 向EEPROM写入数据（支持跨页处理）
 * 
 * 写入算法流程：
 * 1. 计算当前页号和剩余要写入的字节数
 * 2. 检查是否超出EEPROM容量范围
 * 3. 判断是否需要跨页：(tar_addr + cnt) / PAGE_SIZE != cur_page
 * 4. 若跨页：
 *    a. 计算当前页剩余空间：len = PAGE_SIZE - cur_addr % PAGE_SIZE
 *    b. 等待EEPROM就绪
 *    c. 发起DMA写入当前页剩余空间
 *    d. 更新cnt、cur_addr、cur_page
 * 5. 若不跨页：
 *    a. 等待EEPROM就绪
 *    b. 发起DMA写入剩余所有数据
 *    c. cnt = 0，退出循环
 * 
 * @note 使用HAL_I2C_Mem_Write_DMA()进行DMA传输
 * @note 每次DMA传输前都调用EEPROM_WaitUntilReady()
 * @warning 函数中存在一个潜在bug：cur_page = cur_addr + 1 应为 cur_page = cur_addr / PAGE_SIZE
 * 
 * @param tar_addr EEPROM内部目标地址
 * @param data     源数据缓冲区
 * @param size     要写入的字节数
 */
void EEPROM_WriteByte(uint16_t tar_addr, uint8_t *data, uint16_t size) {
  uint16_t cnt = size;                  // 剩余待写入字节数
  uint16_t cur_page = tar_addr / size;  // 当前页号（注：此处计算有bug，应为 tar_addr / PAGE_SIZE）
  uint16_t cur_addr = tar_addr;         // 当前写入地址
  uint16_t len = 0;                     // 当前页剩余空间
  while (cnt != 0) {
    if (cur_page >= PAGE_NUMB) {
      WARN("EEPROM_WriteByte: Out of range\n");
    }
    if ((tar_addr + cnt) / PAGE_SIZE != cur_page) {  // 检测是否跨页
      len = PAGE_SIZE - cur_addr % PAGE_SIZE;        // 计算当前页剩余空间
      EEPROM_WaitUntilReady();
      HAL_I2C_Mem_Write_DMA(&hi2c1, EEPROM_ADDRESS, cur_addr, EEPROM_MEMADD_SIZE, data, len);
      cnt = cnt - len;            // 更新剩余字节数
      cur_addr = cur_addr + len;  // 移动到下一页起始地址
      cur_page = cur_addr + 1;    // 更新页号（注：此处有bug）
    } else {
      EEPROM_WaitUntilReady();
      HAL_I2C_Mem_Write_DMA(&hi2c1, EEPROM_ADDRESS, cur_addr, EEPROM_MEMADD_SIZE, data, cnt);
      cnt = 0;  // 全部写入完成
    }
  }

  return;
}

/**
 * @brief 从EEPROM读取数据
 * 
 * 读取操作无需分页处理，可直接指定任意起始地址和长度。
 * 使用DMA传输，非阻塞。
 * 
 * @param tar_addr EEPROM内部目标地址
 * @param data     目标数据缓冲区
 * @param size     要读取的字节数
 */
void EEPROM_ReadByte(uint16_t tar_addr, uint8_t *data, uint16_t size) {
  EEPROM_WaitUntilReady();
  HAL_I2C_Mem_Read_DMA(&hi2c1, EEPROM_ADDRESS, tar_addr, EEPROM_MEMADD_SIZE, data, size);
}
