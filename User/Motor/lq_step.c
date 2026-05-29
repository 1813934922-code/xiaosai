/**
 ******************************************************************************
 * @file    lq_step.c
 * @brief   雷赛步进电机 UART 控制实现文件
 * @details 本文件实现了通过 UART 串口向雷赛步进电机驱动器发送控制命令的功能。
 *          协议帧格式详解（11字节）：
 *          +--------+--------+--------+--------+--------+
 *          | 偏移0  | 偏移1  | 偏移2  | 偏移3  | 偏移4  |
 *          +--------+--------+--------+--------+--------+
 *          | 帧头   | 地址   | 命令字 | 方向   | 模式   |
 *          | 0x7B   | 0x01   | 见下表 | 0x00/01| 0x20   |
 *          +--------+--------+--------+--------+--------+
 *          | 偏移5  | 偏移6  | 偏移7  | 偏移8  | 偏移9  | 偏移10 |
 *          +--------+--------+--------+--------+--------+--------+
 *          | 数据H  | 数据L  | 速度H  | 速度L  | BCC    | 帧尾   |
 *          +--------+--------+--------+--------+--------+--------+
 *          命令字说明：
 *          - 0x01: 速度控制模式
 *          - 0x02: 相对角度控制模式
 *          - 0x03: 电流控制模式
 *          - 0x04: 绝对角度控制模式
 *          BCC校验：从偏移0到偏移8的所有字节进行异或运算
 ******************************************************************************
 */

#include "lq_step.h"

#include "math_tool.h"

/**
 * @brief  BCC(Block Check Character)校验计算函数
 * @param  data:   待校验的数据缓冲区指针
 * @param  length: 数据长度（字节数）
 * @retval 计算得到的BCC校验值（单字节异或和）
 * @note   算法：将所有数据字节依次进行异或(XOR)运算，得到最终校验值
 *         用于检测UART通信过程中的数据传输错误
 */
uint8_t BCC(uint8_t *data, uint16_t length) {
  uint8_t i;
  uint8_t bcc = 0;  // 初始校验值设为0
  while (length--) {
    bcc ^= *data++;  // 逐字节异或累加
  }
  return bcc;
}

/**
 * @brief  控制步进电机转到绝对角度位置
 * @param  huart: UART 句柄指针，指定用于通信的串口外设
 * @param  angle: 目标绝对角度，单位：度 (°)，有效范围 0~360°
 * @param  speed: 转动速度，单位：rad/s
 * @retval 无
 * @note   命令帧结构：
 *         cmd[0]  = 0x7B          // 帧头
 *         cmd[1]  = 0x01          // 设备地址
 *         cmd[2]  = 0x04          // 命令字：绝对角度控制
 *         cmd[3]  = 0x01          // 固定值
 *         cmd[4]  = 0x20          // 模式标识
 *         cmd[5-6]= 角度值×10     // 目标角度（大端序，单位0.1°）
 *         cmd[7-8]= 速度值×10     // 转动速度（大端序）
 *         cmd[9]  = BCC(cmd, 9)   // 校验和
 *         cmd[10] = 0x7D          // 帧尾
 */
void trun_lq_step_abslute_angle(UART_HandleTypeDef *huart, float angle, float speed) {
  uint8_t cmd[11] = {0};
  cmd[0] = 0x7B;  // 帧头：固定值0x7B，标识一帧数据的开始
  cmd[1] = 0x01;  // 设备地址：0x01表示1号电机
  cmd[2] = 0x04;  // 命令字：0x04表示绝对角度控制模式
  cmd[3] = 0x01;  // 保留字段
  cmd[4] = 0x20;  // 模式标识：0x20

  // 将角度值从度转换为协议格式（×10，单位0.1°），并限制在0~3600范围内
  int16_t turn_angle = (uint16_t)(angle * 10);
  turn_angle = CONFINE(turn_angle, 0, 3600);
  cmd[5] = ((turn_angle >> 8) & 0x00FF);  // 角度高字节（大端序）
  cmd[6] = (turn_angle & 0x00FF);         // 角度低字节

  // 将速度值转换为协议格式（×10）
  int16_t turn_speed = (uint16_t)(speed * 10);
  cmd[7] = (turn_speed >> 8) & 0x00FF;  // 速度高字节（大端序）
  cmd[8] = turn_speed & 0x00FF;         // 速度低字节

  cmd[9] = BCC(cmd, 9);   // 计算前9个字节的BCC校验值
  cmd[10] = 0x7D;         // 帧尾：固定值0x7D，标识一帧数据的结束

  // 通过UART发送11字节命令帧，超时时间10ms
  HAL_UART_Transmit(huart, cmd, 11, 10);

  return;
}

/**
 * @brief  控制步进电机转动指定的相对角度
 * @param  huart: UART 句柄指针，指定用于通信的串口外设
 * @param  angle: 相对转动角度，单位：度 (°)
 * @param  dir:   转动方向
 *               @arg 0x00: 逆时针 (CCW - Counter Clockwise)
 *               @arg 0x01: 顺时针 (CW - Clockwise)
 * @param  speed: 转动速度，单位：rad/s
 * @retval 无
 * @note   命令帧结构与绝对角度控制类似，但命令字为0x02，
 *         角度值为相对当前转过的角度，不是绝对位置
 */
void trun_lq_step_angle(UART_HandleTypeDef *huart, float angle, uint8_t dir, float speed) {
  uint8_t cmd[11] = {0};
  cmd[0] = 0x7B;  // 帧头
  cmd[1] = 0x01;  // 设备地址
  cmd[2] = 0x02;  // 命令字：0x02表示相对角度控制模式
  cmd[3] = dir;   // 方向控制：0x00=逆时针, 0x01=顺时针
  cmd[4] = 0x20;  // 模式标识

  // 将角度值转换为协议格式（×10，单位0.1°）
  int16_t turn_angle = (uint16_t)(angle * 10);
  cmd[5] = ((turn_angle >> 8) & 0x00FF);  // 角度高字节
  cmd[6] = (turn_angle & 0x00FF);         // 角度低字节

  // 将速度值转换为协议格式（×10）
  int16_t turn_speed = (uint16_t)(speed * 10);
  cmd[7] = (turn_speed >> 8) & 0x00FF;  // 速度高字节
  cmd[8] = turn_speed & 0x00FF;         // 速度低字节

  cmd[9] = BCC(cmd, 9);   // BCC校验
  cmd[10] = 0x7D;         // 帧尾

  HAL_UART_Transmit(huart, cmd, 11, 10);

  return;
}

/**
 * @brief  控制步进电机以指定速度连续转动
 * @param  huart: UART 句柄指针，指定用于通信的串口外设
 * @param  speed: 转动速度，单位：rad/s
 * @param  dir:   转动方向
 *               @arg 0x00: 逆时针 (CCW)
 *               @arg 0x01: 顺时针 (CW)
 * @retval 无
 * @note   命令帧结构：
 *         cmd[0]  = 0x7B          // 帧头
 *         cmd[1]  = 0x01          // 设备地址
 *         cmd[2]  = 0x01          // 命令字：速度控制模式
 *         cmd[3]  = 方向          // 0x00/0x01
 *         cmd[4]  = 0x20          // 模式标识
 *         cmd[5-6]= 0x0000        // 角度数据（速度模式下无效）
 *         cmd[7-8]= 速度值×10     // 转动速度
 *         cmd[9]  = BCC           // 校验
 *         cmd[10] = 0x7D          // 帧尾
 */
void trun_lq_step_speed(UART_HandleTypeDef *huart, float speed, uint8_t dir) {
  uint8_t cmd[11] = {0};
  cmd[0] = 0x7B;  // 帧头
  cmd[1] = 0x01;  // 设备地址
  cmd[2] = 0x01;  // 命令字：0x01表示速度控制模式
  cmd[3] = dir;   // 方向控制：0x00=逆时针, 0x01=顺时针
  cmd[4] = 0x20;  // 模式标识
  cmd[5] = 0;     // 角度数据高字节（速度模式下设为0）
  cmd[6] = 0;     // 角度数据低字节

  // 将速度值转换为协议格式（×10）
  int16_t turn_speed = (uint16_t)(speed * 10);
  cmd[7] = (turn_speed >> 8) & 0x00FF;  // 速度高字节
  cmd[8] = turn_speed & 0x00FF;         // 速度低字节

  cmd[9] = BCC(cmd, 9);   // BCC校验
  cmd[10] = 0x7D;         // 帧尾

  HAL_UART_Transmit(huart, cmd, 11, 10);

  return;
}

/**
 * @brief  控制步进电机以指定电流输出
 * @param  huart:   UART 句柄指针，指定用于通信的串口外设
 * @param  current: 输出电流值，单位：毫安 (mA)
 * @param  dir:     电流方向
 *                 @arg 0x00: 逆时针方向 (CCW)
 *                 @arg 0x01: 顺时针方向 (CW)
 * @retval 无
 * @note   命令帧结构：
 *         cmd[0]  = 0x7B          // 帧头
 *         cmd[1]  = 0x01          // 设备地址
 *         cmd[2]  = 0x03          // 命令字：电流控制模式
 *         cmd[3]  = 方向          // 0x00/0x01
 *         cmd[4]  = 0x20          // 模式标识
 *         cmd[5-6]= 电流值        // 目标电流（大端序，单位mA）
 *         cmd[7-8]= 0xFFFF        // 速度数据（电流模式下设为最大值）
 *         cmd[9]  = BCC           // 校验
 *         cmd[10] = 0x7D          // 帧尾
 *         电流模式常用于需要精确控制输出力矩的应用场景
 */
void trun_lq_step_current(UART_HandleTypeDef *huart, uint16_t current, uint8_t dir) {
  uint8_t cmd[11] = {0};
  cmd[0] = 0x7B;  // 帧头
  cmd[1] = 0x01;  // 设备地址
  cmd[2] = 0x03;  // 命令字：0x03表示电流控制模式
  cmd[3] = dir;   // 方向控制：0x00=逆时针, 0x01=顺时针
  cmd[4] = 0x20;  // 模式标识
  cmd[5] = (current >> 8) & 0x00FF;  // 电流值高字节（大端序）
  cmd[6] = current & 0x00FF;         // 电流值低字节
  cmd[7] = 0xff;  // 速度数据高字节（电流模式下设为0xFF）
  cmd[8] = 0xff;  // 速度数据低字节

  cmd[9] = BCC(cmd, 9);   // BCC校验
  cmd[10] = 0x7D;         // 帧尾

  HAL_UART_Transmit(huart, cmd, 11, 10);

  return;
}
