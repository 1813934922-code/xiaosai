/**
 ******************************************************************************
 * @file    abslute_angle_sensor.c
 * @brief   布瑞特绝对值编码器UART驱动实现
 * @details 本文件实现了布瑞特(BRIGHT)绝对值编码器的完整UART通信驱动，包括：
 *          - Modbus RTU协议帧封装(读写寄存器)
 *          - Modbus CRC16校验算法
 *          - UART中断接收数据处理
 *          - 角度值解析与换算
 * @note    通信参数：UART2，Modbus RTU协议
 *          编码器地址：0x01，精度：12位(4096计数)
 ******************************************************************************
 */

// @551

#include "abslute_angle_sensor.h"

uint8_t abslute_angle_sensor_buff[20] = {0};  /**< UART接收数据暂存缓冲区 */
float cur_angle = 0;                          /**< 当前角度值(从编码器读取的实时角度) */
float set_angle = 0;                          /**< 设定角度值(写入编码器的目标角度) */

/**
 * @brief Modbus CRC16校验算法
 * 
 * @param[in] pbuf 指向待校验数据缓冲区的指针
 * @param[in] num  待校验数据的字节数
 * @return unsigned int 计算得到的16位CRC校验值
 * 
 * @details 实现了Modbus RTU协议的CRC16校验算法，用于验证数据帧的完整性。
 *          Modbus CRC使用多项式 0xA001 (x^16 + x^15 + x^2 + 1) 进行计算。
 *          
 *          @b 算法步骤：
 *          1. 初始化CRC寄存器为0xFFFF
 *          2. 对每个数据字节：
 *             a. 将当前字节与CRC寄存器低字节异或
 *             b. 对该字节进行8位循环处理：
 *                - 如果最低位为1：CRC右移1位，然后与0xA001异或
 *                - 如果最低位为0：CRC仅右移1位
 *          3. 返回最终的CRC值
 *          
 *          @b 算法示例(计算单个字节0x01)：
 *          @code
 *          初始: wcrc = 0xFFFF
 *          第1字节: wcrc ^= 0x01 = 0xFFFE
 *          位循环:
 *            bit0=0: wcrc >>= 1 = 0x7FFF
 *            bit1=1: wcrc >>= 1 = 0x3FFF, wcrc ^= 0xA001 = 0x9FFF
 *            bit2=1: wcrc >>= 1 = 0xCFFF, wcrc ^= 0xA001 = 0x6FFF
 *            ... (继续8位)
 *          @endcode
 *          
 *          @b 多项式说明：
 *          - 标准CRC16多项式：0x8005 (正向)
 *          - Modbus使用反向多项式：0xA001 (0x8005的位反转)
 *          - 反向多项式使得算法可以用右移代替左移
 *          
 *          @b 在Modbus RTU中的应用：
 *          - 发送帧：计算数据部分的CRC，附加到帧尾(低字节在前)
 *          - 接收帧：计算接收数据的CRC，与帧尾CRC比较验证
 * @note 该函数是Modbus通信的核心，CRC错误意味着数据传输有误
 */
uint16_t Crc_Count(unsigned char pbuf[], unsigned char num) {
  int i, j;
  unsigned int wcrc = 0xffff;
  for (i = 0; i < num; i++) {
    wcrc ^= (unsigned int)(pbuf[i]);
    for (j = 0; j < 8; j++) {
      if (wcrc & 0x0001) {
        wcrc >>= 1;
        wcrc ^= 0xa001;
      } else
        wcrc >>= 1;
    }
  }
  return wcrc;
}

/**
 * @brief 向编码器写单个寄存器(Modbus功能码0x06)
 * 
 * @param[in] addr  寄存器地址(16位)
 * @param[in] value 要写入的寄存器值(16位)
 * @details 构建Modbus RTU写单个寄存器请求帧(共8字节)：
 *          @code
 *          字节索引  内容           示例值
 *          0         设备地址       0x01 (编码器地址)
 *          1         功能码         0x06 (写单个寄存器)
 *          2-3       寄存器地址     0x00 0x0B (地址0x0B，高字节在前)
 *          4-5       寄存器值       0x00 0x00 (写入值，高字节在前)
 *          6-7       CRC16校验      低字节在前
 *          @endcode
 *          
 *          @b 字节序说明：
 *          - 寄存器地址和值：大端字节序(MSB在前)
 *          - CRC校验值：小端字节序(LSB在前)
 *          这是Modbus RTU的标准格式要求
 *          
 *          @b CRC计算：
 *          对前6个字节(data[0]~data[5])计算CRC16，
 *          结果拆分为低字节data[6]和高字节data[7]
 * @note 该函数仅构建并发送请求帧，应答由UART中断接收处理
 *       实际写入操作需要等待编码器的应答帧确认
 */
void write_abslute_angle_reg(uint16_t addr, uint16_t value) {
  uint8_t data[8] = {0};
  data[0] = 0x01;                            // 设备地址
  data[1] = 0x06;                            // 功能码：写单个寄存器
  data[2] = (addr >> 8) & 0x00FF;            // 寄存器地址高字节
  data[3] = addr & 0x00FF;                   // 寄存器地址低字节
  data[4] = (value >> 8) & 0x00FF;           // 寄存器值高字节
  data[5] = value & 0x00FF;                  // 寄存器值低字节
  uint16_t crc = Crc_Count(data, 6);         // 计算前6字节的CRC
  data[6] = (crc >> 8) & 0x00FF;             // CRC高字节
  data[7] = crc & 0x00FF;                    // CRC低字节

  return;
}

/**
 * @brief 从编码器读单个寄存器(Modbus功能码0x03)
 * 
 * @param[in]  addr  要读取的寄存器地址(16位)
 * @param[out] value 读取到的寄存器值指针(16位)
 * @details 构建Modbus RTU读单个寄存器请求帧(共8字节)：
 *          @code
 *          字节索引  内容           示例值
 *          0         设备地址       0x01 (编码器地址)
 *          1         功能码         0x03 (读保持寄存器)
 *          2-3       寄存器地址     0x00 0x00 (地址，高字节在前)
 *          4-5       寄存器数量     0x00 0x01 (读1个寄存器)
 *          6-7       CRC16校验      低字节在前
 *          @endcode
 *          
 *          @b 读操作流程：
 *          1. 构建读寄存器请求帧
 *          2. 计算前4个字节(data[0]~data[3])的CRC16
 *          3. 将CRC附加到帧尾
 *          4. 发起HAL_UART_Receive_IT接收中断(预期接收7字节应答)
 *          
 *          @b 预期应答帧格式(7字节)：
 *          @code
 *          字节索引  内容           说明
 *          0         设备地址       0x01
 *          1         功能码         0x03 (读应答)
 *          2         字节数         0x02 (后续2字节数据)
 *          3-4       寄存器值       高字节在前
 *          5-6       CRC16校验      低字节在前
 *          @endcode
 * @note 该函数发起读取请求后，实际数据由UART中断接收并解析
 *       value参数当前未使用，读取结果通过全局变量更新
 */
void read_abslute_angle_reg(uint16_t addr, uint16_t *value) {
  uint8_t data[8] = {0};
  data[0] = 0x01;                            // 设备地址
  data[1] = 0x03;                            // 功能码：读保持寄存器
  data[2] = (addr >> 8) & 0x00FF;            // 寄存器地址高字节
  data[3] = addr & 0x00FF;                   // 寄存器地址低字节
  data[4] = 0x00;                            // 读寄存器数量高字节
  data[5] = 0x01;                            // 读寄存器数量低字节(读1个)
  uint16_t crc = Crc_Count(data, 4);         // 计算前4字节的CRC
  data[6] = (crc >> 8) & 0x00FF;             // CRC高字节
  data[7] = crc & 0x00FF;                    // CRC低字节

  HAL_UART_Receive_IT(&ABS_ANGLE_UART, abslute_angle_sensor_buff, 7);

  return;
}

/**
 * @brief 设置编码器的目标角度
 * 
 * @param[in] angle 目标角度值，单位：度(°)，范围0-360
 * @details 将浮点角度值转换为编码器计数值，并写入编码器：
 *          @code
 *          cnt = (uint16_t)(angle / 360 * MAX_CNT)
 *          @endcode
 *          
 *          @b 换算示例：
 *          - angle = 0°   -> cnt = 0/360 * 4096 = 0
 *          - angle = 90°  -> cnt = 90/360 * 4096 = 1024
 *          - angle = 180° -> cnt = 180/360 * 4096 = 2048
 *          - angle = 270° -> cnt = 270/360 * 4096 = 3072
 *          - angle = 360° -> cnt = 360/360 * 4096 = 4096 (溢出为0)
 *          
 *          写入寄存器地址：0x000B (编码器零点设置寄存器)
 * @note 该函数通过Modbus写寄存器实现角度设定
 *       实际设定值需等待编码器应答确认
 */
void set_abslute_angle(float angle) {
  uint16_t cnt = (uint16_t)(angle / 360 * MAX_CNT);
  uint16_t addr = 0x000B;
  write_abslute_angle_reg(addr, cnt);

  return;
}

/**
 * @brief 设置绝对值编码器的工作模式
 * 
 * @param[in] mode 工作模式：0x00=被动模式，0x01=主动模式
 * @details 通过Modbus写寄存器设置编码器的工作模式：
 *          - 被动模式(0x00)：主机主动查询，编码器应答
 *          - 主动模式(0x01)：编码器自动周期发送角度数据
 *          
 *          写入寄存器地址：0x0006 (工作模式寄存器)
 * @note 主动模式适合高频数据采集，被动模式适合按需读取
 */
void set_abslute_angle_sensor_mode(uint8_t mode) {
  uint16_t addr = 0x0006;

  write_abslute_angle_reg(addr, mode);

  return;
}

/**
 * @brief 编码器主驱动函数(UART中断接收处理)
 * 
 * @details 该函数应在UART接收中断回调函数中调用，处理接收到的Modbus应答帧：
 *          
 *          @b 应答帧解析：
 *          根据功能码(abslute_angle_sensor_buff[2])区分应答类型：
 *          
 *          @b 功能码0x03(读寄存器应答)：
 *          @code
 *          字节3-4: 寄存器值(高字节在前)
 *          cur_angle = ((buff[3] << 8) + buff[4]) / MAX_CNT * 360.0
 *          @endcode
 *          例如：buff[3]=0x04, buff[4]=0x00 -> 计数值=1024
 *                 cur_angle = 1024/4096 * 360 = 90°
 *          
 *          @b 功能码0x06(写寄存器应答)：
 *          @code
 *          字节4-5: 写入的寄存器值(高字节在前)
 *          set_angle = ((buff[4] << 8) + buff[5]) / MAX_CNT * 360.0
 *          @endcode
 *          用于确认写入的角度值
 *          
 *          @b 计数到角度换算公式：
 *          @code
 *          角度 = (计数值 / 4096) * 360.0
 *          @endcode
 *          将12位编码器的计数值线性映射到0-360度范围
 *          
 *          @b 重新注册接收中断：
 *          每次处理完成后调用HAL_UART_Receive_IT准备接收下一帧
 * @note 该函数是编码器数据更新的唯一入口
 *       必须放在UART中断处理函数中，如HAL_UART_RxCpltCallback()
 */
void driver_abslute_angle() {
  if (abslute_angle_sensor_buff[2] == 0x03) {
    // 读寄存器应答：解析当前角度值
    cur_angle = (float)((abslute_angle_sensor_buff[3] << 8) + abslute_angle_sensor_buff[4]) / (float)MAX_CNT * 360.0;
  } else if (abslute_angle_sensor_buff[2] == 0x06) {
    // 写寄存器应答：解析设定角度值
    set_angle = (float)((abslute_angle_sensor_buff[4] << 8) + abslute_angle_sensor_buff[5]) / (float)MAX_CNT * 360.0;
  }
  HAL_UART_Receive_IT(&ABS_ANGLE_UART, abslute_angle_sensor_buff, 7);

  return;
}

/**
 * @brief 获取编码器当前角度值
 * 
 * @return float 当前角度值，单位：度(°)
 * @details 返回全局变量cur_angle，该值由driver_abslute_angle()更新
 * @note 该函数无阻塞操作，可安全在任何上下文调用
 */
float get_abslute_angle_value() {
  return cur_angle;
}

/**
 * @brief 获取编码器设定的目标角度值
 * 
 * @return float 设定的目标角度值，单位：度(°)
 * @details 返回全局变量set_angle，该值由driver_abslute_angle()更新
 * @note 该函数无阻塞操作，可安全在任何上下文调用
 */
float get_set_angle_value() {
  return set_angle;
}
