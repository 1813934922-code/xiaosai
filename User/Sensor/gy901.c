/**
 ******************************************************************************
 * @file    gy901.c
 * @brief   GY901陀螺仪I2C驱动实现
 * @details 本文件实现了维特智能GY901 6轴/9轴陀螺仪的完整I2C驱动，包括：
 *          - I2C寄存器配置写入(6轴/9轴模式切换)
 *          - 24字节原始数据批量读取
 *          - 物理量转换(加速度、角速度、姿态角)
 *          - PID角度保持控制器初始化
 * @warning 使用该库时需要开启I2C中断(在STM32CubeMX中配置I2C中断优先级)
 * @note    本文件基于STM32 HAL库的HAL_I2C_Mem_Read/Write函数实现
 ******************************************************************************
 */

// @551

// 警告 使用该库时需要开启I2C中断

#include "gy901.h"

#include "i2c.h"
#include "pid.h"

#define GYR_ADDR 0x50  /**< GY901的7位I2C设备地址 */
#define CALSW 0x01     /**< 校准开关寄存器地址(未使用) */
#define UNLOCK 0x69    /**< 解锁寄存器地址(同REG_UNLOCK) */

/**
 * @brief 初始化GYR结构体
 * 
 * @param[in] gyr 指向GYR结构体的指针
 * @details 执行完整的初始化流程：
 *          1. 设置I2C设备地址：device_addr = 0x50
 *             GY901的默认7位I2C地址，通信时需左移1位(0xA0)
 *          2. 设置数据寄存器起始地址：data_start_addr = 0x34
 *             GY901的数据区从0x34开始，包含加速度、角速度、姿态角
 *          3. 清零数据缓冲区：data_buf[0~23] = 0
 *             确保初始状态无脏数据
 *          4. 初始化目标角度：tar_angle = 0
 *             PID控制器的初始设定值为0度
 *          5. 初始化PID控制器：
 *             - Kp=1：比例增益，误差1度时输出1
 *             - Ki=0：积分增益为0(不使用积分)
 *             - Kd=0：微分增益为0(不使用微分)
 *             - 积分限幅=10：防止积分饱和
 *             - 输出限幅=500：限制PID输出范围
 * @note 该函数应在系统初始化阶段调用一次
 *       PID参数可根据实际控制效果调整
 */
void init_gyr(GYR *gyr) {
  gyr->device_addr = GYR_ADDR;
  gyr->data_start_addr = 0x34;
  for (int i = 0; i < 24; i++) {
    gyr->data_buf[i] = 0;
  }
  gyr->tar_angle = 0;
  gyr->gy901_keep_angle_pid = init_pid(1, 0, 0, 10, 500);
  return;
}

/**
 * @brief 读取GY901陀螺仪的24字节原始数据
 * 
 * @param[in] i2c 指向I2C_HandleTypeDef的指针
 * @param[in] gyr 指向GYR结构体的指针
 * @details 通过I2C内存读操作一次性读取24字节数据：
 *          @code
 *          HAL_I2C_Mem_Read(
 *            i2c,                              // I2C句柄
 *            (uint16_t)(GYR_ADDR << 1),        // 设备地址(7位左移1位=8位写地址)
 *            gyr->data_start_addr,             // 寄存器起始地址(0x34)
 *            I2C_MEMADD_SIZE_8BIT,             // 寄存器地址长度为8位
 *            gyr->data_buf,                    // 数据接收缓冲区
 *            24,                               // 读取24字节
 *            10                                // 超时时间10ms
 *          );
 *          @endcode
 *          
 *          @b 数据布局(data_buf[0~23])：
 *          @code
 *          字节   内容              说明
 *          0-1    加速度X轴低/高字节
 *          2-3    加速度Y轴低/高字节
 *          4-5    加速度Z轴低/高字节
 *          6-7    角速度X轴低/高字节
 *          8-9    角速度Y轴低/高字节
 *          10-11  角速度Z轴低/高字节
 *          12-13  保留
 *          14-15  滚转角(Roll)低/高字节
 *          16-17  俯仰角(Pitch)低/高字节
 *          18-19  偏航角(Yaw)低/高字节
 *          20-23  保留/温度等
 *          @endcode
 * @note 使用HAL_I2C_Mem_Read可一次性读取连续寄存器，效率高
 *       超时10ms对于I2C通信已足够，如频繁超时需检查硬件连接
 */
void get_gyr_raw_data(I2C_HandleTypeDef *i2c, GYR *gyr) {
  HAL_I2C_Mem_Read(i2c, (uint16_t)(GYR_ADDR << 1), gyr->data_start_addr, I2C_MEMADD_SIZE_8BIT, gyr->data_buf, 24, 10);

  return;
}

/**
 * @brief 将原始寄存器数据转换为实际物理量
 * 
 * @param[in] gyr 指向GYR结构体的指针
 * @param[in] key 要读取的传感器数据枚举值(见gyroscope枚举)
 * @return float 转换后的物理量值(加速度m/s²、角速度°/s、角度°)
 * 
 * @details 从data_buf中提取对应寄存器的16位有符号值，并根据传感器类型
 *          应用不同的转换公式。所有数据均为小端字节序(Low Byte在前)。
 *          
 *          @b 字节偏移计算：
 *          @code cnt = (key - data_start_addr) * 2 @endcode
 *          因为key是寄存器地址(每寄存器2字节)，需转换为字节数组偏移。
 *          例如：gyr_x_roll=0x3D，data_start_addr=0x34
 *                cnt = (0x3D - 0x34) * 2 = 0x09 * 2 = 18 (字节偏移)
 *          
 *          @b 数据提取(小端字节序)：
 *          @code value = (short)((data_buf[cnt+1] << 8) | data_buf[cnt]) @endcode
 *          高字节在前，低字节在后，组合成16位有符号整数。
 *          
 *          @b 加速度转换(gyr_a_x/y/z)：
 *          @code value = raw * 16 * 9.8  (单位：m/s²) @endcode
 *          - 量程：±16g (g=9.8m/s²)
 *          - 分辨率：16位有符号(-32768 ~ +32767)
 *          - 灵敏度：16*9.8/32768 ≈ 0.00478 m/s²/LSB
 *          - 示例：raw=1000 -> value=1000*16*9.8=156800 m/s² (约16g)
 *          
 *          @b 角速度转换(gyr_w_x/y/z)：
 *          @code value = raw / 2000  (单位：°/s) @endcode
 *          - 量程：±2000°/s
 *          - 分辨率：16位有符号
 *          - 灵敏度：1/2000 = 0.0005 °/s/LSB
 *          - 示例：raw=4000 -> value=4000/2000=2°/s
 *          
 *          @b 姿态角转换(gyr_x_roll/y_pitch/z_yaw)：
 *          @code value = raw * 180 / 32768  (单位：度) @endcode
 *          - 量程：±180°
 *          - 分辨率：16位有符号
 *          - 灵敏度：180/32768 ≈ 0.00549 °/LSB
 *          - 示例：raw=32768 -> value=32768*180/32768=180°
 * 
 * @note 该函数假设传感器已正确配置量程，如果量程不同需调整转换系数
 *       返回值单位为国际标准单位或常用工程单位
 */
float get_gyr_value(GYR *gyr, enum gyroscope key) {
  uint8_t cnt = (key - gyr->data_start_addr) * 2;
  float value = (short)(((short)gyr->data_buf[cnt + 1] << 8) | gyr->data_buf[cnt]);

  switch (key) {
    case gyr_a_x:
    case gyr_a_y:
    case gyr_a_z:
      return value * 16 * 9.8;
    case gyr_w_x:
    case gyr_w_y:
    case gyr_w_z:
      return value / 2000;
    case gyr_x_roll:
    case gyr_y_pitch:
    case gyr_z_yaw:
      return value * 180 / 32768;
  }
}

/**
 * @brief 向GY901写入16位寄存器配置值
 * 
 * @param[in] i2c 指向I2C_HandleTypeDef的指针
 * @param[in] reg 寄存器地址(8位)
 * @param[in] data 16位配置数据(自动处理小端字节序)
 * @details 该函数用于配置GY901的工作模式，内部自动处理小端字节序：
 *          @code
 *          buf[0] = data & 0xFF;          // 低字节 DataL
 *          buf[1] = (data >> 8) & 0xFF;   // 高字节 DataH
 *          HAL_I2C_Mem_Write(i2c, addr, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
 *          @endcode
 *          
 *          @b 小端字节序说明：
 *          维特智能GY901要求先发送低字节(LSB)，再发送高字节(MSB)。
 *          例如：写入0xB588解锁码
 *          - buf[0] = 0x88 (低字节)
 *          - buf[1] = 0xB5 (高字节)
 *          I2C总线发送顺序：0x88, 0xB5
 *          
 *          @b 参数说明：
 *          - GYR_ADDR << 1: 7位I2C地址左移1位得到8位写地址(0xA0)
 *          - I2C_MEMADD_SIZE_8BIT: 寄存器地址长度为8位
 *          - buf, 2: 发送2字节数据
 *          - 100: 超时时间100ms
 * @note 该函数是配置寄存器的基础函数，write_gyr_reg()内部调用
 *       超时时间100ms已足够，如频繁超时需检查I2C总线
 */
void write_gyr_reg(I2C_HandleTypeDef *i2c, uint8_t reg, uint16_t data) {
  uint8_t buf[2];
  buf[0] = data & 0xFF;         // 低字节 DataL
  buf[1] = (data >> 8) & 0xFF;  // 高字节 DataH

  // I2C_MEMADD_SIZE_8BIT 表示寄存器地址RegAddr是8位的
  HAL_I2C_Mem_Write(i2c, (uint16_t)(GYR_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
}

/**
 * @brief 设置陀螺仪为6轴算法模式
 * 
 * @param[in] i2c 指向I2C_HandleTypeDef的指针
 * @details 完整的6轴模式配置流程(必须按顺序执行)：
 *          
 *          @b 步骤1：解锁配置寄存器
 *          @code write_gyr_reg(i2c, REG_UNLOCK, UNLOCK_CMD); @endcode
 *          - 向寄存器0x69写入解锁码0xB588
 *          - 这是维特智能的安全机制，防止误修改配置
 *          - HAL_Delay(50ms)：等待芯片内部解锁完成
 *          
 *          @b 步骤2：写入算法模式
 *          @code write_gyr_reg(i2c, REG_AXIS6, MODE_6AXIS); @endcode
 *          - 向寄存器0x24写入0x0001(6轴模式)
 *          - 6轴模式：加速度计+陀螺仪融合(不用磁力计)
 *          - 9轴模式：加速度计+陀螺仪+磁力计融合
 *          - HAL_Delay(50ms)：等待芯片内部算法切换完成
 *          
 *          @b 步骤3：保存配置到Flash
 *          @code write_gyr_reg(i2c, REG_SAVE, SAVE_CMD); @endcode
 *          - 向寄存器0x00写入0x0000(保存指令)
 *          - 将当前配置写入内部Flash，掉电不丢失
 *          - HAL_Delay(100ms)：Flash写入操作较慢，需要更长等待时间
 *          
 *          @b 6轴 vs 9轴模式选择：
 *          | 特性        | 6轴模式                    | 9轴模式                      |
 *          |------------|---------------------------|-----------------------------|
 *          | 使用传感器   | 加速度+陀螺仪              | 加速度+陀螺仪+磁力计         |
 *          | 航向角来源   | 陀螺仪积分(相对角度)        | 磁力计绝对方向(绝对角度)     |
 *          | 磁场干扰     | 不受影响                   | 容易受干扰                   |
 *          | 长期漂移     | 有累积误差                 | 无累积误差(磁力计校正)       |
 *          | 适用场景     | 室内、电机附近              | 室外、开阔环境               |
 *          
 * @note 该函数通常在系统初始化时调用一次
 *       如需切换回9轴模式，将MODE_6AXIS改为MODE_9AXIS(0x0000)
 *       延时时间不可省略，否则配置可能不生效
 */
void set_gyr_6axis_mode(I2C_HandleTypeDef *i2c) {
  // 1. 解锁配置寄存器
  write_gyr_reg(i2c, REG_UNLOCK, UNLOCK_CMD);
  HAL_Delay(50); // 必须给出一定延时让芯片内部处理

  // 2. 写入算法模式 (0x01 表示 6轴，0x00 表示 9轴)
  write_gyr_reg(i2c, REG_AXIS6, MODE_6AXIS);
  HAL_Delay(50);

  // 3. 保存配置到Flash (掉电不丢失)
  write_gyr_reg(i2c, REG_SAVE, SAVE_CMD);
  HAL_Delay(100); // 保存操作耗时较长，给够100ms
}
