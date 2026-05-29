/**
 ******************************************************************************
 * @file    gy901.h
 * @brief   GY901陀螺仪传感器驱动头文件
 * @details 本文件定义了维特智能GY901 6轴/9轴陀螺仪的I2C驱动接口。
 *          GY901是一款集成加速度计、陀螺仪、磁力计的9轴运动传感器，
 *          支持6轴(加速度+角速度)和9轴(加速度+角速度+磁场)两种算法模式。
 *          本驱动通过I2C协议读取传感器寄存器数据，提供角度、角速度、加速度等物理量。
 * @warning 使用该库时需要开启I2C中断(在STM32CubeMX中配置I2C中断优先级)
 * @note    传感器I2C地址：0x50 (7位地址)，数据寄存器起始地址：0x34
 ******************************************************************************
 */

// @551

// 警告 使用该库时需要开启I2C中断

#ifndef __GYROSCOPE_H__
#define __GYROSCOPE_H__

#include "main.h"
#include "pid.h"

/**
 * @addtogroup Sensor_Drivers
 * @{
 */

/**
 * @addtogroup GY901_Gyroscope
 * @brief GY901陀螺仪I2C驱动
 * @{
 */

/**
 * @defgroup GY901_Register_Definitions GY901寄存器定义
 * @brief GY901配置寄存器的地址和功能定义
 * 
 * @details 维特智能GY901传感器的关键寄存器：
 *          - REG_SAVE (0x00): 保存配置到Flash，写入后配置掉电不丢失
 *          - REG_AXIS6 (0x24): 算法模式寄存器，控制6轴/9轴切换
 *          - REG_UNLOCK (0x69): 解锁寄存器，必须先写入解锁码才能修改配置
 * 
 * @note 修改配置寄存器前必须先写入UNLOCK_CMD解锁码，修改完成后写入SAVE_CMD保存
 * @{
 */

#define GYR_ADDR 0x50  /**< GY901的I2C设备地址(7位地址)，实际通信时需左移1位 */

#define REG_SAVE      0x00  /**< 保存寄存器：写入SAVE_CMD将当前配置保存到Flash */
#define REG_AXIS6     0x24  /**< 算法寄存器：控制6轴/9轴算法模式切换 */
#define REG_UNLOCK    0x69  /**< 解锁寄存器：必须先写入解锁码才能修改配置寄存器 */

#define UNLOCK_CMD    0xB588 /**< 维特智能官方解锁码(固定为0xB588)，小端模式传输 */
#define MODE_9AXIS    0x0000 /**< 9轴算法模式：使用加速度计+陀螺仪+磁力计融合解算 */
#define MODE_6AXIS    0x0001 /**< 6轴算法模式：仅使用加速度计+陀螺仪融合解算 */
#define SAVE_CMD      0x0000 /**< 保存指令：写入REG_SAVE寄存器，将配置永久保存到Flash */

/** @} */

/**
 * @brief 向GY901写入16位寄存器配置值
 * 
 * @param[in] i2c 指向I2C_HandleTypeDef的指针
 * @param[in] reg 寄存器地址(8位)
 * @param[in] data 16位配置数据(小端模式：低字节先发)
 * @details 该函数用于配置GY901的工作模式，会自动处理小端字节序：
 *          先发送低字节(DataL)，再发送高字节(DataH)
 * @note 必须在写入配置寄存器前先调用此函数向REG_UNLOCK写入UNLOCK_CMD
 */
void write_gyr_reg(I2C_HandleTypeDef *i2c, uint8_t reg, uint16_t data);

/**
 * @brief 设置陀螺仪为6轴算法模式
 * 
 * @param[in] i2c 指向I2C_HandleTypeDef的指针
 * @details 完整配置流程：
 *          1. 向REG_UNLOCK写入UNLOCK_CMD解锁配置
 *          2. 延时50ms等待芯片内部处理
 *          3. 向REG_AXIS6写入MODE_6AXIS(0x0001)
 *          4. 延时50ms等待模式切换完成
 *          5. 向REG_SAVE写入SAVE_CMD保存配置
 *          6. 延时100ms等待Flash写入完成
 * @note 6轴模式不使用磁力计，适合室内或磁场干扰较大的环境
 *       9轴模式使用磁力计，可提供绝对航向角，但易受磁场干扰
 */
void set_gyr_6axis_mode(I2C_HandleTypeDef *i2c);

/**
 * @struct GYR
 * @brief GY901陀螺仪数据结构体
 * 
 * @details 该结构体挂载于全局status.sensor下，用于管理GY901传感器的所有状态：
 *          - data_buf: 存储从I2C读取的24字节原始数据
 *          - device_addr: I2C设备地址(默认0x50)
 *          - data_start_addr: 数据寄存器起始地址(默认0x34)
 *          - gy901_keep_angle_pid: 用于保持目标角度的PID控制器
 *          - tar_angle: 目标角度设定值
 * 
 * @b 数据寄存器布局(从0x34开始，共24字节)：
 * @code
 * 偏移  寄存器地址  内容              量程              分辨率
 * 0-1   0x34-0x35   加速度X轴        ±16g              16位有符号
 * 2-3   0x36-0x37   加速度Y轴        ±16g              16位有符号
 * 4-5   0x38-0x39   加速度Z轴        ±16g              16位有符号
 * 6-7   0x3A-0x3B   角速度X轴        ±2000°/s          16位有符号
 * 8-9   0x3C-0x3D   角速度Y轴        ±2000°/s          16位有符号
 * 10-11 0x3E-0x3F   角速度Z轴        ±2000°/s          16位有符号
 * 12-13 0x40-0x41   滚转角(Roll)     ±180°             16位有符号
 * 14-15 0x42-0x43   俯仰角(Pitch)    ±180°             16位有符号
 * 16-17 0x44-0x45   偏航角(Yaw)      ±180°             16位有符号
 * @endcode
 * @note 每次读取24字节可获取完整的加速度、角速度、姿态角数据
 */
typedef struct GYR {
  uint8_t data_buf[24];      /**< 读取数据暂存区：存储从I2C一次性读取的24字节原始数据 */
  uint8_t device_addr;       /**< I2C设备地址：GY901的7位I2C地址，默认0x50 */
  uint8_t data_start_addr;   /**< 数据寄存器起始地址：GY901数据区首地址，默认0x34 */
  PID gy901_keep_angle_pid;  /**< 保持角度PID控制器：用于控制小车维持目标角度 */
  float tar_angle;           /**< 目标角度：PID控制器的设定值(单位：度) */
} GYR;

/**
 * @enum gyroscope
 * @brief GY901传感器数据寄存器地址枚举
 * 
 * @details 定义了GY901各数据寄存器的地址偏移(以寄存器地址为单位)：
 *          实际访问时需转换为字节偏移：byte_offset = (reg_addr - start_addr) * 2
 *          
 *          @b 寄存器分组：
 *          - 加速度组(0x34-0x36): 三轴加速度，量程±16g，分辨率16位
 *          - 角速度组(0x37-0x39): 三轴角速度，量程±2000°/s，分辨率16位
 *          - 姿态角组(0x3D-0x3F): 三轴欧拉角，量程±180°，分辨率16位
 * 
 * @note 枚举值对应的是16位寄存器地址，不是字节地址
 *       使用时需通过(key - data_start_addr)*2计算字节偏移
 */
enum gyroscope {
  gyr_a_x = 0x34,      /**< X轴加速度寄存器地址，单位：g (1g=9.8m/s²) */
  gyr_a_y = 0x35,      /**< Y轴加速度寄存器地址，单位：g (1g=9.8m/s²) */
  gyr_a_z = 0x36,      /**< Z轴加速度寄存器地址，单位：g (1g=9.8m/s²) */
  gyr_w_x = 0x37,      /**< X轴角速度寄存器地址(滚转角速度)，单位：°/s */
  gyr_w_y = 0x38,      /**< Y轴角速度寄存器地址(俯仰角速度)，单位：°/s */
  gyr_w_z = 0x39,      /**< Z轴角速度寄存器地址(偏航角速度)，单位：°/s */
  gyr_x_roll = 0x3D,   /**< X轴滚转角寄存器地址(Roll)，单位：度(°) */
  gyr_y_pitch = 0x3E,  /**< Y轴俯仰角寄存器地址(Pitch)，单位：度(°) */
  gyr_z_yaw = 0x3F,    /**< Z轴偏航角寄存器地址(Yaw)，单位：度(°) */
};

/** @} */
/** @} */

/**
 * @brief 读取GY901陀螺仪的原始数据
 * 
 * @param[in] i2c 指向I2C_HandleTypeDef的指针
 * @param[in] gyr 指向GYR结构体的指针
 * @details 通过I2C协议从GY901一次性读取24字节数据：
 *          1. 设备地址：GYR_ADDR << 1 (左移1位得到8位写地址)
 *          2. 寄存器地址：data_start_addr (默认0x34)
 *          3. 读取长度：24字节
 *          4. 超时时间：10ms
 *          数据存储到gyr->data_buf[]数组中
 * @note 该函数应在status_update()等周期性任务中调用
 *       需要开启I2C中断以获得最佳性能
 */
void get_gyr_raw_data(I2C_HandleTypeDef *i2c, GYR *gyr);

/**
 * @brief 将原始寄存器数据转换为实际物理量
 * 
 * @param[in] gyr 指向GYR结构体的指针
 * @param[in] key 要读取的传感器数据枚举值(见gyroscope枚举)
 * @return float 转换后的物理量值
 * 
 * @details 从data_buf中提取对应寄存器的16位有符号值，并根据类型转换：
 *          @b 加速度(gyr_a_x/y/z)：
 *          @code value = raw * 16 * 9.8  (单位：m/s²) @endcode
 *          量程±16g，分辨率16位，灵敏度=16*9.8/32768≈0.00478 m/s²/LSB
 *          
 *          @b 角速度(gyr_w_x/y/z)：
 *          @code value = raw / 2000  (单位：°/s) @endcode
 *          量程±2000°/s，分辨率16位，灵敏度=1/2000=0.0005 °/s/LSB
 *          
 *          @b 姿态角(gyr_x_roll/y_pitch/z_yaw)：
 *          @code value = raw * 180 / 32768  (单位：度) @endcode
 *          量程±180°，分辨率16位，灵敏度=180/32768≈0.00549 °/LSB
 * @note 返回值的小端字节序由data_buf的存储顺序保证
 */
float get_gyr_value(GYR *gyr, enum gyroscope key);

/**
 * @brief 初始化GYR结构体
 * 
 * @param[in] gyr 指向GYR结构体的指针
 * @details 初始化包括：
 *          1. 设置I2C设备地址为0x50
 *          2. 设置数据寄存器起始地址为0x34
 *          3. 清零data_buf数组(24字节)
 *          4. 初始化目标角度为0
 *          5. 初始化PID控制器(kp=1, ki=0, kd=0, 积分限幅10, 输出限幅500)
 * @note 该函数应在init_sensor()等系统初始化函数中调用
 */
void init_gyr(GYR *gyr);

#endif /* !__GYROSCOPE_H__ */
