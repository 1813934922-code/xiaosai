// @551

// 警告 使用该库时需要开启I2C中断

#ifndef __GYROSCOPE_H__
#define __GYROSCOPE_H__

#include "main.h"
#include "pid.h"


// 在 #define GYR_ADDR 0x50 下面添加寄存器地址和配置值宏定义
#define REG_SAVE      0x00  // 保存寄存器
#define REG_AXIS6     0x24  // 算法寄存器 (0x24)
#define REG_UNLOCK    0x69  // 解锁寄存器 (0x69)

#define UNLOCK_CMD    0xB588 // 维特智能官方解锁码 (固定为0x88B5)
#define MODE_9AXIS    0x0000 // 9轴算法
#define MODE_6AXIS    0x0001 // 6轴算法
#define SAVE_CMD      0x0000 // 保存指令

#define REG_CALSW     0x01   // 校准模式寄存器
#define CAL_NORMAL    0x00   // 正常工作模式
#define CAL_ANGLE_REF 0x08   // 设置角度参考模式

// 新增函数声明
void write_gyr_reg(I2C_HandleTypeDef *i2c, uint8_t reg, uint16_t data);
void set_gyr_6axis_mode(I2C_HandleTypeDef *i2c);
// GYR结构体
// 挂载于status sensor
// 用于驱动gy901
typedef struct GYR {
  uint8_t data_buf[24];      // 读取数据暂存
  uint8_t device_addr;       // 设备iic地址 默认0xa1
  uint8_t data_start_addr;   // gy901数据寄存器起始地址 默认0x34
  PID gy901_keep_angle_pid;  // 陀螺仪保持角度PID
  float tar_angle;
} GYR;

enum gyroscope {
  gyr_a_x = 0x34,      // Acceleration of the sensor along the x-axis
  gyr_a_y = 0x35,      // Acceleration of the sensor along the y-axis
  gyr_a_z = 0x36,      // Acceleration of the sensor along the z-axis
  gyr_w_x = 0x37,      // The angular velocity of the sensor around the x-axis
  gyr_w_y = 0x38,      // The angular velocity of the sensor around the y-axis
  gyr_w_z = 0x39,      // The angular velocity of the sensor around the z-axis
  gyr_x_roll = 0x3D,   // The angle of the sensor around the x-axis
  gyr_y_pitch = 0x3E,  // The angle of the sensor around the y-axis
  gyr_z_yaw = 0x3F,    // The angle of the sensor around the z-axis
};

// 读取gy901的原始数据 放在status_update()中
void get_gyr_raw_data(I2C_HandleTypeDef *i2c, GYR *gyr);
// 将原始数据转化为实际物理量 key传入参数枚举 gyroscope见上
float get_gyr_value(GYR *gyr, enum gyroscope key);
// 初始化gyr 放在init_sensor()中
void init_gyr(GYR *gyr);

void set_gyr_angle_reference(I2C_HandleTypeDef *i2c);

// 在 #define REG_CALSW 0x01 下面添加：
#define REG_RRATE     0x03   // 输出速率寄存器
#define RATE_200HZ    0x0B   // 200Hz 回传速率配置值

// 在文件底部的函数声明区添加：
void set_gyr_rate_200hz(I2C_HandleTypeDef *i2c);

#endif /* !__GYROSCOPE_H__ */
