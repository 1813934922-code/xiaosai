// @551

// 警告 使用该库时需要开启I2C中断

#include "gy901.h"

#include "i2c.h"
#include "pid.h"

#define GYR_ADDR 0x50
#define CALSW 0x01
#define UNLOCK 0x69

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

void get_gyr_raw_data(I2C_HandleTypeDef *i2c, GYR *gyr) {
  HAL_I2C_Mem_Read(i2c, (uint16_t)(GYR_ADDR << 1), gyr->data_start_addr, I2C_MEMADD_SIZE_8BIT, gyr->data_buf, 24, 10);

  return;
}

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

// 封装的 IIC 写入寄存器函数
// data参数为16位数据，函数内部自动处理 先低字节(DataL) 后高字节(DataH) 的小端模式
void write_gyr_reg(I2C_HandleTypeDef *i2c, uint8_t reg, uint16_t data) {
  uint8_t buf[2];
  buf[0] = data & 0xFF;         // 低字节 DataL
  buf[1] = (data >> 8) & 0xFF;  // 高字节 DataH

  // I2C_MEMADD_SIZE_8BIT 表示寄存器地址RegAddr是8位的
  HAL_I2C_Mem_Write(i2c, (uint16_t)(GYR_ADDR << 1), reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
}

// 设置陀螺仪为6轴算法模式
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

