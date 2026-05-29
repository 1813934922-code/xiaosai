#include "status.h"

#include "log.h"
#include "math_tool.h"
#include "status.h"
#include "wheel.h"
#include "uart_it.h"
#include "gy901.h"
#include "gw_anagloge.h"
#include "buzzer.h"
#include "i2c.h"
#include "pendulum.h"
STATUS status;

PID balance_pid;
PID speed_pid;
PID yaw_pid;
PID find_line_pid;

extern PID gyro_turn_pid;


void init_motor() {
  init_wheel(&status.motor.wheel[0], 1, 1);
  init_wheel(&status.motor.wheel[1], 2, -1);

  return;
}

void init_device() {
  init_button(&status.device.button1, 1, 0);
  init_button(&status.device.button2, 2, 0);
  init_LED(&status.device.led1, 1, 1);
  init_LED(&status.device.led2, 2, 1);
  init_LED(&status.device.led3, 3, 1);
  init_BUZZER(&status.device.buzzer1,1,1);
  return;
}

void init_sensor(STATUS *status) {
  init_gw_analogue(&status->sensor.gw_analogue);
  init_gyr(&status->sensor.gyr);
}

void init_state(STATUS *status, uint8_t T) {
  status->state.T = T;
  status->state.time = 0;

  return;
}

void init_status(STATUS *status, uint8_t T) {
  init_state(status, T);

  init_sensor(status);

  init_motor();

  init_device();

  init_uart_idle_it();

  gyro_turn_pid = init_pid(0.5f, 0.0f, 10.0f, 10.0f, 200.0f);

  set_gyr_6axis_mode(&hi2c1);

  init_pendulum();

  return;
}

void driver_status(STATUS *status) {
    update_task_clock(&status->task, status->state.time);
    driver_button(&status->device.button1);
    driver_button(&status->device.button2);
    driver_LED(&status->device.led1);
    driver_LED(&status->device.led2);
    driver_LED(&status->device.led3);
    //driver_wheel(&status->motor.wheel[0]);
    //driver_wheel(&status->motor.wheel[1]);
    //driver_gw_analogue(&status->sensor.gw_analogue);
    driver_BUZZER(&status->device.buzzer1);
    //get_gyr_raw_data(&hi2c1,&status->sensor.gyr);
    //driver_gyr(&status->sensor.gyr);
}
