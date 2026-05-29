/**
 ******************************************************************************
 * @file    timer_it.h
 * @brief   定时器中断与任务调度头文件
 *
 * @details 本文件定义了定时器中断相关的公共接口、枚举类型和外部变量声明。
 *          主要用于：
 *          - 任务启动与电机控制接口
 *          - 陀螺仪转向模式定义（圆弧弯/原地打转/单轮转弯）
 *          - 陀螺仪测试状态机枚举（用于调试陀螺仪PID锁死功能）
 *          - 跨模块访问陀螺仪测试变量的外部声明（供button.c使用）
 *
 * @note    timer_it.c中的全局测试变量通过extern声明暴露给button.c，
 *          使得按钮中断可以触发/停止陀螺仪测试程序。
 ******************************************************************************
 */

#ifndef XIAOSAI_TIMER_IT_H
#define XIAOSAI_TIMER_IT_H

#include <stdint.h>

/**
 * @defgroup 任务控制接口
 * @{
 */

/**
 * @brief  启动任务（触发1秒倒计时）
 * @details 被调用后系统进入COUNTDOWN状态，1秒后自动开始执行巡线任务。
 *          同时触发蜂鸣器报警和全LED亮起。
 */
void start_task(void);

/**
 * @brief  开启蜂鸣器（提示音）
 * @details 检测到路口、启动任务或到达终点时调用。
 */
void beep(void);

/**
 * @brief  停止电机
 * @details 将左右轮目标速度设为0，用于路口停车、任务完成等场景。
 */
void stop_motor(void);

/** @} */

/**
 * @defgroup 陀螺仪转向模式
 * @brief 定义三种陀螺仪控制的转弯方式
 * @{
 */

/** @brief 圆弧弯模式 - 双轮以基速前进，通过差速实现弧线转弯 */
#define GYRO_TURN_ARC 0

/** @brief 原地打转模式 - 两轮反向旋转，实现原地转向（如U-turn掉头） */
#define GYRO_TURN_PIVOT 1

/** @brief 单轮转弯模式 - 内侧轮减速、外侧轮加速，适合直角转弯 */
#define GYRO_TURN_SINGLE 2

/** @} */

/**
 * @brief 陀螺仪测试状态机
 *
 * @details 状态流转：
 *          GYRO_TEST_IDLE → GYRO_TEST_START → GYRO_TEST_HOLDING → GYRO_TEST_DONE
 *          - IDLE:    静止状态，电机停止
 *          - START:   过渡状态，自动抓取当前陀螺仪角度作为基准，清空PID积分
 *          - HOLDING: 持续锁死状态，angle_handler持续运行维持目标角度
 *          - DONE:    手动停止，电机停止
 *
 * @note    用于调试陀螺仪PID控制参数，验证角度闭环稳定性
 */
typedef enum {
    GYRO_TEST_IDLE = 0,     /**< 静止状态，电机停止 */
    GYRO_TEST_START = 1,    /**< 自动抓取初始角度的过渡状态 */
    GYRO_TEST_HOLDING = 2,  /**< 持续保持（锁死）状态 */
    GYRO_TEST_DONE = 3      /**< 手动停止 */
} GYRO_TEST_STATE;

/**
 * @defgroup 陀螺仪测试外部变量
 * @brief 供button.c访问的测试变量声明
 * @{
 */

/** @brief 陀螺仪测试状态机当前状态 */
extern GYRO_TEST_STATE gyro_test_state;

/** @brief 测试目标角度（正=左转角度增大，负=右转角度减小） */
extern float gyro_test_tar;

/** @brief 测试转向模式（0=GYRO_TURN_ARC, 1=GYRO_TURN_PIVOT, 2=GYRO_TURN_SINGLE） */
extern uint8_t gyro_test_mode;

/** @brief 陀螺仪转向起始角度（抓取自gyr_z_yaw轴） */
extern float gyro_start_angle;

/** @} */

#endif //XIAOSAI_TIMER_IT_H
