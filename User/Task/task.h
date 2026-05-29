#ifndef __TASK_H_
#define __TASK_H_

#include <stdbool.h>
#include <stdint.h>

#include "para.h"

/**
 * @defgroup TaskController 任务控制器模块
 * @brief 基于条件触发和超时机制的任务调度系统
 * 
 * 该模块实现了一个轻量级的任务调度器，支持：
 * - 条件触发：当监控变量达到目标值时执行任务
 * - 超时保护：任务等待超过设定时间后强制执行
 * - 泛型支持：通过TYPE枚举支持多种数据类型的条件变量
 * - 队列管理：最多支持MAX_TASK_NUM个任务并发等待
 * 
 * @{
 */

/** @brief 任务队列最大容量 */
#define MAX_TASK_NUM 40

/**
 * @struct task_cell
 * @brief 任务单元结构体，封装单个任务的所有信息
 * 
 * 每个task_cell包含：
 * - 任务函数指针及参数
 * - 条件判断函数及监控变量
 * - 超时时间戳
 * 
 * 工作流程：
 * 1. 通过add_task()创建任务单元加入等待队列
 * 2. driver_task()轮询检查每个等待中的任务
 * 3. 读取cur_addr指向的变量当前值，与tar比较
 * 4. 若judge()返回true或超时，则执行task()并移除任务
 */
typedef struct task_cell {
  void (*task)(void* para);             /**< @brief 任务函数指针，条件满足时执行 */
  bool (*judge)(float cur, float tar);  /**< @brief 条件判断函数，传入当前值和目标值，返回是否满足执行条件 */
  void* para;                           /**< @brief 传递给任务函数的参数指针 */
  float cur;                            /**< @brief 条件变量的当前采样值（由driver_task更新） */
  float tar;                            /**< @brief 条件触发目标值 = 初始值 + offset偏移量 */
  TYPE type;                            /**< @brief 条件变量的数据类型，用于get_cur_val()类型萃取 */
  void* cur_addr;                       /**< @brief 条件变量的内存地址，用于实时读取当前值 */
  int32_t timeout;                      /**< @brief 任务超时时间戳（绝对时间），<=0表示无超时保护 */
} task_cell;

/**
 * @struct TASK
 * @brief 任务控制器主结构体，管理整个任务队列
 * 
 * 包含任务槽位数组、等待状态标记和系统时钟。
 * 使用前必须调用init_TASK()进行初始化。
 */
typedef struct TASK {
  uint32_t task_num;             /**< @brief 当前等待执行的任务数量 */
  bool waiting[MAX_TASK_NUM];    /**< @brief 任务等待状态标记：true=等待执行，false=已执行/空闲槽位 */
  task_cell cell[MAX_TASK_NUM];  /**< @brief 任务单元数组，存储所有注册的任务 */
  uint64_t task_clock;           /**< @brief 任务系统时钟（ms），由update_task_clock()同步，用于超时检测 */
} TASK;

/**
 * @brief 向任务控制器添加一个条件触发任务
 * 
 * 该函数创建一个任务单元并加入等待队列。任务将在以下任一条件满足时执行：
 * 1. judge(cur_addr的值, tar) 返回 true
 * 2. 当前系统时钟 >= timeout 时间戳
 * 
 * @param ctrl       任务控制器指针
 * @param task       任务函数指针，签名: void task(void* para)
 * @param para       传递给任务函数的参数
 * @param judge      条件判断函数，签名: bool judge(float cur, float tar)
 *                   cur为cur_addr实时读取的值，tar为get_cur_val(cur_addr)+offset
 * @param cur_addr   被监控变量的地址，用于实时采样当前值
 * @param offset     目标值偏移量，tar = get_cur_val(cur_addr) + offset
 * @param type       被监控变量的数据类型，用于正确解析内存值
 * @param timeout    超时时间（ms），从当前task_clock开始的相对时间
 * @return true      任务添加成功
 * @return false     任务队列已满或参数无效
 * 
 * @note 超时时间设为0表示无超时保护，任务将无限等待直到条件满足
 * @see driver_task() - 驱动任务执行
 * @see find_unused_cell() - 查找空闲任务槽位
 */
bool add_task(TASK* ctrl, void (*task)(void* para), void* para, bool(*judge), void* cur_addr, float offset, TYPE type, uint32_t timeout);

/**
 * @brief 驱动任务控制器执行轮询检查
 * 
 * 该函数应被周期性调用（通常在主循环或定时器回调中）。
 * 遍历所有等待中的任务单元，执行以下操作：
 * 1. 通过get_cur_val()读取条件变量当前值
 * 2. 调用judge()判断是否满足执行条件
 * 3. 调用is_timeout()检查是否超时
 * 4. 若任一条件满足，则执行task()并将任务标记为已完成
 * 
 * @param ctrl 任务控制器指针
 * 
 * @note 该函数会修改ctrl->task_num和waiting数组状态
 * @note 必须在调用前通过update_task_clock()更新系统时钟
 * @see add_task() - 添加任务
 * @see update_task_clock() - 更新时钟
 */
void driver_task(TASK* ctrl);

/**
 * @brief 初始化任务控制器
 * 
 * 将所有任务槽位标记为空闲，清零计数器。
 * 使用TASK结构体前必须调用此函数。
 * 
 * @param ctrl 任务控制器指针
 */
void init_TASK(TASK* ctrl);

/**
 * @brief 更新任务控制器的系统时钟
 * 
 * 将外部系统时间同步到任务控制器，用于超时检测。
 * 应在每次系统tick或主循环开始时调用。
 * 
 * @param ctrl        任务控制器指针
 * @param system_time 当前系统时间（ms），通常为HAL_GetTick()返回值
 */
void update_task_clock(TASK* ctrl, uint64_t system_time);

/**
 * @defgroup BuiltInJudgeFunctions 内置条件判断函数
 * @brief 预定义的judge函数，可直接用于add_task()
 * @{
 */

/**
 * @brief 判断当前值是否大于等于目标值
 * @param cur 当前值
 * @param tar 目标值
 * @return cur >= tar
 */
bool Greater_or_Equal(float cur, float tar);

/**
 * @brief 判断当前值是否小于等于目标值
 * @param cur 当前值
 * @param tar 目标值
 * @return cur <= tar
 */
bool Less_or_Equal(float cur, float tar);

/**
 * @brief 判断误差是否小于1（|cur-tar| < 1）
 * @param cur 当前值
 * @param tar 目标值
 * @return |cur - tar| < 1.0f
 */
bool Error_less_1(float cur, float tar);

/**
 * @brief 判断误差是否小于10
 * @param cur 当前值
 * @param tar 目标值
 * @return |cur - tar| < 10.0f
 */
bool Error_less_10(float cur, float tar);

/**
 * @brief 判断误差是否小于100
 * @param cur 当前值
 * @param tar 目标值
 * @return |cur - tar| < 100.0f
 */
bool Error_less_100(float cur, float tar);

/**
 * @brief 判断误差是否小于1000
 * @param cur 当前值
 * @param tar 目标值
 * @return |cur - tar| < 1000.0f
 */
bool Error_less_1000(float cur, float tar);

/** @} */  // end of BuiltInJudgeFunctions

/**
 * @brief 添加一个定时延迟任务
 * 
 * 该函数添加一个仅由超时触发的任务，judge函数固定为All_false（永远返回false），
 * 因此任务一定会在timeout时间到达时执行。
 * 
 * @param ctrl 任务控制器指针
 * @param task 任务函数指针
 * @param para 任务参数
 * @param later 延迟时间（ms）
 * @return 返回设置的延迟时间
 * 
 * @note 内部使用UINT8类型和NULL地址作为占位符
 */
uint32_t add_task_with_time_line(TASK* ctrl, void (*task)(void* para), void* para, uint32_t later);

/** @} */  // end of TaskController

#endif
