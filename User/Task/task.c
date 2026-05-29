/**
 * @file task.c
 * @brief 任务控制器实现文件
 * 
 * 实现了基于条件触发和超时机制的任务调度系统。
 * 核心功能：
 * - 任务槽位管理（查找空闲槽位）
 * - 条件轮询检测（driver_task）
 * - 超时判定机制（is_timeout）
 * - 内置判断函数集（大于/小于/误差范围）
 */

#include "task.h"

#include <stdint.h>
#include <stdio.h>

#include "log.h"

/** @brief 绝对值宏 */
#define ABS(x) ((x) < 0 ? -(x) : (x))

/**
 * @brief 查找空闲的任务单元槽位
 * 
 * 线性扫描waiting数组，找到第一个标记为false的槽位索引。
 * 
 * @param ctrl 任务控制器指针
 * @return int32_t 空闲槽位索引，-1表示队列已满或ctrl为NULL
 */
int32_t find_unused_cell(TASK* ctrl) {
  if (ctrl == NULL) {
    return -1;  // 控制器空指针
  } else if (ctrl->task_num >= MAX_TASK_NUM) {
    return -1;  // 任务队列已满
  } else {
    for (int i = 0; i < MAX_TASK_NUM; i++) {
      if (ctrl->waiting[i] == false) {
        return i;  // 找到第一个空闲槽位
      }
    }
  }
  return -1;  // 理论上不会到达这里
}

/**
 * @brief 向任务控制器添加一个新任务
 * 
 * 实现流程：
 * 1. 调用find_unused_cell()查找空闲槽位
 * 2. 填充task_cell各字段
 * 3. 计算目标值tar = get_cur_val(cur_addr) + offset
 * 4. 设置超时时间戳timeout = task_clock + timeout
 * 5. 标记槽位为等待状态，task_num++
 * 
 * @return true  添加成功
 * @return false 队列已满
 */
bool add_task(TASK* ctrl, void (*task)(void* para), void* para, bool(*judge), void* cur_addr, float offset, TYPE type, uint32_t timeout) {
  int32_t pos = find_unused_cell(ctrl);
  if (pos == -1) {
    return false;  // 无空闲槽位
  } else {
    ctrl->cell[pos].task = task;
    ctrl->cell[pos].para = para;
    ctrl->cell[pos].judge = judge;
    ctrl->cell[pos].tar = get_cur_val(cur_addr, type) + offset;  // 计算触发目标值
    ctrl->cell[pos].cur_addr = cur_addr;
    ctrl->cell[pos].type = type;
    ctrl->cell[pos].timeout = ctrl->task_clock + timeout;  // 转换为绝对时间戳
    ctrl->waiting[pos] = true;                             // 标记为等待中
    ctrl->task_num++;
    return true;
  }
}

/**
 * @brief 检查任务是否超时
 * 
 * 超时判定逻辑：
 * - timeout <= 0：无超时保护，返回false
 * - task_clock >= timeout：已超时，返回true
 * 
 * @param ctrl 任务控制器
 * @param cell 待检查的任务单元
 * @return true  已超时
 * @return false 未超时或无超时保护
 */
bool is_timeout(TASK* ctrl, task_cell* cell) {
  if (cell->timeout <= 0) {
    return false;  // 无超时设置
  }
  if (ctrl->task_clock >= cell->timeout) {
    return true;  // 当前时间已超过超时时间戳
  }
  return false;
}

/**
 * @brief 驱动任务控制器执行
 * 
 * 核心调度循环，执行流程：
 * 1. 遍历所有MAX_TASK_NUM个槽位
 * 2. 对每个waiting[i]==true的任务：
 *    a. 读取cur_addr的当前值存入cell[i].cur
 *    b. 调用judge(cur, tar)判断条件是否满足
 *    c. 调用is_timeout()判断是否超时
 *    d. 若任一条件满足，执行task(para)并移除任务
 * 
 * 时间复杂度：O(MAX_TASK_NUM)
 * 
 * @param ctrl 任务控制器指针
 */
void driver_task(TASK* ctrl) {
  if (ctrl == NULL) {
    return;  // 空指针保护
  } else if (ctrl->task_num == 0) {
    return;  // 无等待任务，直接返回
  }
  for (int i = 0; i < MAX_TASK_NUM; i++) {
    if (ctrl->waiting[i] == true) {
      // 从实际变量地址读取当前值
      ctrl->cell[i].cur = get_cur_val(ctrl->cell[i].cur_addr, ctrl->cell[i].type);
      // 条件满足或超时则执行任务
      if (ctrl->cell[i].judge(ctrl->cell[i].cur, ctrl->cell[i].tar) || is_timeout(ctrl, &ctrl->cell[i])) {
        ctrl->cell[i].task(ctrl->cell[i].para);  // 执行任务
        ctrl->waiting[i] = false;                // 标记为已完成
        ctrl->task_num--;                        // 等待任务数减1
      }
    }
  }
}

/**
 * @brief 初始化任务控制器
 * 
 * 将所有字段重置为默认值，确保干净的初始状态。
 * 
 * @param ctrl 任务控制器指针
 */
void init_TASK(TASK* ctrl) {
  if (ctrl == NULL) {
    return;  // 空指针保护
  }
  ctrl->task_num = 0;
  ctrl->task_clock = 0;
  for (int i = 0; i < MAX_TASK_NUM; i++) {
    ctrl->waiting[i] = false;
    ctrl->cell[i].task = NULL;
    ctrl->cell[i].para = NULL;
    ctrl->cell[i].judge = NULL;
    ctrl->cell[i].cur_addr = NULL;
    ctrl->cell[i].type = UNKNOWN;
    ctrl->cell[i].timeout = 0;
  }
}

/**
 * @brief 更新任务时钟
 * 
 * 同步外部系统时间到任务控制器。
 * 
 * @param ctrl        任务控制器指针
 * @param system_time 系统时间（ms）
 */
void update_task_clock(TASK* ctrl, uint64_t system_time) {
  if (ctrl == NULL) {
    return;
  }
  ctrl->task_clock = system_time;
}

/**
 * @brief 判断当前值是否大于等于目标值
 */
bool Greater_or_Equal(float cur, float tar) {
  return cur >= tar;
}

/**
 * @brief 判断当前值是否小于等于目标值
 */
bool Less_or_Equal(float cur, float tar) {
  return cur <= tar;
}

/**
 * @brief 判断误差绝对值是否小于1
 */
bool Error_less_1(float cur, float tar) {
  return ABS(cur - tar) < 1.0f;
}

/**
 * @brief 判断误差绝对值是否小于10
 */
bool Error_less_10(float cur, float tar) {
  return ABS(cur - tar) < 10.0f;
}

/**
 * @brief 判断误差绝对值是否小于100
 */
bool Error_less_100(float cur, float tar) {
  return ABS(cur - tar) < 100.0f;
}

/**
 * @brief 判断误差绝对值是否小于1000
 */
bool Error_less_1000(float cur, float tar) {
  return ABS(cur - tar) < 1000.0f;
}

/**
 * @brief 始终返回false的判断函数
 * 
 * 用于add_task_with_time_line()，使任务仅通过超时触发。
 */
bool All_false(float cur, float tar) {
  return false;
}

/**
 * @brief 添加定时延迟任务
 * 
 * 内部调用add_task()，使用：
 * - judge = All_false（永不满足）
 * - cur_addr = NULL（无监控变量）
 * - type = UINT8（占位类型）
 * - timeout = later（延迟时间）
 * 
 * @return 返回设置的延迟时间
 */
uint32_t add_task_with_time_line(TASK* ctrl, void (*task)(void* para), void* para, uint32_t later) {
  add_task(ctrl, task, para, All_false, NULL, 0, UINT8, later);
  return later;
}
