/**
 * @file array.c
 * @brief 数组工具函数实现
 * 
 * 实现了常用的数组操作算法：
 * - 基本操作：求和、拷贝、串口显示
 * - 极值查找：最小/最大索引
 * - 统计计数：阈值计数、连续计数
 * - 信号处理：卷积、前向差分
 * 
 * 应用场景：
 * - 巡线传感器ADC值处理
 * - 信号边缘检测（差分）
 * - 滑动窗口滤波（卷积）
 */

#include "array.h"

#include "usart.h"

/**
 * @brief 计算数组元素之和
 * 
 * 时间复杂度：O(len)
 * 空间复杂度：O(1)
 * 
 * @return int 累加和
 */
int array_sum(unsigned len, const short array[len]) {
  int sum = 0;
  for (unsigned int i = 0; i < len; i++)
    sum += array[i];
  return sum;
}

/**
 * @brief 查找最小元素索引
 * 
 * 从索引1开始遍历，维护当前最小值和其索引。
 * 时间复杂度：O(len)
 * 
 * @return unsigned 最小值索引
 */
unsigned array_find_min_index(unsigned len, const short array[len]) {
  unsigned min_index = 0;
  short min = array[min_index];
  for (unsigned i = 1; i < len; i++)
    if (array[i] < min) {
      min = array[i];
      min_index = i;
    }

  return min_index;
}

/**
 * @brief 查找最大元素索引
 * 
 * 算法同array_find_min_index，比较方向相反。
 * 
 * @return unsigned 最大值索引
 */
unsigned array_find_max_index(unsigned len, const short array[len]) {
  unsigned max_index = 0;
  short max = array[max_index];
  for (unsigned i = 1; i < len; i++)
    if (array[i] > max) {
      max = array[i];
      max_index = i;
    }

  return max_index;
}

/**
 * @brief 数组拷贝
 * 
 * 逐元素复制，不使用memcpy以保持类型安全。
 * 
 * @param len  数组长度
 * @param src  源数组
 * @param dest 目标数组
 */
void array_copy(unsigned len, const short src[len], short dest[len]) {
  for (unsigned i = 0; i < len; i++)
    dest[i] = src[i];
}

/**
 * @brief 一阶前向差分
 * 
 * 计算公式：dest[i] = src[i+1] - src[i]
 * 差分结果反映信号的局部变化率。
 * 
 * 边界情况：len=0时返回0，输出数组为空。
 * 
 * @return unsigned 输出数组长度 (len-1)
 */
unsigned forward_difference(unsigned len, const short src[len],
                            short dest[len - 1]) {
  if (len == 0)
    return 0;

  for (unsigned i = 0; i < len - 1; i++)
    dest[i] = src[i + 1] - src[i];

  return len - 1;
}

/**
 * @brief 多步前向差分
 * 
 * 计算公式：dest[i] = src[i+forward] - src[i]
 * 
 * 优化策略：
 * - forward=0：直接拷贝（无差分）
 * - forward=1：复用forward_difference()
 * - forward>=len：返回0（无法差分）
 * 
 * @param len     输入数组长度
 * @param forward 差分步长
 * @param src     输入数组
 * @param dest    输出数组
 * @return unsigned 输出数组长度
 */
unsigned forward_difference_multiple(unsigned len, unsigned forward,
                                     const short src[len],
                                     short dest[len - forward]) {
  if (len == 0)
    return 0;

  if (forward == 0) {
    array_copy(len, src, dest);
    return len;
  }

  if (forward >= len)
    return 0;

  if (forward == 1)
    return forward_difference(len, src, dest);

  unsigned dest_len = len - forward;
  for (unsigned i = 0; i < dest_len; i++)
    dest[i] = src[i + forward] - src[i];
  return dest_len;
}

/**
 * @brief 统计小于阈值的元素个数
 * 
 * 线性扫描数组，统计满足 array[i] < compare 的元素数量。
 * 
 * @return unsigned 符合条件的元素个数
 */
unsigned array_count_less_than(unsigned len, const short array[len],
                               short compare) {
  unsigned count = 0;
  for (unsigned i = 0; i < len; i++)
    if (array[i] < compare)
      count++;
  return count;
}

/**
 * @brief 统计连续小于阈值的最大长度
 * 
 * 算法流程：
 * 1. 维护两个计数器：count（当前连续长度）和max_count（历史最大长度）
 * 2. 遍历时：
 *    - 若满足条件：count++
 *    - 若不满足：更新max_count并重置count
 * 3. 注意：循环结束后未更新最后一次count，可能存在bug
 *    若数组末尾是连续段，其长度不会被记录到max_count
 * 
 * 应用场景：巡线传感器检测最长连续黑线
 * 
 * @return unsigned 最大连续长度
 */
unsigned array_count_continue_less_than(unsigned len, const short array[len],
                                        short compare) {
  int max_count = 0;
  int count = 0;

  for (unsigned i = 0; i < len; i++) {
    if (array[i] < compare)
      count++;
    else if (count > 0) {
      if (count > max_count)
        max_count = count;
      count = 0;
    }
  }
  return max_count;
}

/**
 * @brief 计算满足条件元素的索引和与个数
 * 
 * 遍历数组，对满足 array[i] < compare 的元素：
 * - 累加其索引值（而非元素值）到sum
 * - 累加计数器count
 * 
 * 应用：可用于质心计算，质心位置 = sum / count
 * 例如巡线小车通过传感器阵列确定黑线中心位置。
 * 
 * @return struct SumAndCount 包含索引和与元素个数
 */
struct SumAndCount array_mean_index_less_than(unsigned len,
                                              const short array[len],
                                              short compare) {
  int sum = 0;
  int count = 0;

  for (unsigned i = 0; i < len; i++) {
    if (array[i] < compare) {
      sum += i;       // 累加索引而非元素值
      count += 1;
    }
  }

  struct SumAndCount result = {.sum = sum, .count = count};
  return result;
}
