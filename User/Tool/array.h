/**
 * @file array.h
 * @brief 数组工具函数头文件
 * 
 * 提供常用的数组操作算法，适用于传感器数据处理和巡线算法。
 * 核心功能：
 * - 基本操作：求和、拷贝、显示
 * - 极值查找：最小值/最大值索引
 * - 统计计数：小于阈值的元素个数、连续小于阈值的最大长度
 * - 信号处理：卷积、前向差分
 * 
 * 所有函数使用short类型数组，适用于ADC采样值等场景。
 * 
 * @addtogroup ArrayTool
 * @{
 */

#ifndef __ARRAY_H__
#define __ARRAY_H__

/**
 * @brief 计算数组元素之和
 * @param len   数组长度
 * @param array 输入数组（const short类型）
 * @return int  所有元素的累加和
 */
int array_sum(unsigned len, const short array[len]);

/**
 * @brief 拷贝数组
 * @param len  数组长度
 * @param src  源数组
 * @param dest 目标数组
 */
void array_copy(unsigned len, const short src[len], short dest[len]);

/**
 * @brief 通过串口打印数组内容（调试用）
 * @param len   数组长度
 * @param array 要显示的数组
 */
void array_display(unsigned len, const short array[len]);

/**
 * @brief 查找数组中最小元素的索引
 * @param len   数组长度
 * @param array 输入数组
 * @return unsigned 最小元素的索引（若有多个最小值，返回第一个）
 */
unsigned array_find_min_index(unsigned len, const short array[len]);

/**
 * @brief 查找数组中最大元素的索引
 * @param len   数组长度
 * @param array 输入数组
 * @return unsigned 最大元素的索引（若有多个最大值，返回第一个）
 */
unsigned array_find_max_index(unsigned len, const short array[len]);

/**
 * @brief 统计数组中小于比较值的元素个数
 * @param len     数组长度
 * @param array   输入数组
 * @param compare 比较阈值
 * @return unsigned 小于compare的元素数量
 */
unsigned array_count_less_than(unsigned len, const short array[len],
                               short compare);

/**
 * @brief 统计数组中连续小于比较值的最大长度
 * 
 * 遍历数组，找出连续满足array[i] < compare的最长子序列长度。
 * 适用于巡线传感器中检测连续黑线的最大长度。
 * 
 * @param len     数组长度
 * @param array   输入数组
 * @param compare 比较阈值
 * @return unsigned 连续小于compare的最大元素个数
 */
unsigned array_count_continue_less_than(unsigned len, const short array[len],
                                        short compare);

/**
 * @struct SumAndCount
 * @brief 求和与计数结果结构体
 */
struct SumAndCount {
  int sum;    /**< @brief 索引之和 */
  int count;  /**< @brief 满足条件的元素个数 */
};

/**
 * @brief 计算满足条件元素的索引之和与个数
 * 
 * 遍历数组，对所有满足array[i] < compare的元素：
 * - 累加其索引值到sum
 * - 累加计数器count
 * 
 * 可用于计算质心位置：质心 = sum / count
 * 
 * @param len     数组长度
 * @param array   输入数组
 * @param compare 比较阈值
 * @return struct SumAndCount 包含索引和与元素个数
 */
struct SumAndCount
array_mean_index_less_than(unsigned len, const short array[len], short compare);

/**
 * @brief 单位卷积核卷积运算
 * 
 * 使用长度为kernel_len的全1核进行卷积。
 * 输出数组长度 = len - kernel_len
 * dest[i] = sum(src[i] ~ src[i+kernel_len-1])
 * 
 * 常用于滑动窗口求和或均值滤波。
 * 
 * @param len         输入数组长度
 * @param kernel_len  卷积核长度
 * @param src         输入数组
 * @param dest        输出数组（长度至少为 len - kernel_len）
 * @return unsigned   输出数组的实际长度
 */
unsigned convolve_unit(unsigned len, unsigned kernel_len, const short src[len],
                       short dest[len - kernel_len]);

/**
 * @brief 一阶前向差分
 * 
 * 计算相邻元素的差值：dest[i] = src[i+1] - src[i]
 * 输出数组长度 = len - 1
 * 
 * 常用于检测信号变化率或边缘检测。
 * 
 * @param len  输入数组长度
 * @param src  输入数组
 * @param dest 输出数组（长度至少为 len - 1）
 * @return unsigned 输出数组长度
 */
unsigned forward_difference(unsigned len, const short src[len],
                            short dest[len - 1]);

/**
 * @brief 多步前向差分
 * 
 * 计算间隔forward个元素的差值：dest[i] = src[i+forward] - src[i]
 * 输出数组长度 = len - forward
 * 
 * 特殊情况处理：
 * - forward == 0：直接拷贝数组
 * - forward == 1：调用forward_difference()
 * - forward >= len：返回0
 * 
 * @param len     输入数组长度
 * @param forward 差分步长
 * @param src     输入数组
 * @param dest    输出数组（长度至少为 len - forward）
 * @return unsigned 输出数组长度
 */
unsigned forward_difference_multiple(unsigned len, unsigned forward,
                                     const short src[len],
                                     short dest[len - forward]);

/** @} */  // end of ArrayTool

#endif  // !__ARRAY_H__
