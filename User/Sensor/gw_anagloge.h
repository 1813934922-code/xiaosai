/**
 ******************************************************************************
 * @file    gw_anagloge.h
 * @brief   感为8路红外巡线传感器驱动头文件
 * @details 本文件定义了感为GW-8路模拟量输出传感器的数据结构、枚举和函数接口。
 *          该传感器通过8路红外对管检测地面黑线位置，输出模拟电压信号。
 *          本驱动实现了ADC采集、迟滞比较、归一化插值和路口识别等功能。
 * @note    传感器使用前必须调用correct_gw_analogue()进行白面/黑面校准
 ******************************************************************************
 */

// @551
// 驱动介绍:   此为感为8路模拟量输出传感器的驱动文件
// 传感器链接: https://item.taobao.com/item.htm?id=902128042528
// 功能实现:   将八路传感器的模拟值进行插值输出线性的黑线位置
//            使用迟滞比较器将八路模拟量转换为一个uint8_t数字量
//            通过数字量进行路口判断
// 注意事项:   传感器使用前需要校准，调用correct_gw_analogue()函数进行校准,详细此函数

#ifndef __GW_ANALOGUE_H
#define __GW_ANALOGUE_H

#include "main.h"
#include "pid.h"

/**
 * @addtogroup Sensor_Drivers
 * @{
 */

/**
 * @addtogroup GW_Analogue_Sensor
 * @brief 感为8路红外巡线传感器驱动
 * @{
 */

/**
 * @enum Road
 * @brief 路口类型枚举，基于中间3路传感器(L/F/R)的状态组合
 * 
 * @details 使用3位二进制编码表示路口特征：
 *          - Bit2(L): 左侧传感器是否检测到黑线
 *          - Bit1(F): 前方(中间)传感器是否检测到黑线  
 *          - Bit0(R): 右侧传感器是否检测到黑线
 *          
 *          例如: CrossRoad=0b111 表示左中右都检测到黑线，判定为十字路口
 *                TBRoad=0b101 表示左右有线、中间无线，判定为T型横向路口
 * @note 该枚举通过路口识别状态机自动判断，用于巡线小车的决策控制
 */
typedef enum Road {    // L F R
  CrossRoad = 0b111,   /**< 1 1 1 - 十字路口: 左中右三线同时检测到黑线 */
  TBRoad = 0b101,      /**< 1 0 1 - T型横向路口: 左右有线，前方无线(横向道路) */
  TLRoad = 0b011,      /**< 1 1 0 - T型左路口: 左前有线，右侧无线 */
  TRRoad = 0b110,      /**< 0 1 1 - T型右路口: 右前有线，左侧无线 */
  LeftRoad = 0b001,    /**< 1 0 0 - 左转弯: 仅左侧检测到黑线 */
  RightRoad = 0b100,   /**< 0 0 1 - 右转弯: 仅右侧检测到黑线 */
  Straight = 0b010,    /**< 0 1 0 - 直行: 仅中间检测到黑线 */
  UnknowRoad = 0b000,  /**< 0 0 0 - 未知/断线: 三路均未检测到黑线 */
} Road;

/**
 * @struct Cross
 * @brief 路口识别状态机结构体
 * 
 * @details 该结构体实现了带记忆功能的路口识别状态机，核心原理：
 *          1. 当传感器外侧(第0,1,6,7路)检测到黑线时，进入路口检测模式
 *          2. 在跨越路口过程中，累积(OR操作)所有帧的传感器状态到integral字段
 *          3. 当外侧连续多帧检测不到黑线时，触发结算逻辑
 *          4. 根据integral的累积状态判断最终路口类型
 *          
 *          防干扰机制：
 *          - 连续3帧外侧检测到黑线才进入路口模式(防抖动)
 *          - 连续3帧外侧无线才触发结算(防误判)
 *          - is_first_frame快照记录离开路口瞬间的中间状态
 * 
 * @note 该结构体实现了"连续性记忆"算法，能有效抵抗高速巡线时的漏判
 */
typedef struct Cross {
  uint8_t data_buf;           /**< 当前帧8路传感器的数字量状态(每位对应1路传感器) */
  uint8_t integral;           /**< 路口积分器：跨越路口过程中所有帧状态的逻辑或累积值 */
  uint8_t maybe;              /**< 路口检测状态标志：0=正常巡线，>0=正在跨越路口(记录黑线帧数) */
  uint8_t cross_cnt;          /**< 路口检测总计数器 */
  Road cross;                 /**< 最终判定的路口类型 */
  uint8_t integral_times;     /**< 白帧计数器：离开路口后外侧连续无线的帧数 */
  uint8_t cross_delay;        /**< 路口屏蔽计数器：用于转弯时暂时屏蔽传感器，该参数等于屏蔽剩余次数 */
  uint8_t is_first_frame;     /**< 离开路口第一帧标记：1=中间断线(直角弯)，0=中间有线(十字/T型) */
  
  /** @name 路口类型统计计数器 */
  /** @{ */
  uint8_t CrossRoad_cnt;      /**< 十字路口出现次数统计 */
  uint8_t TBRoad_cnt;         /**< T型横向路口出现次数统计 */
  uint8_t TLRoad_cnt;         /**< T型左路口出现次数统计 */
  uint8_t TRRoadd_cnt;        /**< T型右路口出现次数统计 */
  uint8_t LeftRoad_cnt;       /**< 左转弯出现次数统计 */
  uint8_t RightRoad_cnt;      /**< 右转弯出现次数统计 */
  uint8_t Straight_cnt;       /**< 直道出现次数统计 */
  uint8_t UnknowRoad_cnt;     /**< 未知/断线出现次数统计 */
  /** @} */

} Cross;

/**
 * @struct GW_ANALOGUE
 * @brief 感为8路巡线传感器主结构体
 * 
 * @details 该结构体包含了传感器的所有状态数据、校准参数和控制信息。
 *          使用前需初始化并校准，校准后自动生成迟滞比较器的阈值参数。
 * @note 数字量处理使用迟滞比较器防止临界值抖动，具体见get_gw_analoge_digital_data()
 */
typedef struct GW_ANALOGUE {
  uint8_t channel[8];                 /**< 8路传感器的原始ADC采样值(0-255)，对应通道0-7 */
  uint8_t sta;                        /**< 工作状态：0=正常工作模式，1=校准模式 */
  uint8_t correction_data_w[8];       /**< 白色校准数据：在白面上采集的各通道ADC基准值 */
  uint8_t correction_data_b[8];       /**< 黑色校准数据：在黑面上采集的各通道ADC基准值 */
  uint8_t digital_8bit;               /**< 8位数字量输出：每位对应1路传感器的黑白状态(1=黑，0=白) */
  uint8_t digital_high_threshold[8];  /**< 高阈值数组：用于迟滞比较器，ADC值高于此判定为白色 */
  uint8_t digital_low_threshold[8];   /**< 低阈值数组：用于迟滞比较器，ADC值低于此判定为黑色 */
  float diff;                         /**< 黑线偏离中心的位置偏差值，用于PID控制(单位：mm) */
  PID gw_analogue_pid;                /**< 巡线PID控制器实例 */
  Cross cross;                        /**< 路口判断状态机实例 */

} GW_ANALOGUE;

/** @} */
/** @} */

/**
 * @brief 初始化巡线传感器结构体
 * 
 * @param[in] aw_analogue 指向GW_ANALOGUE结构体的指针
 * @details 初始化包括：
 *          - 清零路口识别状态机
 *          - 清零通道数组和校准数据
 *          - 加载默认的迟滞比较器阈值(经验值)
 *          - 根据默认阈值反推校准数据
 *          - 初始化ADC多路复用选择器到通道0
 *          - 初始化PID控制器参数
 * @note 如果传感器已有校准数据，可跳过correct_gw_analogue()直接使用默认阈值
 */
void init_gw_analogue(GW_ANALOGUE *aw_analogue);

/**
 * @brief 获取8路传感器的原始ADC数据
 * 
 * @param[in] aw_analogue 指向GW_ANALOGUE结构体的指针
 * @details 通过ADC多路复用器依次读取8个通道的模拟值：
 *          1. 通过GPIO控制AD0/AD1/AD2引脚选择通道(3位二进制编码)
 *          2. 启动ADC3转换并等待完成
 *          3. 读取转换结果存入channel数组
 * @note 该函数采用阻塞式ADC轮询，8路全部读取完成约需数百微秒
 */
void get_gw_raw_data(GW_ANALOGUE *aw_analogue);

/**
 * @brief 校准传感器(白面/黑面两步校准法)
 * 
 * @param[in] gw_analogue 指向GW_ANALOGUE结构体的指针
 * @details 校准流程(需调用两次)：
 *          @b 第一次调用：将传感器放置在白色地面上
 *          - 采集8路通道的ADC值存入correction_data_w[]
 *          - LED1亮表示进入校准状态
 *          - 状态切换到sta=1
 *          
 *          @b 第二次调用：将传感器放置在黑色地面上  
 *          - 采集8路通道的ADC值存入correction_data_b[]
 *          - LED1灭表示校准完成
 *          - 自动计算迟滞比较器的高低阈值
 *          - 状态切换回sta=0
 *          
 *          @b 阈值计算公式：
 *          @code
 *          low_threshold[i]  = black[i] + (white[i] - black[i]) * 0.33
 *          high_threshold[i] = black[i] + (white[i] - black[i]) * 0.66
 *          @endcode
 *          迟滞窗口宽度 = high_threshold - low_threshold = (white-black)*0.33
 *          该设计确保在黑白交界处有充足的防抖动余量
 * @note 校准是可选的，如果不校准会使用init_gw_analogue()中的经验阈值
 */
void correct_gw_analogue(GW_ANALOGUE *gw_analogue);

/**
 * @brief 选择ADC多路复用器的通道
 * 
 * @param[in] channel 通道编号(0-7)，使用3位二进制编码
 * @details 通过3个GPIO引脚(AD0/AD1/AD2)实现8选1模拟多路复用：
 *          @code
 *          channel = 0b000 -> AD0=0, AD1=0, AD2=0 -> 选择通道0
 *          channel = 0b001 -> AD0=1, AD1=0, AD2=0 -> 选择通道1
 *          channel = 0b111 -> AD0=1, AD1=1, AD2=1 -> 选择通道7
 *          @endcode
 *          硬件原理：AD0对应Bit0，AD1对应Bit1，AD2对应Bit2
 * @note 此为内部调用函数，外部无需直接调用
 */
void select_channel(uint8_t channel);

/**
 * @brief 将原始ADC数据转换为8位数字量(迟滞比较器)
 * 
 * @param[in] gw_analogue 指向GW_ANALOGUE结构体的指针
 * @details 使用迟滞比较器原理将模拟量转换为数字量：
 *          - 若ADC值 > high_threshold -> 该位清零(判定为白色)
 *          - 若ADC值 < low_threshold  -> 该位置1(判定为黑色)
 *          - 若在两者之间 -> 保持原状态不变(迟滞区)
 *          
 *          @b 迟滞比较器的优势：
 *          当ADC值在阈值附近波动时，普通比较器会频繁翻转，
 *          而迟滞比较器通过设置高/低两个阈值，形成一个"死区"，
 *          只有信号超出死区才会改变输出状态，有效消除抖动。
 * @note digital_8bit的Bit0对应通道0，Bit7对应通道7
 */
void get_gw_analoge_digital_data(GW_ANALOGUE *gw_analogue);

/**
 * @brief 将数字量状态以ASCII格式输出(调试用)
 * 
 * @param[in] gw_analogue 指向GW_ANALOGUE结构体的指针
 * @details 将8位数字量转换为可视化的字符串：
 *          - Bit为1(黑线) -> 输出'#'字符
 *          - Bit为0(白色) -> 输出'.'字符
 *          例如：digital_8bit = 0b00010000 输出 "..#....."
 * @note 调试时可将str数组内容通过串口打印，直观查看传感器状态
 */
void gw_analogue_gray_show(GW_ANALOGUE *gw_analogue);

/**
 * @brief 计算黑线偏离中心的模拟量偏差值
 * 
 * @param[in] gw_analogue 指向GW_ANALOGUE结构体的指针
 * @details 基于加权平均算法计算黑线相对于传感器中心的位置偏移：
 *          1. 提取digital_8bit中检测到黑线的通道
 *          2. 每个通道对应一个物理距离值(distance数组)
 *          3. 偏差 = Σ(检测到黑线的通道 × 对应距离)
 *          4. 应用一阶低通滤波: output = input*0.7 + last_output*0.3
 *          
 *          @b 距离映射(distance数组)：
 *          @code
 *          通道:  0    1    2    3    4    5    6    7
 *          距离: -25  -20  -15  -10   10   15   20   25 (mm)
 *          @endcode
 *          注意通道3和4之间有空隙(中心区域)，负值表示偏左，正值表示偏右
 * @note 输出的diff值可直接用于PID巡线控制
 */
void get_gw_analogue_analogue_diff(GW_ANALOGUE *gw_analogue);

/**
 * @brief 巡线传感器主驱动函数
 * 
 * @param[in] gw_analogue 指向GW_ANALOGUE结构体的指针
 * @details 该函数应在主循环中周期性调用，执行流程：
 *          1. 采集8路原始ADC数据
 *          2. 通过迟滞比较器转换为数字量
 *          3. 如果cross_delay>0则跳过路口检测(屏蔽期)
 *          4. 执行路口识别状态机判断路口类型
 * @note cross_delay用于转弯时屏蔽传感器，防止误判，每调用一次递减
 */
void driver_gw_analogue(GW_ANALOGUE *gw_analogue);

#endif  /* __GW_ANALOGUE_H */
