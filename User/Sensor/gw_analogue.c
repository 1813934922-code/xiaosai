/**
 ******************************************************************************
 * @file    gw_analogue.c
 * @brief   感为8路红外巡线传感器驱动实现
 * @details 本文件实现了8路红外巡线传感器的完整驱动，包括：
 *          - ADC多路复用通道采集
 *          - 白面/黑面校准算法
 *          - 迟滞比较器数字量转换
 *          - 加权平均位置偏差计算
 *          - 带记忆功能的路口识别状态机
 * @note    本文件基于STM32 HAL库实现
 ******************************************************************************
 */

#include "adc.h"
#include "gpio.h"
#include "gw_anagloge.h"
#include "log.h"
#include "status.h"
#include "main.h"
#include "pid.h"

/**
 * @brief 各通道距离传感器中心的物理距离映射表(单位:mm)
 * @details 用于将离散的数字量传感器状态转换为连续的位置偏差值
 *          通道3和4之间留有空隙(中心区域)，负值表示偏左，正值表示偏右
 * @note 该数组的索引对应digital_8bit的位编号
 */
float distance[8] = {-25, -20, -15, -10, 10, 15, 20, 25};

/**
 * @brief 将8位数字量状态转换为ASCII字符串(调试用)
 * 
 * @param[in] gw_analogue 指向传感器结构体的指针
 * @details 将digital_8bit的每一位转换为可见字符：
 *          - Bit=1(检测到黑线) -> '#'字符
 *          - Bit=0(白色地面)   -> '.'字符
 *          输出示例：digital_8bit=0b00011000 生成 "....##.."
 * @note 调试时可通过串口打印str数组，直观查看传感器状态
 */
void gw_analogue_gray_show(GW_ANALOGUE *gw_analogue) {
  uint8_t buf = gw_analogue->digital_8bit;
  char str[9];
  str[8] = '\0';
  for (int i = 0; i < 8; i++) {
    str[i] = buf & 0x80 ? '#' : '.';
    buf <<= 1;
  }
}

/**
 * @brief 初始化路口识别状态机
 * 
 * @param[in] cross 指向路口状态机结构体的指针
 * @details 将路口状态机的所有字段清零，并设置默认参数：
 *          - integral_times=6: 白帧判定阈值(连续6帧外侧无线则结算)
 *          - cross=Stright: 初始状态为直行
 * @note 应在系统启动时调用一次
 */
void init_road_determine(Cross *cross) {
  cross->integral = 0;
  cross->data_buf = 0;
  cross->cross = Straight;
  cross->cross_cnt = 0;
  cross->integral_times = 6;
  cross->maybe = 0;
  cross->cross_delay = 0;

  return;
}

/**
 * @brief 清零路口类型统计计数器
 * 
 * @param[in] cross 指向路口状态机结构体的指针
 * @details 将所有路口类型的出现次数统计清零，用于重新统计或调试
 */
void init_road_cnt(Cross *cross){
  cross->CrossRoad_cnt=0;
  cross->LeftRoad_cnt=0;
  cross->RightRoad_cnt=0;
  cross->Straight_cnt=0;
  cross->TBRoad_cnt=0;
  cross->TLRoad_cnt=0;
  cross->TRRoadd_cnt=0;
  cross->UnknowRoad_cnt=0;
  return;
}

/**
 * @brief 初始化巡线传感器结构体
 * 
 * @param[in] gw_analogue 指向传感器结构体的指针
 * @details 执行完整的初始化流程：
 *          1. 初始化路口识别状态机(清零所有状态)
 *          2. 清零通道数组和校准数据数组
 *          3. 加载经验阈值(针对特定传感器标定好的默认值)
 *          4. 根据经验阈值反推校准数据：
 *             correction_data_w = 2*high_threshold - low_threshold
 *             correction_data_b = 2*low_threshold - high_threshold
 *             这是阈值计算公式的逆运算，确保数据结构完整性
 *          5. 设置工作状态为正常模式(sta=0)
 *          6. 初始化ADC多路复用器到通道0
 *          7. 初始化PID控制器
 * @note 如果不执行校准，传感器将使用这些经验阈值工作
 */
void init_gw_analogue(GW_ANALOGUE *gw_analogue) {
  init_road_determine(&gw_analogue->cross);
  init_road_cnt(&gw_analogue->cross);

  // Initialize the ADC and GPIO for the analogue channels
  for (int i = 0; i < 8; i++) {
    gw_analogue->channel[i] = 0;  // Initialize channel values to 0
  }
  for (int i = 0; i < 8; i++) {
    gw_analogue->correction_data_w[i] = 0;  // Initialize correction data to 0
    gw_analogue->correction_data_b[i] = 0;  // Initialize correction data to 0
  }
  gw_analogue->digital_high_threshold[0] = 81;   // Initialize high threshold to 0
  gw_analogue->digital_high_threshold[1] = 137;  // Initialize high threshold to 0
  gw_analogue->digital_high_threshold[2] = 77;   // Initialize high threshold to 0
  gw_analogue->digital_high_threshold[3] = 82;   // Initialize high threshold to 0
  gw_analogue->digital_high_threshold[4] = 61;   // Initialize high threshold to 0
  gw_analogue->digital_high_threshold[5] = 115;  // Initialize high threshold to 0
  gw_analogue->digital_high_threshold[6] = 129;  // Initialize high threshold to 0
  gw_analogue->digital_high_threshold[7] = 75;   // Initialize high threshold to 0

  gw_analogue->digital_low_threshold[0] = 49;  // Initialize low threshold to 0
  gw_analogue->digital_low_threshold[1] = 95;  // Initialize low threshold to 0
  gw_analogue->digital_low_threshold[2] = 49;  // Initialize low threshold to 0
  gw_analogue->digital_low_threshold[3] = 55;  // Initialize low threshold to 0
  gw_analogue->digital_low_threshold[4] = 37;  // Initialize low threshold to 0
  gw_analogue->digital_low_threshold[5] = 77;  // Initialize low threshold to 0
  gw_analogue->digital_low_threshold[6] = 89;  // Initialize low threshold to 0
  gw_analogue->digital_low_threshold[7] = 47;  // Initialize low threshold to 0

  for (int i = 0; i < 8; i++) {
    gw_analogue->correction_data_w[i] = 2 * gw_analogue->digital_high_threshold[i] - gw_analogue->digital_low_threshold[i];  // Initialize high threshold to 0
    gw_analogue->correction_data_b[i] = 2 * gw_analogue->digital_low_threshold[i] - gw_analogue->digital_high_threshold[i];  // Initialize low threshold to 0
  }

  gw_analogue->sta = 0;           // Set the state to 0 (normal mode)
  gw_analogue->digital_8bit = 0;  // Initialize the 8-bit digital value to 0

  gw_analogue->diff = 0.0f;  // Initialize the difference value to 0.0f

  select_channel(0);  // Select channel 0 for initial setup

  gw_analogue->gw_analogue_pid = init_pid(gw_analogue->gw_analogue_pid.kp,gw_analogue->gw_analogue_pid.ki,gw_analogue->gw_analogue_pid.kd,10.0,50.0);
}

/**
 * @brief 选择ADC多路复用器的通道
 * 
 * @param[in] channel 通道编号(0-7)，使用3位二进制编码
 * @details 通过3个GPIO引脚(AD0/AD1/AD2)实现8选1模拟多路复用器控制：
 *          @code
 *          channel bit0 -> AD0引脚 (LSB)
 *          channel bit1 -> AD1引脚
 *          channel bit2 -> AD2引脚 (MSB)
 *          @endcode
 *          例如：
 *          - channel=0 (0b000): AD0=0, AD1=0, AD2=0 -> 选择通道0
 *          - channel=3 (0b011): AD0=1, AD1=1, AD2=0 -> 选择通道3
 *          - channel=7 (0b111): AD0=1, AD1=1, AD2=1 -> 选择通道7
 * @note 这是硬件多路复用器的标准控制方式，节省ADC引脚资源
 */
void select_channel(uint8_t channel) {
  // Select the ADC channel to read from
  if (channel & 0x01) {
    HAL_GPIO_WritePin(AD0_GPIO_Port, AD0_Pin, GPIO_PIN_SET);  // Set PA0 high
  } else {
    HAL_GPIO_WritePin(AD0_GPIO_Port, AD0_Pin, GPIO_PIN_RESET);  // Set PA0 low
  }
  if (channel & 0x02) {
    HAL_GPIO_WritePin(AD1_GPIO_Port, AD1_Pin, GPIO_PIN_SET);  // Set PA1 high
  } else {
    HAL_GPIO_WritePin(AD1_GPIO_Port, AD1_Pin, GPIO_PIN_RESET);  // Set PA1 low
  }
  if (channel & 0x04) {
    HAL_GPIO_WritePin(AD2_GPIO_Port, AD2_Pin, GPIO_PIN_SET);  // Set PA2 high
  } else {
    HAL_GPIO_WritePin(AD2_GPIO_Port, AD2_Pin, GPIO_PIN_RESET);  // Set PA2 low
  }
}

/**
 * @brief 获取8路传感器的原始ADC采样值
 * 
 * @param[in] gw_analogue 指向传感器结构体的指针
 * @details 通过ADC多路复用器依次读取8个通道的模拟电压值：
 *          1. 循环8次，每次select_channel(i)选择对应通道
 *          2. HAL_ADC_Start() 启动ADC转换
 *          3. HAL_ADC_PollForConversion() 阻塞等待转换完成(超时1ms)
 *          4. HAL_ADC_GetValue() 读取转换结果(8位精度，0-255)
 *          5. HAL_ADC_Stop() 停止ADC转换
 * @note 该函数采用阻塞式轮询，8路全部读取完成约需数百微秒到1毫秒
 *       如果对实时性要求高，可考虑使用ADC DMA连续扫描模式
 */
void get_gw_raw_data(GW_ANALOGUE *gw_analogue) {
  // Read the ADC value for the selected channel
  for (int i = 0; i < 8; i++) {
    select_channel(i);                                   // Select the channel to read from
    HAL_ADC_Start(&hadc3);                               // Start the ADC conversion
    HAL_ADC_PollForConversion(&hadc3, 1);                // Wait for conversion to complete
    gw_analogue->channel[i] = HAL_ADC_GetValue(&hadc3);  // Get the ADC value
    HAL_ADC_Stop(&hadc3);                                // Stop the ADC conversion
  }
}

/**
 * @brief 校准传感器(白面/黑面两步校准法)
 * 
 * @param[in] gw_analogue 指向传感器结构体的指针
 * @details 该函数需要被调用两次，完成完整的校准流程：
 *          
 *          @b 第一次调用(sta=0 -> sta=1)：白色校准
 *          - 依次采集8个通道在白面上的ADC原始值
 *          - 存储到correction_data_w[]数组
 *          - 点亮LED1作为视觉反馈
 *          - 状态切换到sta=1(等待黑色校准)
 *          
 *          @b 第二次调用(sta=1 -> sta=0)：黑色校准
 *          - 依次采集8个通道在黑面上的ADC原始值
 *          - 存储到correction_data_b[]数组
 *          - 熄灭LED1表示校准完成
 *          - 自动计算迟滞比较器的高/低阈值：
 *            @code
 *            low_threshold[i]  = black[i] + (white[i] - black[i]) * 0.33
 *            high_threshold[i] = black[i] + (white[i] - black[i]) * 0.66
 *            @endcode
 *            物理含义：
 *            - 33%位置：从黑到白转换的下阈值，留33%余量防止黑线误判为白
 *            - 66%位置：从白到黑转换的上阈值，留34%余量防止白底误判为黑
 *            - 迟滞窗口宽度 = (white-black) * 0.33，确保临界状态稳定
 *          
 *          @b 校准原理说明：
 *          不同环境光线下，传感器输出的绝对ADC值会变化。
 *          通过采集当前环境的黑白基准值，可以动态调整阈值，
 *          使传感器适应各种光照条件。
 * @note 校准是可选操作，如果不校准会使用init_gw_analogue()中的经验阈值
 *       校准前请确保传感器完全覆盖白色/黑色区域
 */
void correct_gw_analogue(GW_ANALOGUE *gw_analogue) {
  if (gw_analogue->sta == 0) {
    for (int i = 0; i < 8; i++) {
      select_channel(i);                                             // Select the channel to read from
      HAL_ADC_Start(&hadc3);                                         // Start the ADC conversion
      HAL_ADC_PollForConversion(&hadc3, 1);                          // Wait for conversion to complete
      gw_analogue->correction_data_w[i] = HAL_ADC_GetValue(&hadc3);  // Get the ADC value      HAL_ADC_Stop(&hadc1);                                // Stop the ADC conversion
    }
    status.device.led1.on = 1;
    gw_analogue->sta = 1;  // Set the state to calibration mode 1
    return;
  }
  if (gw_analogue->sta == 1) {
    for (int i = 0; i < 8; i++) {
      select_channel(i);                                             // Select the channel to read from
      HAL_ADC_Start(&hadc3);                                         // Start the ADC conversion
      HAL_ADC_PollForConversion(&hadc3, 1);                          // Wait for conversion to complete
      gw_analogue->correction_data_b[i] = HAL_ADC_GetValue(&hadc3);  // Get the ADC value
      HAL_ADC_Stop(&hadc3);                                          // Stop the ADC conversion
    }
    status.device.led1.on = 0;
    gw_analogue->sta = 0;  // Set the state to calibration mode 2
    for (int i = 0; i < 8; i++) {
      gw_analogue->digital_low_threshold[i] = gw_analogue->correction_data_b[i] +
                                              (gw_analogue->correction_data_w[i] - gw_analogue->correction_data_b[i]) * 0.33;
      // Calculate the low threshold
      gw_analogue->digital_high_threshold[i] = gw_analogue->correction_data_b[i] +
                                               (gw_analogue->correction_data_w[i] - gw_analogue->correction_data_b[i]) * 0.66;
      // Calculate the high threshold
    }
    return;
  }
}

/**
 * @brief 通过迟滞比较器将模拟量转换为数字量
 * 
 * @param[in] gw_analogue 指向传感器结构体的指针
 * @details 对8个通道的ADC值逐一进行迟滞比较：
 *          @code
 *          if (ADC[i] > high_threshold[i])  -> digital_8bit的bit i = 0 (白色)
 *          if (ADC[i] < low_threshold[i])   -> digital_8bit的bit i = 1 (黑色)
 *          if (low <= ADC <= high)          -> 保持bit i原状态不变 (迟滞区)
 *          @endcode
 *          
 *          @b 迟滞比较器的工作原理：
 *          当ADC值在高阈值和低阈值之间时，不改变输出状态。
 *          这种"记忆"特性使得输出不会在临界值附近频繁翻转。
 *          例如：某通道从黑色(ADC=30)逐渐变亮：
 *          - ADC=30 < low=49 -> 输出1(黑)
 *          - ADC=60 在49~81之间 -> 仍输出1(保持)
 *          - ADC=85 > high=81 -> 输出0(白)
 *          反向变暗时同理，形成不对称的切换特性，有效滤除噪声。
 * @note digital_8bit使用位操作，bit0对应通道0，bit7对应通道7
 *       该函数依赖channel数组中的最新ADC值，需先调用get_gw_raw_data()
 */
void get_gw_analoge_digital_data(GW_ANALOGUE *gw_analogue) {
  for (int i = 0; i < 8; i++) {
    if (gw_analogue->channel[i] > gw_analogue->digital_high_threshold[i]) {
      gw_analogue->digital_8bit &= ~(1 << i);
    } else if (gw_analogue->channel[i] < gw_analogue->digital_low_threshold[i]) {
      gw_analogue->digital_8bit |= (1 << i);
    }
  }
}

/**
 * @brief 将ADC原始值归一化到0-100范围
 * 
 * @param[in] max 该通道的最大值(白色校准值)
 * @param[in] min 该通道的最小值(黑色校准值)
 * @param[in] now 当前ADC采样值
 * @return float 归一化后的百分比值(0-100)
 * @details 计算公式：((now - min) / (max - min)) * 100
 *          将不同通道的ADC值统一映射到相同量程，消除个体差异
 * @note 该函数当前未被调用，可能用于更精细的模拟量处理
 */
float normalize_gray_data(uint8_t max, uint8_t min, uint8_t now) {
  return (((float)(now - min) / (float)(max - min)) * 100);
}

/**
 * @brief 对中间6路传感器的权重进行归一化处理
 * 
 * @param[in,out] raw_data 指向包含6个权重值的数组(索引1-6)
 * @details 将中间6路传感器的权重值归一化，使总和为1.0：
 *          1. 计算raw_data[1]到raw_data[6]的总和
 *          2. 如果总和为0则直接返回(避免除零)
 *          3. 每个元素除以总和，得到相对权重
 *          例如：[0, 0.2, 0.3, 0.5, 0.4, 0.1, 0.0, 0] -> 
 *                总和=1.5 -> [0, 0.133, 0.2, 0.333, 0.267, 0.067, 0.0, 0]
 * @note 该函数当前未被调用，可能用于更高级的加权平均算法
 */
void normalize_gray_weight(float *raw_data) {
  float total = 0;
  for (int i = 1; i < 7; i++) {
    total += raw_data[i];
  }
  if (total == 0) {
    return;
  }
  for (int i = 1; i < 7; i++) {
    raw_data[i] = (raw_data[i] / total);
  }

  return;
}

/**
 * @brief 计算黑线偏离中心的模拟量偏差值
 * 
 * @param[in] gw_analogue 指向传感器结构体的指针
 * @details 基于离散传感器阵列的加权平均算法计算黑线位置：
 *          @b 算法步骤：
 *          1. 提取digital_8bit的每一位到buff数组(0或1)
 *          2. 计算加权偏差：diff = Σ(buff[i] * distance[i])
 *             其中distance数组定义了各通道距离中心的物理距离
 *          3. 应用一阶低通滤波器平滑输出：
 *             output = input * 0.7 + last_output * 0.3
 *             该滤波器时间常数约为3个采样周期，有效抑制高频噪声
 *          
 *          @b 物理意义：
 *          - diff = 0: 黑线在传感器正中心
 *          - diff > 0: 黑线偏右(数值越大越靠右)
 *          - diff < 0: 黑线偏左(数值越小越靠左)
 *          - 理论范围：-55 ~ +55 (所有左侧/右侧通道都检测到黑线)
 *          - 实际范围：-25 ~ +25 (单通道检测时)
 *          
 *          @b 示例：
 *          如果只有通道2检测到黑线(digital_8bit=0b00000100)：
 *          diff = buff[2] * distance[2] = 1 * (-15) = -15mm (偏左15mm)
 * @note 结果存储在status全局变量中，供PID控制器使用
 *       低通滤波使用静态变量last_diff保持历史状态
 */
void get_gw_analogue_analogue_diff(GW_ANALOGUE *gw_analogue) {
  uint8_t buff[8] = {0};
  float diff = 0;

  // 从 digital_8bit 提取每个传感器的二值状态（1 表示黑线有效）
  for (int i = 0; i < 8; i++) {
    if (gw_analogue->digital_8bit & (1 << i)) {
      buff[i] = 1;
    } else {
      buff[i] = 0;
    }
  }
  for (int i=0;i<8;i++)
  {
    diff+=buff[i]*distance[i];
  }

  static float last_diff = 0;
  status.sensor.gw_analogue.diff = diff * 0.7f + last_diff * 0.3f;
  last_diff = status.sensor.gw_analogue.diff;
  // status.sensor.gw_analogue.diff = diff;
}

/**
 * @brief 根据左中右三路状态生成路口类型枚举值
 * 
 * @param[in] L 左侧传感器是否检测到黑线(true=有线)
 * @param[in] F 前方(中间)传感器是否检测到黑线(true=有线)
 * @param[in] R 右侧传感器是否检测到黑线(true=有线)
 * @return enum Road 对应的路口类型枚举值
 * @details 将3路布尔值组合成3位二进制编码：
 *          @code
 *          L=1,F=1,R=1 -> 0b111 -> CrossRoad (十字路口)
 *          L=1,F=0,R=1 -> 0b101 -> TBRoad (T型横向)
 *          L=1,F=1,R=0 -> 0b011 -> TLRoad (T型左)
 *          以此类推...
 *          @endcode
 * @note 该函数为路口识别的基础工具函数
 */
enum Road road_new_from_bit(bool L, bool F, bool R) {
  uint8_t left = L ? 0b100 : 0;
  uint8_t font = F ? 0b010 : 0;
  uint8_t right = R ? 0b001 : 0;

  return left | font | right;
}

/**
 * @brief 记录路口类型的出现次数
 * 
 * @param[in] cross 指向路口状态机结构体的指针
 * @param[in] road 当前判定的路口类型
 * @details 根据road类型累加对应的计数器，用于统计各种路口的出现频率
 * @note 该函数在get_road_type()中调用，自动记录每次路口判定结果
 */
void serve_road(Cross *cross, Road road) {
    switch (road) {
      case CrossRoad:
        cross->CrossRoad_cnt++;
        break;
      case TBRoad:
        cross->TBRoad_cnt++;
        break;
      case TLRoad:
        cross->TLRoad_cnt++;
        break;
      case TRRoad:
        cross->TRRoadd_cnt++;
        break;
      case LeftRoad:
        cross->LeftRoad_cnt++;
        break;
      case RightRoad:
        cross->RightRoad_cnt++;
        break;
      case Straight:
        cross->Straight_cnt++;
        break;
      case UnknowRoad:
        cross->UnknowRoad_cnt++;
        break;
    }

  cross->cross = road;
}

/**
 * @brief 路口识别状态机(核心算法)
 * 
 * @param[in] cross 指向路口状态机结构体的指针
 * @param[in] road_data 当前帧8路传感器的数字量状态
 * @details 实现了带"连续性记忆"的路口识别算法，有效抵抗高速巡线时的干扰。
 *          
 *          @b 状态机工作流程：
 *          
 *          @b 状态0：正常巡线，寻找路口入口
 *          - 检测外侧4路传感器(位0,1,6,7对应掩码0xC3)
 *          - 如果外侧有线，启动防抖计数器(noise_filter_cnt)
 *          - 连续3帧外侧都有线才确认进入路口(maybe=1)
 *          - 记录当前帧状态到integral(路口积分器)
 *          
 *          @b 状态1：正在跨越路口
 *          - 持续累积所有帧的状态：integral |= data_buf (逻辑或)
 *          - 检测外侧是否全白(掩码0xC3=0)：
 *            * 第1帧全白时，快照记录中间状态(is_first_frame)：
 *              - 中间有线 -> is_first_frame=0 (十字/T型)
 *              - 中间无线 -> is_first_frame=1 (直角弯/断头路)
 *            * 继续累积白帧计数(integral_times)
 *          - 如果外侧再次出现黑线，打断白帧计数，继续累积
 *          - 连续4帧外侧全白，触发结算：
 *            * 根据integral的累积状态和is_first_frame快照判断路口类型
 *            * 重置状态机(maybe=0, integral=0)
 *          
 *          @b 核心位掩码说明：
 *          - 0xC3 (0b11000011): 外侧4路传感器(通道0,1,6,7)
 *          - 0x18 (0b00011000): 中间2路传感器(通道3,4)
 *          - 0x01/0x02: 左侧传感器(通道0,1)
 *          - 0x80/0x40: 右侧传感器(通道6,7)
 *          
 *          @b 路口判定逻辑：
 *          @code
 *          if (center_lost) {  // 前方断头(直角弯类)
 *            if (left && right)  -> TBRoad (T型横向)
 *            if (left only)      -> LeftRoad (左转)
 *            if (right only)     -> RightRoad (右转)
 *          } else {  // 前方有路(十字/T型类)
 *            if (left && right)  -> CrossRoad (十字)
 *            if (left only)      -> TLRoad (T型左)
 *            if (right only)     -> TRRoad (T型右)
 *          }
 *          @endcode
 *          
 *          @b 防干扰机制：
 *          - 入口防抖：连续3帧外侧有线才进入路口模式
 *          - 出口防抖：连续4帧外侧无线才结算
 *          - 快照技术：离开瞬间捕捉中间状态，避免PID干扰
 *          - 积分记忆：累积整个跨越过程的状态，不依赖单帧
 * @note 该函数是巡线算法的核心，应每帧调用
 *       参数设计(3帧/4帧)可根据车速和传感器采样率调整
 */
void get_road_type(Cross *cross, uint8_t road_data) {
  cross->data_buf = road_data;
  static uint8_t noise_filter_cnt = 0;
  // ================= 状态 0：正常巡线，寻找路口 =================
  if (cross->maybe == 0) {
    if ((cross->data_buf & 0xC3) != 0) { // 外侧摸到线了
      noise_filter_cnt++;
      // 🌟 核心防抖：连续 3 帧（具体数值可调）外侧都有线，才确信是路口
      if (noise_filter_cnt >= 3) {
        cross->maybe = 1;
        cross->integral = cross->data_buf;
        cross->integral_times = 0;
        noise_filter_cnt = 0; // 触发后清零
      }
    } else {
      noise_filter_cnt = 0; // 一旦断开，立刻清零，滤除瞬间干扰
    }
  }
  // ================= 状态 1：正在跨越路口 =================
  else if (cross->maybe > 0) {
    cross->integral |= cross->data_buf;

    // 🌟 动态出口：连续3帧外侧全白
    if ((cross->data_buf & 0xC3) == 0) {

      // 【核心黑科技：快照定格】
      // 就在左右探头刚刚离开黑线的这第一帧，立刻抓取中间探头的状态！
      // 此时车身已经完全跨出横线，且 PID 还没来得及因为断线而乱扭。
      if (cross->integral_times == 0) {
        if ((cross->data_buf & 0x18) == 0) {
          cross->is_first_frame = 1; // 中间没线 -> 确定是断头直角弯
        } else {
          cross->is_first_frame = 0; // 中间有线 -> 确定是十字或T型
        }
      }

      cross->integral_times++; // 白帧计数

    } else {
      // 只要中间有任何一帧摸到外侧黑线，立刻打断白帧计数
      cross->integral_times = 0;
      if (cross->maybe < 250) cross->maybe++;
    }

    // 🌟 结算条件：确定已经离开路口 3 帧以上
    if (cross->integral_times >= 4) {

      // 降低门槛：只要经历过至少 1 帧的黑线，就认定为真路口（免疫高速漏判）
      if (cross->maybe >= 1) {
        bool left = (cross->integral & 0x01) || (cross->integral & 0x02);
        bool right = (cross->integral & 0x80) || (cross->integral & 0x40);
        bool center_lost = (cross->is_first_frame == 1); // 提取刚刚的快照

        Road final_road = Straight;

        if (center_lost) { // 前方断头
          if (left && right) final_road = TBRoad;
          else if (left) final_road = LeftRoad;
          else if (right) final_road = RightRoad;
        } else { // 前方有路
          if (left && right) final_road = CrossRoad;
          else if (left) final_road = TLRoad;
          else if (right) final_road = TRRoad;
        }

        serve_road(cross, final_road);
        //log_uprintf(&huart1, "[ROAD] Type:%d, BlackFrames:%d, CenterLost:%d\n", final_road, cross->maybe, center_lost);
      } else {
        serve_road(cross, Straight);
      }

      // 干净利落，状态重置
      cross->maybe = 0;
      cross->integral = 0;
    }
  }
}

/**
 * @brief 巡线传感器主驱动函数
 * 
 * @param[in] gw_analogue 指向传感器结构体的指针
 * @details 该函数应在主循环或定时器中断中周期性调用，执行完整的传感器处理流程：
 *          1. 采集8路原始ADC数据 -> get_gw_raw_data()
 *          2. 通过迟滞比较器转换为数字量 -> get_gw_analoge_digital_data()
 *          3. 检查路口屏蔽计数器(cross_delay)：
 *             - 如果>0：跳过路口检测(转弯后短暂屏蔽，防止误判)
 *             - 每调用一次递减，直到归零恢复正常检测
 *          4. 执行路口识别状态机 -> get_road_type()
 *          
 *          @b 调用频率建议：
 *          - 低速巡线(1m/s以下)：50-100Hz
 *          - 高速巡线(1-2m/s)：100-200Hz
 *          - 超高速(2m/s以上)：200Hz以上
 * @note 模拟量偏差计算(get_gw_analogue_analogue_diff)当前被注释，
 *       如需使用PID巡线控制，应取消注释
 */
void driver_gw_analogue(GW_ANALOGUE *gw_analogue) {
  get_gw_raw_data(gw_analogue);
  get_gw_analoge_digital_data(gw_analogue);
  // get_gw_analogue_analogue_diff(gw_analogue);
  if (gw_analogue->cross.cross_delay > 0) {
    gw_analogue->cross.cross_delay--;
    return;
  }
  get_road_type(&gw_analogue->cross, gw_analogue->digital_8bit);
}
