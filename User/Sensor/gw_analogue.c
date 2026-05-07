#include "adc.h"
#include "gpio.h"
#include "gw_anagloge.h"
#include "log.h"
#include "status.h"
#include "main.h"
#include "pid.h"
float distance[8] = {-25, -20, -15, -10, 10, 15, 20, 25};

void gw_analogue_gray_show(GW_ANALOGUE *gw_analogue) {
  uint8_t buf = gw_analogue->digital_8bit;
  char str[9];
  str[8] = '\0';
  for (int i = 0; i < 8; i++) {
    str[i] = buf & 0x80 ? '#' : '.';
    buf <<= 1;
  }
}

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

void get_gw_analoge_digital_data(GW_ANALOGUE *gw_analogue) {
  for (int i = 0; i < 8; i++) {
    if (gw_analogue->channel[i] > gw_analogue->digital_high_threshold[i]) {
      gw_analogue->digital_8bit &= ~(1 << i);
    } else if (gw_analogue->channel[i] < gw_analogue->digital_low_threshold[i]) {
      gw_analogue->digital_8bit |= (1 << i);
    }
  }
}

float normalize_gray_data(uint8_t max, uint8_t min, uint8_t now) {
  return (((float)(now - min) / (float)(max - min)) * 100);
}

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

enum Road road_new_from_bit(bool L, bool F, bool R) {
  uint8_t left = L ? 0b100 : 0;
  uint8_t font = F ? 0b010 : 0;
  uint8_t right = R ? 0b001 : 0;

  return left | font | right;
}
// ==========================================
// 修改1：重构路口特征判定逻辑（核心抗干扰点）
// ==========================================
// ==========================================
// 升级版：带“连续性记忆”的路口特征判定
// ==========================================


  // 针对正方形赛道的暴力过滤逻辑：
  // 赛道上根本不存在真正的十字路口或T型路口！
  // 任何包含前方有线的“路口”，统统都是 BC 边上的干扰！
  //if (raw_road == CrossRoad || raw_road == TBRoad || raw_road == TLRoad || raw_road == TRRoad) {
     //return Straight; // 强行滤除干扰，当做直道处理
  //}

  // 如果前方没线，且左右有线，那才是真正的 A,B,C,D 顶点弯道
  //return raw_road;
//}

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
// if (cross->cross == Straight) {
  //   if ((cross->data_buf & 0xC3)) {
  //     if (cross->maybe == 0) {
  //       cross->maybe = cross->integral_times;
  //       cross->integral = cross->integral | cross->data_buf;
  //     }
  //   }
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