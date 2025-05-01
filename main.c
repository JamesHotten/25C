#include "ti/driverlib/dl_adc12.h"

extern uint32_t adc_fs;

/**
 * @brief 设置 ADC 采样率为目标频率
 *
 * @param target_freq 目标采样频率（Hz）
 */
void set_adc_sampling_rate(float target_freq) {
  // 假设系统时钟为 32MHz
  uint32_t system_clock = 32000000;
  uint32_t adc_clock = system_clock; // 默认不分频

  DL_ADC12_setClockPrescaler(ADC12_0_INST,
                             DL_ADC12_CLOCK_PRESCALER_DIV_1); // 默认不分频

  // 如果目标频率过高，尝试用分频降低 ADC 时钟
  if (target_freq < 1000000) {
    DL_ADC12_setClockPrescaler(ADC12_0_INST, DL_ADC12_CLOCK_PRESCALER_DIV_4);
    adc_clock = system_clock / 4;
  } else if (target_freq < 5000000) {
    DL_ADC12_setClockPrescaler(ADC12_0_INST, DL_ADC12_CLOCK_PRESCALER_DIV_2);
    adc_clock = system_clock / 2;
  }

  // 计算所需总周期数：Total Cycles = ADC_Clock / Target_Frequency
  float total_cycles = (float)adc_clock / target_freq;

  // 转换时间约为 13 个时钟周期，因此采样时间 = 总周期 - 13
  int sample_time = (int)(total_cycles - 13);

  if (sample_time < 1)
    sample_time = 1; // 最小采样时间为 1 cycle
  if (sample_time > 1024)
    sample_time = 1024; // 最大限制

  // 设置采样时间
  if (sample_time <= 4) {
    DL_ADC12_setSampleTime(ADC12_0_INST, DL_ADC12_SAMPLE_TIME_1);
  } else if (sample_time <= 8) {
    DL_ADC12_setSampleTime(ADC12_0_INST, DL_ADC12_SAMPLE_TIME_4);
  } else if (sample_time <= 16) {
    DL_ADC12_setSampleTime(ADC12_0_INST, DL_ADC12_SAMPLE_TIME_8);
  } else if (sample_time <= 32) {
    DL_ADC12_setSampleTime(ADC12_0_INST, DL_ADC12_SAMPLE_TIME_16);
  } else if (sample_time <= 64) {
    DL_ADC12_setSampleTime(ADC12_0_INST, DL_ADC12_SAMPLE_TIME_32);
  } else if (sample_time <= 128) {
    DL_ADC12_setSampleTime(ADC12_0_INST, DL_ADC12_SAMPLE_TIME_64);
  } else if (sample_time <= 256) {
    DL_ADC12_setSampleTime(ADC12_0_INST, DL_ADC12_SAMPLE_TIME_128);
  } else if (sample_time <= 512) {
    DL_ADC12_setSampleTime(ADC12_0_INST, DL_ADC12_SAMPLE_TIME_256);
  } else {
    DL_ADC12_setSampleTime(ADC12_0_INST, DL_ADC12_SAMPLE_TIME_512);
  }

  // 更新全局变量供 findfreq 使用
  adc_fs = (uint32_t)target_freq;
}