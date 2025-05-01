/*
 * Copyright (c) 2021, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "arm_const_structs.h"
#include "arm_math.h"
#include "ti/driverlib/dl_adc12.h"
#include "ti_msp_dl_config.h"

#define ADC_SAMPLE_SIZE 2048
#define maxn(a, b) (a > b ? a : b)
#define minn(a, b) (a < b ? a : b)

float FFT_INPUT[ADC_SAMPLE_SIZE * 2];
float FFT_OUTPUT[ADC_SAMPLE_SIZE];
float FFT_OUTPUT_MAX = 0;
uint32_t FFT_OUTPUT_MAX_index = 0;

uint16_t gADCSamples[ADC_SAMPLE_SIZE];
volatile bool gCheckADC;
float freq;
float vpp;
uint32_t adc_fs = 4e5;
uint16_t point;

void StartAdc(int freq) {
  DL_TimerA_stopCounter(TIMER_0_INST);
  uint16_t sampletime = 32e6 / freq;
  DL_TimerA_setLoadValue(TIMER_0_INST, sampletime);
  int check = DL_TimerA_getLoadValue(TIMER_0_INST);
  DL_TimerA_startCounter(TIMER_0_INST);
  gCheckADC = false;
  // DL_ADC12_enableConversions(ADC12_0_INST);
  // DL_ADC12_startConversion(ADC12_0_INST);
  while (gCheckADC == false)
    ;
  DL_TimerA_stopCounter(TIMER_0_INST);
  return;
}

int switchwave() {
  uint16_t waveform = 0;
  float k =
      FFT_OUTPUT[FFT_OUTPUT_MAX_index] / FFT_OUTPUT[3 * FFT_OUTPUT_MAX_index];
  if (k < 9 && k > 1)
    waveform = 2; // 方波
  if (k < 20 && k > 9)
    waveform = 3; // 三角波
  if (k > 20)
    waveform = 1; // 正弦波
  return waveform;
}

float vppfft;

float findvpp() {
  StartAdc(adc_fs);
  float ans = -1;
  float e = 0.0;
  float power = 0.0;
  point = adc_fs / freq;
  uint16_t i;
  int multi = 3;
  for (i = 0; i < point * multi; i++) {
    e = e + ((powf((gADCSamples[i] * 3.3 / 4096 - vppfft / 2), 2) +
              powf((gADCSamples[i + 1] * 3.3 / 4096 - vppfft / 2), 2)) /
             adc_fs / 2);
  }
  __BKPT();
  power = e * freq / multi;
  int waveform = switchwave();
  switch (waveform) {
  case 1: // 正弦波
    ans = 2 * sqrtf(2 * power);
    break;
  case 2: // 方波
    ans = 2 * sqrtf(power);
    break;
  case 3: // 三角波
    ans = 2 * sqrtf(3 * power);
    break;
  default:
    break;
  }
  return ans;
}

void findbase() {
  adc_fs = 400e3;
  StartAdc(adc_fs);
  StartAdc(adc_fs);
  uint16_t i;
  for (i = 0; i < ADC_SAMPLE_SIZE; i++) {
    FFT_INPUT[i * 2] = (float)(gADCSamples[i]);
    FFT_INPUT[i * 2 + 1] = 0;
  }

  arm_cfft_f32(&arm_cfft_sR_f32_len2048, FFT_INPUT, 0, 1);
  arm_cmplx_mag_f32(FFT_INPUT, FFT_OUTPUT, ADC_SAMPLE_SIZE);

  FFT_OUTPUT[0] = 0;
  arm_max_f32(FFT_OUTPUT, ADC_SAMPLE_SIZE, &FFT_OUTPUT_MAX,
              &FFT_OUTPUT_MAX_index);

  float base_frequency = adc_fs * FFT_OUTPUT_MAX_index / ADC_SAMPLE_SIZE;
  if (base_frequency <= 500) {
    adc_fs = 5e3;
    // __BKPT();
    return;
  } else if (500 < base_frequency && base_frequency < 1e3) {
    adc_fs = base_frequency * 20;
    // __BKPT();
    return;
  } else if (base_frequency < 70e3) {
    adc_fs = base_frequency * 20;
    // __BKPT();
    return;
  } else {
    adc_fs = 400e3;
    // __BKPT();
    return;
  }
}

float findfreq() {
  StartAdc(adc_fs);
  StartAdc(adc_fs);
  StartAdc(adc_fs);
  StartAdc(adc_fs);
  uint16_t check = DL_TimerA_getLoadValue(TIMER_0_INST);
  uint16_t i;
  for (i = 0; i < ADC_SAMPLE_SIZE; i++) {
    FFT_INPUT[i * 2] = (float)(gADCSamples[i]);
    FFT_INPUT[i * 2 + 1] = 0;
  }

  int Vmax[5], Vmin[5];

  // 初始化最大值数组为最小值
  Vmax[0] = Vmax[1] = Vmax[2] = Vmax[3] = Vmax[4] = -1;
  // 初始化最小值数组为最大值
  Vmin[0] = Vmin[1] = Vmin[2] = Vmin[3] = Vmin[4] = 0xFFFF;

  for (int i = 0; i < ADC_SAMPLE_SIZE; i++) {
    uint16_t current = gADCSamples[i];

    // 更新前五大最大值
    if (current > Vmax[0]) {
      Vmax[4] = Vmax[3];
      Vmax[3] = Vmax[2];
      Vmax[2] = Vmax[1];
      Vmax[1] = Vmax[0];
      Vmax[0] = current;
    } else if (current > Vmax[1]) {
      Vmax[4] = Vmax[3];
      Vmax[3] = Vmax[2];
      Vmax[2] = Vmax[1];
      Vmax[1] = current;
    } else if (current > Vmax[2]) {
      Vmax[4] = Vmax[3];
      Vmax[3] = Vmax[2];
      Vmax[2] = current;
    } else if (current > Vmax[3]) {
      Vmax[4] = Vmax[3];
      Vmax[3] = current;
    } else if (current > Vmax[4]) {
      Vmax[4] = current;
    }

    // 更新前五最小值
    if (current < Vmin[0]) {
      Vmin[4] = Vmin[3];
      Vmin[3] = Vmin[2];
      Vmin[2] = Vmin[1];
      Vmin[1] = Vmin[0];
      Vmin[0] = current;
    } else if (current < Vmin[1]) {
      Vmin[4] = Vmin[3];
      Vmin[3] = Vmin[2];
      Vmin[2] = Vmin[1];
      Vmin[1] = current;
    } else if (current < Vmin[2]) {
      Vmin[4] = Vmin[3];
      Vmin[3] = Vmin[2];
      Vmin[2] = current;
    } else if (current < Vmin[3]) {
      Vmin[4] = Vmin[3];
      Vmin[3] = current;
    } else if (current < Vmin[4]) {
      Vmin[4] = current;
    }
  }

  // 计算峰峰值：使用第2~5大/小值平均，避免异常点干扰
  float avg_max = (Vmax[0] + Vmax[4]) / 2.0f;
  float avg_min = (Vmin[0] + Vmin[4]) / 2.0f;

  vppfft = (avg_max - avg_min) * 3.3f / 4096.0f;
  arm_cfft_f32(&arm_cfft_sR_f32_len2048, FFT_INPUT, 0, 1);
  arm_cmplx_mag_f32(FFT_INPUT, FFT_OUTPUT, ADC_SAMPLE_SIZE);

  FFT_OUTPUT[0] = 0;
  arm_max_f32(FFT_OUTPUT, ADC_SAMPLE_SIZE / 2, &FFT_OUTPUT_MAX,
              &FFT_OUTPUT_MAX_index);
  // FFT_OUTPUT[0] = 0;
  // arm_max_f32(FFT_OUTPUT, ADC_SAMPLE_SIZE / 2, &FFT_OUTPUT_MAX,
  //             &FFT_OUTPUT_MAX_index);

  float base_frequency = adc_fs * FFT_OUTPUT_MAX_index / ADC_SAMPLE_SIZE;
  return base_frequency;
}

int main(void) {
  SYSCFG_DL_init();
  DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
                    (uint32_t)0x40556280); // FIFO模式：降低DMA的使用频率
  DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&gADCSamples[0]);
  DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

  /* Setup interrupts on device */
  NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
  // DL_ADC12_startConversion(ADC12_0_INST);

  while (1) {
    // findbase();
    // freq = findfreq();
    // vpp = findvpp();
    int cntwa = 0;
    findbase();
    freq = findfreq();
    vpp = findvpp();
    __BKPT();
  }
}

void ADC12_0_INST_IRQHandler(void) {
  switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
  case DL_ADC12_IIDX_DMA_DONE:
    // DL_TimerA_disableClock(TIMER_0_INST);
    // DL_ADC12_disableConversions(ADC12_0_INST);
    gCheckADC = true;
    break;
  default:
    break;
  }
}
