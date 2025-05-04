/*
 * Copyright (c) 2020, Texas Instruments Incorporated
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
#include "oled.h"
#include "ti_msp_dl_config.h"
#include "waveform.h"
#define ADC_SAMPLE_SIZE 1024
// 变量定义
///////////////////////////////////////////////////////////////////////
// 测频法模块定义变量
volatile uint32_t count = 0;
volatile float freq_pin;
// 蓝牙模块定义变量，引脚定义RX对应PA11，TX对应PA10

#define USART_TX_LEN_SRC 15 // 发送蓝牙数据包

uint8_t USART_TX_BUF_SRC[USART_TX_LEN_SRC];

uint32_t adc_fs = 4e5;
float avg_max = 0;
float avg_min = 3;
// ADC采样
// PA27引脚
float base_frequency = 0;
float FFT_INPUT[ADC_SAMPLE_SIZE * 2];
float FFT_OUTPUT[ADC_SAMPLE_SIZE];
uint16_t waveform;
float FFT_OUTPUT_MAX = 0;
uint32_t FFT_OUTPUT_MAX_index = 0;
uint16_t gADCSamples[ADC_SAMPLE_SIZE];
volatile bool gCheckADC;

uint32_t point = 0;
uint8_t multipler = 1;
float_t vppadc = 0;
volatile uint8_t ledState = 0;
////////////////////////////////////////////////////////////////////////

// 函数封装
////////////////////////////////////////////////////////////////////////

// 蓝牙模块发送函数
void sendBluetoothData(const uint8_t *data, uint32_t length) {
  uint32_t i;
  for (i = 0; i < length; i++) {
    while (DL_UART_Extend_isTXFIFOEmpty(UART_0_INST) == 0)
      ; // 等待发送缓冲区有空间
    DL_UART_Extend_transmitData(UART_0_INST, data[i]);

    while (DL_UART_isBusy(UART_0_INST))
      ;
  }
}

// 钟控ADC,改为TIMER1启用
void StartAdc(int freq) {
  DL_TimerA_stopCounter(TIMER_1_INST);
  uint16_t sampletime = 32e6 / freq;
  DL_TimerA_setLoadValue(TIMER_1_INST, sampletime);
  int check = DL_TimerA_getLoadValue(TIMER_1_INST);
  DL_TimerA_startCounter(TIMER_1_INST);
  gCheckADC = false;
  while (gCheckADC == false)
    ;
  DL_TimerA_stopCounter(TIMER_1_INST);

  return;
}

void findbase(void) {
  // adc_fs = 400e3;
  // StartAdc(adc_fs);
  // StartAdc(adc_fs);

  //   uint16_t i;
  //   for (i = 0; i < ADC_SAMPLE_SIZE; i++) {
  //     FFT_INPUT[i * 2] = (float)(gADCSamples[i]);
  //     FFT_INPUT[i * 2 + 1] = 0;
  //   }

  //   arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_INPUT, 0, 1);
  //   arm_cmplx_mag_f32(FFT_INPUT, FFT_OUTPUT, ADC_SAMPLE_SIZE);

  //   FFT_OUTPUT[0] = 0;
  //   arm_max_f32(FFT_OUTPUT, ADC_SAMPLE_SIZE, &FFT_OUTPUT_MAX,
  //               &FFT_OUTPUT_MAX_index);

  //   base_frequency = adc_fs * FFT_OUTPUT_MAX_index / ADC_SAMPLE_SIZE;
  if (freq_pin <= 50) {
    multipler = 40;
  } else if (freq_pin < 5000) {
    multipler = 10;
  } else {
    multipler = 10;
  }
  adc_fs = freq_pin * multipler;
}

/////寻找V
void findV(void) {
  StartAdc(adc_fs);
  StartAdc(adc_fs);
  // __BKPT();
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
  avg_max = (Vmax[0] + Vmax[4]) / 2.0f;
  avg_min = (Vmin[0] + Vmin[4]) / 2.0f;
  vppadc = (avg_max - avg_min) * 3.3f / 4096.0f;
}

////

int switchwave() {
  // 准确判断波形类别
  waveform = 0;
  float_t line = 0;

  point = 0;
  int t = 0;
  float harm;
  line = (avg_max - avg_min) * 0.25 + (avg_max + avg_min) * 0.5;
  for (t = 10; t < 12 * multipler + 10; t++) {
    if (gADCSamples[t] > line) {
      point++;
    }
  }
  if (point < 3 * multipler + 5) {
    waveform = 3; // 三角波
  } else if (point < 4 * multipler + 5) {
    waveform = 1; // 正弦波
  } else if (point < 6 * multipler + 25) {
    waveform = 2; // 方波
  }
  return waveform;
}

// 法一：利用FFT结果去计算
// int switchwave() {
//   uint16_t waveform = 0;
//   float harm;
//   harm = maxn(maxn(FFT_OUTPUT[3 * FFT_OUTPUT_MAX_index],
//                    FFT_OUTPUT[3 * FFT_OUTPUT_MAX_index + 1]),
//               FFT_OUTPUT[3 * FFT_OUTPUT_MAX_index + 2]);
//   float k = FFT_OUTPUT[FFT_OUTPUT_MAX_index] / harm; // 需要加一个for循环
//   if (k < 9 && k > 1)
//     waveform = 2; // 方波
//   if (k < 20 && k > 9)
//     waveform = 3; // 三角波
//   if (k > 20)
//     waveform = 1; // 正弦波
//   return waveform;
// }

int getlen(float val) {
  int reg = val;
  int len = 0;
  while (val >= 1) {
    val /= 10;
    len++;
  }
  return len;
}

char str[6] = {'.'};
void getdec(float val, int bit) {
  int ans = val * pow(10, bit);
  for (int i = bit; i > 0; i--) {
    str[i] = ans % 10 + '0';
    ans /= 10;
  }
  str[bit + 1] = '\0';
  return;
}
// OLED显示模块部分
// SCL接PA12，SDA接PA13
void OLEDSHOW() {
  int waveform = switchwave();
  OLED_ShowString(0, 0, (uint8_t *)"wave:", 16);
  if (waveform == 1) {
    OLED_ShowString(50, 0, (uint8_t *)"sin", 16);
  } else if (waveform == 2) {
    OLED_ShowString(50, 0, (uint8_t *)"square", 16);
  } else if (waveform == 3) {
    OLED_ShowString(50, 0, (uint8_t *)"triangle", 16);
  } else {
    { OLED_ShowString(50, 0, (uint8_t *)"wait", 16); }
  }

  float vpp = vppadc * 1e3;
  int len = getlen(vpp);
  OLED_ShowString(0, 4, (uint8_t *)"vpp:", 16);
  OLED_ShowNum(50, 4, (int)(vpp), len, 16);
  getdec(vpp, 2);
  OLED_ShowString(50 + len * 8, 4, (uint8_t *)str, 16);
  OLED_ShowString(50 + (len + 3) * 8, 4, (uint8_t *)"mV", 16);

  len = getlen(freq_pin);
  if (freq_pin < 10) {
    OLED_ShowString(0, 2, (uint8_t *)"freq:", 16);
    OLED_ShowNum(50, 2, (int)(freq_pin), len, 16);
    getdec(freq_pin, 4);
    OLED_ShowString(50 + len * 8, 2, (uint8_t *)str, 16);
    OLED_ShowString(50 + (len + 5) * 8, 2, (uint8_t *)"Hz", 16);
  } else if (freq_pin < 100) {
    OLED_ShowString(0, 2, (uint8_t *)"freq:", 16);
    OLED_ShowNum(50, 2, (int)(freq_pin), len, 16);
    getdec(freq_pin, 3);
    OLED_ShowString(50 + len * 8, 2, (uint8_t *)str, 16);
    OLED_ShowString(50 + (len + 4) * 8, 2, (uint8_t *)"Hz", 16);
  } else if (freq_pin < 1e3) {
    OLED_ShowString(0, 2, (uint8_t *)"freq:", 16);
    OLED_ShowNum(50, 2, (int)(freq_pin), len, 16);
    getdec(freq_pin, 2);
    OLED_ShowString(50 + len * 8, 2, (uint8_t *)str, 16);
    OLED_ShowString(50 + (len + 3) * 8, 2, (uint8_t *)"Hz", 16);
  } else if (freq_pin < 1e4) {
    float freq = freq_pin / 1e3;
    len -= 3;
    OLED_ShowString(0, 2, (uint8_t *)"freq:", 16);
    OLED_ShowNum(50, 2, (int)(freq), len, 16);
    getdec(freq, 4);
    OLED_ShowString(50 + len * 8, 2, (uint8_t *)str, 16);
    OLED_ShowString(50 + (len + 5) * 8, 2, (uint8_t *)"kHz", 16);
  } else {
    len -= 3;
    float freq = freq_pin / 1e3;
    OLED_ShowString(0, 2, (uint8_t *)"freq:", 16);
    OLED_ShowNum(50, 2, (int)(freq), len, 16);
    OLED_ShowString(50 + (len + 4) * 8, 2, (uint8_t *)"kHz", 16);
    getdec(freq, 3);
    OLED_ShowString(50 + len * 8, 2, (uint8_t *)str, 16);
    OLED_ShowString(50 + (len + 4) * 8, 2, (uint8_t *)"kHz", 16);
  }

  delay_cycles(32e6); // 每秒刷新一次
  OLED_Clear();
}

// 蓝牙模块发送数据定义
// RX PA11   TX PA10

void BT_send_Src() {
  // 初始化数据包
  uint16_t i;
  for (i = 0; i < 100; i++) {
    USART_TX_BUF_SRC[0] = 0xA5;
    float wave = 0.0f;
    if (waveform == 1)
      wave = sin_table[i] * vppadc / 4096;
    else if (waveform == 2)
      wave = square_table[i] * vppadc / 4096;
    else if (waveform == 3)
      wave = triangle_table[i] * vppadc / 4096;
    memcpy(&USART_TX_BUF_SRC[1], &wave, sizeof(wave));
    *((float *)(&USART_TX_BUF_SRC[5])) = freq_pin;
    *((float *)(&USART_TX_BUF_SRC[9])) = vppadc;
    // 计算校验和
    uint8_t checksum = 0;
    for (int i = 1; i < USART_TX_LEN_SRC - 2;
         i++) { // 从第二个字节到倒数第二个字节（不包括包尾）
      checksum += USART_TX_BUF_SRC[i];
    }
    USART_TX_BUF_SRC[USART_TX_LEN_SRC - 2] = checksum; // 校验位

    // 添加包尾
    USART_TX_BUF_SRC[USART_TX_LEN_SRC - 1] = 0x5A; // 包尾字节
    while (DL_UART_isBusy(UART_0_INST))
      ;
    // 发送数据包到蓝牙
    sendBluetoothData(USART_TX_BUF_SRC, USART_TX_LEN_SRC);
  }
  return;
}

int main(void) {
  SYSCFG_DL_init();

  OLED_Init();
  OLED_Clear();

  // DMA传输
  DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)0x40556280);
  DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&gADCSamples[0]);
  DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
  NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);

  // 测频法（50赫兹以上肯定正确，精度非常高，但是50赫兹以下小概率出错，但是误差也很小）
  // 引脚PB10//显示有一点点小问题，不能实时显示
  DL_TimerG_setCoreHaltBehavior(CAPTURE_0_INST, DL_TIMER_CORE_HALT_IMMEDIATE);
  NVIC_SetPriority(TIMER_0_INST_INT_IRQN, 0); // 设置 TIMER_0 中断优先级为 0
  NVIC_SetPriority(CAPTURE_0_INST_INT_IRQN, 2); // 设置 CAPTURE_0 中断优先级为 2
  NVIC_EnableIRQ(TIMER_0_INST_INT_IRQN);
  NVIC_EnableIRQ(CAPTURE_0_INST_INT_IRQN);
  DL_TimerA_startCounter(TIMER_0_INST);
  DL_TimerG_startCounter(CAPTURE_0_INST);
  // 按键使能
  DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
  NVIC_EnableIRQ(GPIO_SWITCHES_INT_IRQN);
  // OLED初始化

  while (1) {
    // 为了防止ADC和测频法互相干扰，必须要先加入延时
    delay_cycles(32e6);
    findbase();
    findV();
    OLEDSHOW();

    BT_send_Src();
  }
  return 0;
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

// 测频法捕获中断
void CAPTURE_0_INST_IRQHandler(void) {
  switch (DL_TimerG_getPendingInterrupt(CAPTURE_0_INST)) {
  case DL_TIMERG_IIDX_CC1_DN:
    count++;
    break;
  case DL_TIMERG_IIDX_ZERO:
    break;
  default:
    break;
  }
}

// 测频法总时钟控制，以1s为测量时间，时间不用担心
void TIMER_0_INST_IRQHandler(void) {
  switch (DL_TimerA_getPendingInterrupt(TIMER_0_INST)) {
  case DL_TIMER_IIDX_ZERO:
    DL_Timer_clearInterruptStatus(TIMER_0_INST, DL_TIMER_INTERRUPT_ZERO_EVENT);
    freq_pin = (float)count * 1.01106;
    DL_TimerA_stopCounter(TIMER_0_INST);
    DL_TimerA_startCounter(TIMER_0_INST);
    count = 0;
    break;
  default:
    break;
  }
}

void GROUP1_IRQHandler(void) {
  switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
  case GPIO_SWITCHES_INT_IIDX:
    /* If SW is high, turn the LED off */
    for (volatile int i = 0; i < 10000; i++)
      ; // 简单的延时
    if (!DL_GPIO_readPins(GPIO_SWITCHES_PORT,
                          GPIO_SWITCHES_USER_SWITCH_1_PIN)) {
      DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
      ledState = !ledState;
    }
    if (ledState) {
      DL_GPIO_setPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);

    }
    /* Otherwise, turn the LED on */
    else {
      DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
    }

    DL_Interrupt_clearGroup(DL_INTERRUPT_GROUP_1, GPIO_SWITCHES_INT_IIDX);
    break;
  }
}

////////////////////////////////////////////////
