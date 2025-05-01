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
// #include "ti/driverlib/dl_timer.h"
// #include "ti/driverlib/dl_timera.h"
// #include "ti/driverlib/dl_timerg.h"
#include "ti_msp_dl_config.h"

#define ADC_SAMPLE_SIZE 2048

float FFT_INPUT[ADC_SAMPLE_SIZE * 2];
float FFT_OUTPUT[ADC_SAMPLE_SIZE];
float FFT_OUTPUT_MAX = 0;
uint32_t FFT_OUTPUT_MAX_index = 0;

uint16_t gADCSamples[ADC_SAMPLE_SIZE];
volatile bool gCheckADC;

uint32_t adc_fs = 4e5;

void StartAdc(int freq) {
  uint16_t sampletime = 32e6 / freq;
  DL_ADC12_setSampleTime0(ADC12_0_INST, sampletime);
  int check = DL_ADC12_getSampleTime0(ADC12_0_INST);
  gCheckADC = false;
  DL_ADC12_enableConversions(ADC12_0_INST);
  DL_ADC12_startConversion(ADC12_0_INST);
  while (gCheckADC == false)
    ;
  return;
}

void findbase() {
  adc_fs = 400e3;
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
    __BKPT();
    return;
  } else if (500 < base_frequency && base_frequency < 1e3) {
    adc_fs = 10e3;
    __BKPT();
    return;
  } else if (base_frequency < 10e3) {
    adc_fs = 100e3;
    __BKPT();
    return;
  } else {
    adc_fs = 400e3;
    __BKPT();
    return;
  }
}

float findfreq() {
  StartAdc(adc_fs);
  uint16_t check = DL_ADC12_getSampleTime0(ADC12_0_INST);
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
  // FFT_OUTPUT[0] = 0;
  // arm_max_f32(FFT_OUTPUT, ADC_SAMPLE_SIZE / 2, &FFT_OUTPUT_MAX,
  //             &FFT_OUTPUT_MAX_index);

  float base_frequency = adc_fs * FFT_OUTPUT_MAX_index / ADC_SAMPLE_SIZE;
  return base_frequency;
}

int main(void) {
  SYSCFG_DL_init();
  DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID,
                    (uint32_t)DL_ADC12_getFIFOAddress(
                        ADC12_0_INST)); // FIFO模式：降低DMA的使用频率
  DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&gADCSamples[0]);
  DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

  /* Setup interrupts on device */
  NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
  DL_ADC12_startConversion(ADC12_0_INST);

  while (1) {
    findbase();
    float freq = findfreq();
    int a = 10;
  }
}

void ADC12_0_INST_IRQHandler(void) {
  switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
  case DL_ADC12_IIDX_DMA_DONE:
    DL_ADC12_disableConversions(ADC12_0_INST);
    gCheckADC = true;
    break;
  default:
    break;
  }
}
