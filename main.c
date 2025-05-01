#include "arm_const_structs.h"
#include "arm_math.h"
#include "ti_msp_dl_config.h"

#define NUM_SAMPLES 256
#define ADC_SAMPLE_SIZE 1024

uint16_t gADCSamples[ADC_SAMPLE_SIZE];
volatile bool gCheckADC;

float FFT_INPUT[NUM_SAMPLES] = {0};
float FFT_OUTPUT[NUM_SAMPLES * 2];
float FFT_OUTPUT_MAX = 0;
uint32_t FFT_OUTPUT_MAX_index = 0;
float findfreq() {
  gCheckADC = false;
  DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);
  DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, (uint32_t)&gADCSamples[0]);
  DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, ADC_SAMPLE_SIZE);
  DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
  DL_ADC12_enableDMA(ADC12_0_INST);
  DL_ADC12_startConversion(ADC12_0_INST);
  while (false == gCheckADC) {
    __WFE();
  }
  int i;
  for (i = 0; i < ADC_SAMPLE_SIZE; i++) {
    FFT_INPUT[i * 2] = (float)(gADCSamples[i]);
    FFT_INPUT[i * 2 + 1] = 0;
  }
  arm_cfft_f32(&arm_cfft_sR_f32_len1024, FFT_INPUT, 0, 1);
  arm_cmplx_mag_f32(FFT_INPUT, FFT_OUTPUT, NUM_SAMPLES);

  // arm_max_f32(FFT_OUTPUT, NUM_SAMPLES, &FFT_OUTPUT_MAX,
  // &FFT_OUTPUT_MAX_index);
  FFT_OUTPUT[0] = 0;
  arm_max_f32(FFT_OUTPUT, NUM_SAMPLES / 2, &FFT_OUTPUT_MAX,
              &FFT_OUTPUT_MAX_index);
  float base_frequency = adc_fs * FFT_OUTPUT_MAX_index / 1024;

  return base_frequency;
}
int main(void) {
  SYSCFG_DL_init();
  DL_DMA_setSrcAddr(
      DMA, DMA_CH0_CHAN_ID,
      (uint32_t)DL_ADC12_getMemResultAddress(ADC12_0_INST, DL_ADC12_MEM_IDX_0));

  NVIC_EnableIRQ(ADC12_0_INST_INT_IRQN);
  gCheckADC = false;

  while (1) {
  }
}
// arm_cfft_q15(&arm_cfft_sR_q15_len256, (q15_t *)gDstBuffer, IFFTFLAG,
//              BITREVERSE);
// arm_cmplx_mag_q15((q15_t *)gDstBuffer, (q15_t *)gFFTOutput, NUM_SAMPLES);
// arm_max_q15((q15_t *)gFFTOutput, NUM_SAMPLES, (q15_t *)&gFFTmaxValue,
//             (uint32_t *)&gFFTmaxFreqIndex);
void ADC12_0_INST_IRQHandler(void) {
  switch (DL_ADC12_getPendingInterrupt(ADC12_0_INST)) {
  case DL_ADC12_IIDX_DMA_DONE:
    gCheckADC = true;
    break;
  default:
    break;
  }
}
