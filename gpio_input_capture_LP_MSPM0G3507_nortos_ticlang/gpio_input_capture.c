
#include "ti_msp_dl_config.h"
volatile uint8_t ledState = 0; // 0表示LED关闭，1表示LED打开

int main(void) {
  SYSCFG_DL_init();

  /*
   * Turn OFF LED if SW is open, ON if SW is closed.
   * LED starts OFF by default.
   */
  DL_GPIO_clearPins(GPIO_LEDS_PORT, GPIO_LEDS_USER_LED_1_PIN);
  NVIC_EnableIRQ(GPIO_SWITCHES_INT_IRQN);

  while (1) {
    __WFI();
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
              // 如果ledState为1，则打开LED
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
// 二级放大  、、放大倍数
