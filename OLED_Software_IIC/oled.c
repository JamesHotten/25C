//////////////////////////////////////////////////////////////////////////////////
//  文 件 名   : oled.c
//  版 本 号   : v2.0
//  作    者   : Torris
//  生成日期   : 2024-07-08
//  最近修改   :
//  功能描述   : 0.96寸OLED 接口演示例程(MSPM0G系列)
//  驱动IC     : SSD1306/SSD1315
//              说明:
//              ----------------------------------------------------------------
//              GND    电源地
//              VCC    接3.3v电源
//              SCL    PA12（时钟）
//              SDA    PA13（数据）
//******************************************************************************/
#include "oled.h"
#include "bmp.h"
#include "ti_msp_dl_config.h"

int waveform = 3;

int main(void) {
  SYSCFG_DL_init();

  OLED_Init(); // 初始化OLED
  while (1) {

    OLED_ShowString(0, 2, (uint8_t *)"wave:", 16);
    if (waveform == 1) {
      OLED_ShowString(50, 2, (uint8_t *)"sin", 16);
    } else if (waveform == 2) {
      OLED_ShowString(50, 2, (uint8_t *)"square", 16);
    } else if (waveform == 3) {
      OLED_ShowString(50, 2, (uint8_t *)"triangle", 16);
    } else {
      { OLED_ShowString(50, 2, (uint8_t *)"wait", 16); }
    }
    OLED_ShowString(0, 4, (uint8_t *)"frequency:", 16);
    OLED_ShowString(0, 6, (uint8_t *)"peaktopeak:", 16);
  }
}
