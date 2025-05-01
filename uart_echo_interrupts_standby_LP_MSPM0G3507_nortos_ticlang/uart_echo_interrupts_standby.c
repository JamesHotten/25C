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

#include "ti_msp_dl_config.h"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define USART_TX_LEN 27
uint8_t USART_TX_BUF[USART_TX_LEN];
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

int main(void) {

  // 初始化数据包
  USART_TX_BUF[0] = 0xA5;                  // 数据包头
  *((float *)(&USART_TX_BUF[1])) = 123.45; // float值
  *((float *)(&USART_TX_BUF[5])) = 65.78;  // float值
  *((float *)(&USART_TX_BUF[9])) = 30.67;  // float值
  *((float *)(&USART_TX_BUF[13])) = 15.25; // float值
  *((float *)(&USART_TX_BUF[17])) = 8.99;  // float值
  *((float *)(&USART_TX_BUF[21])) = 0.08;  // float值

  // 主循环
  while (1) {
    // 发送简单的“hello”消息到蓝牙
    const char hello_msg[] = "Hello, Bluetooth!\n";
    sendBluetoothData((const uint8_t *)hello_msg, strlen(hello_msg));

    // 计算校验和
    uint8_t checksum = 0;
    for (uint8_t i = 1; i < USART_TX_LEN - 2;
         i++) { // 从第二个字节到倒数第二个字节（不包括包尾）
      checksum += USART_TX_BUF[i];
    }
    USART_TX_BUF[USART_TX_LEN - 2] = checksum; // 校验位

    // 添加包尾
    USART_TX_BUF[USART_TX_LEN - 1] = 0x5A; // 包尾字节

    // 发送数据包到蓝牙
    sendBluetoothData(USART_TX_BUF, USART_TX_LEN);
  }
}
