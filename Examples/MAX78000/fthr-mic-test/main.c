/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All rights Reserved.
*
* This software is protected by copyright laws of the United States and
* of foreign countries. This material may also be protected by patent laws
* and technology transfer regulations of the United States and of foreign
* countries. This software is furnished under a license agreement and/or a
* nondisclosure agreement and may only be used or reproduced in accordance
* with the terms of those agreements. Dissemination of this information to
* any party or parties not specified in the license agreement and/or
* nondisclosure agreement is expressly prohibited.
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
* OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL MAXIM INTEGRATED BE LIABLE FOR ANY CLAIM, DAMAGES
* OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
* ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
* OTHER DEALINGS IN THE SOFTWARE.
*
* Except as contained in this notice, the name of Maxim Integrated
* Products, Inc. shall not be used except as stated in the Maxim Integrated
* Products, Inc. Branding Policy.
*
* The mere transfer of this software does not imply any licenses
* of trade secrets, proprietary technology, copyrights, patents,
* trademarks, maskwork rights, or any other form of intellectual
* property whatsoever. Maxim Integrated Products, Inc. retains all
* ownership rights.
*******************************************************************************/
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "mxc_sys.h"
#include "board.h"
#include "led.h"
#include "bbfc_regs.h"
#include "icc.h"
#include "gpio_regs.h"
#include "mxc_delay.h"
#include "i2s_regs.h"
#include "i2s.h"
#include "nvic_table.h"
#include "hpf.h"

#undef CONSOLE_METER

#define CHUNK               128   // number of data points to read at a time and average for threshold, keep multiple of 128

#define I2S_RX_BUFFER_SIZE  64    // I2S buffer size

#define SAMPLE_SCALE_FACTOR 4     // multiplies 16-bit samples by this scale factor before converting to 8-bit
#define THRESHOLD_HIGH      350   // voice detection threshold to find beginning of a keyword


int16_t Max, Min;
uint16_t thresholdHigh = THRESHOLD_HIGH;

volatile uint8_t i2s_flag = 0;
int32_t i2s_rx_buffer[I2S_RX_BUFFER_SIZE];

void i2s_isr(void)
{
    i2s_flag = 1;
    /* Clear I2S interrupt flag */
    MXC_I2S_ClearFlags(MXC_F_I2S_INTFL_RX_THD_CH0);
}

uint8_t MicReadChunk(uint8_t *pBuff, uint16_t * avg)
{
  static uint16_t chunkCount = 0;
  static uint16_t sum = 0;

  static uint32_t index = 0;

  int32_t sample = 0;
  int16_t temp = 0;
  uint32_t rx_size = 0;

  /* sample not ready */
  if (!i2s_flag) {
    *avg = 0;
    return 0;
  }

  /* Clear flag */
  i2s_flag = 0;
  /* Read number of samples in I2S RX FIFO */
  rx_size = MXC_I2S->dmach0 >> MXC_F_I2S_DMACH0_RX_LVL_POS;

  /* read until fifo is empty or enough samples are collected */
  while ((rx_size--) && (chunkCount < CHUNK)) {
    /* Read microphone sample from I2S FIFO */
    sample = (int32_t)MXC_I2S->fifoch0;
    /* The actual value is 18 MSB of 32-bit word */
    temp = sample >> 14;

    /* Remove DC from microphone signal */
    sample = HPF((int16_t)temp); // filter needs about 1K sample to converge

    /* Discard first 10k samples due to microphone charging cap effect */
    if (index++ < 10000)
      continue;

    /* absolute for averaging */
    if (sample >= 0)
      sum += sample;
    else
      sum -= sample;

    /* Convert to 8 bit unsigned */
    pBuff[chunkCount] = (uint8_t)((sample)*SAMPLE_SCALE_FACTOR/256);

    temp=(int8_t)pBuff[chunkCount];

    chunkCount++;

    /* record max and min */
    if (temp > Max)
      Max = temp;
    if (temp < Min)
      Min = temp;
  }

  /* if not enough samples, return 0 */
  if (chunkCount < CHUNK) {
    *avg = 0;
    return 0;
  }

  /* enough samples are collected, calculate average and return 1 */
  *avg = ((uint16_t)(sum / CHUNK));

  chunkCount = 0;
  sum = 0;
  return 1;
}

void I2SInit(void)
{
  mxc_i2s_req_t req;
  int32_t err;

  /* Initialize High Pass Filter */
  HPF_init();
  /* Initialize I2S RX buffer */
  memset(i2s_rx_buffer, 0, sizeof(i2s_rx_buffer));
  /* Configure I2S interface parameters */
  req.wordSize    = MXC_I2S_DATASIZE_WORD;
  req.sampleSize  = MXC_I2S_SAMPLESIZE_THIRTYTWO;
  req.justify     = MXC_I2S_MSB_JUSTIFY;
  req.wsPolarity  = MXC_I2S_POL_NORMAL;
  req.channelMode = MXC_I2S_INTERNAL_SCK_WS_0;
  /* Get only left channel data from on-board microphone. Right channel samples are zeros */
  req.stereoMode  = MXC_I2S_MONO_LEFT_CH;
  req.bitOrder  = MXC_I2S_MSB_FIRST;
  /* I2S clock = PT freq / (2*(req.clkdiv + 1)) */
  /* I2S sample rate = I2S clock/64 = 16kHz */
  req.clkdiv      = 5;
  req.rawData     = NULL;
  req.txData      = NULL;
  req.rxData      = i2s_rx_buffer;
  req.length      = I2S_RX_BUFFER_SIZE;


  if((err = MXC_I2S_Init(&req)) != E_NO_ERROR) {
    printf("\nError in I2S_Init: %d\n", err);
    while (1);
  }

  /* Set I2S RX FIFO threshold to generate interrupt */
  MXC_I2S_SetRXThreshold(4);
  NVIC_SetVector(I2S_IRQn, i2s_isr);
  NVIC_EnableIRQ(I2S_IRQn);
  /* Enable RX FIFO Threshold Interrupt */
  MXC_I2S_EnableInt(MXC_F_I2S_INTEN_RX_THD_CH0);
  MXC_I2S_RXEnable();
  __enable_irq();
}

int main()
{
#ifdef CONSOLE_METER
  uint32_t sampleCounter = 0;
#endif
  uint8_t pChunkBuff[CHUNK];
  uint16_t avg = 0;
  uint32_t fade_cnt = 0;

  // Wait for PMIC 1.8V to become available, about 180ms after power up.
  MXC_Delay(200000);

  MXC_ICC_Enable(MXC_ICC0); // Enable cache

  // Switch to 100 MHz clock
  MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
  SystemCoreClockUpdate();

  /* Enable microphone power */
  Microphone_Power(1);

  LED_On(LED_GREEN);
  MXC_Delay(1000000);
  LED_Off(LED_GREEN);

  /* initialize I2S interface to Mic */
  I2SInit();

  for (;;) {
    if (fade_cnt) {
      if (!(--fade_cnt)) {
        LED_Off(LED_GREEN);
      }
    }

    /* Read from Mic driver to get CHUNK worth of samples, otherwise next sample*/
    if (MicReadChunk(pChunkBuff, &avg) == 0) {
      continue;
    }

#ifdef CONSOLE_METER
    sampleCounter += CHUNK;

    /* Display average envelope as a bar */
    printf("%.6d|",sampleCounter);
    for (int i = 0; i < avg / 10; i++)
      printf("=");
    if (avg >= thresholdHigh)
      printf("*");
    printf("[%d]\n",avg);
#endif

    if (avg >= thresholdHigh) {
      fade_cnt = 5000;
      LED_On(LED_GREEN);
    }
  }
}
