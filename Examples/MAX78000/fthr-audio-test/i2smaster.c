/*
    i2smaster.c
*/

#include <stdint.h>
#include <stdlib.h>
#include "mxc_sys.h"
#include "i2s.h"
#include "i2s_regs.h"

#define EXT_CLK            12288000
#define BITS_PER_CHANNEL   16
#define CHANNELS_PER_FRAME 2
#define SAMPLE_RATE        24000

#define BIT_CLK (SAMPLE_RATE * CHANNELS_PER_FRAME * BITS_PER_CHANNEL)
#define CLK_DIV (((EXT_CLK / 2) / BIT_CLK) - 1)

volatile int i2s_flag = 0;
volatile uint32_t i2s_irq_flags = 0;

void I2S_IRQHandler(void)
{
    i2s_irq_flags = MXC_I2S_GetFlags();
    MXC_I2S_ClearFlags(i2s_irq_flags);
    i2s_flag = 1;
}

int i2s_init(void)
{
    int32_t err;
    mxc_i2s_req_t req;

    #define FAKE_BUFFER_SIZE 2
    static int32_t fake_buffer[FAKE_BUFFER_SIZE];

    req.wordSize    = MXC_I2S_DATASIZE_HALFWORD;
    req.sampleSize  = MXC_I2S_SAMPLESIZE_SIXTEEN;
    req.justify     = MXC_I2S_MSB_JUSTIFY;
    req.wsPolarity  = MXC_I2S_POL_NORMAL;
    req.channelMode = MXC_I2S_INTERNAL_SCK_WS_0;
    req.stereoMode  = 0;

    req.bitOrder    = MXC_I2S_MSB_FIRST;
    req.clkdiv      = CLK_DIV;

    req.rawData     = NULL;
    req.txData      = fake_buffer;
    req.rxData      = fake_buffer;
    req.length      = FAKE_BUFFER_SIZE;

    if ((err = MXC_I2S_Init(&req)) != E_NO_ERROR) {
        return err;
    }

    MXC_I2S_SetRXThreshold(4);
    NVIC_EnableIRQ(I2S_IRQn);
    MXC_I2S_EnableInt(MXC_F_I2S_INTEN_RX_THD_CH0);
    MXC_I2S_EnableInt(MXC_F_I2S_INTEN_TX_HE_CH0);
    MXC_I2S_RXEnable();
    MXC_I2S_TXEnable();

    return E_NO_ERROR;
}
