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
#include <stdio.h>
#include "mxc_sys.h"
#include "board.h"
#include "led.h"
#include "mxc_delay.h"
#include "icc.h"
#include "i2s.h"
#include "i2smaster.h"
#include "i2s_regs.h"
#include "max9867.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "Hann_1024.h"


#define SAMPLE_LENGTH 1024

#define FFT_SIZE SAMPLE_LENGTH

// I2S sample rate
#define SAMPLE_FREQ 24000
// I2S external clock frequency
#define EXT_CLK 12288000


/*
    The tone should appear in or around index i,
    i = round(TONE_FREQ / (SAMPLE_FREQ / SAMPLE_LENGTH))
*/
#define LEFT_FREQ 1000
#define RIGHT_FREQ 2000
#define LEFT_BIN (int)round((float)LEFT_FREQ / ((float)SAMPLE_FREQ / (float)SAMPLE_LENGTH))
#define RIGHT_BIN (int)round((float)RIGHT_FREQ / ((float)SAMPLE_FREQ / (float)SAMPLE_LENGTH))

#define MAX9867_IRQ_PORT MXC_GPIO0
#define MAX9867_IRQ_PIN  MXC_GPIO_PIN_13

int sample_idx = 0;
int16_t samples_left[SAMPLE_LENGTH];
int16_t samples_right[SAMPLE_LENGTH];
float32_t input[SAMPLE_LENGTH * 2]; // 2x for real and imaginary parts
float32_t output[SAMPLE_LENGTH];

int lut_idx = 0;
/*  15-bit sine lookup table, packed into 32-bit entries
    right channel bits 31:16  left channel bits 15:0 */
const uint32_t sine_lut[] = {
    // 2kHz right, 1kHz left
    0x00000000,
    0x1FFF108F,
    0x376C1FFF,
    0x3FFF2D40,
    0x376C376C,
    0x1FFF3DD0,
    0x00003FFF,
    0xE0003DD0,
    0xC893376C,
    0xC0002D40,
    0xC8931FFF,
    0xE000108F,
    0x00000000,
    0x1FFFEF70,
    0x376CE000,
    0x3FFFD2BF,
    0x376CC893,
    0x1FFFC22F,
    0x0000C000,
    0xE000C22F,
    0xC893C893,
    0xC000D2BF,
    0xC893E000,
    0xE000EF70,
};

void max9867_handler(void* cbdata)
{
    int res;

    res = max9867_status();
    if (res < 0) {
        // I2C communications error!
    } else {
        printf("%02X\n", res);
    }
}

int audio_setup(void)
{
    mxc_gpio_cfg_t cfg;

    cfg.port = MAX9867_IRQ_PORT;
    cfg.mask = MAX9867_IRQ_PIN;
    cfg.pad = MXC_GPIO_PAD_NONE;
    cfg.func = MXC_GPIO_FUNC_IN;
    cfg.vssel = MXC_GPIO_VSSEL_VDDIO;
    MXC_GPIO_Config(&cfg);
    MXC_GPIO_RegisterCallback(&cfg, max9867_handler, NULL);
    MXC_GPIO_IntConfig(&cfg, MXC_GPIO_INT_FALLING);
    MXC_GPIO_EnableInt(cfg.port, cfg.mask);
    NVIC_EnableIRQ(MXC_GPIO_GET_IRQ(MXC_GPIO_GET_IDX(MAX9867_IRQ_PORT)));

    i2s_init();

    return max9867_init(MXC_I2C1, EXT_CLK, SAMPLE_FREQ);
}

void unload_fifo(void)
{
    uint32_t n;
    uint32_t tmp;

    n = MXC_I2S->dmach0 >> MXC_F_I2S_DMACH0_RX_LVL_POS;
    n &= 0xFFFFFFFE;

    while (n--) {
        tmp = MXC_I2S->fifoch0;
        samples_left[sample_idx] = tmp & 0x0000FFFF;
        samples_right[sample_idx] = tmp >> 16;
        sample_idx += 1;
        sample_idx &= (SAMPLE_LENGTH - 1);
    }
}

void load_fifo(void)
{
    MXC_I2S->fifoch0 = sine_lut[lut_idx++];
    MXC_I2S->fifoch0 = sine_lut[lut_idx++];
    MXC_I2S->fifoch0 = sine_lut[lut_idx++];
    MXC_I2S->fifoch0 = sine_lut[lut_idx++];

    if (lut_idx >= (sizeof(sine_lut)/sizeof(sine_lut[0]))) {
        lut_idx = 0;
    }
}

int check_for_tone(int16_t* samples, int index)
{
    int i, x;
    float32_t maxVal;
    uint32_t maxIdx;

    /*  Copy integer samples to complex buffer and apply Hann window, the test tones
        are integer multiple of sample frequency, but it's good practice. */
    for (i = x = 0; i < SAMPLE_LENGTH; i++) {
        input[x++] = samples[sample_idx++] * Hann_lut[i]; // real part
        input[x++] = 0.0f;                                // imaginary part
        sample_idx &= SAMPLE_LENGTH - 1;
    }

    arm_cfft_f32(&arm_cfft_sR_f32_len1024, input, 0, 1);
    arm_cmplx_mag_f32(input, output, SAMPLE_LENGTH);
    arm_max_f32(output, SAMPLE_LENGTH / 2, &maxVal, &maxIdx);

    /* Allow for system timing errors in FFT bin resolving */
    if ((maxIdx >= (index - 1)) && (maxIdx <= (index + 1))) {
        return 0;
    }

    return 1;
}

int main()
{
    int ticks = 0;
    int err = 0;

    // Wait for PMIC 1.8V to become available, about 180ms after power up.
    MXC_Delay(200000);

    MXC_ICC_Enable(MXC_ICC0); // Enable cache

    // Switch to 100 MHz clock
    MXC_SYS_Clock_Select(MXC_SYS_CLOCK_IPO);
    SystemCoreClockUpdate();

    setbuf(stdout, NULL); // disable stdout buffering
    printf("MAX78000FTHR Audio Test -- %s %s\n", __DATE__, __TIME__);

    // YELLOW (RED + GREEN) for 1 second, validates LED result is viewable
    LED_On(LED_GREEN);
    LED_On(LED_RED);
    MXC_Delay(1000000);
    LED_Off(LED_GREEN);
    LED_Off(LED_RED);

    audio_setup();

    // Preload output FIFO
    load_fifo();
    load_fifo();

    for (;;) {
        while (!i2s_flag);
        i2s_flag = 0;

        if (i2s_irq_flags & MXC_F_I2S_INTFL_RX_THD_CH0) {
            unload_fifo();
        }

        if (i2s_irq_flags & MXC_F_I2S_INTFL_TX_HE_CH0) {
            load_fifo();
        }

        /*  Allow output signal to stabilize for a time before testing input.
            i2s_flag is set at a constant rate and usable as a periodic reference. */
        if (++ticks == 4500) {
            MXC_I2S_Shutdown();

            /*
                Check left and right tones individually, detects left/right shorts.
            */
            err |= check_for_tone(samples_left, LEFT_BIN);
            err |= check_for_tone(samples_right, RIGHT_BIN);

            for (;;) {
                LED_On(err ? LED_RED : LED_GREEN);
                MXC_Delay(500000);
                LED_Off(err ? LED_RED : LED_GREEN);
                MXC_Delay(500000);
            }
        }
    }
}
