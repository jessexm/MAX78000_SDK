/*******************************************************************************
* Copyright (C) Maxim Integrated Products, Inc., All Rights Reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
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
*
******************************************************************************/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "camera.h"
#include "sccb.h"
#include "ov7692_regs.h"
#include "mxc_delay.h"
#include "mxc_device.h"

#define cambus_writeb(addr, x)      sccb_write_byt(g_slv_addr, addr, x)
#define cambus_readb(addr, x)       sccb_read_byt(g_slv_addr, addr, x)

static int g_slv_addr;
static pixformat_t g_pixelformat = PIXFORMAT_RGB565;

static const uint8_t default_regs[][2] = {
    {0x12, 0x80}, // System reset
    {0x69, 0x52}, // BLC window selection, BLC enable (default is 0x12)
    {0x1e, 0xb3}, // AddLT1F (default 0xb1)
    {0x48, 0x42},
    {0xff, 0x01}, // Select MIPI register bank
    {0xb5, 0x30},
    {0xff, 0x00}, // Select system control register bank
    {0x16, 0x03}, // Default
    {0x0c, 0xd6},
    {0x82, 0x03},
    {0x11, 0x00},  // Set clock prescaler
    {0x12, 0x06},
    {0x61, 0x00},
    {0x64, 0x21},
    {0xc3, 0x80},
    {0x81, 0x3f},
    {0x16, 0x07},
    {0x31, 0x82},
    {0x37, 0x04},
    {0x3e, 0x20},
    {0x5e, 0x10},
    {0x69, 0x02},
    {0xc4, 0x01},
    {0xc5, 0x80},
    {0xc6, 0x01},
    {0xc7, 0x80},
    {0xc8, 0x02},
    {0xc9, 0x80},
    {0xca, 0x01},
    {0xcb, 0xe0},
    {0xcc, 0x00},
    {0xcd, 0x40}, // Default to 64 line width
    {0xce, 0x00},
    {0xcf, 0x40}, // Default to 64 lines high
    {0x1c, 0x7f},
    {0x1d, 0xa2},
    {0x0e, 0x00},
    {0xee, 0xee}  // End of register list marker 0xee
};

/******************************** Static Functions ***************************/
static int init(void)
{
    int ret = 0;
    
    g_slv_addr = sccb_scan();
    if (g_slv_addr == -1) {
        return -1;
    }
    
    return ret;
}

static int get_slave_address(void)
{
    return g_slv_addr;
}

static int get_product_id(int *id)
{
    int ret = 0;
    uint8_t id_high;
    uint8_t id_low;
    
    ret |= cambus_readb(PIDH, &id_high);
    ret |= cambus_readb(PIDL, &id_low);
    *id = (int)(id_high << 8) + id_low;
    return ret;
}

static int get_manufacture_id(int *id)
{
    int ret = 0;
    uint8_t id_high;
    uint8_t id_low;
    
    ret |= cambus_readb(MIDH, &id_high);
    ret |= cambus_readb(MIDL, &id_low);
    *id = (int)(id_high << 8) + id_low;
    return ret;
}

static int dump_registers(void)
{
    int ret = 0;
    uint8_t byt = 0;
    uint32_t i;
    uint8_t buf[64] = {0};
    uint8_t *ptr = buf;
    
    for (i = 0; ; i++) {
    
        if ((i != 0) && !(i % 16)) {
            *ptr = '\0';
            printf("%04X:%s\n", i - 16, buf);
            ptr = buf;
        }
        if (i == 256) {
            break;
        }
        ret = cambus_readb(i, &byt);
        if (ret == 0) {
            ret = sprintf((char*)ptr, " %02X", byt);
            if (ret < 0) {
                return ret;
            }
            ptr += 3;// XX + space
        }
        else {
            *ptr++ = '!';
            *ptr++ = '!';
            *ptr++ = ' ';
        }
    }
    return ret;
}

static int reset(void)
{
    int ret = 0;
    uint8_t value;
    ret |= cambus_writeb(REG12, REG12_RESET);
	// Read from the register, when the reset bit is cleared then reset is done.
	for (value = 0xff; value != 0;) {
		ret |= cambus_readb(REG12, &value);
		MXC_Delay(MSEC(10));
	}
    // Write default registers
    for (int i = 0; (default_regs[i][0] != 0xee); i++) {
        ret |= cambus_writeb(default_regs[i][0], default_regs[i][1]);
    }
    return ret;
}

static int sleep(int enable)
{
    int ret = 0;
#if 0
    ret = cambus_readb(REG_COM2, &reg);
    
    if (ret == 0) {
        if (enable) {
            reg |= COM2_SOFT_SLEEP_MODE;
        }
        else {
            reg &= ~COM2_SOFT_SLEEP_MODE;
        }
        
        // Write back register
        ret |= cambus_writeb(REG_COM2, reg);
    }
    
#endif
    return ret;
}

static int read_reg(uint8_t reg_addr, uint8_t* reg_data)
{
    *reg_data = 0xff;
    if (cambus_readb(reg_addr, reg_data) != 0) {
        return -1;
    }
    return 0;
}

static int write_reg(uint8_t reg_addr, uint8_t reg_data)
{
    return cambus_writeb(reg_addr, reg_data);
}

static int set_pixformat(pixformat_t pixformat)
{
    int ret = 0;
    
    g_pixelformat = pixformat;
    switch (pixformat) {
    case PIXFORMAT_YUV422:
    case PIXFORMAT_GRAYSCALE:
        ret |= cambus_writeb(REG12, COLOR_YUV422);
        break;
    case PIXFORMAT_RGB444:
        ret |= cambus_writeb(REG12, COLOR_RGB444);
        break;
    case PIXFORMAT_RGB565:
    case PIXFORMAT_RGB888:
        ret |= cambus_writeb(REG12, COLOR_RGB565);
        break;
    case PIXFORMAT_BAYER:
        ret |= cambus_writeb(REG12, COLOR_BAYER);
        break;
    default:
        ret = -1;
        break;
    }
    
    return ret;
}

static int get_pixformat(pixformat_t *pixformat)
{
    int ret = 0;
    *pixformat = g_pixelformat;
    return ret;
}

static int set_framesize(int width, int height)
{
    int ret = 0;
    uint8_t input_factor_large[4] = { 0 };
    // Default input factor for large images
    if ((width < 320) || (height < 240)) { // check if size is QVGA or less
        input_factor_large[0] = 0x02;
        input_factor_large[1] = 0x80;
        input_factor_large[2] = 0x00;
        input_factor_large[3] = 0xf0;
        uint8_t value;
        ret |= cambus_readb(REG12, &value);
        ret |= cambus_writeb(REG12, (value | 0x40)); // enable skip mode
        ret |= cambus_writeb(0x17, 0x65); // Horizontal Window Start Point Control (LSBs), default is 0x69
        ret |= cambus_writeb(0x18, 0xa4); // Horizontal sensor size (default)
        ret |= cambus_writeb(0x19, 0x06); // Vertical Window Start Line Control
        ret |= cambus_writeb(0x1a, 0x7b); // Vertical sensor size
    } else {
        input_factor_large[0] = 0x02;
        input_factor_large[1] = 0x80;
        input_factor_large[2] = 0x01;
        input_factor_large[3] = 0xe0;
    }

    uint8_t input_factor_small[] = { 0x01, 0xbf, 0x00, 0xf0 };
    uint8_t *input_factor_ptr = input_factor_large;
    
    // Image typically outputs one line short, add a line to account.
    height = height + 1;
    // Apply passed in resolution as output resolution.
    ret |= cambus_writeb(OH_HIGH, (width >> 8) & 0xff);
    ret |= cambus_writeb(OH_LOW, (width >> 0) & 0xff);
    ret |= cambus_writeb(OV_HIGH, (height >> 8) & 0xff);
    ret |= cambus_writeb(OV_LOW, (height >> 0) & 0xff);
    // Check and see if the target resolution is very small
    // that is less than 42 x 42, if so then apply and
    // x and y scaling factor.
    if ((width < 42) || (height < 42)) {
        input_factor_ptr = input_factor_small;
    }
    // Apply the appropriate input image factor.
    ret |= cambus_writeb(0xc8, input_factor_ptr[0]);
    ret |= cambus_writeb(0xc9, input_factor_ptr[1]);
    ret |= cambus_writeb(0xca, input_factor_ptr[2]);
    ret |= cambus_writeb(0xcb, input_factor_ptr[3]);
    return ret;
}

static int set_windowing(int width, int height, int hsize, int vsize)
{
    /* Note: width and height is used to control scaling size of the image
       width: horizontal input size
       height: vertical input size 
       hsize: horizontal size of cropped image
       vsize: vertical size of cropped image
    */
    int ret = 0;
    if (width<hsize || height<vsize) {
        ret = -1;
    }
    ret |= cambus_writeb(0x11, 0x0);
    ret |= cambus_writeb(0x51, 0x7f);
    ret |= cambus_writeb(0x50, 0x99);
    ret |= cambus_writeb(0x21, 0x23);
    ret |= cambus_writeb(0x20, 0x00);
    // Apply passed in resolution as input resolution.
    ret |= cambus_writeb(0xc8, (width >> 8) & 0xff);
    ret |= cambus_writeb(0xc9, (width >> 0) & 0xff);
    ret |= cambus_writeb(0xca, (height >> 8) & 0xff);
    ret |= cambus_writeb(0xcb, (height >> 0) & 0xff);

    // Apply passed in hsize & vsize as output resolution.
    ret |= cambus_writeb(OH_HIGH, (hsize >> 8) & 0xff);
    ret |= cambus_writeb(OH_LOW, (hsize >> 0) & 0xff);
    ret |= cambus_writeb(OV_HIGH, (vsize >> 8) & 0xff);
    ret |= cambus_writeb(OV_LOW, (vsize >> 0) & 0xff);

    // adjust center position
    ret |= cambus_writeb(0x9, 0x10);
    ret |= cambus_writeb(HSTART, 0x55);
    ret |= cambus_writeb(VSTART, 0xc2);

    return ret;
}

static int set_contrast(int level)
{
    int ret = 0;
#if 0
    
    switch (level) {
    case -4:
        level =   0;
        break;
        
    case -3:
        level =  30;
        break;
        
    case -2:
        level =  60;
        break;
        
    case -1:
        level =  90;
        break;
        
    case  0:
        level = 120;
        break;
        
    case  1:
        level = 150;
        break;
        
    case  2:
        level = 180;
        break;
        
    case  3:
        level = 210;
        break;
        
    case  4:
        level = 255;
        break;
        
    default:
        return -1;
    }
    
    ret = cambus_writeb(REG_CONTRAS, level);
#endif
    return ret;
}

static int set_brightness(int level)
{
    int ret = 0;
#if 0
    switch (level) {
    case -4:
        level =   0;
        break;
        
    case -3:
        level =  30;
        break;
        
    case -2:
        level =  60;
        break;
        
    case -1:
        level =  90;
        break;
        
    case  0:
        level = 120;
        break;
        
    case  1:
        level = 150;
        break;
        
    case  2:
        level = 180;
        break;
        
    case  3:
        level = 210;
        break;
        
    case  4:
        level = 255;
        break;
        
    default:
        return -1;
    }
    
    // update REG_COM8
    ret = cambus_readb(REG_COM8, &reg);
    reg &= ~COM8_AEC;
    ret = cambus_writeb(REG_COM8, reg);
    
    // update REG_BRIGHT
    level &= 0xFF;
    reg = (level > 127) ? (level & 0x7f) : ((128 - level) | 0x80);
    ret = cambus_writeb(REG_BRIGHT, reg);
#endif
    return ret;
}

static int set_saturation(int level)
{
    int ret = 0;
    
    return ret;
}

static int set_gainceiling(gainceiling_t gainceiling)
{
    int ret = 0;
#if 0
    ret = cambus_readb(REG_COM9, &reg);
    
    // Set gain ceiling
    reg = COM9_SET_AGC(reg, gainceiling);
    ret |= cambus_writeb(REG_COM9, reg);
#endif
    return ret;
}

static int set_colorbar(int enable)
{
    int ret = 0;
    
    return ret;
}

static int set_hmirror(int enable)
{
    int ret = 0;
    uint8_t reg;
    
    ret = cambus_readb(REG0C, &reg);
    if (enable) {
        reg |= HORIZONTAL_FLIP;
    }
    else {
        reg &= ~HORIZONTAL_FLIP;
    }
    ret |= cambus_writeb(REG0C, reg);
    return ret;
}

static int set_negateimage(int enable)
{
    int ret = 0;
    
    if (enable) {
        ret |= cambus_writeb(REG81, 0x3f);
        ret |= cambus_writeb(REG28, 0x82);
        ret |= cambus_writeb(REGD2, 0x00);
        ret |= cambus_writeb(REGDA, 0x80);
        ret |= cambus_writeb(REGDB, 0x80);
    } else {
        ret |= cambus_writeb(REG81, 0x3f);
        ret |= cambus_writeb(REG28, 0x02);
        ret |= cambus_writeb(REGD2, 0x00);
        ret |= cambus_writeb(REGDA, 0x80);
        ret |= cambus_writeb(REGDB, 0x80);
    }
    return ret;
}

static int set_vflip(int enable)
{
    int ret = 0;
    uint8_t reg;
    
    ret = cambus_readb(REG0C, &reg);
    if (enable) {
        reg |= VERTICAL_FLIP;
    }
    else {
        reg &= ~VERTICAL_FLIP;
    }
    ret |= cambus_writeb(REG0C, reg);
    return ret;
}

/******************************** Public Functions ***************************/
int sensor_register(camera_t* camera)
{
    // Initialize sensor structure.
    camera->init                = init;
    camera->get_slave_address   = get_slave_address;
    camera->get_product_id      = get_product_id;
    camera->get_manufacture_id  = get_manufacture_id;
    camera->dump_registers      = dump_registers;
    camera->reset               = reset;
    camera->sleep               = sleep;
    camera->read_reg            = read_reg;
    camera->write_reg           = write_reg;
    camera->set_pixformat       = set_pixformat;
    camera->get_pixformat       = get_pixformat;
    camera->set_framesize       = set_framesize;
    camera->set_windowing       = set_windowing;
    camera->set_contrast        = set_contrast;
    camera->set_brightness      = set_brightness;
    camera->set_saturation      = set_saturation;
    camera->set_gainceiling     = set_gainceiling;
    camera->set_colorbar        = set_colorbar;
    camera->set_hmirror         = set_hmirror;
    camera->set_vflip           = set_vflip;
    camera->set_negateimage     = set_negateimage;
    return 0;
}

