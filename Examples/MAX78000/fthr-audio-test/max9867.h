/*

*/
#pragma once

#include "i2c_regs.h"

int max9867_init(mxc_i2c_regs_t *i2c_inst, int mclk, int lrclk);
int max9867_status(void);
