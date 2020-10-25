/*
	i2smaster.h
*/

extern volatile int i2s_flag;
extern volatile uint32_t i2s_irq_flags;

int i2s_init(void);
