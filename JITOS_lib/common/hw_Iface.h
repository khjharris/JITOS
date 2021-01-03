#ifndef _HW_IFACE_H
#define _HW_FACE_H

#include "hw_Defines.h"

#define TIM_CR1_EN              0x1UL
#define UPDATE_INTERRUPT_FLAG   0x1

void init_general_timer();
void start_pwm_timer();
void start_general_timer();
void read_general_timer();
void stop_general_timer();

#endif



