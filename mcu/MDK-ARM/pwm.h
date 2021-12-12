#ifndef __pwm_v0_H
#define __pwm_v0_H
#include "stm32f4xx_hal.h"

#define MAX_CHANNELS  13
#define MAX_CAPTURE_CNT MAX_CHANNELS+2



typedef struct 
{
	uint8_t capture_cnt;
	uint16_t capture_buf[MAX_CAPTURE_CNT];
	uint16_t pwm_rawbuf[MAX_CHANNELS+1];
	
	uint8_t pwm_buf1_ready_flag;
	uint8_t pwm_buf1_new_flag;
	uint16_t pwm_buf1[MAX_CHANNELS+1];
	
	uint8_t pwm_buf2_ready_flag;
	uint8_t pwm_buf2_new_flag;
	uint16_t pwm_buf2[MAX_CHANNELS+1];
	
	uint16_t pwm_buf[MAX_CHANNELS+1];
	
	
}PWM_HANDLE;




extern PWM_HANDLE pwm_info;
void init_capture_pwm();
void capture_pwm_tim_callback(PWM_HANDLE* handle);
void get_pwm(PWM_HANDLE* handle);
void  print_pwm(PWM_HANDLE* handle);
#endif