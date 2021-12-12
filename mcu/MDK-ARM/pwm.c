#include "pwm.h"
#include <tim.h>
#include <string.h>
#include <stdio.h>

#define PWM_TIMER_CHANNEL      TIM_CHANNEL_1
#define PWM_TIMER  htim5


PWM_HANDLE pwm_info;


void init_capture_pwm()
{
	HAL_TIM_IC_Start_IT(&PWM_TIMER,PWM_TIMER_CHANNEL);
	__HAL_TIM_SET_CAPTUREPOLARITY(&PWM_TIMER,PWM_TIMER_CHANNEL,TIM_INPUTCHANNELPOLARITY_RISING);
}
	



void capture_pwm_tim_callback(PWM_HANDLE* handle)
{
	if(handle->capture_cnt>=MAX_CAPTURE_CNT)
	{
		handle->capture_cnt = 0;
		
	}
	handle->capture_buf[handle->capture_cnt] =__HAL_TIM_GET_COMPARE(&PWM_TIMER,PWM_TIMER_CHANNEL);
	if(handle->capture_cnt==0)
	{
		handle->capture_cnt = 1;
	}
	else
	{
		handle->pwm_rawbuf[handle->capture_cnt-1]=handle->capture_buf[handle->capture_cnt]-handle->capture_buf[handle->capture_cnt-1];
		if(handle->pwm_rawbuf[handle->capture_cnt-1]>4000)
		{
			
			if(handle->pwm_buf1_ready_flag==1)
			{
				memcpy(handle->pwm_buf1,handle->pwm_rawbuf,handle->capture_cnt*sizeof(uint16_t));
				handle->pwm_buf1_new_flag = 1;
			}
			else
			{
				memcpy(handle->pwm_buf2,handle->pwm_rawbuf,handle->capture_cnt*sizeof(uint16_t));
				handle->pwm_buf2_new_flag = 1;
			}
			handle->capture_buf[0]=handle->capture_buf[handle->capture_cnt];
			handle->capture_cnt =0;
		}
		handle->capture_cnt+=1;
	}
	
}


void get_pwm(PWM_HANDLE*handle)
{
	if(handle->pwm_buf1_new_flag==1)
	{
		handle->pwm_buf1_ready_flag = 0;
		memcpy(handle->pwm_buf,handle->pwm_buf1,MAX_CHANNELS*2);
		handle->pwm_buf1_new_flag=0;
		handle->pwm_buf1_ready_flag = 1;
	}else if(handle->pwm_buf2_new_flag==1)
	{
		handle->pwm_buf2_ready_flag = 0;
		memcpy(handle->pwm_buf,handle->pwm_buf2,MAX_CHANNELS*2);
		handle->pwm_buf2_new_flag=0;
		handle->pwm_buf2_ready_flag = 1;
	}else
	{
		handle->pwm_buf1_ready_flag = 0;
		memcpy(handle->pwm_buf,handle->pwm_buf1,MAX_CHANNELS*2);
		handle->pwm_buf1_new_flag=0;
		handle->pwm_buf1_ready_flag = 1;
	}

}
void  print_pwm(PWM_HANDLE* handle)
{
		printf("%8d%8d%8d%8d%8d%8d%8d%8d%8d%8d%8d%8d%8d%8d\r\n",pwm_info.pwm_buf[0],pwm_info.pwm_buf[1],pwm_info.pwm_buf[2],
		pwm_info.pwm_buf[3],pwm_info.pwm_buf[4],pwm_info.pwm_buf[5],pwm_info.pwm_buf[6],
		pwm_info.pwm_buf[7],pwm_info.pwm_buf[8],pwm_info.pwm_buf[9],pwm_info.pwm_buf[10],
		pwm_info.pwm_buf[11],pwm_info.pwm_buf[12],pwm_info.pwm_buf[13]);
}
	
	