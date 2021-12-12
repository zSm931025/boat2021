#ifndef __control_boat_H
#define __control_boat_H

//#include "stm32f4xx_hal.h"
#include <tim.h>

#define SING_LEN  500.0
#define MAX_SING  2000.0
#define MIN_SING  1000.0
#define MID_SING  1500.0


#define LEFT_MOTOR_TIMER  htim3
#define LEFT_MOTOR_TIMER_CHANNEL    TIM_CHANNEL_1

#define RIGHT_MOTOR_TIMER htim3
#define RIGHT_MOTOR_TIMER_CHANNEL   TIM_CHANNEL_2





typedef struct
{
	float throttle;     //ͨ��3��-1��1����ǰΪ��
	float rudder;		    //ͨ��1��-1��1����ʱ��Ϊ��
	uint16_t acc_speed;    //ͨ��6��0��1000
	uint8_t gear;		    //ͨ��7��1��30%����2��60%����3��100%��
	uint8_t mode;			  //ͨ��8��0:��Ĭ��1:������2:ң����
	uint8_t status;      //����
	uint16_t aim_value_left;    //left motor
	uint16_t aim_value_right;   
	uint16_t true_value_left;    //left motor
	uint16_t true_value_right;   

}CONTROL_INFO;

extern CONTROL_INFO control_info;
void ctrl_init();
void contorl_boat(CONTROL_INFO* comd);
void print_boat_info(CONTROL_INFO* comd);
#endif
