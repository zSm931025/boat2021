#include "control_boat.h"
#include "pwm.h"
#include "stdio.h"
#include "pc.h"
#include "../../MDK-ARM/pid.h"


CONTROL_INFO control_info;

void get_control_info(CONTROL_INFO* comd);
void send_control_info();


void ctrl_init()
{
	HAL_TIM_PWM_Start(&LEFT_MOTOR_TIMER,LEFT_MOTOR_TIMER_CHANNEL);//the start of pwm_generation
	HAL_TIM_PWM_Start(&RIGHT_MOTOR_TIMER,RIGHT_MOTOR_TIMER_CHANNEL);
	__HAL_TIM_SET_COMPARE(&LEFT_MOTOR_TIMER,LEFT_MOTOR_TIMER_CHANNEL,MID_SING);
	__HAL_TIM_SET_COMPARE(&RIGHT_MOTOR_TIMER,RIGHT_MOTOR_TIMER_CHANNEL,MID_SING);
	
}

void contorl_boat(CONTROL_INFO* comd)
{
	get_control_info(comd);
	//
	//
	//�˴��ɼ�pid,��ң�غ�
	//process_pid();
	//
	//
	//
	send_control_info(comd);
}

	

void get_control_info(CONTROL_INFO* comd)
{
	get_pwm(&pwm_info);
	//7ͨ��Ϊģʽѡ��ͨ����ʧ�ر���ͨ����С��1300Ϊ����������1700 Ϊң��
	if(pwm_info.pwm_buf[6]<1300 && pcHandle.connect_status==1)  //С��1300����������ô��Ϊ ����ģʽ1
	{
		comd->mode = 1;
	}
	else if (pwm_info.pwm_buf[6]>1700) //����1700����ô��Ϊң��ģʽ
	{
		comd->mode = 2;
	}
	else                       //ʧ�ػ��߾�ĬΪ0
	{
		comd->mode = 0;
	}
	
	
	
	if(comd->mode==2)
	{
		//����������
		
		comd->throttle = (MID_SING-pwm_info.pwm_buf[2])/SING_LEN;
		comd->rudder = (pwm_info.pwm_buf[0]-MID_SING)/SING_LEN;
		comd->acc_speed = (pwm_info.pwm_buf[5]-MIN_SING)/30;
		
		if(pwm_info.pwm_buf[8]<1300)
		{
			comd->gear = 1;
		}
		else if (pwm_info.pwm_buf[8]>1800)
		{
			comd->gear =3;
		}
		else
		{
			comd->gear = 2;
		}
		//ģʽ ch8
		//printf("throttle:%f    rudder:%f     accelerate:%d\r\n", comd->throttle,comd->rudder,comd->acc_speed);
		
	}
	else if(comd->mode == 1)
	{
		//������������
		comd->throttle = pcHandle.ctlCmd.velocity/100.0;
		comd->rudder  = pcHandle.ctlCmd.rotation/100.0;
		comd->acc_speed = pcHandle.ctlCmd.accelerate;
		comd->gear = pcHandle.ctlCmd.gear;
	}
	else
	{
		comd->throttle = 0;
		comd->rudder  = 0;
		comd->acc_speed = 10;
		comd->gear = 1;
	}
	//printf("pc connect status:%d\n",pcHandle.connect_status);
	
	//�Ϸ��Լ��
	if(comd->throttle<-1 || comd->throttle>1)
	{
		comd->throttle=0;
	}
	if(comd->rudder<-1 || comd->rudder>1)
	{
		comd->rudder=0;
	}
	if(comd->acc_speed<0 || comd->acc_speed>100)
	{
		comd->acc_speed=0;
	}
	if(comd->gear!=1 && comd->gear!=2 && comd->gear!=3)
	{
		comd->gear=1;
	}
		
}







void send_control_info(CONTROL_INFO* comd)
{
	
	
	//���
	//printf("%f  %f\r\n",control_info.throttle,control_info.rudder);
	
	comd->aim_value_left  = 1500-(comd->throttle+comd->rudder)*SING_LEN*comd->gear/3.0;
	comd->aim_value_right = 1500-(comd->throttle-comd->rudder)*SING_LEN*comd->gear/3.0;
	
	//printf("left:%d    right:%d \r\n", comd.aim_value_left,comd.aim_value_right);
	
	//����Լ��
	if(comd->aim_value_left<MIN_SING)
	{
		comd->aim_value_left  =MIN_SING;
	}
	if(comd->aim_value_left>MAX_SING)
	{
		comd->aim_value_left = MAX_SING;
	}
	if(comd->aim_value_right<MIN_SING)
	{
		comd->aim_value_right = MIN_SING;
	}
	if(comd->aim_value_right>MAX_SING)
	{
		comd->aim_value_right  =MAX_SING;
	}
	
	
	//������
	comd->true_value_left=__HAL_TIM_GET_COMPARE(&LEFT_MOTOR_TIMER,LEFT_MOTOR_TIMER_CHANNEL);//left motor
	comd->true_value_right=__HAL_TIM_GET_COMPARE(&RIGHT_MOTOR_TIMER,RIGHT_MOTOR_TIMER_CHANNEL);//right motor

	//printf("true:      left:%d    right:%d    \r\n", comd->true_value_left,comd->true_value_right);
	//printf("aim:       left:%d    right:%d     accelerate:%d \r\n", comd->aim_value_left,comd->aim_value_right,comd->acc_speed);
	
	if(comd->aim_value_left-comd->true_value_left>comd->acc_speed)
	{
		__HAL_TIM_SET_COMPARE(&LEFT_MOTOR_TIMER,LEFT_MOTOR_TIMER_CHANNEL,comd->true_value_left+comd->acc_speed);
	}
	else if(comd->true_value_left-comd->aim_value_left>comd->acc_speed)
	{
		__HAL_TIM_SET_COMPARE(&LEFT_MOTOR_TIMER,LEFT_MOTOR_TIMER_CHANNEL,comd->true_value_left-comd->acc_speed);
				
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&LEFT_MOTOR_TIMER,LEFT_MOTOR_TIMER_CHANNEL,comd->aim_value_left);
	}
	
	
	if(comd->aim_value_right-comd->true_value_right>comd->acc_speed)
	{
		__HAL_TIM_SET_COMPARE(&RIGHT_MOTOR_TIMER,RIGHT_MOTOR_TIMER_CHANNEL,comd->true_value_right+comd->acc_speed);
	}
	else if(comd->true_value_right-comd->aim_value_right>comd->acc_speed)
	{
		__HAL_TIM_SET_COMPARE(&RIGHT_MOTOR_TIMER,RIGHT_MOTOR_TIMER_CHANNEL,comd->true_value_right-comd->acc_speed);
	}
	else
	{
		__HAL_TIM_SET_COMPARE(&RIGHT_MOTOR_TIMER,RIGHT_MOTOR_TIMER_CHANNEL,comd->aim_value_right);
	}
	
	
}


void print_boat_info(CONTROL_INFO* comd)
{
	printf("����:%f ת��:%f ����:%d ��λ:%d ģʽ:%d ״̬:%d Ŀ����:%d ʵ����:%d Ŀ����:%d ʵ����:%d\r\n\r\n",
	comd->throttle,comd->rudder,comd->acc_speed,comd->gear,comd->mode,comd->status,comd->aim_value_left,comd->true_value_left,
	comd->aim_value_right,comd->true_value_right);
		
}


