#include "../MDK-ARM/pid.h"
#include <math.h>

PID pid;
PID * p_pid = &pid;
volatile float aim_angle =0;

void init_pid()
{
	p_pid->p_p = 1;
	p_pid->p_i = 1;
	p_pid->p_d = 1;
	p_pid->p_integral = 0;
	p_pid->p_integral_max = 10;
	p_pid->p_integral_threshold = 5;
	p_pid->p_max_output = 10;
	
	p_pid->v_p = 1;
	p_pid->v_i = 1;
	p_pid->v_d = 1;
	p_pid->v_integral = 0;
	p_pid->v_integral_max = 10;
	p_pid->v_integral_threshold = 5;
	p_pid->v_max_output = 10;
}

float process_pid(float aim_angle, float now_angle, float now_angleSpeed)
{
	float error_temp,output1,output2;
	error_temp = aim_angle - now_angle;
	if (error_temp>0 && error_temp<=180)
			p_pid->p_error = error_temp;
	else if (error_temp>0 && error_temp>180)
			p_pid->p_error = error_temp-360;
	else if (error_temp<=0 && error_temp>-180)
			p_pid->p_error = error_temp;
	else
			p_pid->p_error = 360+error_temp;
	
	if(fabs(p_pid->p_error) < p_pid->p_integral_threshold)
			p_pid->p_integral += p_pid->p_error;
			if(p_pid->p_integral > p_pid->p_integral_max)
				p_pid->p_integral = p_pid->p_integral_max;
	else
			p_pid->p_integral = 0;
	output1 = p_pid->p_p * p_pid->p_error + p_pid->p_i * p_pid->p_integral + p_pid->p_d * (p_pid->p_error-p_pid->p_last_error);
	p_pid->p_last_error = p_pid->p_error;
	if (output1 > p_pid->p_max_output)
			output1 = p_pid->p_max_output;
	if (output1<-p_pid->p_max_output)
			output1 = -p_pid->p_max_output;

	p_pid->v_error  = output1-now_angleSpeed;
	if(fabs(p_pid->v_error) < p_pid->v_integral_threshold)
			p_pid->v_integral += p_pid->v_error;
			if(p_pid->v_integral > p_pid->v_integral_max)
				p_pid->v_integral = 0;
	output2 = p_pid->v_p * p_pid->v_error + p_pid->v_i * p_pid->v_integral + p_pid->v_d * (p_pid->v_error - p_pid->v_last_error);
	p_pid->v_last_error = p_pid->v_error;
	if (output2 > p_pid->v_max_output)
		output2 = p_pid->v_max_output;
	if (output2<-p_pid->v_max_output)
		output2 = -p_pid->v_max_output;
	return output2;
}