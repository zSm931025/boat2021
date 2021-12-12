#ifndef __pid_H
#define __pid_H




typedef struct
{
	volatile float p_p;
	volatile float p_i;
	volatile float p_d;
	volatile float p_error;
	volatile float p_last_error;
	volatile float p_integral;
	volatile float p_integral_threshold;
	volatile float p_integral_max;
	volatile float p_max_output;
	
	volatile float v_p;
	volatile float v_i;
	volatile float v_d;
	volatile float v_error;
	volatile float v_last_error;
	volatile float v_integral;
	volatile float v_integral_threshold;
	volatile float v_integral_max;
	volatile float v_max_output;
}PID;



void init_pid();
float process_pid(float aim_angle, float now_angle, float now_angleSpeed);
extern PID * p_pid;
extern volatile float aim_angle;








#endif