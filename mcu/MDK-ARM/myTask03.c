#include "myTask03.h"
#include "cmsis_os.h"
#include "../../MDK-ARM/pwm.h"
#include "../../MDK-ARM/gps.h"
#include "../../MDK-ARM/yis100.h"
#include "control_boat.h"
#include <stdio.h>

void StartTask03(void *argument)
{
	ctrl_init();
	init_capture_pwm();
	init_gps(&gps_handle);
	init_yis(&yis_Handle);
	uint8_t num1=0;
	for(;;)
	{
		get_pwm(&pwm_info);
		contorl_boat(&control_info);
		num1++;
		if(num1==50)
		{
			//print_pwm(&pwm_info);
			//print_boat_info(&control_info);
			//parse_gps(&gps_handle);
			//print_gps(&gps_handle);
			//parse_yis(&yis_Handle);
			//print_yis(&yis_Handle);
			num1=0;
		}
		osDelay(20);
		
	}
	
}