#include "myTask02.h"
#include <stdio.h>
#include "usart.h"
#include "../../MDK-ARM/pc.h"





void StartTask02(void *argument)
{
	init_pc_interface(&pcHandle);
	uint8_t num=0;
	for(;;)
	{
		parse_pc(&pcHandle);
		//(just for test)HAL_UART_Transmit(&PCUART,pcHandle.send_buf,1,0xFFFF);
		send_pc(&pcHandle);
		num++;
		if(num==100) 
		{
			if(pcHandle.connect_status==0)
			{
				pc_send_recv_debug(&pcHandle);
			}
			num = 0;
		}
		HAL_Delay(1);
	}
	
		
}



