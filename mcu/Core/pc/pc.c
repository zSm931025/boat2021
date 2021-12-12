#include "pc.h"
#include "stdio.h"
#include <string.h>




uint8_t recvBuf[30];
uint8_t recvBuf1[30];
uint8_t recvBuf2[30];
uint8_t recv_index;
uint8_t recv_new;
uint8_t sendBuf[30];
void pc_task()
{
	recv_index=0;
	recv_new = 0;
	uint8_t res=0;
	HAL_UART_Receive_IT(&PCUART,recvBuf+recv_index,1);
	while(1)
	{
		
		HAL_NVIC_DisableIRQ(PC_IRQn);
		if(recv_new==1)
		{
			res = 1;
			recv_new=0;
			memcpy(recvBuf2,recvBuf1,20);
		}
		HAL_NVIC_EnableIRQ(PC_IRQn);
		
		
		HAL_UART_Transmit(&PCUART,sendBuf,30,0xFFF);
		if(res==1)
		{
			printf("receive new info!\n");
			res = 0;
		}
		else
		{
			printf("no\n");
		}
		
		HAL_Delay(20);
	
	}

}