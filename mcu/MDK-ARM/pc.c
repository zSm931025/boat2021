#include "pc.h"
#include <string.h>
#include <stdio.h>
#include "../MDK-ARM/gps.h"
#include "../MDK-ARM/yis100.h"
#include "../MDK-ARM/control_boat.h"



uint8_t prepare_gps(PCHANDLE*handle);
uint8_t prepare_acc(PCHANDLE*handle);
uint8_t prepare_eul(PCHANDLE*handle);
uint8_t prepare_gro(PCHANDLE*handle);
uint8_t prepare_qua(PCHANDLE*handle);
uint8_t prepare_tim(PCHANDLE*handle);
uint8_t prepare_sys(PCHANDLE*handle);
uint16_t checkSumPcmsg(uint8_t* data,uint8_t len);

PCHANDLE pcHandle;


void init_pc_interface(PCHANDLE* handle)
{
	handle->connect_status = 0;
	handle->lose_connect_times = 0;
	handle->time_since_last_recv = 0;
	handle->lose_parse_times = 0;
	handle->wrong_frame_times = 0;
	handle->wrong_count_times = 0;
	handle->recv_head = -1;
	handle->recv_tail = -1;
	handle->recv_index = 0;
	handle->recv_payload_len = 0;
	handle->parse_done = 1;
	handle->send_head = -1;
	handle->send_tail = -1;
	handle->send_done = 1;
	handle->send_count = 0;
	handle->ctlCmd.mode = 1;
	handle->ctlCmd.velocity = 0;
	handle->ctlCmd.rotation = 0;
	handle->ctlCmd.accelerate = 1;
	handle->ctlCmd.gear = 1;
	handle->send_msg_enable[MSGGPS] = 1; 
	handle->send_msg_enable[MSGACC] = 1; 
	handle->send_msg_enable[MSGEUL] = 1; 
	handle->send_msg_enable[MSGGRO] = 1; 
	handle->send_msg_enable[MSGQUA] = 1; 
	handle->send_msg_enable[MSGTIM] = 1; 
	handle->send_msg_enable[MSGSYS] = 1; 
	
	handle->send_msg_mode[MSGGPS] = 0; 
	handle->send_msg_mode[MSGACC] = 0; 
	handle->send_msg_mode[MSGEUL] = 0; 
	handle->send_msg_mode[MSGGRO] = 0; 
	handle->send_msg_mode[MSGQUA] = 0; 
	handle->send_msg_mode[MSGTIM] = 0; 
	handle->send_msg_mode[MSGSYS] = 0; 
	
	handle->send_msg_flag[MSGGPS] = 0; 
	handle->send_msg_flag[MSGACC] = 0; 
	handle->send_msg_flag[MSGEUL] = 0; 
	handle->send_msg_flag[MSGGRO] = 0; 
	handle->send_msg_flag[MSGQUA] = 0; 
	handle->send_msg_flag[MSGTIM] = 0; 
	handle->send_msg_flag[MSGSYS] = 0; 
	
	handle->send_msg_delay[MSGGPS] = MSGGPS_DELAY; 
	handle->send_msg_delay[MSGACC] = MSGACC_DELAY; 
	handle->send_msg_delay[MSGEUL] = MSGEUL_DELAY; 
	handle->send_msg_delay[MSGGRO] = MSGGRO_DELAY; 
	handle->send_msg_delay[MSGQUA] = MSGQUA_DELAY; 
	handle->send_msg_delay[MSGTIM] = MSGTIM_DELAY; 
	handle->send_msg_delay[MSGSYS] = MSGSYS_DELAY; 
	
	handle->send_msg_ticker[MSGGPS] = 0; 
	handle->send_msg_ticker[MSGACC] = 0; 
	handle->send_msg_ticker[MSGEUL] = 0; 
	handle->send_msg_ticker[MSGGRO] = 0; 
	handle->send_msg_ticker[MSGQUA] = 0; 
	handle->send_msg_ticker[MSGTIM] = 0; 
	handle->send_msg_ticker[MSGSYS] = 0; 
	HAL_UART_Receive_IT(&PCUART,handle->recv_buf+handle->recv_index,1);
}

void parse_pc(PCHANDLE*handle)
{
	static uint8_t count=0;
	HAL_NVIC_DisableIRQ(PC_IRQn);
	if(handle->recv_head!=-1)
	{
		handle->parse_done=0;
		memcpy(handle->parse_buf,handle->recv_raw_data[handle->recv_head],SIZE_PC_RECV_BUF);
		handle->recv_head = (handle->recv_head+1)%SIZE_PC_RECV_QUEUE;
		if(handle->recv_tail==handle->recv_head) handle->recv_head=handle->recv_tail = -1;
		handle->lose_parse_times--;
	}
	HAL_NVIC_EnableIRQ(PC_IRQn);
	if(handle->parse_done==0)
	{
		handle->parse_done=1;
		if(handle->connect_status==0)
		{
			handle->connect_status=1;
			handle->lose_connect_times++;
		}
		handle->time_since_last_recv=0;
		//printf("1:%d  2:%d\n",count,handle->parse_buf[1]);
		if(count!=handle->parse_buf[1])
		{
			count=handle->parse_buf[1];
			handle->wrong_count_times++;
		}
		count=(count+1)%256;
		uint16_t check_sum = checkSumPcmsg(handle->parse_buf,handle->parse_buf[0]+1-2);
		uint16_t check_sum1 = (*(handle->parse_buf+handle->parse_buf[0]-1)<<8)+*(handle->parse_buf+handle->parse_buf[0]);
		if(check_sum==check_sum1)
		{
			uint8_t last_index = handle->parse_buf[0];
			uint8_t index = 2;
			while(index<last_index-1)
			{
				uint8_t id = handle->parse_buf[index]>>4;
				uint8_t len = handle->parse_buf[index]&0x0F;
				if(len==0)
				{
					len =16;
				}
				switch(id)
				{
					case CMDCTL:
					{
						handle->ctlCmd.mode = handle->parse_buf[index+1];
						handle->ctlCmd.velocity = handle->parse_buf[index+2];
						handle->ctlCmd.rotation = handle->parse_buf[index+3];
						handle->ctlCmd.accelerate = handle->parse_buf[index+4];
						handle->ctlCmd.gear = handle->parse_buf[index+5];
						//printf("speed: %d     rotation: %d\r\n",handle->ctlCmd.velocity,handle->ctlCmd.rotation);
					}
					break;
					case CMDSET:
					{
						handle->send_msg_mode[1]= (handle->parse_buf[index+1]&0x02)>>1;
						handle->send_msg_mode[2]= (handle->parse_buf[index+1]&0x04)>>2;
						handle->send_msg_mode[3]= (handle->parse_buf[index+1]&0x08)>>3;
						handle->send_msg_mode[4]= (handle->parse_buf[index+1]&0x10)>>4;
						handle->send_msg_mode[5]= (handle->parse_buf[index+1]&0x20)>>5;
						handle->send_msg_mode[6]= (handle->parse_buf[index+1]&0x40)>>6;
						handle->send_msg_mode[7]= (handle->parse_buf[index+1]&0x80)>>7;
					}
					break;
					default:
					break;
				}
				index+=len+1;
			}
		}
		else
		{
			handle->wrong_frame_times++;
		}
	}		
}

void send_pc(PCHANDLE* handle)
{
	uint8_t send_len = 0;
	for(int i=SENDMINID;i<=SENDMAXID;i++)
	{
		send_len = 0;
		if(handle->send_msg_enable[i]==1 && handle->send_msg_flag[i])
		{
			switch(i)
			{
				case MSGGPS:
					parse_gps(&gps_handle);
					send_len = prepare_gps(handle);
					break;
				
				case MSGACC:
					parse_yis(&yis_Handle);
					send_len = prepare_acc(handle);
					break;
				
				case MSGEUL:
					parse_yis(&yis_Handle);
					send_len = prepare_eul(handle);
					break;
				
				case MSGGRO:
					parse_yis(&yis_Handle);
					send_len = prepare_gro(handle);
					break;
					
				case MSGQUA:
					parse_yis(&yis_Handle);
					send_len = prepare_qua(handle);
					break;
					
				case MSGTIM:
					parse_gps(&gps_handle);
					send_len = prepare_tim(handle);
					break;
					
				case MSGSYS:
					send_len = prepare_sys(handle);
					break;
				
				default:
					send_len = 0;
					break;
			}
			handle->send_msg_flag[i]=0;
			
		}
		if(send_len!=0)
		{
			HAL_NVIC_DisableIRQ(PC_IRQn);
			if(handle->send_done==1)
			{
				handle->send_done = 0;
				memcpy(handle->send_buf,handle->prepare_send_buf,send_len);
				handle->send_buf[3]= handle->send_count;
				handle->send_count++;
				uint16_t temp3;
				uint8_t len = handle->send_buf[2];
				temp3 = checkSumPcmsg(handle->send_buf+2,len-2+1);
				handle->send_buf[len+3-2] = temp3>>8;
				handle->send_buf[len+3-1] = temp3&(0x00FF);
				HAL_UART_Transmit_IT(&PCUART,handle->send_buf,send_len);
			
			}
			else
			{
				if(handle->send_tail==-1)
				{
					memcpy(handle->send_raw_data[0],handle->prepare_send_buf,send_len);
					handle->send_raw_data_len[0] = send_len;
					handle->send_head = 0;
					handle->send_tail = (handle->send_head+1)%SIZE_PC_SEND_QUEUE;
					
				}
				else
				{
					memcpy(handle->send_raw_data[handle->send_tail],handle->prepare_send_buf,send_len);
					handle->send_raw_data_len[handle->send_tail] = send_len;
					if(handle->send_head==handle->send_tail) handle->send_head = (handle->send_head+1)%SIZE_PC_SEND_QUEUE;
					handle->send_tail=(handle->send_tail+1)%SIZE_PC_SEND_QUEUE;
					
				}
			}
			
			HAL_NVIC_EnableIRQ(PC_IRQn);
		}
	}
}

uint8_t prepare_gps(PCHANDLE*handle)
{
	uint8_t *p = handle->prepare_send_buf;
	*p++ = PCHEAD1;
	*p++ = PCHEAD2;
	*p++ = 19;
	*p++ = 0;
	//id/len number
	*p++ = (MSGGPS<<4)+15;
	//latitude label,longitude label, mode, state ..
	if(gps_handle.gps_info.EW=='W'){
		*p = 0;
	}else{
		*p = 1;
	}
	
	if(gps_handle.gps_info.NS=='S'){
		*p+= 0;
	}else{
		*p+=2;
	}
	
	if(gps_handle.gps_info.mode=='A')
	{
		*p+=0;
	}else if(gps_handle.gps_info.mode=='D'){
		*p+=4;
	}else if(gps_handle.gps_info.mode=='E'){
		*p+=8;
	}else{
		*p+=12;
	}
	
	if(gps_handle.gps_info.state=='A'){
		*p+=0;
	}else{
		*p+=16;
	}
	p++;
	//longitude
	*p++ = gps_handle.gps_info.longitude_degree;
	//longitude cent
	*p++ = gps_handle.gps_info.longitude_cent;
	//longitude second
	memcpy(p,&gps_handle.gps_info.longitude_second,sizeof(float));
	p+=sizeof(float);
	//latitude 
	*p++ = gps_handle.gps_info.latitude_degree;
	//latitude cent
	*p++ = gps_handle.gps_info.latitude_cent;
	//latitude second
	memcpy(p,&gps_handle.gps_info.latitude_second,sizeof(float));
	p+=sizeof(float);
	//speed
	uint16_t speed = gps_handle.gps_info.speed*100;
	memcpy(p,&speed,sizeof(uint16_t));
	p+=sizeof(uint16_t);
	return 22;
}



uint8_t prepare_acc(PCHANDLE*handle)
{
	uint8_t *p = handle->prepare_send_buf;
	*p++ = PCHEAD1;
	*p++ = PCHEAD2;
	*p++ = 16;
	*p++ = 0;
	
	//id/len number
	*p++ = (MSGACC<<4)+12;
	//x
	
	memcpy(p,&yis_Handle.yis_data.acc_x,sizeof(float));
	p+=sizeof(float);
	//y
	
	memcpy(p,&yis_Handle.yis_data.acc_y,sizeof(float));
	p+=sizeof(float);
	//z
	
	memcpy(p,&yis_Handle.yis_data.acc_z,sizeof(float));
	p+=sizeof(float);
	return 19;
	
}

uint8_t prepare_eul(PCHANDLE*handle)
{
	uint8_t *p = handle->prepare_send_buf;
	*p++ = PCHEAD1;
	*p++ = PCHEAD2;
	*p++ = 16;
	*p++ = 0;
	
	//id/len number
	*p++ = (MSGEUL<<4)+12;
	//x
	
	memcpy(p,&yis_Handle.yis_data.eul_x,sizeof(float));
	p+=sizeof(float);
	//y

	memcpy(p,&yis_Handle.yis_data.eul_y,sizeof(float));
	p+=sizeof(float);
	//z
	
	memcpy(p,&yis_Handle.yis_data.eul_z,sizeof(float));
	p+=sizeof(float);
	return 19;
	
}

uint8_t prepare_gro(PCHANDLE*handle)
{
	uint8_t *p = handle->prepare_send_buf;
	*p++ = PCHEAD1;
	*p++ = PCHEAD2;
	*p++ = 16;
	*p++ = 0;
	
	//id/len number
	*p++ = (MSGGRO<<4)+12;
	//x
	
	memcpy(p,&yis_Handle.yis_data.gyr_x,sizeof(float));
	p+=sizeof(float);
	//y

	memcpy(p,&yis_Handle.yis_data.gyr_y,sizeof(float));
	p+=sizeof(float);
	//z
	
	memcpy(p,&yis_Handle.yis_data.gyr_z,sizeof(float));
	p+=sizeof(float);
	return 19;
}

uint8_t prepare_qua(PCHANDLE*handle)
{
	uint8_t *p = handle->prepare_send_buf;
	*p++ = PCHEAD1;
	*p++ = PCHEAD2;
	*p++ = 20;
	*p++ = 0;
	
	//id/len number
	*p++ = (MSGQUA<<4)+0;
	
	//1
	memcpy(p,&yis_Handle.yis_data.qua_1,sizeof(float));
	p+=sizeof(float);
	
	//2
	memcpy(p,&yis_Handle.yis_data.qua_2,sizeof(float));
	p+=sizeof(float);
	
	//3
	memcpy(p,&yis_Handle.yis_data.qua_3,sizeof(float));
	p+=sizeof(float);
	
	//4
	memcpy(p,&yis_Handle.yis_data.qua_4,sizeof(float));
	p+=sizeof(float);
	return 23;
}

uint8_t prepare_tim(PCHANDLE*handle)
{
	uint8_t *p = handle->prepare_send_buf;
	*p++ = PCHEAD1;
	*p++ = PCHEAD2;
	*p++ = 10;
	*p++ = 0;
	
	//id/len number
	*p++ = (MSGTIM<<4)+6;
	//
	
	uint16_t temp = (gps_handle.gps_info.year<<9)+(gps_handle.gps_info.month<<5)+gps_handle.gps_info.day;
	memcpy(p,&temp,sizeof(uint16_t));
	p+=sizeof(uint16_t);
	*p++= gps_handle.gps_info.hour;
	*p++= gps_handle.gps_info.minute;
	uint16_t second = gps_handle.gps_info.second*100;
	memcpy(p,&second,sizeof(uint16_t));
	p+=sizeof(uint16_t);
	return 13;
}

uint8_t prepare_sys(PCHANDLE*handle)
{
	uint8_t *p = handle->prepare_send_buf;
	*p++ = PCHEAD1;
	*p++ = PCHEAD2;
	*p++ = 10;
	*p++ = 0;
	
	//id/len number
	*p++ = (MSGSYS<<4)+6;
	//x
		
	*p++=control_info.mode;
	*p++=control_info.status;
	memcpy(p,&control_info.true_value_left,sizeof(uint16_t));
	p+=sizeof(uint16_t);
	memcpy(p,&control_info.true_value_left,sizeof(uint16_t));
	p+=sizeof(uint16_t);
	
	uint16_t temp3;
	temp3 = checkSumPcmsg(handle->prepare_send_buf+2,p-handle->prepare_send_buf-2);
	*p++ = temp3>>8;
	*p = temp3&(0x00FF);
	return 13;
}
uint16_t checkSumPcmsg(uint8_t* data,uint8_t len)
{
	uint8_t ck1 = 0, ck2 = 0;
	for(uint8_t i=0;i<len;i++)
	{
		ck1 = ck1+data[i];
		ck2 = ck1+ck2;
	}
	return (ck1<<8)+ck2;
}


void pc_send_callback(PCHANDLE* handle)
{
	if(handle->send_head==-1)
	{
		handle->send_done = 1;
	}
	else
	{
		memcpy(handle->send_buf,handle->send_raw_data[handle->send_head],handle->send_raw_data_len[handle->send_head]);
		handle->send_buf[3]= handle->send_count;
		handle->send_count++;
		uint16_t temp3;
		uint8_t len = handle->send_buf[2];
		temp3 = checkSumPcmsg(handle->send_buf+2,len-2+1);
		handle->send_buf[len+3-2] = temp3>>8;
		handle->send_buf[len+3-1] = temp3&(0x00FF);
		HAL_UART_Transmit_IT(&PCUART,handle->send_buf,handle->send_raw_data_len[handle->send_head]);
		handle->send_head = (handle->send_head+1)%SIZE_PC_SEND_QUEUE;
		if(handle->send_head==handle->send_tail)
		{
			handle->send_head=handle->send_tail = -1;
		}
	}
	
}

void pc_recv_callback(PCHANDLE* handle)
{
	if (handle->recv_index==0)
	{
		if(handle->recv_buf[0]==PCHEAD1)
		{
			handle->recv_index=1;
		}
		HAL_UART_Receive_IT(&PCUART,handle->recv_buf+handle->recv_index,1);
		return;
	}
	
	
	if(handle->recv_index==1)
	{
		if(handle->recv_buf[1]==PCHEAD2)
		{
			handle->recv_index=2;
		}
		else
		{
			handle->recv_index=0;
		}
		HAL_UART_Receive_IT(&PCUART,handle->recv_buf+handle->recv_index,1);
		return ;
	}
	
	
	if(handle->recv_index==2)
	{
		handle->recv_payload_len  = handle->recv_buf[handle->recv_index];
		if(handle->recv_payload_len<MAX_PC_LEN && handle->recv_payload_len>MIN_PC_LEN)
		{
			handle->recv_index+=handle->recv_payload_len;
			HAL_UART_Receive_IT(&PCUART,handle->recv_buf+3,handle->recv_payload_len);
		}
		else
		{
			handle->recv_index=0;
			HAL_UART_Receive_IT(&PCUART,handle->recv_buf,1);
		}
		return;
	}
	
	//checksum
	
	if(handle->recv_index>2)
	{
		if(handle->recv_tail == -1)
		{
			handle->recv_tail  = 1;
			handle->recv_head =0;
			memcpy(handle->recv_raw_data[0],handle->recv_buf+2,handle->recv_payload_len+1);
			handle->lose_parse_times++;
		
		}
		else
		{
			memcpy(handle->recv_raw_data[handle->recv_tail],handle->recv_buf+2,handle->recv_payload_len+1);
			if(handle->recv_tail==handle->recv_head) handle->recv_head=(handle->recv_head+1)%SIZE_PC_RECV_QUEUE;
			handle->recv_tail=(handle->recv_tail+1)%SIZE_PC_RECV_QUEUE;
			handle->lose_parse_times++;
		}
		
	}
	handle->recv_index=0;
	handle->recv_payload_len = 0;
	HAL_UART_Receive_IT(&PCUART,handle->recv_buf,1);
}

void pc_time_callback(PCHANDLE* handle)
{

	for(int i = SENDMINID;i<=SENDMAXID;i++)
	{
		if(handle->send_msg_enable[i]==1&&handle->send_msg_mode[i]==0)
		{
			handle->send_msg_ticker[i]++;
			if(handle->send_msg_ticker[i]==handle->send_msg_delay[i]) 
			{
				handle->send_msg_flag[i]=1;
				handle->send_msg_ticker[i]=0;
			}
		}

	}
	
	handle->time_since_last_recv++;
	if(handle->time_since_last_recv>MAX_LOSE_CONNECT_TIME)
	{
		handle->connect_status = 0;
	}
	
}

void  pc_send_recv_debug(PCHANDLE* handle)
{
		printf("status:%d  lose_times:%d   sin_time:%d   loseparse:%d    wrongparse:%d    wrongcount:%d\n",handle->connect_status,
		handle->lose_connect_times,handle->time_since_last_recv,handle->lose_parse_times,handle->wrong_frame_times,handle->wrong_count_times);
}


