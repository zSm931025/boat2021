#include "yis100.h"
#include "string.h"
#include "stdio.h"


YIS_HANDLE yis_Handle;
uint8_t cheak_sum(uint8_t* handle);
int get_signed_int(uint8_t *data);

void init_yis(YIS_HANDLE* handle)
{
	handle->recv_buf_prt = 0;
	handle->pUsartHandle = &YIS_USART;
	handle->parse_done = 1;
	handle->new_yis_arrival = 0;
	HAL_UART_Receive_IT(handle->pUsartHandle,handle->recv_buf+handle->recv_buf_prt,1);
	
}


void usart_recv_callback(YIS_HANDLE* handle)
{
	
	if(handle->recv_buf_prt==0 && *(handle->recv_buf+handle->recv_buf_prt)==YIS_HEAD1)
	{
		handle->recv_buf_prt+=1;
	}
	else if(handle->recv_buf_prt==1 && *(handle->recv_buf+handle->recv_buf_prt)==YIS_HEAD2)
	{
		handle->recv_buf_prt+=1;
		
	}
	else if(handle->recv_buf_prt==2 || handle->recv_buf_prt==3)
	{
		handle->recv_buf_prt+=1;
		
	}
	else if(handle->recv_buf_prt==4 && handle->recv_buf[handle->recv_buf_prt]>= MIN_RECV_LEN && handle->recv_buf[handle->recv_buf_prt]<= MAX_RECV_LEN)
	{
		
		handle->recv_payload_len = handle->recv_buf[handle->recv_buf_prt];
		handle->recv_buf_prt+=1;
	}
	else if(handle->recv_buf_prt<(handle->recv_payload_len+6)&& handle->recv_buf_prt>4)
	{
		handle->recv_buf_prt+=1;
	}
	else if(handle->recv_buf_prt==(handle->recv_payload_len+6))
	{
		memcpy(handle->parse_ready_buf,handle->recv_buf,handle->recv_buf_prt+1);
		handle->new_yis_arrival=1;
		handle->recv_buf_prt=0;
	}
	else
	{
		handle->recv_buf_prt = 0;
	}
	HAL_UART_Receive_IT(handle->pUsartHandle,handle->recv_buf+handle->recv_buf_prt,1);
}



void parse_yis(YIS_HANDLE* handle)
{
	HAL_NVIC_DisableIRQ(YIS_USART_IRQn);
	if(handle->new_yis_arrival==1)
	{
		handle->new_yis_arrival=0;
		handle->parse_done=0;
		memcpy(handle->parse_buf,handle->parse_ready_buf,SIZE_RECV_YISBUF);
	}
	HAL_NVIC_EnableIRQ(YIS_USART_IRQn);
	
	
	if(handle->parse_done==0)
	{
		
		handle->parse_done=1;
	
		if(cheak_sum(handle->parse_buf))
		{
		
			handle->tid = *((uint16_t*)(handle->parse_buf+TID_POS));
			handle->parse_payload_len = handle->parse_buf[PAYLOAD_LEN_POS];
			uint8_t pos = PAYLOAD_POS;
			uint8_t end = CRC_DATA_POS(handle->parse_payload_len);
			uint8_t id;
			uint8_t len;
			//printf("0 pos:%d   end:%d",pos,end);
			while(pos!=end)
			{
			
				id = handle->parse_buf[pos];
				len = handle->parse_buf[pos+1];
		
				switch(id)
				{
					case ACCEL_ID:
						{
							if(len==ACCEL_DATA_LEN)
							{
								handle->yis_data.acc_x = get_signed_int(handle->parse_buf+pos+2)*DATA_FACTOR;
								handle->yis_data.acc_y = get_signed_int(handle->parse_buf+pos+2+4*1)*DATA_FACTOR;
								handle->yis_data.acc_z = get_signed_int(handle->parse_buf+pos+2+4*2)*DATA_FACTOR;
								pos = pos+2+12;
							}
							else
							{
								
								pos++;
							}
						}
						break;
					case GYROS_ID:{
						if(len==GYROS_DATA_LEN)
						{
							handle->yis_data.gyr_x = get_signed_int(handle->parse_buf+pos+2)*DATA_FACTOR;  //roll_dot
							handle->yis_data.gyr_y = get_signed_int(handle->parse_buf+pos+2+4*1)*DATA_FACTOR;  //pitch_dot
							handle->yis_data.gyr_z = get_signed_int(handle->parse_buf+pos+2+4*2)*DATA_FACTOR;  //yaw_dot
							pos = pos+2+12;
						}
						else
							{
							pos++;
						}
						}
						break;
					case EULER_ID:{
						if(len==EULER_DATA_LEN)
						{
							handle->yis_data.eul_y = get_signed_int(handle->parse_buf+pos+2)*DATA_FACTOR;  //pitch
							handle->yis_data.eul_x = get_signed_int(handle->parse_buf+pos+2+4*1)*DATA_FACTOR;  //roll
							handle->yis_data.eul_z = get_signed_int(handle->parse_buf+pos+2+4*2)*DATA_FACTOR;  //yaw
							pos = pos+2+12;
						}
						else
						{
							pos++;
						}
						}
						break;
					case QUATERNION_ID:{
						if(len==QUATERNION_DATA_LEN)
						{
							handle->yis_data.qua_1 = get_signed_int(handle->parse_buf+pos+2)*DATA_FACTOR;
							handle->yis_data.qua_2 = get_signed_int(handle->parse_buf+pos+2+4*1)*DATA_FACTOR;
							handle->yis_data.qua_3 = get_signed_int(handle->parse_buf+pos+2+4*2)*DATA_FACTOR;
							handle->yis_data.qua_4 = get_signed_int(handle->parse_buf+pos+2+4*3)*DATA_FACTOR;
							pos = pos+2+16;
						}
						else
						{
							pos++;
						}
						}
						break;
					default:
						pos++;
						break;
				}

			}

		}
	}
		
		
	}


uint8_t cheak_sum(uint8_t* data)
{
	uint8_t payload_len = data[4];
	uint8_t crc_len = CRC_CALC_LEN(payload_len);
	uint8_t check_a = 0;
	uint8_t check_b = 0;
	uint16_t checksum;

	if(NULL == data || 0 == payload_len)
	{
		return 0;
	}
	for(uint8_t i = 0; i < crc_len; i++)
	{
		check_a += data[i+CRC_CALC_START_POS];
		check_b += check_a;
	}
	return ((uint16_t )(check_b << 8) | check_a) == *((uint16_t *)(data + CRC_DATA_POS(payload_len)));
}

int get_signed_int(uint8_t *data)
{
	int temp = 0;

	temp = (int)((data[3] << 24) | (data[2] << 16) | (data[1] << 8) | data[0]);

	return temp;
}
	




void print_yis(YIS_HANDLE* handle)
{
	printf("euler_x:%f  _y:%f   _z:%f\r\n  acc_x:%f    _y:%f    _z:%f \r\n\r\n",handle->yis_data.eul_x,handle->yis_data.eul_y,
	handle->yis_data.eul_z,handle->yis_data.acc_x,handle->yis_data.acc_y,handle->yis_data.acc_z);
}

//#endif //USE_YIS100
