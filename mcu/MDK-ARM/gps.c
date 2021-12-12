#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "gps.h"


GPS_HANDLE gps_handle;

void str_to_hexdata(uint8_t* pstrdata,uint8_t* result);
uint8_t  check_gps(char * DATA);
void split(char*src,char**dest);

void print_gps(GPS_HANDLE*handle)
{
		printf("year:%d   month:%d   day:%d   hour:%d   minute:%d   second:%.f\r\n\
		NS:%c   lati:%f   lati_d:%d   lati_c:%d   lati_s:%f \r\n\
		EW:%c   long:%f   long_d:%d   long_c:%d   long_s:%f \r\n\
		speed:%f   direction:%f   mode:%c   state:%c\r\n\r\n\r\n",
		gps_handle.gps_info.year,gps_handle.gps_info.month,gps_handle.gps_info.day,
		gps_handle.gps_info.hour,gps_handle.gps_info.minute,gps_handle.gps_info.second,
		gps_handle.gps_info.NS,gps_handle.gps_info.latitude,gps_handle.gps_info.latitude_degree,gps_handle.gps_info.latitude_cent,gps_handle.gps_info.latitude_second,
		gps_handle.gps_info.EW,gps_handle.gps_info.longitude,gps_handle.gps_info.longitude_degree,gps_handle.gps_info.longitude_cent,gps_handle.gps_info.longitude_second,
		gps_handle.gps_info.speed,gps_handle.gps_info.direction,gps_handle.gps_info.mode,gps_handle.gps_info.state);
		
}
void init_gps(GPS_HANDLE*handle)
{
	handle->recv_buf_prt = 0;
	handle->recv_buf_last_prt = 0;
	handle->new_gps_arrival = 0;
	handle->is_gps_parse_done = 1;
	printf("gps init:%d\n",HAL_UART_Receive_IT(&GPS_USART,handle->recv_buf+handle->recv_buf_prt,1));
	
}
	
	
	


void parse_gps(GPS_HANDLE* handle)
{
		HAL_NVIC_DisableIRQ(GPS_USART_IRQn);
		if (handle->new_gps_arrival==1)
		{
				memcpy(handle->parse_buf,handle->parse_ready_buf,SIZE_RECV_GPSBUF);
				handle->new_gps_arrival=0;
				handle->is_gps_parse_done=0;
		}
		HAL_NVIC_EnableIRQ(GPS_USART_IRQn);
		if(handle->is_gps_parse_done==0)
		{
			handle->is_gps_parse_done=1;
			if(check_gps((char*)handle->parse_buf)==1)
			{
				char *p = strchr((char*)handle->parse_buf,'*');
				*p = '\0';
				char *revbuf[13] ={0};
				
				split((char *)handle->parse_buf,revbuf);
				
				//mode
				handle->gps_info.mode = revbuf[12][0];

				//NS_latitude
				handle->gps_info.NS = revbuf[4][0];
				handle->gps_info.latitude = strtod(revbuf[3],NULL);//double
				char latitude_degree[3];
				char latitude_cent[3];
				char latitude_second[10]="0";
				strncpy(latitude_degree,revbuf[3],2);
				strncpy(latitude_cent,revbuf[3]+2,2);
				strncpy(latitude_second+1,revbuf[3]+4,7);
				handle->gps_info.latitude_degree = atoi(latitude_degree);
				handle->gps_info.latitude_cent = atoi(latitude_cent);
				handle->gps_info.latitude_second = strtod(latitude_second,NULL)*60.0;
				//EW_longitude
				handle->gps_info.EW = revbuf[6][0];
				handle->gps_info.longitude = strtod(revbuf[5],NULL);
				char longitude_degree[4];
				char longitude_cent[3];
				char longitude_second[10]="0";
				strncpy(longitude_degree,revbuf[5],3);
				strncpy(longitude_cent,revbuf[5]+3,2);
				strncpy(longitude_second+1,revbuf[5]+5,7);
				handle->gps_info.longitude_degree = atoi(longitude_degree);
				handle->gps_info.longitude_cent = atoi(longitude_cent);
				handle->gps_info.longitude_second = strtod(longitude_second,NULL)*60.0;
				//state
				handle->gps_info.state = revbuf[2][0];
				//speed
				handle->gps_info.speed = atof(revbuf[7]);
				//direction
				handle->gps_info.direction = atof(revbuf[8]);
				//Date
				char day[3];
				char month[3];
				char year[3];
				strncpy(day,revbuf[9],2);
				strncpy(month,revbuf[9]+2,2);
				strncpy(year,revbuf[9]+4,2);
				handle->gps_info.day  = atoi(day);
				handle->gps_info.month = atoi(month);
				handle->gps_info.year = atoi(year);
				char hour[3];
				char minute[3];
				char second[10];
				strncpy(hour,revbuf[1],2);
				strncpy(minute,revbuf[1]+2,2);
				strncpy(second,revbuf[1]+4,7);
				handle->gps_info.hour = atoi(hour);
				handle->gps_info.minute  = atoi(minute);
				handle->gps_info.second  = strtod(second,NULL);
			}
	 }
}
	



uint8_t check_gps(char * DATA)
{
	char* p_0=NULL;
	uint8_t cheak_value = 0;
	uint8_t cheak_sum = 0;
	p_0 = strchr(DATA,'*');
	if (p_0!=NULL)
	{
		cheak_value = DATA[1];
		for (int i =2;DATA[i]!='*';i++)
		{
			cheak_value ^= DATA[i];
		}
		str_to_hexdata((uint8_t *)p_0+1,&cheak_sum);
		if (cheak_sum==cheak_value)
		{
			return 1;
		}
		else
		{
			return 0;
		}
	}
	else
	{
		return 0;
	}
	
}




void usart_recv_gps_callback(GPS_HANDLE* handle)
{
	
	if(handle->recv_buf_prt==0)
	{
		if(*(handle->recv_buf+handle->recv_buf_prt)==GPS_HEAD)
		{
			handle->recv_buf_prt+=1;
		}
		else
		{
			handle->recv_buf_prt+=0;
		}
	}
	else if(handle->recv_buf_prt<5)
	{
		
		handle->recv_buf_prt+=1;
	}
	else if(handle->recv_buf_prt==5)
	{
		char gps_label[6] = "";
		strncpy(gps_label,(char*)(handle->recv_buf+1),5);
		if(strcmp(gps_label,GPS_TYPE)==0)
		{
			handle->recv_buf_prt+=1;
		}
		else
		{
			handle->recv_buf_prt=0;
		}
	}
	else if(handle->recv_buf_prt< MAX_GPS_RECV_LEN-1)
	{	
		if(*(handle->recv_buf+handle->recv_buf_prt)=='*')
		{
			if(handle->recv_buf_last_prt==0)
			{
				handle->recv_buf_last_prt=1;
				handle->recv_buf_prt+=1;
			}else
			{
				handle->recv_buf_last_prt=0;
				handle->recv_buf_prt=0;
				
			}
		}
		else if(handle->recv_buf_last_prt==1)
		{
			handle->recv_buf_last_prt=2;
			handle->recv_buf_prt+=1;
		}
		else if(handle->recv_buf_last_prt==2)
		{
			memcpy(handle->parse_ready_buf,handle->recv_buf,handle->recv_buf_prt+1);
			handle->new_gps_arrival=1;
			handle->recv_buf_last_prt=0;
			handle->recv_buf_prt=0;
		}
		else
		{
			handle->recv_buf_last_prt=0;
			handle->recv_buf_prt+=1;
		}
	}
	else
	{
		handle->recv_buf_prt = 0;
	}
	HAL_UART_Receive_IT(&GPS_USART,handle->recv_buf+handle->recv_buf_prt,1);
}


void str_to_hexdata(uint8_t* pstrdata,uint8_t* result)
{
	uint8_t low_buf;
	if ((*pstrdata)>=97){
	(*result)=(*pstrdata)-87;}
	else if ((*pstrdata)>=65){
	(*result)=(*pstrdata)-55;}
	else if ((*pstrdata)>=48){
	(*result)=(*pstrdata)-48;}
	
	if ((*(pstrdata+1))>=97){
	low_buf=*(pstrdata+1)-87;}
	else if ((*(pstrdata+1))>=65){
	low_buf = *(pstrdata+1)-55;}
	else if ((*(pstrdata+1))>=48){
	low_buf = *(pstrdata+1)-48;}
	
	*result = (*(result)<<4)|low_buf;
}

void split(char*src,char**dest)
{
	uint8_t i =0;
	while(1)
	{
		dest[i++]=src;
		while(*src!=','&&*src!='\0')
		{
			src++;
		}
		if(*src==',')
		{
			*src='\0';
		}
		else
		{
			break;
		}
		src++;
	}
}



	