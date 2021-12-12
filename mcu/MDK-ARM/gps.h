#ifndef __gps_H
#define __gps_H


#include "usart.h"

#define GPS_USART huart2
#define GPS_USART_IRQn  USART2_IRQn



#define SIZE_RECV_GPSBUF   80
#define GPS_HEAD  '$'
#define GPS_TYPE  "GNRMC"
#define MAX_GPS_RECV_LEN 80




typedef struct 
{
	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	float second;
	
	char NS;
	double latitude;
	uint8_t latitude_degree;
	uint8_t latitude_cent;
	float latitude_second;
	
	char EW;
	double longitude;
	uint8_t longitude_degree;
	uint8_t longitude_cent;
	float longitude_second;
	
	float speed;
	float direction;
	char mode;     //A=自主定位，D=差分，E=估算，N=数据无效
	char state;    //A=有效定位，V=无效定位
	
}GPS_INFO;



typedef struct
{
	uint8_t recv_buf_prt;
	uint8_t recv_buf_last_prt;
	uint8_t recv_buf[SIZE_RECV_GPSBUF];
	uint8_t parse_ready_buf[SIZE_RECV_GPSBUF];
	uint8_t parse_buf[SIZE_RECV_GPSBUF];
	uint8_t new_gps_arrival;
	uint8_t is_gps_parse_done;
	GPS_INFO gps_info;
	
}GPS_HANDLE;


extern GPS_HANDLE gps_handle;
void init_gps(GPS_HANDLE*handle);
void parse_gps(GPS_HANDLE* handle);
void usart_recv_gps_callback(GPS_HANDLE* handle);
void print_gps(GPS_HANDLE*handle);

#endif    //USE_GPS