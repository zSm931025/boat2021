//#include "module_config.h"
//#if  USE_YIS100


//使用说明：
//1. define YIS_USART,接受数据
//2. define YIS_USART_IRQn,关中断用于数据保护
//3. 在串口中断回调函数加入“usart_recv_callback（yis_Handle）；”
//4. 调用初始化函数init_yis（yis_Handle）
//5. 100hz  调用 parse_yis（yis_Handle）解析，取数据在 yis_Handle 中获得











#ifndef __yis100_H
#define __yis100_H
#include "stm32f4xx_hal.h"
#include "usart.h"




#define YIS_USART huart3
#define YIS_USART_IRQn  USART3_IRQn




#define MIN_RECV_LEN 0x3c
#define MAX_RECV_LEN 0x3c

#define SIZE_RECV_YISBUF  0x43


#define YIS_HEAD1 0x59
#define YIS_HEAD2 0x53
#define TID_POS 2
#define PAYLOAD_LEN_POS 4
#define PAYLOAD_POS 5
#define CRC_CALC_START_POS				2
#define CRC_CALC_LEN(parse_payload_len)		((parse_payload_len) + 3)	      /*3 = tid(2B) + len(1B)*/
#define CRC_DATA_POS(parse_payload_len)			(PAYLOAD_POS + parse_payload_len)

#define ACCEL_ID				 0x10
#define GYROS_ID				 0x20
#define EULER_ID				 0x40
#define QUATERNION_ID		 0x41

#define ACCEL_DATA_LEN				 12
#define GYROS_DATA_LEN				 12
#define EULER_DATA_LEN				 12
#define QUATERNION_DATA_LEN		 16

#define DATA_FACTOR	0.000001f

typedef struct _yis_handle
{
	
	uint8_t recv_payload_len;
	uint8_t recv_buf_prt;
	uint8_t recv_buf[SIZE_RECV_YISBUF];
	uint8_t new_yis_arrival;
	uint8_t parse_ready_buf[SIZE_RECV_YISBUF];
	uint8_t parse_buf[SIZE_RECV_YISBUF];
	uint8_t parse_payload_len;
	uint8_t parse_done;
	uint16_t tid;
	UART_HandleTypeDef *pUsartHandle;
	
	struct _data
	{
		float acc_x;
		float acc_y;
		float acc_z;
	
		float qua_1;
		float qua_2;
		float qua_3;
		float qua_4;
	
		float gyr_x;
		float gyr_y;
		float gyr_z;

		float eul_x;
		float eul_y;
		float eul_z;
	
	}yis_data;
	
}YIS_HANDLE;

	







extern YIS_HANDLE yis_Handle;
void init_yis(YIS_HANDLE* handle);
void parse_yis(YIS_HANDLE* handle);
void usart_recv_callback(YIS_HANDLE* handle);
void print_yis(YIS_HANDLE* handle);








#endif


//#endif //USE_YIS100