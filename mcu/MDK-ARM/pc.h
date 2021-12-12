#ifndef pc_H
#define pc_H
#include "cmsis_os.h"
#include "usart.h"


#define PCUART  huart5
#define PC_IRQn UART5_IRQn



#define MAX_PC_LEN 30
#define MIN_PC_LEN 3

#define PCHEAD1 0xF1
#define PCHEAD2 0xF2


#define SIZE_PC_RECV_QUEUE 3  
// TO DO:  try not to use division, use shiftting method
#define SIZE_PC_RECV_BUF 100
#define SIZE_PC_SEND_QUEUE 3
#define SIZE_PC_SEND_BUF  60

#define NUM_MSGTYPE 8  //actual num + 1
#define SENDMINID 1
#define SENDMAXID 7
#define MSGGPS_DELAY    800       //1hz
#define MSGACC_DELAY    1000   //50hz
#define MSGEUL_DELAY    30    //20hz
#define MSGGRO_DELAY    30
#define MSGQUA_DELAY    1000
#define MSGTIM_DELAY    1000
#define MSGSYS_DELAY    1000	

#define MAX_LOSE_CONNECT_TIME 1000 

typedef enum 
{
	CMDCTL=1,
	CMDSET
}CMDTYPE;

typedef enum
{
	MSGGPS=1,
	MSGACC,
	MSGEUL,
	MSGGRO,
	MSGQUA,
	MSGTIM,
	MSGSYS
}MSGTYPE;


typedef struct 
{
	
	//statistic
  	uint8_t connect_status;                    // connect or disconnect
	  uint16_t lose_connect_times;                // the count of losing connect
		uint16_t time_since_last_recv;              // time since last recv  

		uint16_t lose_parse_times;                 // the number of frame recieved which did't get parsed
		uint16_t wrong_frame_times;                // the number of frame which did't through parse
		uint16_t wrong_count_times;                // 
			
	//recv
		int8_t  recv_head;
		int8_t  recv_tail;	
		uint8_t recv_index;
		uint8_t recv_payload_len;
		uint8_t recv_buf[SIZE_PC_RECV_BUF];
		uint8_t recv_raw_data[SIZE_PC_RECV_QUEUE][SIZE_PC_RECV_BUF];
	
	//parse

		uint8_t parse_done;
		uint8_t parse_buf[SIZE_PC_RECV_BUF];
	
	
	//send
	  uint8_t send_msg_mode[NUM_MSGTYPE];      //0: cycle    1: trigger
		uint8_t send_msg_enable[NUM_MSGTYPE];    //enable:1       disenable:0
		uint8_t send_msg_flag[NUM_MSGTYPE];      //0:not ready    1:ready
		uint16_t send_msg_delay[NUM_MSGTYPE];
		uint16_t send_msg_ticker[NUM_MSGTYPE];
		
		int8_t  send_head;
		int8_t  send_tail;
		uint8_t send_done;
		uint8_t send_buf[SIZE_PC_SEND_BUF];
		uint8_t send_raw_data[SIZE_PC_SEND_QUEUE][SIZE_PC_SEND_BUF];
		uint8_t send_raw_data_len[SIZE_PC_SEND_QUEUE];
		uint8_t send_count;
		uint8_t prepare_send_buf[SIZE_PC_SEND_BUF];
		
		
		struct 
		{
			uint8_t mode; //0,1,2   (0:静默，1:数传，2:遥控器)
			int8_t velocity;//-100-100（前进为正）
			int8_t rotation;//-100-100（顺时针为正）
      uint8_t accelerate;//0-100
			uint8_t gear;//1,2,3(对应最大速度30%，60%，100%)
		}ctlCmd;
	
	
}PCHANDLE;



extern  PCHANDLE pcHandle;
void pc_recv_callback(PCHANDLE* handle);
void pc_send_callback(PCHANDLE* handle);
void pc_time_callback(PCHANDLE* handle);
void init_pc_interface(PCHANDLE* handle);
void parse_pc(PCHANDLE*handle);
void send_pc(PCHANDLE*handle);
void pc_send_recv_debug(PCHANDLE* handle);





#endif