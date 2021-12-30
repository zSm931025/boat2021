#ifndef __cjson_com_port_H
#define __cjson_com_port_H
#include "stdint.h"

/*
{"set":{"yaw":90,"kp1":1.5,"ki1":0.05,"kd1":0.1,"integral_threshold_err":5}\n
{"dbg":{"rst":1}}\n
*/

#define JSON_REC_BUF_LEN   (200)
#define JSON_UART        huart1

typedef struct _JsonUartRec {
    uint8_t buf[JSON_REC_BUF_LEN];
    uint8_t cnt;
    uint8_t flg;
    uint8_t byte;
} JsonUartRecTyp;




void JsonReceiveInit(void);
void JsonUartRecCallback(void);
void JsonParseRoutine(void);





#endif