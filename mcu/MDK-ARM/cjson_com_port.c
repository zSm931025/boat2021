#include "cjson_com_port.h"
#include "usart.h"
#include "cjson.h"
#include "pid.h"
#include <stdio.h>

static JsonUartRecTyp  mJsonRec;


void JsonReceiveInit(void)
{
    HAL_UART_Receive_IT(&JSON_UART, &mJsonRec.byte, 1);
    
}

void JsonUartRecCallback(void)
{
    HAL_UART_Receive_IT(&JSON_UART, &mJsonRec.byte, 1);
		if (mJsonRec.flg) {
        return;
    }
    if (JSON_REC_BUF_LEN <= mJsonRec.cnt) {
        mJsonRec.cnt = 0;
    }
    mJsonRec.buf[mJsonRec.cnt] = mJsonRec.byte;
    if ('\n' == mJsonRec.buf[mJsonRec.cnt]) {
        mJsonRec.flg = 1;
        mJsonRec.buf[mJsonRec.cnt] = 0;
    }
		else
			++mJsonRec.cnt;
}

void JsonParseRoutine(void)
{
    if (!mJsonRec.flg) {
			
        return;
    }
		printf("debug0");
    cJSON *cjson = cJSON_Parse((const char*)mJsonRec.buf);
    if (cjson){
        cJSON *json_val = NULL;
			
        json_val = cJSON_GetObjectItem(cjson, "set_pp");
        if (json_val){if (json_val) p_pid->p_p = json_val->valuedouble;}
				
				json_val = cJSON_GetObjectItem(cjson, "set_pi");
        if (json_val){if (json_val) p_pid->p_i = json_val->valuedouble;}
				
				json_val = cJSON_GetObjectItem(cjson, "set_pd");
        if (json_val){if (json_val) p_pid->p_d = json_val->valuedouble;}
				
				json_val = cJSON_GetObjectItem(cjson, "set_pin");
        if (json_val){if (json_val) p_pid->p_integral = json_val->valuedouble;}
				
				json_val = cJSON_GetObjectItem(cjson, "set_pinmax");
        if (json_val){if (json_val) p_pid->p_integral_max = json_val->valuedouble;}
				
				json_val = cJSON_GetObjectItem(cjson, "set_pinthre");
        if (json_val){if (json_val) p_pid->p_integral_threshold = json_val->valuedouble;}
				
				json_val = cJSON_GetObjectItem(cjson, "set_pomax");
        if (json_val){if (json_val) p_pid->p_max_output = json_val->valuedouble;}
           
        json_val = cJSON_GetObjectItem(cjson, "set_vp");
        if (json_val){if (json_val) p_pid->v_p = json_val->valuedouble;}
				
				json_val = cJSON_GetObjectItem(cjson, "set_vi");
        if (json_val){if (json_val) p_pid->v_i = json_val->valuedouble;}
				
				json_val = cJSON_GetObjectItem(cjson, "set_vd");
        if (json_val){if (json_val) p_pid->v_d = json_val->valuedouble;}
				
				json_val = cJSON_GetObjectItem(cjson, "set_vin");
        if (json_val){if (json_val) p_pid->v_integral = json_val->valuedouble;}
				
				json_val = cJSON_GetObjectItem(cjson, "set_vinmax");
        if (json_val){if (json_val) p_pid->v_integral_max = json_val->valuedouble;}
				
				json_val = cJSON_GetObjectItem(cjson, "set_vinthre");
        if (json_val){if (json_val) p_pid->v_integral_threshold = json_val->valuedouble;}
				
				json_val = cJSON_GetObjectItem(cjson, "set_vomax");
        if (json_val){if (json_val) p_pid->v_max_output = json_val->valuedouble;}
				
				json_val = cJSON_GetObjectItem(cjson, "test0");
        if (json_val){if (json_val) printf("test0  %f",json_val->valuedouble);}
				
				
				json_val = cJSON_GetObjectItem(cjson, "rst");
        if (json_val){if (json_val) NVIC_SystemReset();}
        
    }
		cJSON_Delete(cjson);
    mJsonRec.cnt = 0;
    mJsonRec.flg = 0;
}


