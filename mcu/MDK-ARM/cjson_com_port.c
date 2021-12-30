#include "cjson_com_port.h"
#include "usart.h"
#include "cjson.h"
#include "pid.h"
#include <stdio.h>
#include "control_boat.h"
#include "yis100.h"

static JsonUartRecTyp  mJsonRec;


void JsonReceiveInit(void)
{
		printf("init cjson!\n");
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
		cJSON *cjson = cJSON_Parse((const char*)mJsonRec.buf);
    if (cjson){
        cJSON *json_val = NULL;
				
				json_val = cJSON_GetObjectItem(cjson, "set_angle");
        if (json_val) set_angle = json_val->valuedouble;
			
        json_val = cJSON_GetObjectItem(cjson, "set_pp");
        if (json_val) p_pid->p_p = json_val->valuedouble;
				
				json_val = cJSON_GetObjectItem(cjson, "set_pi");
        if (json_val) p_pid->p_i = json_val->valuedouble;
				
				json_val = cJSON_GetObjectItem(cjson, "set_pd");
        if (json_val) p_pid->p_d = json_val->valuedouble;
				
				json_val = cJSON_GetObjectItem(cjson, "set_pinmax");
        if (json_val) p_pid->p_integral_max = json_val->valuedouble;
				
				json_val = cJSON_GetObjectItem(cjson, "set_pinthre");
        if (json_val) p_pid->p_integral_threshold = json_val->valuedouble;
				
				json_val = cJSON_GetObjectItem(cjson, "set_pomax");
        if (json_val) p_pid->p_max_output = json_val->valuedouble;
           
        json_val = cJSON_GetObjectItem(cjson, "set_vp");
        if (json_val) p_pid->v_p = json_val->valuedouble;
				
				json_val = cJSON_GetObjectItem(cjson, "set_vi");
        if (json_val) p_pid->v_i = json_val->valuedouble;
				
				json_val = cJSON_GetObjectItem(cjson, "set_vd");
        if (json_val) p_pid->v_d = json_val->valuedouble;
				
				json_val = cJSON_GetObjectItem(cjson, "set_vinmax");
        if (json_val) p_pid->v_integral_max = json_val->valuedouble;
				
				json_val = cJSON_GetObjectItem(cjson, "set_vinthre");
        if (json_val) p_pid->v_integral_threshold = json_val->valuedouble;
				
				json_val = cJSON_GetObjectItem(cjson, "set_vomax");
        if (json_val) p_pid->v_max_output = json_val->valuedouble;
				
				json_val = cJSON_GetObjectItem(cjson, "rst");
        if (json_val) NVIC_SystemReset();
				
				json_val = cJSON_GetObjectItem(cjson, "test0");
        if (json_val) printf("test0  %f",json_val->valuedouble);
					
				printf("PID mode:%d  Now angle:%.2f  set_angle:%.2f\n",pid_mode,yis_Handle.yis_data.eul_z,set_angle);
				printf("set_pp:%.2f set_pi:%.2f set_pd:%.2f set_pinmax:%.2f set_pinthre:%.2f set_pomax:%.2f\n",p_pid->p_p,p_pid->p_i,p_pid->p_d,p_pid->p_integral_max,p_pid->p_integral_threshold,p_pid->p_max_output);
				printf("set_vp:%.2f set_vi:%.2f set_vd:%.2f set_vinmax:%.2f set_vinthre:%.2f set_vomax:%.2f\n",p_pid->v_p,p_pid->v_i,p_pid->v_d,p_pid->v_integral_max,p_pid->v_integral_threshold,p_pid->v_max_output);
        
    }
		cJSON_Delete(cjson);
    mJsonRec.cnt = 0;
    mJsonRec.flg = 0;
}


