/***************************************************************************** 
* 
* File Name : main.c
* 
* Description: main 
* 
* Copyright (c) 2014 Winner Micro Electronic Design Co., Ltd. 
* All rights reserved. 
* 
* Author : dave
* 
* Date : 2014-6-14
*****************************************************************************/ 
#include "wm_include.h"

void UserMain(void)
{
	printf("\n user task \n");
    int demo_bt_enable();
    demo_bt_enable();
    int get_bt_adapter_state();
    while ((get_bt_adapter_state() == 0))tls_os_time_delay(10);
    int demo_bt_app_on();
    demo_bt_app_on();
#if DEMO_CONSOLE
	CreateDemoTask();
#endif
//�û��Լ���task
}

