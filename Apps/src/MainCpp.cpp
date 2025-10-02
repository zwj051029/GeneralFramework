#include "MainCpp.hpp"
#include "tim.h"
#include "spi.h"
#include "main.h"
#include "std_cpp.h"
#include "odo_ops.hpp"
#include "led_ws2812.hpp"
#include "bsp_dwt.h"
#include "WS2812_yx.h"
#include "motor_dji.hpp"

Ops9 myOdo;
LedWs2812 myLedWs;

MotorDji myMotor;

Color Up70_Green(105, 209, 25);
Color Up70_Purple(70, 0, 190);


void MainCpp()
{   
    // 初始化DWT计时器 
    DWT_Init(CPU_HERT_A_BOARD_MHZ);
    
    // 初始化里程计
    myOdo.Init(&huart6);

    // 初始化电机
    myMotor.Init(&hcan1, 1, Speed_Control);

    // 初始化LED
    myLedWs.Init(&htim5, TIM_CHANNEL_4, 11);
    
    while (1)
    {
        myLedWs.GradientFlow(Up70_Green, Up70_Purple, 5);
        myLedWs.SendData();
        HAL_Delay(20);
    }
    
    // 
}