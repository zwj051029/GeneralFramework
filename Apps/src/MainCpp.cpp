#include "MainCpp.hpp"
#include "tim.h"
#include "spi.h"
#include "main.h"
#include "std_cpp.h"
#include "odo_ops.hpp"
#include "led_ws2812.hpp"
#include "bsp_dwt.h"
#include "WS2812_yx.h"

Ops9 myOdo;
LedWs2812 myLedWs;

int a = 1;

void WS2812_Refresh()
{
    HAL_SPI_Transmit_DMA(&hspi4, (uint8_t*)WS2812buf2send, 24 * (LED_Nums + 1));
}


void MainCpp()
{   
    // 初始化DWT计时器 
    DWT_Init(CPU_HERT_A_BOARD_MHZ);
    
    // 初始化里程计
    myOdo.Init(&huart6);
    
    // 初始化WS2812
    myLedWs.Init(&htim5, TIM_CHANNEL_4, 7);
    WS2812_InitBuffer();

    WS2812_AddStateLink(&a, &Yellow_Flow);



    // 设置LED颜色
    myLedWs.SetColor(0, 255, 0, 0);
    myLedWs.SetColor(1, 255, 0, 0);
    myLedWs.SetColor(2, 255, 0, 0);
    myLedWs.SetColor(3, 255, 0, 0);
    myLedWs.SetColor(4, 255, 0, 0);
    myLedWs.SetColor(5, 255, 0, 0);
    myLedWs.SetColor(6, 0, 255, 0);

    while (1)
    {
        // 发送到灯带
        myLedWs.SendData();
        WS2812_State_Handler();

        HAL_Delay(10);
    }
    
    
}