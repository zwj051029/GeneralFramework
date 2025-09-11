#include "led_ws2812.hpp"

#define MaxLedNums 16

static LedWs2812* targ_led;
static uint32_t WS2812buf2send[MaxLedNums][24] = {0};

void LedWs2812::Init(TIM_HandleTypeDef *htim, uint32_t Channel, uint8_t LedNums)
{
    // 记录所用的PWM通道，灯数
    this->htim = htim;
    this->Channel = Channel;
    this->LedNums = LedNums;

    for(int i = 0; i < MaxLedNums ; i++)
    {
        SetColor(i, 0, 0, 0);
    }

    // 初始化PWM
    PwmMaxValue = __HAL_TIM_GET_AUTORELOAD(this->htim);
    HIGH_WS2812 = PwmMaxValue * 0.67f;    // PWM高电平数值
    LOW_WS2812 = PwmMaxValue * 0.33f;           // PWM低电平数值   

    // 注册
    targ_led = this;
}

/**
 * @brief 设置某个LED的颜色
 */
void LedWs2812::SetColor(int8_t Led_id, uint8_t R, uint8_t G, uint8_t B)
{
    // 不允许 Led_id 大于 实际拥有的灯数
    while(Led_id > (LedNums - 1))   Led_id -= LedNums;
    while(Led_id < 0)   Led_id += LedNums;


    // 向 数组 覆写颜色
	int i = 0;
	for(i=0;i<8;i++) WS2812buf2send[Led_id][i]   = ( G & (1 << (7 -i))? (HIGH_WS2812):LOW_WS2812 ); 
	for(i=8;i<16;i++) WS2812buf2send[Led_id][i]  = ( R & (1 << (15-i))? (HIGH_WS2812):LOW_WS2812 ); 
	for(i=16;i<24;i++) WS2812buf2send[Led_id][i] = ( B & (1 << (23-i))? (HIGH_WS2812):LOW_WS2812 ); 
}


/**
 * @brief 发送数据，更新LED的颜色（使用PWM + DMA）
 * @details 对于某一个LED的RGB信息，共24bit，每个bit发送：{0 ：取占空比33%，1：取占空比66%}
 * @warning PWM DMA被配置为byte了
 */
void LedWs2812::SendData()
{
    HAL_TIM_PWM_Start_DMA(htim, Channel, (uint32_t*)WS2812buf2send, LedNums * 24);
}

/**
 * @brief 覆写PWM + DMA发送完成中断
 */
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == targ_led->htim)
    {
        HAL_TIM_PWM_Stop_DMA(targ_led->htim, targ_led->Channel);
    }
}