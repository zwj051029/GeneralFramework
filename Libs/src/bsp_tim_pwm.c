#include "bsp_tim_pwm.h"

void BspTIMPWM_InstRegist(BspTIMPWM_TypeDef *tim_inst, TIM_HandleTypeDef *htim, uint32_t channel)
{
    tim_inst->htim = htim;
    tim_inst->channel = channel;

    // 获取ARR的值
    tim_inst->auto_reload_value = __HAL_TIM_GET_AUTORELOAD(tim_inst->htim);
    // 获取CCR的值
    tim_inst->compare_value = __HAL_TIM_GET_COMPARE(tim_inst->htim, tim_inst->channel);

    // 设置初始占空比为 0
    BspTIMPWM_SetDuty(tim_inst, 0.0f);
}


void BspTIMPWM_SetDuty(BspTIMPWM_TypeDef *tim_inst, float duty)
{
    if(duty < 0.0f) duty = 0.0f;
    if(duty > 1.0f) duty = 1.0f;

    tim_inst->duty = duty;
    
    // 计算CCR的对应值
    tim_inst->compare_value = (uint32_t)(tim_inst->auto_reload_value * duty);
    __HAL_TIM_SET_COMPARE(tim_inst->htim, tim_inst->channel, tim_inst->compare_value);
}


void BspTIMPWM_Enable(BspTIMPWM_TypeDef *tim_inst)
{
    HAL_TIM_PWM_Start(tim_inst->htim, tim_inst->channel);
    tim_inst->enabled = 1;
}


