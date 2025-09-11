#include "RtosCpp.hpp"
#include "std_cpp.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "bsp_dwt.h"


/**
 * @brief 机器人主要的应用层任务
 * @note 负载较低，以10Hz运行
 */
void RobotApplicationCpp()
{
    uint32_t AppTick = xTaskGetTickCount();

    while (1)
    {   
        // 维护DWT计时器
        DWT_CntUpdate();
        osDelayUntil(&AppTick, 100);    // 10Hz
    }
    
}

void RobotMainCpp()
{
    while (1)
    {
        osDelay(1);
    }
}

void SlowControlCpp()
{
    while (1)
    {
        osDelay(1);
    }
}

void FastControlCpp()
{
    while (1)
    {
        osDelay(1);
    }
}
