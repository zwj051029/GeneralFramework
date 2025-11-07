#include "StateCenter.hpp"
#include "EventCenter.hpp"
#include "msg_coder.hpp" 
#include "Action.hpp"
#include "Chassis.hpp"
#include "StateCore.hpp"

RobotStateCenter StateCenter;

/**
 * @brief 进程模块的初始化部分
 * @note 只会执行一次，在其中 组织机器人状态机的结构
 */
void RobotStateCenter::Regist()
{
    UartMsgCoder uart_coder;
    uart_coder.Init(&huart2);
    
    // 进入简并模式
    System.Automatic_Core.Degenerate();

    // 绘制状态机图到串口，检验状态机结构
    System.Automatic_Core.CoreGraph(uart_coder.uart_inst);
}

/**     覆写简并函数    **/
void Roboworking(StateCore *core)
{
}
