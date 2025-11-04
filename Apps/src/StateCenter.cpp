#include "StateCenter.hpp"
#include "EventCenter.hpp"
#include "msg_coder.hpp" 
#include "Action.hpp"
#include "Chassis.hpp"

RobotStateCenter StateCenter;

void RoboInit(StateCore *core);
void RoboIdle(StateCore *core);
void RoboError(StateCore *core);
void RoboStop(StateCore *core);
void RoboArea1(StateCore *core);
bool Area1Flag;

/**
 * @brief 进程模块的初始化部分
 * @note 只会执行一次，在其中 组织机器人状态机的结构
 */
void RobotStateCenter::Regist()
{
    UartMsgCoder uart_coder;
    uart_coder.Init(&huart2);
    
    // 注册状态机的所有状态块
    NEW_STATE(System.Automatic_Core, Init);
    NEW_STATE(System.Automatic_Core, Idle);
    NEW_STATE(System.Automatic_Core, Stop);
    NEW_STATE(System.Automatic_Core, Error);

    NEW_STATE(System.Automatic_Core, Area1);

    // 建立状态块之间的链接
    St_Init.incore->AddLink(&(EvtSParam.RoboReady), St_Idle.incore);
    St_Idle.incore->AddLink(&(EvtCParam.StopRobot), St_Stop.incore);

    // 绘制状态机图到串口，检验状态机结构
    System.Automatic_Core.CoreGraph(uart_coder.uart_inst);
}


void RoboInit(StateCore *core)
{
}

/**
 * @brief 机器人区域1的状态
 * @details R1 在1区时的具体行为
 */
void RoboArea1(StateCore *core)
{
    SEQLIZE
    {
        // case 0:
        // p_act_0 = Action.LaunchInstant(Arm.FetchBlock());
        // p_act_1 = Action.LaunchAsync(Chassis.MoveAt(Vec2(2.0f, 2.0f));
        // ACTEND
        
        // case 1:
        // Action.WaitUntil(p_act_0->Completed() && p_act_1->Completed());
        // ACTEND

        // case 2:
        // SEQEND;
    }
}


void RoboIdle(StateCore *core)
{

}

void RoboError(StateCore *core)
{
    
}

void RoboStop(StateCore *core)
{

}