#include "Chassis.hpp"
ChassisClass Chassis;

void ChassisClass::Init(MotorDji* m1, MotorDji* m2, MotorDji* m3, MotorDji* m4, ChassisType t)
{
    Motors[0] = m1;
    Motors[1] = m2;
    Motors[2] = m3;
    Motors[3] = m4;
    type = t;
}

BaseAction* ChassisClass::MoveAt(Vec2 Pos)
{
    // 如果当前动作正在运行，先取消
    if (move_action.GetState() == Action.RUNNING)
    {
        move_action.Cancel();
    }
    
    move_action.movetype = AtPos;
    move_action.Reset(Pos);

    return &move_action;
}

BaseAction* ChassisClass::MoveAt(Vec2 Pos, float MaxVelo, float MaxAccel)
{
    // 如果当前动作正在运行，先取消
    if (move_action.GetState() == Action.RUNNING)
    {
        move_action.Cancel();
    }
    
    move_action.movetype = AtPos;
    move_action.Reset(Pos);

    return &move_action;
}


void _MoveAct::Reset(Vec2 new_target_vel)
{
    target_pos = new_target_vel;       // 更新目标速度
    
    state = Action.READY;        // 重置状态为就绪
    timeout_ms = 0;                    // 重置超时
    dwt_tick = 0;                    // 重置启动时间
}


/**
 * @brief 移动动作的更新函数
 * @note 这玩意会以200Hz被循环调用
 */
bool _MoveAct::OnUpdate()
{
    // 这里实现移动到target_pos的逻辑
    
}