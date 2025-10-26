#ifndef EVTCENTER_HPP
#define EVTCENTER_HPP

#include "RobotSystem.hpp"


/*****      系统状态变量        *****/
class EventStatusParam
{
    public:
    EventStatusParam(){};
    bool RoboReady;            // 机器人准备就绪
};


/*****      系统控制变量        *****/
class EventControlParam
{
    public:
    EventControlParam(){};
    bool StopRobot;
};



/*****      用户自定义变量        *****/




/***    全局实例    ***/
extern EventControlParam EvtCParam;
extern EventStatusParam EvtSParam;
#endif