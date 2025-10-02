#ifndef _CHASSIS_HPP_
#define _CHASSIS_HPP_

#include "stm32f4xx_hal.h"
#include "std_math.hpp"
#include "motor_dji.hpp"

/**
 * @brief 路径类，包含了一系列路径点
 */
class Path
{
    Vec2 key_points[32];            // 路径点数组
};


class Chassis
{
    private:
        // 配置参数
        bool EnAccelLim;            // 是否启用加速度限制
        float Accel = 0.0f;         // 最大加速度，单位m/s^2
        bool EnVeloLim;             // 是否启用速度限制
        float VeloLim = 0.0f;       // 最大线速度，单位m/s

    public:
        Chassis(){};
        ~Chassis(){};

        typedef enum
        {
            Omni,       // 全向轮
            Meca,       // 麦克纳姆轮
            Steer,      // 舵轮
        }ChassisType;
        
        // 底盘类型，默认为全向轮
        ChassisType type = Omni;

        // 分别对应四个底盘电机
        MotorDji *Motors[4] = {nullptr, nullptr, nullptr, nullptr};

        // 属性参数
        Vec3 Speed;     // 期望速度，车体右手系，x向前，y向左，z左转，单位m/s，rad/s
        float Velo;     // 对应的线速度，单位m/s

        void Init(MotorDji* m1, MotorDji* m2, MotorDji* m3, MotorDji* m4, ChassisType t = Omni);
        void Config();
        void Move(Vec3 Spd);
        void MoveAt(Vec2 Pos);
        void MoveAlong(Path path_t);
};


#endif