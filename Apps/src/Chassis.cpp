#include "Chassis.hpp"


void Chassis::Init(MotorDji* m1, MotorDji* m2, MotorDji* m3, MotorDji* m4, ChassisType t)
{
    Motors[0] = m1;
    Motors[1] = m2;
    Motors[2] = m3;
    Motors[3] = m4;
    type = t;
}
