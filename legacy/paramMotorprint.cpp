#include "paramMotor.h"
#include "unitreeMotor.h"


void param_allprint(MOTOR_recv motor_r)
{
    std::cout << "Motor_id: " << motor_r.motor_id << std::endl;

    std::cout << "error: " << motor_r.MError << std::endl;

    std::cout << "位置pos: " << motor_r.Pos << std::endl;
    std::cout << "速度elocity: " << motor_r.W << std::endl;
    std::cout << "力矩torque: " << motor_r.T << std::endl;
    std::cout << "温度temp " << motor_r.Temp << std::endl;
}
