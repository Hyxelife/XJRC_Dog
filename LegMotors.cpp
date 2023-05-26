#include "LegMotors.h"
#include <iostream>
#include <fstream>
using namespace std;
//void __init();

float LegMotors::ms_motorScalar;
bool LegMotors::ms_systemSafe = true;
std::vector<LegMotors*> LegMotors::ms_registerMotor;

#define DEG(deg)    deg*3.141592653589/180
#define RAD(rad)    rad*180/3.141592653589

#define SAFE_TRAP(msg)      {\
                                ms_systemSafe = false;\
                                int motorID;\
                                for(motorID = 0;motorID < ms_registerMotor.size();motorID++)\
                                {if(this == ms_registerMotor[motorID])break;}\
                                while(true){cout<<"[LegMotor]:system unsafe!!!motor:"<<m_name<<msg<<endl;usleep(10000000);}\
                            }


void LegMotors::_checkPos(float shoulderPos,float armPos,float feetPos)
{
    return;
    const float safetyAngle = DEG(5);
    while(!ms_systemSafe)usleep(10000000);

    float deltaAngle = fabsf(m_currentAngle.shoulderHorizontal-shoulderPos);
    if(deltaAngle >= safetyAngle)
        SAFE_TRAP("[LegMotor]:1电机控制角度相差过大!");
    deltaAngle = fabsf(m_currentAngle.armRotation-armPos);
    if(deltaAngle >= safetyAngle)
        SAFE_TRAP("[LegMotor]:2电机控制角度相差过大!");
    deltaAngle = fabsf(m_currentAngle.armFeetIntersect - feetPos);
    if(deltaAngle >= safetyAngle)
        SAFE_TRAP("[LegMotor]:3电机控制角度相差过大!");
}
void LegMotors::_checkRange(float shoudlerAngle,float armAngle,float feetAngle)
{
    const float safetyShoulder[2] = {DEG(-40),DEG(40)};
    const float safetyArm[2] = {DEG(30),DEG(175)};
    const float safetyFeet[2] = {DEG(15),DEG(140)};
    while(!ms_systemSafe);

    if(shoudlerAngle < safetyShoulder[0])
        SAFE_TRAP("[LegMotor]:shoudler angle, too small!");
    if(shoudlerAngle > safetyShoulder[1])
        SAFE_TRAP("[LegMotor]:shoudler angle, too large!");

    if(armAngle < safetyArm[0])
        SAFE_TRAP("[LegMotor]:arm angle, too small!");
    if(armAngle > safetyArm[1])
        SAFE_TRAP("[LegMotor]:arm angle, too large!");

    if(feetAngle < safetyFeet[0])
        SAFE_TRAP("[LegMotor]:feet angle, too small!");
    if(feetAngle > safetyFeet[1])
        SAFE_TRAP("[LegMotor]:feet angle, too large!");
}

LegMotors::LegMotors(AxisMovement zeroPos,std::vector<float> motorSign,float motorScalar,std::string portName)
:m_serial(portName),
m_motorSign(motorSign),
m_zeros(zeroPos),
m_name(portName)
{
    MOTOR_recv recv = position_get(m_serial, 0);
    m_currentAngle.shoulderHorizontal = m_motorSign[0] * (recv.Pos - m_zeros.shoulderHorizontal) / ms_motorScalar;
    m_currentTorque.shoulderTorque = m_motorSign[0]*recv.T;

    recv = position_get(m_serial, 1);
    m_currentAngle.armRotation = m_motorSign[1] * (recv.Pos - m_zeros.armRotation) / ms_motorScalar;
    m_currentTorque.armTorque = m_motorSign[1]*recv.T;

    recv = position_get(m_serial, 2);
    m_currentAngle.armFeetIntersect = m_motorSign[2] * (recv.Pos - m_zeros.armFeetIntersect) / ms_motorScalar;
    m_currentTorque.armTorque = m_motorSign[2]*recv.T;
}


// #include <Windows.h>
// HANDLE log_mutex;
// fstream file;
// bool init = false;

// void __init()
// {
//     if (init)return;
//     file.open("./leg.txt", ios::out);
//     log_mutex = CreateMutex(NULL, FALSE, NULL);
//     init = true;
// }


void LegMotors::PositionCtrl(float shoulderAngle,float armAngle,float armFeetInterAngle)
{
    // static int step = 0;
    // step++;
    // if (step % 300000 != 0)return;
    // WaitForSingleObject(log_mutex, INFINITE);
     //stringstream formater;
     //formater <<m_name<<" "<< RAD(shoulderAngle) << "," << RAD(armAngle) << "," << RAD(armFeetInterAngle) << endl;
     //string str = formater.str();
    // file.write(str.c_str(),str.size());
    // file.flush();
    // ReleaseMutex(log_mutex);
    //cout << "position control:\n"<< str;
    //   usleep(100000);
    //return;
    //_checkPos(shoulderAngle, armAngle, armFeetInterAngle);
    //_checkRange(shoulderAngle, armAngle, armFeetInterAngle);
    MOTOR_recv recv;
    recv = postion_control(m_serial,0,m_motorSign[0]*shoulderAngle*ms_motorScalar+m_zeros.shoulderHorizontal);
    m_currentAngle.shoulderHorizontal = m_motorSign[0]*(recv.Pos-m_zeros.shoulderHorizontal)/ms_motorScalar;

    recv = postion_control(m_serial,1,m_motorSign[1]*armAngle*ms_motorScalar+m_zeros.armRotation);
    m_currentAngle.armRotation = m_motorSign[1]*(recv.Pos-m_zeros.armRotation)/ms_motorScalar;

    recv = postion_control(m_serial,2,m_motorSign[2]*armFeetInterAngle*ms_motorScalar+m_zeros.armFeetIntersect);
    m_currentAngle.armFeetIntersect = m_motorSign[2]*(recv.Pos-m_zeros.armFeetIntersect)/ms_motorScalar;

}
void LegMotors::TorqueCtrl(float shoulderTorque,float armTorque,float armFeetInterTorque)
{
        // cout << "torque control:\n" << shoulderTorque << ", " << armTorque << ", " << armFeetInterTorque << endl;

    MOTOR_recv recv;
    recv = torque_control(m_serial,0,shoulderTorque);
    m_currentAngle.shoulderHorizontal = m_motorSign[0]*(recv.Pos-m_zeros.shoulderHorizontal)/ms_motorScalar;

    recv = postion_control(m_serial,1,armTorque);
    m_currentAngle.armRotation = m_motorSign[1]*(recv.Pos-m_zeros.armRotation)/ms_motorScalar;

    recv = postion_control(m_serial,2,armFeetInterTorque);
    m_currentAngle.armFeetIntersect = m_motorSign[2]*(recv.Pos-m_zeros.armFeetIntersect)/ms_motorScalar;

}

void BlendCtrl(AxisMovement angle,AxisTorque torque)
{

}

AxisMovement LegMotors::GetCurrentMotorAngle()
{
    return m_currentAngle;
}
