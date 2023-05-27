#include "Controller.h"
#include<iostream>
using  namespace std;
#define DEBUG_MODE

#define STOP_LEG    0
#ifdef DEBUG_MODE
bool enableMap[4] = {true,true,true,true};
#endif // DEBUG_MODE



Controller::Controller(
    std::vector<std::string> serialName,
	std::vector<AxisMovement> motorAngle,
	std::vector<AxisMovement> realAngle,
	std::vector<std::vector<float>> motorSign,
	LegController::VMCParam param)
	:m_planner(9,9,0.1,41,15),
	m_pos(4,FeetMovement(0,0,0)),
	m_touchStatus(4,true)
{
m_time = -1;
	AxisMovement angle;
	float scalar = LegMotors::GetMotorScalar();
	for (int i = 0; i < 4; ++i)
	{
		angle.shoulderHorizontal = motorAngle[i].shoulderHorizontal - motorSign[i][0]*realAngle[i].shoulderHorizontal * scalar;
		angle.armRotation = motorAngle[i].armRotation - motorSign[i][1]*realAngle[i].armRotation * scalar;
		angle.armFeetIntersect = motorAngle[i].armFeetIntersect - motorSign[i][2]*realAngle[i].armFeetIntersect * scalar;
        cout<<"init leg controller["<<i<<"] zeros:"<<angle.shoulderHorizontal<<","<<angle.armRotation<<","<<angle.armFeetIntersect<<endl;
        #ifdef DEBUG_MODE
        if(enableMap[i])
        #endif // DEBUG_MODE
            m_pControllers[i] = new LegController(param, i, serialName[i],angle,motorSign[i]);
	}
}

Controller::~Controller()
{
	for (int i = 0; i < 4; ++i)
	{
	#ifdef DEBUG_MODE
	if(enableMap[i])
	#endif // DEBUG_MODE
		delete m_pControllers[i];
    }
}

void Controller::Start(float startUpTime)
{
	Update(0, 0, 0);
	for (int i = 0; i < 4; ++i)
	{
	#ifdef DEBUG_MODE
	if(enableMap[i])
	#endif // DEBUG_MODE
        {m_pControllers[i]->Start(startUpTime);}
	}

	for (int i = 0; i < 4; ++i)
	{
	#ifdef DEBUG_MODE
        if(enableMap[i])
	#endif // DEBUG_MODE
        {
            while (!m_pControllers[i]->Ready());
		}
	}
	m_time = -1;
}

void Controller::Exit()
{
	for (int i = 0; i < 4; ++i)
	{
	#ifdef DEBUG_MODE
	if(enableMap[i])
	#endif // DEBUG_MODE
		{m_pControllers[i]->Exit();}
	}
}

void Controller::ClearControlHistory()
{
	m_time = -1;
	m_planner.Reset();
}

void Controller::ClearTime()
{
    m_time = -1;
}
#define CLAMP(num,low,up)	{if(num > up)num = up;\
							else if(num < low)num = low;}

bool Controller::Update(float velX, float velY, float velYaw)
{
	CLAMP(velX,-1,1);
	CLAMP(velY,-1,1);
	CLAMP(velYaw,-1,1);
	//printf("ctrl:[%.3f,%.3f,%.3f]\n",velX,velY,velYaw);

    if(m_time == -1)
        m_time = clock();
	clock_t time = clock();
	float dt = (float)(time - m_time)/CLOCKS_PER_SEC;
	m_time = time;
	m_planner.SetVelocity(velY, velX, velYaw);
	LegController::CtrlParam param;

	bool status = m_planner.Update(dt, m_pos, m_touchStatus);
	if (m_bVMCCtrl)
	{
		for (int i = 0; i < 4; ++i)
		{
			param.ctrlMask = LegController::feetPos|LegController::orientation|LegController::topSpeed;
			param.feetPosX = m_pos[i].x;
			param.feetPosY = m_pos[i].y;
			param.feetPosZ = m_pos[i].z;
			param.feetSpeedX = 0;
			param.feetSpeedY = 0;
			param.feetSpeedZ = 0;
			param.feetTouchDown = m_touchStatus[i];
			//param.roll
			param.yaw = 0;
			param.yawVel = velYaw;
			param.topSpeedX = velX;
			param.topSpeedY = velY;
			#ifdef DEBUG_MODE
			if(enableMap[i])
			#endif // DEBUG_MODE
			{m_pControllers[i]->ApplyCtrlParam(param);}

		}
	}
	else
	{

		for (int i = 0; i < 4; ++i)
		{
			param.ctrlMask = LegController::feetPos;
			param.feetPosX = m_pos[i].x;
			param.feetPosY = m_pos[i].y;
			param.feetPosZ = m_pos[i].z;
			#ifdef DEBUG_MODE
			if(enableMap[i])
			#endif // DEBUG_MODE
			{m_pControllers[i]->ApplyCtrlParam(param);}
			//cout<<"applay motor:"<<i<<endl;
			//if(i == LB)
			//cout<<"applay motor:"<<i<<"pos:"<<m_pos[i].x<<","<<m_pos[i].y<<","<<m_pos[i].z<<endl;
		}
	}
	return status;
}

void Controller::RawControl(std::vector<FeetMovement> moves)
{
	LegController::CtrlParam param;
	for (int i = 0; i < 4; ++i)
	{
		param.ctrlMask = LegController::feetPos;
		param.feetPosX = moves[i].x;
		param.feetPosY = moves[i].y;
		param.feetPosZ = moves[i].z;
		m_pControllers[i]->ApplyCtrlParam(param);
		//cout<<"applay motor:"<<i<<endl;
		if (i == STOP_LEG)break;
	}
}


void Controller::EnableVMC(bool enable)
{
	m_bVMCCtrl = enable;
	m_planner.EnableVMC(enable);
	for (int i = 0; i < 4; ++i){
			#ifdef DEBUG_MODE
			if(enableMap[i])
			#endif // DEBUG_MODE
        {m_pControllers[i]->SetCtrlMode(LegController::Position);}
	}

}

PacePlanner& Controller::GetPacePlanner()
{
	return m_planner;
}

