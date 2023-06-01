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
	:m_planner(9,9,0.3,41,15+9.41f+9.41f),
	m_pos(4,FeetMovement(0,0,0)),
	hopTimeouchStatus(4,true)
{
hopTimeime = -1;
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
	hopTimeime = -1;
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
	hopTimeime = -1;
	m_planner.Reset();
}

void Controller::ClearTime()
{
    hopTimeime = -1;
}

#define CLAMP(num,low,up)	{if(num > up)num = up;\
							else if(num < low)num = low;}

bool Controller::Update(float velX, float velY, float velYaw)
{
	CLAMP(velX,-1,1);
	CLAMP(velY,-1,1);
	CLAMP(velYaw,-1,1);
	//printf("ctrl:[%.3f,%.3f,%.3f]\n",velX,velY,velYaw);

    if(hopTimeime == -1)
        hopTimeime = clock();
	clock_t time = clock();
	float dt = (float)(time - hopTimeime)/CLOCKS_PER_SEC;
	hopTimeime = time;
	m_planner.SetVelocity(velY, velX, velYaw);
	LegController::CtrlParam param;

	bool status = m_planner.Update(dt, m_pos, hopTimeouchStatus);
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
			param.feetTouchDown = hopTimeouchStatus[i];
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

void Controller::Hop()
{
	const float leanTime = 0.5f,hopTime = 1.0f;
	const float exp_y1 = -4,exp_z1 = -15,exp_x1 = 9.41;
	const float ;

	float timer = 0;
	clock_t stime = clock(),etime = clock();
	FeetMovement pos[4];
	LegController::CtrlParam params[4];
	for(int i = 0;i<4;++i)
	{
		pos[i] = m_pControllers[i]->GetCurrentPosition();
		params[i].ctrlMask = LegController::Position;
	}

	//lie down
	while(timer < leanTime)
	{
		float t = timer / leanTime;
		for(int i = 0;i<4;++i)
		{
			if(i == 1 || i == 2)
				params[i].feetPosX = pos[i].x*(1.10f-t)-t*exp_y1;
			else
				params[i].feetPosX = pos[i].x*(1.10f-t)+t*exp_y1;
			params[i].feetPosY = pos[i].y*(1.10f-t)+t*exp_y1;
			params[i].feetPosZ = pos[i].z*(1.10f-t)+t*exp_z1;
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}


		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = stime;
	}
	for(int i = 0;i<4;++i)
	{
		if(i == 1 || i == 2)
			params[i].feetPosX = -t*exp_y1;
		else
			params[i].feetPosX = t*exp_y1;
		params[i].feetPosY = exp_y1;
		params[i].feetPosZ = exp_z1;
		
		pos[i].x = params[i].feetPosX;
		pos[i].y = params[i].feetPosY;
		pos[i].z = params[i].feetPosZ;
		m_pControllers[i]->ApplyCtrlParam(params[i]);

	}


	//hop
	const float x0 = -0.21;  //   //qi dian zuo biao 
	const float 	z0 = -0.31;
	const float 	x1 = -0.25;  //qi shi su du fang xiang 
	const float 	z1 = -0.27;
	const float 	x2 = -0.3;   //jie shu su du fang xiang		
	const float 	z2 = 0.01;
	const float 	x3 = 0.05;  //jie shu zuo biao
	const float 	z3 = -0.2;
	const float	x_start = 0;
	const float	z_start = -0.2;
	
	timer = 0;
	etime = stime = clock();
	while(timer < hopTime)
	{
		float t = timer/hopTime;
		for (int i = 0; legnumber < 4; ++i)
		{
			params[i].feetPosY = x0 * (1 - t / (hopTime * 0.5)) * (1 - t / (hopTime * 0.5)) * (1 - t / (hopTime * 0.5)) +
				3 * x1 * t / (hopTime * 0.5) * (1 - t / (hopTime * 0.5)) * (1 - t / (hopTime * 0.5)) +
				3 * x2 * (t / (hopTime * 0.5)) * (t / (hopTime * 0.5)) * (1 - t / (hopTime * 0.5)) + x3 * (t / (hopTime * 0.5)) * (t / (hopTime * 0.5)) * (t / (hopTime * 0.5));
			params[i].feetPosZ = z0 * (1 - t / (hopTime * 0.5)) * (1 - t / (hopTime * 0.5)) * (1 - t / (hopTime * 0.5)) +
				3 * z1 * t / (hopTime * 0.5) * (1 - t / (hopTime * 0.5)) * (1 - t / (hopTime * 0.5)) +
				3 * z2 * (t / (hopTime * 0.5)) * (t / (hopTime * 0.5)) * (1 - t / (hopTime * 0.5)) + z3 * (t / (hopTime * 0.5)) * (t / (hopTime * 0.5)) * (t / (hopTime * 0.5));
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = stime;
	}


	//touch down
	LegMotors::MotorParams mt_params[4];
	for(int i = 0;i<4;++i)
	{
		mt_params[i] = m_pControllers[i]->GetMotors()->GetMotorParams();
		m_pControllers[i]->GetMotors()->SetMotorParams(LegMotors::SetMotorParams(0.2,6));
	}

	for (int i = 0; i < 4; i++)
	{
		params[i].feetPosY = x3;
		params[i].feetPosZ = z3;
		m_pControllers[i]->ApplyCtrlParam(params[i]);
	}
	
	
	for (int i = 0; i < 4; i++)
	{
		params[i].feetPosY = x_start;
		params[i].feetPosZ = z_start;
		m_pControllers[i]->ApplyCtrlParam(params[i]);
	}

	for(int i = 0;i<4;++i)
		m_pControllers[i]->GetMotors()->SetMotorParams(mt_params[i]);
}


