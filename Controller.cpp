#include "Controller.h"
#include<iostream>
#include "Debug.h"
using  namespace std;

#define DEBUG_MODE
#ifdef DEBUG_MODE
const bool enableMap[4] = {true,true,true,true};
#endif // DEBUG_MODE

#define X	0
#define Y	1
#define R	2

#define CLAMP(num,low,up)	{if(num > up)num = up;\
							else if(num < low)num = low;}

Controller::Controller(
    std::vector<std::string> serialName,
	std::vector<AxisMovement> motorAngle,
	std::vector<AxisMovement> realAngle,
	std::vector<std::vector<float>> motorSign,
	LegController::VMCParam param,
	CtrlInitParam initParam,MechParam mcParam)
	:m_planner(initParam.maxVelFw,initParam.maxVelVt,initParam.maxVelRt,mcParam.dogWidth,mcParam.dogLength),
	m_pos(4,FeetMovement(0,0,0)),
	m_touchStatus(4,true),
	m_kp(initParam.kp),m_kw(initParam.kw),
	m_movingThres(initParam.movingThreshold)
	//m_usingGlobalRecord(initParam.usingGlobal)
{
	m_maxVel[X] = initParam.maxVelFw;
	m_maxVel[Y] = initParam.maxVelVt;
	m_maxVel[R] = initParam.maxVelRt;

	for(int i = 0;i<3;++i)
		m_incVel[i] = m_hisVel[i] = m_outVel[i] = 0;
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
	m_smthCtrl = true;
	m_needHop = m_needStop = false;
	m_stop = true;
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
    m_planner.SetVelocity(0,0,0);
	LegController::CtrlParam param;
	m_planner.Update(0, m_pos, m_touchStatus);

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
    }




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
}

void Controller::EnableSmoothCtrl(bool enable)
{
	m_smthCtrl = enable;
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

void Controller::_updateVel(float x,float y,float r,float dt)
{
	float ctrlVel[3] = {x,y,r};
    float err_x = ctrlVel[X] - m_outVel[X],err_y = ctrlVel[Y]-m_outVel[Y],err_r = ctrlVel[R]-m_outVel[R];

	if(m_smthCtrl)
	{
		m_incVel[X] += m_kp*err_x*dt+m_kw*(err_x-m_hisVel[X]);
		m_incVel[Y] += m_kp*err_y*dt+m_kw*(err_y-m_hisVel[Y]);
		m_incVel[R] += m_kp*err_r*dt+m_kw*(err_r-m_hisVel[R]);

		m_outVel[X] += m_incVel[X]*dt;
		m_outVel[Y] += m_incVel[Y]*dt;
		m_outVel[R] += m_incVel[R]*dt;

		m_hisVel[X] = err_x;
		m_hisVel[Y] = err_y;
		m_hisVel[R] = err_r;
	}else
	{
		m_outVel[X] = ctrlVel[X];
		m_outVel[Y] = ctrlVel[Y];
		m_outVel[R] = ctrlVel[R];
	}
	if(fabsf(m_outVel[X])<m_movingThres && x == 0)m_outVel[X] = 0;
	if(fabsf(m_outVel[Y])<m_movingThres && y == 0)m_outVel[Y] = 0;
	if(fabsf(m_outVel[R])<m_movingThres && r == 0)m_outVel[R] = 0;


	m_moving = fabsf(m_outVel[X])>m_movingThres ||
				fabsf(m_outVel[Y])>m_movingThres ||
				fabsf(m_outVel[R])>m_movingThres;
    CLAMP(m_outVel[X],-1,1);
    CLAMP(m_outVel[Y],-1,1);
    CLAMP(m_outVel[R],-1,1);
}

bool Controller::Update(float velX, float velY, float velYaw,bool Hop,bool restrictHop)
{
	CLAMP(velX,-1,1);
	CLAMP(velY,-1,1);
	CLAMP(velYaw,-1,1);

	if(m_needStop)
		velX = velY = velYaw = 0;


	if(m_stop)
	{
		if(m_needHop || Hop)
		{
			_doHop();
			m_needHop =false;
			return true;
		}
		if(velX != 0 || velY != 0 || velYaw != 0)StartMoving();
		else return false;
	}else
	{
        if(Hop)
        {
            if(restrictHop)
            {
                m_needHop = true;
                m_needStop = true;
            }
        }
	}


    if(m_time == -1)
        m_time = clock();
	clock_t time = clock();
	float dt = (float)(time - m_time)/CLOCKS_PER_SEC;
	m_time = time;
	_updateVel(velX,velY,velYaw,dt);
    //printf("x:%f,y:%f,r:%f,dt:%f\n",m_outVel[Y], m_outVel[X], m_outVel[R],dt);
	m_planner.SetVelocity(m_outVel[Y], m_outVel[X], m_outVel[R]);
	LegController::CtrlParam param;
	bool status = m_planner.Update(dt, m_pos, m_touchStatus);

	if(status)
	{
		if(m_needStop)
		{
			if(!m_moving)
			{
				m_stop = true;
				m_needStop = false;
				m_planner.Reset();
				m_planner.Update(0,m_pos, m_touchStatus);
			}
		}
	}
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

void Controller::StartMoving()
{
    //printf("sm\n");
	m_time = -1;
	m_planner.Reset();
	m_stop = false;
}

void Controller::StopMoving()
{
	if(m_stop)return;
	m_needStop = true;
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

void Controller::_doHop()
{
    printf("[Controller-Hopping]start Hop\n");

	const float leanTime = 1.0f,hopTime = 0.5f,hopbackTime = 0.5f,restTime = 1.0f;
	const float exp_y1 = -4,exp_z1 = -15,exp_x1 = 9.41,exp_y2 = -10,exp_z2 = -31;

	float timer = 0;
	clock_t stime = clock(),etime = clock();
	FeetMovement pos[4];
	LegController::CtrlParam params[4];
	for(int i = 0;i<4;++i)
	{
        if(enableMap[i])
		pos[i] = m_pControllers[i]->GetCurrentPosition();
		params[i].ctrlMask = LegController::feetPos;
	}
    //printf("current:%.3f,%.3f,%.3f\n",pos[0].x,pos[0].y,pos[0].z);
	printf("[Controller-Hopping]do lie down\n");
	//lie down
	while(timer < leanTime)
	{
		float t = timer / leanTime;
		for(int i = 0;i<4;++i)
		{
			if(i == 1 || i == 2)
				params[i].feetPosX = pos[i].x*(1.0f-t)-t*exp_x1;
			else
				params[i].feetPosX = pos[i].x*(1.0f-t)+t*exp_x1;
			params[i].feetPosY = pos[i].y*(1.0f-t)+t*exp_y1;
			params[i].feetPosZ = pos[i].z*(1.0f-t)+t*exp_z1;
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}

        //printf("%f\n",timer);
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}
	//char ch = getchar();
	//hop
	printf("[Controller-Hopping]do hop\n");
    timer = 0;
	etime = stime = clock();
	while(timer < hopTime)
	{
        float t = timer / hopTime;
		for(int i = 0;i<4;++i)
		{
			params[i].feetPosY = exp_y1*(1.0f-t)+t*exp_y2;
			params[i].feetPosZ = exp_z1*(1.0f-t)+t*exp_z2;
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
        etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}
    //ch = getchar();
	//hop back
	const float x0 = -10;  //   //qi dian zuo biao
	const float 	z0 = -31;
	const float 	x1 = -25;  //qi shi su du fang xiang
	const float 	z1 = -27;
	const float 	x2 = -30;   //jie shu su du fang xiang
	const float 	z2 = 1;
	const float 	x3 = 5;  //jie shu zuo biao
	const float 	z3 = -20;
	const float	x_start = 0;
	const float	z_start = -20;

	timer = 0;
	etime = stime = clock();
	printf("[Controller-Hopping]do retrive\n");
	while(timer < hopbackTime)
	{
		float t = timer/hopbackTime;
		for (int i = 0; i < 4; ++i)
		{
			params[i].feetPosY = x0 * (1 - t) * (1 - t) * (1 - t) +
				3 * x1 * t  * (1 - t) * (1 - t) +
				3 * x2 * (t) * (t ) * (1 - t ) + x3 * (t) * (t) * (t);
			params[i].feetPosZ = z0 * (1 - t) * (1 - t) * (1 - t) +
				3 * z1 * t * (1 - t) * (1 - t) +
				3 * z2 * (t) * (t) * (1 - t) + z3 * (t) * (t) * (t);
				//if(i == 0)
				//printf("x=%f,y=%f,z=%f\n",params[i].feetPosX,params[i].feetPosY,params[i].feetPosZ);
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}


	//touch down
	//ch = getchar();
	printf("[Controller-Hopping]do touch down\n");
	LegMotors::MotorParams mt_params[4];
	for(int i = 0;i<4;++i)
	{
        if(enableMap[i])
		mt_params[i] = m_pControllers[i]->GetMotors()->GetMotorParams();
		if(enableMap[i])
		m_pControllers[i]->GetMotors()->SetMotorParams(LegMotors::MotorParams(0.2,6));
	}

	for (int i = 0; i < 4; i++)
	{
		params[i].feetPosY = x3;
		params[i].feetPosZ = z3;
		if(enableMap[i])
		m_pControllers[i]->ApplyCtrlParam(params[i]);
	}

    timer = 0;etime = stime = clock();
	while(timer < restTime)
	{
		float t = timer / restTime;
		for(int i = 0;i<4;++i)
		{
			params[i].feetPosY = x3*(1.0f-t)+t*pos[i].y;
			params[i].feetPosZ = z3*(1.0f-t)+t*pos[i].z;
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}

        //printf("%f\n",timer);
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}


	for(int i = 0;i<4;++i)
	{
        if(enableMap[i])
		m_pControllers[i]->GetMotors()->SetMotorParams(mt_params[i]);
    }

    m_time = -1;
	m_planner.Reset();
    printf("[Controller-Hopping]end hop\n");
}


