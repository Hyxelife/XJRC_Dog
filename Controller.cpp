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
    m_planner.SetDogOffsetX(9.41f);
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
    m_planner.DebugShow();
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
        printf("leg[%d],x:%.3f,y:%.3f,z:%.3f\n",m_pos[i].x,m_pos[i].y,m_pos[i].z);
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

bool Controller::Update(float velX, float velY, float velYaw,bool Hop,HopType type,bool restrictHop)
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
            if(Hop)
                _doHop(type);
            else _doHop(m_hopType);
			m_needHop =false;
			return true;
		}
		if(velX != 0 || velY != 0 || velYaw != 0)StartMoving();
		else return true;
	}else
	{
        if(Hop)
        {
            if(restrictHop)
            {
                m_needHop = true;
                m_needStop = true;
                m_hopType = type;
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

void Controller::GetCurrentVelocity(float &x,float &y,float &r)
{
    x = m_outVel[X];
    y = m_outVel[Y];
    r = m_outVel[R];
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

struct HopParams
{
    float leanTime,leanStopTime,hopTime,hopStopTime,hopbackTime,restTime;
    float start_x,start_y,start_z;
    float hop_y,hop_z;
    float end_y,end_z;
    float bz_y1,bz_z1,bz_y2,bz_z2;
};

void Controller::_doHop(HopType type)
{
    switch(type)
    {
        case HopForward:_hopForward();break;
        //case TestMotor:hop = &hopTest;break;
		case Restore:_restore();break;
		case HopAndSpan:_hopAndSpan();break;
        case HopAndLean:_hopAndLean();return;
    }
}

void Controller::_hopForward()
{
const HopParams hop = {
    .leanTime = 3.0f,.leanStopTime = 1.0f,.hopTime = 0.07f,.hopStopTime = 0.02f,.hopbackTime = 0.5f,.restTime = 0.5f,
    .start_x = 9.41,.start_y = -7,.start_z = -10,//预备点
    .hop_y = -21,.hop_z = -30,//蹬腿点
    .end_y = 5,.end_z = -20,//结束点
    .bz_y1 = -25,.bz_z1 = -27,
    .bz_y2 = -30,.bz_z2 = 1
};
    printf("[Controller-Hopping]start Hop\n");

	float timer = 0;
	clock_t stime = clock(),etime = clock();
	FeetMovement pos[4];
	LegController::CtrlParam params[4];
	LegMotors::MotorParams mt_params[4];
	for(int i = 0;i<4;++i)
	{
        if(enableMap[i])
		pos[i] = m_pControllers[i]->GetCurrentPosition();
		params[i].ctrlMask = LegController::feetPos;

		if(enableMap[i])
		mt_params[i] = m_pControllers[i]->GetMotors()->GetMotorParams();
		if(enableMap[i])
		m_pControllers[i]->GetMotors()->SetMotorParams(LegMotors::MotorParams(0.7,4));
	}
	printf("[Controller-Hopping]do lie down\n");
	//lie down
	while(timer < hop.leanTime)
	{
		float t = timer / hop.leanTime;
		for(int i = 0;i<4;++i)
		{
			if(i == 1 || i == 2)
				params[i].feetPosX = pos[i].x*(1.0f-t)-t*hop.start_x;
			else
				params[i].feetPosX = pos[i].x*(1.0f-t)+t*hop.start_x;
			params[i].feetPosY = pos[i].y*(1.0f-t)+t*hop.start_y;
			params[i].feetPosZ = pos[i].z*(1.0f-t)+t*hop.start_z;
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}

	printf("[Controller-Hopping]lean stop\n");
    timer = 0;
	etime = stime = clock();
	while(timer < hop.leanStopTime)
	{
        float t = timer / hop.leanStopTime;
		for(int i = 0;i<4;++i)
		{
			params[i].feetPosY = hop.start_y;
			params[i].feetPosZ = hop.start_z;
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
        etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}

	//hop
	printf("[Controller-Hopping]do hop\n");
    timer = 0;
	etime = stime = clock();
	while(timer < hop.hopTime)
	{
        float t = timer / hop.hopTime;
		for(int i = 0;i<4;++i)
		{
			params[i].feetPosY = hop.start_y*(1.0f-t*t)+t*t*hop.hop_y;
			params[i].feetPosZ = hop.start_z*(1.0f-t*t)+t*t*hop.hop_z;
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
        etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}

	timer = 0;
	etime = stime = clock();
	printf("[Controller-Hopping]do hold on\n");
	while(timer < hop.hopStopTime)
	{
		float t = timer/hop.hopStopTime;
		for (int i = 0; i < 4; ++i)
		{
			params[i].feetPosY = hop.hop_y;
			params[i].feetPosZ = hop.hop_z;
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}


	timer = 0;
	etime = stime = clock();
	printf("[Controller-Hopping]do retrive\n");


	for(int i = 0;i<4;++i)
	{
		if(enableMap[i])
		m_pControllers[i]->GetMotors()->SetMotorParams(LegMotors::MotorParams(0.1,2));
	}
	while(timer < hop.hopbackTime)
	{
		float t = timer/hop.hopbackTime;
		for (int i = 0; i < 4; ++i)
		{
			params[i].feetPosY = hop.hop_y * (1 - t) * (1 - t) * (1 - t) +
				3 * hop.bz_y1 * t  * (1 - t) * (1 - t) +
				3 * hop.bz_y2 * (t) * (t ) * (1 - t ) + hop.end_y * (t) * (t) * (t);
			params[i].feetPosZ = hop.hop_z * (1 - t) * (1 - t) * (1 - t) +
				3 * hop.bz_z1 * t * (1 - t) * (1 - t) +
				3 * hop.bz_z2 * (t) * (t) * (1 - t) + hop.end_z * (t) * (t) * (t);
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}
	//touch down
	printf("[Controller-Hopping]do touch down\n");


	for (int i = 0; i < 4; i++)
	{
		params[i].feetPosY = hop.end_y;
		params[i].feetPosZ = hop.end_z;
		if(enableMap[i])
		m_pControllers[i]->ApplyCtrlParam(params[i]);
	}

    timer = 0;etime = stime = clock();
	while(timer < hop.restTime)
	{
		float t = timer / hop.restTime;
		for(int i = 0;i<4;++i)
		{
			params[i].feetPosY = hop.end_y*(1.0f-t)+t*pos[i].y;
			params[i].feetPosZ = hop.end_z*(1.0f-t)+t*pos[i].z;
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

void Controller::_hopAndLean()
{
    std::vector<FeetMovement> posVec(4);
    std::vector<bool> preserve(4);
    m_planner.SetClimbAngle(15.0/180.0*3.141592653589);
    m_planner.Reset();
    m_planner.Update(0,posVec,preserve);
    const HopParams hop = {
        .leanTime = 1.0f,.hopTime = 0.01f,.hopStopTime = 0.1f,.hopbackTime = 0.2f,.restTime = 0.5f,
        .start_x = 9.41,.start_y = -5,.start_z = -15,
        .hop_y = -25,.hop_z = -35,
        .end_y = posVec[0].y,.end_z = posVec[0].z+5,
        .bz_y1 = -25.3,.bz_z1 = -25.1,
        .bz_y2 = -23.3,.bz_z2 = -10
    };
    printf("[Controller-HopWithAngle]:start\n");


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


	printf("[Controller-HopWithAngle]do lie down\n");
	//lie down
	while(timer < hop.leanTime)
	{
		float t = timer / hop.leanTime;
		for(int i = 0;i<4;++i)
		{
			if(i == 1 || i == 2)
				params[i].feetPosX = pos[i].x*(1.0f-t)-t*hop.start_x;
			else
				params[i].feetPosX = pos[i].x*(1.0f-t)+t*hop.start_x;
			params[i].feetPosY = pos[i].y*(1.0f-t)+t*hop.start_y;
			params[i].feetPosZ = pos[i].z*(1.0f-t)+t*hop.start_z;
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}


	//hop
	printf("[Controller-HopWithAngle]do hop\n");
    timer = 0;
	etime = stime = clock();
	while(timer < hop.hopTime)
	{
        float t = timer / hop.hopTime;
		for(int i = 0;i<4;++i)
		{
			params[i].feetPosY = hop.start_y*(1.0f-t)+t*hop.hop_y;
			params[i].feetPosZ = hop.start_z*(1.0f-t)+t*hop.hop_z;
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
        etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}
	timer = 0;
	etime = stime = clock();

	printf("[Controller-HopWithAngle]do retrive\n");
	while(timer < hop.hopbackTime)
	{
		float t = timer/hop.hopbackTime;
		for (int i = 0; i < 4; ++i)
		{
			params[i].feetPosY = hop.hop_y * (1 - t) * (1 - t) * (1 - t) +
				3 * hop.bz_y1 * t  * (1 - t) * (1 - t) +
				3 * hop.bz_y2 * (t) * (t ) * (1 - t ) + hop.end_y * (t) * (t) * (t);
			params[i].feetPosZ = hop.hop_z * (1 - t) * (1 - t) * (1 - t) +
				3 * hop.bz_z1 * t * (1 - t) * (1 - t) +
				3 * hop.bz_z2 * (t) * (t) * (1 - t) + hop.end_z * (t) * (t) * (t);
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}

	//touch down
	printf("[Controller-HopWithAngle]do touch down\n");
	LegMotors::MotorParams mt_params[4];
	for(int i = 0;i<4;++i)
	{
        if(enableMap[i])
		mt_params[i] = m_pControllers[i]->GetMotors()->GetMotorParams();
		if(enableMap[i])
		m_pControllers[i]->GetMotors()->SetMotorParams(LegMotors::MotorParams(0.1,2));
	}

	for (int i = 0; i < 4; i++)
	{
		params[i].feetPosY = hop.end_y;
		params[i].feetPosZ = hop.end_z;
		if(enableMap[i])
		m_pControllers[i]->ApplyCtrlParam(params[i]);
	}

    printf("[Controller-HopWithAngle]do resting\n");
    timer = 0;etime = stime = clock();
	while(timer < hop.restTime)
	{
		float t = timer / hop.restTime;
		for(int i = 0;i<4;++i)
		{
			params[i].feetPosY = hop.end_y*(1.0f-t)+t*posVec[i].y;
			params[i].feetPosZ = hop.end_z*(1.0f-t)+t*posVec[i].z;
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
    printf("[Controller-HopWithAngle]end hop\n");
}

void Controller::_restore()
{
    std::vector<FeetMovement> pos_old(4),pos_new(4);
    std::vector<bool> preserve(4);
	LegController::CtrlParam params[4];
	for(int i = 0;i<4;++i)
	{
        if(enableMap[i])
		pos_old[i] = m_pControllers[i]->GetCurrentPosition();
		params[i].ctrlMask = LegController::feetPos;
		params[i].feetPosX = pos_old[i].x;
	}
    m_planner.Reset();
    m_planner.SetClimbAngle(0);
	m_planner.SetDogOffsetX(9.41);
    m_planner.Update(0,pos_new,preserve);
    printf("[Controller-restore]:start lerp\n");
    float lerpTime = 1.0f;
    float timer = 0;
    clock_t etime = clock(),stime = clock();
	while(timer < lerpTime)
	{
		float t = timer / lerpTime;
		for(int i = 0;i<4;++i)
		{
			params[i].feetPosY = pos_old[i].y*(1.0f-t)+t*pos_new[i].y;
			params[i].feetPosZ = pos_old[i].z*(1.0f-t)+t*pos_new[i].z;
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}

        //printf("%f\n",timer);
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}
	printf("[Controller-restore]:end lerp\n");
}

void Controller::_hopAndSpan()
{
    std::vector<FeetMovement> posVec(4);
    std::vector<bool> preserve(4);
    //m_planner.SetDogOffsetX(15.0*180.0/3.141592653589);
    m_planner.Reset();
    m_planner.Update(0,posVec,preserve);
    const HopParams hop = {
        .leanTime = 1.0f,.hopTime = 5.0f,.hopbackTime = 5.0f,.restTime = 1.0f,
        .start_x = 9.41,.start_y = -5,.start_z = -15,
        .hop_y = -10,.hop_z = -31,
        .end_y = posVec[0].y,.end_z = posVec[0].z+5,
        .bz_y1 = -25,.bz_z1 = -27,
        .bz_y2 = -30,.bz_z2 = 1
    };
    printf("[Controller-HopWithAngle]:start\n");


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


	printf("[Controller-HopWithAngle]do lie down\n");
	//lie down
	while(timer < hop.leanTime)
	{
		float t = timer / hop.leanTime;
		for(int i = 0;i<4;++i)
		{
			if(i == 1 || i == 2)
				params[i].feetPosX = pos[i].x*(1.0f-t)-t*hop.start_x;
			else
				params[i].feetPosX = pos[i].x*(1.0f-t)+t*hop.start_x;
			params[i].feetPosY = pos[i].y*(1.0f-t)+t*hop.start_y;
			params[i].feetPosZ = pos[i].z*(1.0f-t)+t*hop.start_z;
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}


	//hop
	printf("[Controller-HopWithAngle]do hop\n");
    timer = 0;
	etime = stime = clock();
	while(timer < hop.hopTime)
	{
        float t = timer / hop.hopTime;
		for(int i = 0;i<4;++i)
		{
			params[i].feetPosY = hop.start_y*(1.0f-t)+t*hop.hop_y;
			params[i].feetPosZ = hop.start_z*(1.0f-t)+t*hop.hop_z;
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
        etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}
	timer = 0;
	etime = stime = clock();

	printf("[Controller-HopWithAngle]do retrive\n");
	while(timer < hop.hopbackTime)
	{
		float t = timer/hop.hopbackTime;
		for (int i = 0; i < 4; ++i)
		{
			params[i].feetPosY = hop.hop_y * (1 - t) * (1 - t) * (1 - t) +
				3 * hop.bz_y1 * t  * (1 - t) * (1 - t) +
				3 * hop.bz_y2 * (t) * (t ) * (1 - t ) + hop.end_y * (t) * (t) * (t);
			params[i].feetPosZ = hop.hop_z * (1 - t) * (1 - t) * (1 - t) +
				3 * hop.bz_z1 * t * (1 - t) * (1 - t) +
				3 * hop.bz_z2 * (t) * (t) * (1 - t) + hop.end_z * (t) * (t) * (t);
			if(enableMap[i])
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}

	//touch down
	printf("[Controller-HopWithAngle]do touch down\n");
	LegMotors::MotorParams mt_params[4];
	for(int i = 0;i<4;++i)
	{
        if(enableMap[i])
		mt_params[i] = m_pControllers[i]->GetMotors()->GetMotorParams();
		if(enableMap[i])
		m_pControllers[i]->GetMotors()->SetMotorParams(LegMotors::MotorParams(0.1,2));
	}

	for (int i = 0; i < 4; i++)
	{
		params[i].feetPosY = hop.end_y;
		params[i].feetPosZ = hop.end_z;
		if(enableMap[i])
		m_pControllers[i]->ApplyCtrlParam(params[i]);
	}

    printf("[Controller-HopWithAngle]do resting\n");
    timer = 0;etime = stime = clock();
	while(timer < hop.restTime)
	{
		float t = timer / hop.restTime;
		for(int i = 0;i<4;++i)
		{
			params[i].feetPosY = hop.end_y*(1.0f-t)+t*posVec[i].y;
			params[i].feetPosZ = hop.end_z*(1.0f-t)+t*posVec[i].z;
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
    printf("[Controller-HopWithAngle]end hop\n");
}
