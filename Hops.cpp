#include "Controller.h"
#include "Kinematics.h"


struct HopParams
{
    float leanTime,leanStopTime,hopBackTime,hopTime,hopStopTime,retriveTime,restTime;
    float start_x,start_y,start_z;
	float back_y,back_z;
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
		case LerpToRestore:_lerpRestore();break;
		case StepToRestore:_stepToChange(9.41,0);break;
		case StepAndSpan:_stepToChange(9.41+10,10);break;
		case StepAndClaw:break;
		case Claw:
		{
            if(!m_startClaw)
            {
                m_startClaw = true;
                m_clawDown = false;
            }
            if(m_clawDown)
                ClawFwdOneStep__tp(-15.0/180.0*3.141592653589);
            else
                ClawFwdOneStep__tp(-15.0/180.0*3.141592653589);

		}break;
		case ClawDown:
		{
            m_clawDown = true;
		}break;

		case ClawLeft:VerticalAdjOneStep__tp(-1);break;
		case ClawRight:VerticalAdjOneStep__tp(1);break;
    }
}

void Controller::_hopForward()
{
    const HopParams hop = {
        .leanTime = 2.0f,.leanStopTime = 1.0f,
        .hopBackTime = 0.09f,.hopTime = 0.11f,.hopStopTime = 0.05f,.retriveTime = 0.3f,.restTime = 0.5f,
        .start_x = 9.41,.start_y = -7,.start_z = -10,//预备点
        .back_y = -12,.back_z = -10,
        .hop_y = -21,.hop_z = -37,//蹬腿点
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
		pos[i] = m_pControllers[i]->GetCurrentPosition();
		params[i].ctrlMask = LegController::feetPos;
		mt_params[i] = m_pControllers[i]->GetMotors()->GetMotorParams();
		m_pControllers[i]->GetMotors()->SetMotorParams(LegMotors::MotorParams(0.8,2.5));
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
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}
        etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}

	printf("[Controller-Hopping] do hop back");
	timer = 0;
	etime = stime = clock();
	while(timer < hop.hopBackTime)
	{
        float t = timer / hop.hopBackTime;
		for(int i = 0;i<4;++i)
		{
			params[i].feetPosY = hop.start_y*(1.0f-t*t)+t*t*hop.back_y;
			params[i].feetPosZ = hop.start_z*(1.0f-t*t)+t*t*hop.back_y;
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
			params[i].feetPosY = hop.back_y*(1.0f-t*t)+t*t*hop.hop_y;
			params[i].feetPosZ = hop.back_y*(1.0f-t*t)+t*t*hop.hop_z;
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
		m_pControllers[i]->GetMotors()->SetMotorParams(LegMotors::MotorParams(0.1,3));
	}
	while(timer < hop.retriveTime)
	{
		float t = timer/hop.retriveTime;
		for (int i = 0; i < 4; ++i)
		{
			params[i].feetPosY = hop.hop_y * (1 - t) * (1 - t) * (1 - t) +
				3 * hop.bz_y1 * t  * (1 - t) * (1 - t) +
				3 * hop.bz_y2 * (t) * (t ) * (1 - t ) + hop.end_y * (t) * (t) * (t);
			params[i].feetPosZ = hop.hop_z * (1 - t) * (1 - t) * (1 - t) +
				3 * hop.bz_z1 * t * (1 - t) * (1 - t) +
				3 * hop.bz_z2 * (t) * (t) * (1 - t) + hop.end_z * (t) * (t) * (t);
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
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}

        //printf("%f\n",timer);
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}


	for(int i = 0;i<4;++i)
	{
		m_pControllers[i]->GetMotors()->SetMotorParams(mt_params[i]);
    }

    m_time = -1;
	m_planner.Reset();
    printf("[Controller-Hopping]end hop\n");
}

void Controller::_lerpRestore()
{
    std::vector<FeetMovement> pos_old(4),pos_new(4);
    std::vector<bool> preserve(4);
	LegController::CtrlParam params[4];
	for(int i = 0;i<4;++i)
	{
		pos_old[i] = m_pControllers[i]->GetCurrentPosition();
		params[i].ctrlMask = LegController::feetPos;
		params[i].feetPosX = pos_old[i].x;
	}
    m_planner.Reset();
    m_planner.SetClimbAngle(0);
	m_planner.SetDogOffset(9.41);
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
			params[i].feetPosX = pos_old[i].x*(1.0f-t)+t*pos_new[i].x;
			params[i].feetPosY = pos_old[i].y*(1.0f-t)+t*pos_new[i].y;
			params[i].feetPosZ = pos_old[i].z*(1.0f-t)+t*pos_new[i].z;
			m_pControllers[i]->ApplyCtrlParam(params[i]);
		}

        //printf("%f\n",timer);
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}
	printf("[Controller-restore]:end lerp\n");
}

float cycloidCurve_(float time,float height)
{
    const float pi_2 = 3.141592653589f*2.0f;
    float phi = time*pi_2;
    return 0.5f*height*(1.0f-cos(phi));
}

void Controller::_stepToChange(float offsetX,float offsetY)
{
    std::vector<FeetMovement> pos_old(4),pos_new(4);
    std::vector<bool> preserve(4);
	LegController::CtrlParam params[4];
	for(int i = 0;i<4;++i)
	{
		pos_old[i] = m_pControllers[i]->GetCurrentPosition();
		params[i].ctrlMask = LegController::feetPos;
		params[i].feetPosX = pos_old[i].x;
	}
    m_planner.Reset();
    m_planner.SetClimbAngle(0);
	m_planner.SetDogOffset(offsetX,offsetY);
    m_planner.Update(0,pos_new,preserve);
    printf("[Controller-restore]:start step\n");
    float stepTime = 1.0f;
    float timer = 0;
    clock_t etime = clock(),stime = clock();
	while(timer < stepTime)
	{
		float t = timer / stepTime;
		if(t < 0.5)
		{
            t *= 2.0f;
            params[0].feetPosX = pos_old[0].x*(1.0f-t)+t*pos_new[0].x;
            params[0].feetPosY = pos_old[0].y*(1.0f-t)+t*pos_new[0].y;
            params[0].feetPosZ = cycloidCurve_(t,10.0f)+pos_new[0].z;
            m_pControllers[0]->ApplyCtrlParam(params[0]);

            params[2].feetPosX = pos_old[2].x*(1.0f-t)+t*pos_new[2].x;
            params[2].feetPosY = pos_old[2].y*(1.0f-t)+t*pos_new[2].y;
            params[2].feetPosZ = cycloidCurve_(t,10.0f)+pos_new[2].z;
            m_pControllers[2]->ApplyCtrlParam(params[2]);
		}else
		{
            t = (t-0.5f)*2.0f;
            params[1].feetPosX = pos_old[1].x*(1.0f-t)+t*pos_new[1].x;
            params[1].feetPosY = pos_old[1].y*(1.0f-t)+t*pos_new[1].y;
            params[1].feetPosZ = cycloidCurve_(t,10.0f)+pos_new[1].z;
            m_pControllers[1]->ApplyCtrlParam(params[1]);

            params[3].feetPosX = pos_old[3].x*(1.0f-t)+t*pos_new[3].x;
            params[3].feetPosY = pos_old[3].y*(1.0f-t)+t*pos_new[3].y;
            params[3].feetPosZ = cycloidCurve_(t,10.0f)+pos_new[3].z;
            m_pControllers[3]->ApplyCtrlParam(params[3]);
		}


        //printf("%f\n",timer);
		etime = clock();
		timer += (float)(etime-stime)/(float)CLOCKS_PER_SEC;
		stime = etime;
	}

    for(int i = 0;i<4;++i)
    {
        params[i].feetPosX = pos_new[i].x;
        params[i].feetPosY = pos_new[i].y;
        params[i].feetPosZ = pos_new[i].z;
        m_pControllers[i]->ApplyCtrlParam(params[i]);
    }

	printf("[Controller-restore]:end step\n");
}


//////////////////////////////////////////
//////////////transplant//////////////////
//////////////////////////////////////////

#define PI		3.14159265359

typedef FeetMovement(*STEP_GENERATE)(FeetMovement,FeetMovement,float);

typedef unsigned char PROCID;



#define NONE			0
#define LINE			2
#define CUBEPACE	3

FeetMovement generateLiner(FeetMovement initPos,FeetMovement targetPos,double duration)
{
	FeetMovement pos;
	pos.x = targetPos.x*duration+(1.0-duration)*initPos.x;
	pos.y = targetPos.y*duration+(1.0-duration)*initPos.y;
	pos.z = targetPos.z*duration+(1.0-duration)*initPos.z;
	return pos;
}

FeetMovement generateTriStep(FeetMovement initPos,FeetMovement targetPos,double duration)
{
	const double topHeight = 15.0;
	FeetMovement pos;
	if(3.0*duration <= 1.0)
	{
		pos.x = initPos.x;
		pos.y = initPos.y;
		pos.z = initPos.z+duration*topHeight*3.0;
	}else if(3.0*duration<=2.0)
	{
		pos.x = initPos.x*(2.0-duration*3.0)+targetPos.x*(duration*3.0-1.0);
		pos.y = initPos.y*(2.0-duration*3.0)+targetPos.y*(duration*3.0-1.0);
		pos.z = initPos.z+topHeight;
	}else
	{
		pos.x = targetPos.x;
		pos.y = targetPos.y;
		pos.z = (initPos.z+topHeight)*(3.0-duration*3.0)+targetPos.z*(duration*3.0-2.0);
	}
	//pos.z *= -1;
	return pos;
}

void Controller::doMovement__tp(struct Step__tp* animation[4],int stepCount)//index:LF  LB  RF  RB
{
	struct Step__tp* indicator[4];
	FeetMovement initPos[4];
	float timer[4] = {0};
	int step = 0;
	for(int i = 0;i<4;i++)
	{
		indicator[i] = &animation[i][step];
		initPos[i] = m_pControllers[i]->GetCurrentPosition();
	}

    clock_t stime = clock(),etime = clock();

    LegController::CtrlParam params[4];
    FeetMovement feetMovement[4];
	while(step < stepCount)
	{
        float dt = etime-stime;
        dt /= (float)CLOCKS_PER_SEC;

		for(int i = 0;i<4;i++)
		{
			if(timer[i] > indicator[i]->duration)
			{
				timer[i]-=indicator[i]->duration;
				if(indicator[i]->procID)
					initPos[i] = indicator[i]->targetPos;
				else
					initPos[i] = feetMovement[i];
				if(i == 0)
				//{
                    //printf("step :%d\n",step);
					step++;
                //}
				if(step < stepCount)
					indicator[i] = &animation[i][step];
				else
					break;
			}
			switch(indicator[i]->procID)
			{
				case NONE:break;
				case LINE:feetMovement[i] = generateLiner(initPos[i],indicator[i]->targetPos,(double)timer[i]/(double)indicator[i]->duration);break;
				case CUBEPACE:feetMovement[i] = generateTriStep(initPos[i],indicator[i]->targetPos,(double)timer[i]/(double)indicator[i]->duration);break;
			}
			timer[i]+=dt;

		}
		for(int i = 0;i<4;++i)
        {
            params[i].ctrlMask = LegController::feetPos;
            params[i].feetPosX = feetMovement[i].x;
            params[i].feetPosY = feetMovement[i].y;
            params[i].feetPosZ = feetMovement[i].z;
            m_pControllers[i]->ApplyCtrlParam(params[i]);
        }
        stime = etime;
        etime = clock();
	}

}


#define CLAW_OFFXR			8.5
#define CLAW_OFFXL			8.5
//#define CLAW_OFFZ			5

#define CLAW_FUL_OFFZ		30
#define CLAW_STEP_DURAT		0.8f
#define CLAW_LEAN_DURAT	    0.8f
#define CLAW_OFFY			-7


#define CLAWSTEP_YLEN		20
#define CLAWSTEP_XLEN		5.0

#define MIN(a,b)		a<b?a:b
#define MAX(a,b)		a>b?a:b


void Controller::ClawFwdOneStep__tp(double angle)
{
    printf("start claw\n");
	struct Step__tp LF_[6];
	struct Step__tp RF_[6];
	struct Step__tp LR_[6];
	struct Step__tp RR_[6];
	struct Step__tp* animation[4] = { RF_,LF_,LR_,RR_};
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			animation[i][j].duration = CLAW_STEP_DURAT;
			animation[i][j].procID = NULL;
			animation[i][j].targetPos.z = -CLAW_FUL_OFFZ;
			//if (i == 0 || i == 2)
			//	animation[i][j].targetPos.z += CLAW_OFFZ;
		}
	}
	FeetMovement old[4];
	for(int i = 0;i<4;++i)old[i] = m_pControllers[i]->GetCurrentPosition();

	LF_[0].duration = CLAW_LEAN_DURAT;
	LF_[0].procID = LINE;
	LF_[0].targetPos.x = CLAW_OFFXR;
	LF_[0].targetPos.y = old[LF].y;

	LR_[0].duration = CLAW_LEAN_DURAT;
	LR_[0].procID = LINE;
	LR_[0].targetPos.x = + CLAW_OFFXR;
	LR_[0].targetPos.y = old[LB].y;

	RF_[0].duration = CLAW_LEAN_DURAT;
	RF_[0].procID = LINE;
	RF_[0].targetPos.x = +CLAW_OFFXR;
	RF_[0].targetPos.y = old[RF].y;

	RR_[0].duration = CLAW_LEAN_DURAT;
	RR_[0].procID = LINE;
	RR_[0].targetPos.x = +CLAW_OFFXR;
	RR_[0].targetPos.y = old[RB].y;



	LF_[1].duration = CLAW_STEP_DURAT;
	LF_[1].procID = LINE;
	LF_[1].targetPos.x = +CLAW_OFFXR;
	LF_[1].targetPos.y = 0;

	LR_[1].duration = CLAW_STEP_DURAT;
	LR_[1].procID = LINE;
	LR_[1].targetPos.x = +CLAW_OFFXR;
	LR_[1].targetPos.y = 0;

	RF_[1].duration = CLAW_STEP_DURAT;
	RF_[1].procID = LINE;
	RF_[1].targetPos.x = CLAW_OFFXR;
	RF_[1].targetPos.y = CLAWSTEP_YLEN / -2.0;


	RR_[1].duration = CLAW_STEP_DURAT;
	RR_[1].procID = CUBEPACE;
	RR_[1].targetPos.x = + CLAW_OFFXR;
	RR_[1].targetPos.y = (CLAWSTEP_YLEN) / 2.0;


	RF_[2].duration = CLAW_STEP_DURAT;
	RF_[2].procID = CUBEPACE;
	RF_[2].targetPos.x =  + CLAW_OFFXR;
	RF_[2].targetPos.y =  CLAWSTEP_YLEN / 2.0;


	LF_[3].duration = CLAW_LEAN_DURAT;
	LF_[3].procID = LINE;
	LF_[3].targetPos.x = -CLAW_OFFXL;
	LF_[3].targetPos.y = 0;

	LR_[3].duration = CLAW_LEAN_DURAT;
	LR_[3].procID = LINE;
	LR_[3].targetPos.x = -CLAW_OFFXL;
	LR_[3].targetPos.y = 0;

	RR_[3].duration = CLAW_LEAN_DURAT;
	RR_[3].procID = LINE;
	RR_[3].targetPos.x = - CLAW_OFFXL;
	RR_[3].targetPos.y = (CLAWSTEP_YLEN) / 2.0;

	RF_[3].duration = CLAW_LEAN_DURAT;
	RF_[3].procID = LINE;
	RF_[3].targetPos.x = - CLAW_OFFXL;
	RF_[3].targetPos.y = (CLAWSTEP_YLEN) / 2.0;



	LF_[4].duration = CLAW_STEP_DURAT;
	LF_[4].procID = LINE;
	LF_[4].targetPos.x = - CLAW_OFFXL;
	LF_[4].targetPos.y = (CLAWSTEP_YLEN) / -2.0;


	LR_[4].duration = CLAW_STEP_DURAT;
	LR_[4].procID = CUBEPACE;
	LR_[4].targetPos.x = - CLAW_OFFXL;
	LR_[4].targetPos.y = (CLAWSTEP_YLEN) / 2.0;

	RR_[4].duration = CLAW_STEP_DURAT;
	RR_[4].procID = LINE;
	RR_[4].targetPos.x = -CLAW_OFFXL;
	RR_[4].targetPos.y = 0;

	RF_[4].duration = CLAW_STEP_DURAT;
	RF_[4].procID = LINE;
	RF_[4].targetPos.x = -CLAW_OFFXL;
	RF_[4].targetPos.y = 0;


	LF_[4].duration = CLAW_STEP_DURAT;
	LF_[4].procID = LINE;
	LF_[4].targetPos.x = - CLAW_OFFXL;
	LF_[4].targetPos.y = (CLAWSTEP_YLEN) / -2.0;

	LF_[5].duration = CLAW_STEP_DURAT;
	LF_[5].procID = CUBEPACE;
	LF_[5].targetPos.x =  - CLAW_OFFXL;
	LF_[5].targetPos.y = (CLAWSTEP_YLEN) / 2.0;


	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 6; j++)
		{
            if(i == RF | i == RB)
                animation[i][j].targetPos.x += 9.41;
            else
                animation[i][j].targetPos.x -= 9.41;

			if(animation[i][j].targetPos.y >= 0 && (i == 0 ||i == 1))
				animation[i][j].targetPos.z += sin(angle) * animation[i][j].targetPos.y;
			else
				animation[i][j].targetPos.z += sin(angle/3.0) * animation[i][j].targetPos.y;
			animation[i][j].targetPos.y += CLAW_FUL_OFFZ * tan(angle);
			if(i == 0 || i == 2)
				animation[i][j].targetPos.y += CLAW_OFFY;
		}
	}
	doMovement__tp(animation, 6);
	printf("end claw\n");
}



#define ADJ_DURAT			1.0f
#define ADJ_OFFX			4.5
#define ADJ_STEP			8.0
void Controller::VerticalAdjOneStep__tp(double vertical)
{
    printf("start claw adjust\n");
	struct Step__tp LF_[6];
	struct Step__tp RF_[6];
	struct Step__tp LR_[6];
	struct Step__tp RR_[6];
	struct Step__tp* animation[4] = { RF_,LF_,LR_,RR_ };
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			animation[i][j].duration = CLAW_STEP_DURAT;
			animation[i][j].procID = NULL;
			animation[i][j].targetPos.x = animation[i][j].targetPos.y = animation[i][j].targetPos.z = 0;

		}
	}

	FeetMovement old[4];
	for(int i = 0;i<4;++i)old[i] = m_pControllers[i]->GetCurrentPosition();

	LF_[0].duration = CLAW_LEAN_DURAT;
	LF_[0].procID = LINE;
	LF_[0].targetPos.x = ADJ_OFFX;

	LR_[0].duration = CLAW_LEAN_DURAT;
	LR_[0].procID = LINE;
	LR_[0].targetPos.x = +ADJ_OFFX;

	RF_[0].duration = CLAW_LEAN_DURAT;
	RF_[0].procID = LINE;
	RF_[0].targetPos.x = +ADJ_OFFX;

	RR_[0].duration = CLAW_LEAN_DURAT;
	RR_[0].procID = LINE;
	RR_[0].targetPos.x = +ADJ_OFFX;

	RR_[1].duration = CLAW_STEP_DURAT;
	RR_[1].procID = CUBEPACE;
	RR_[1].targetPos.x = + ADJ_OFFX + ADJ_STEP / +2.0 * (vertical);
	RR_[1].targetPos.y = old[RB].y;


	RF_[2].duration = CLAW_STEP_DURAT;
	RF_[2].procID = CUBEPACE;
	RF_[2].targetPos.x = + ADJ_OFFX + ADJ_STEP / +2.0 * (vertical);
	RF_[2].targetPos.y = old[RF].y;


	LF_[3].duration = CLAW_LEAN_DURAT;
	LF_[3].procID = LINE;
	LF_[3].targetPos.x = - ADJ_OFFX + ADJ_STEP / -2.0 * (vertical);
	LF_[3].targetPos.y = old[LF].y;


	LR_[3].duration = CLAW_LEAN_DURAT;
	LR_[3].procID = LINE;
	LR_[3].targetPos.x =  - ADJ_OFFX + ADJ_STEP / -2.0 * (vertical);
	LR_[3].targetPos.y = old[LB].y;

	RR_[3].duration = CLAW_LEAN_DURAT;
	RR_[3].procID = LINE;
	RR_[3].targetPos.x = -ADJ_OFFX;
	RR_[3].targetPos.y = 0;


	RF_[3].duration = CLAW_LEAN_DURAT;
	RF_[3].procID = LINE;
	RF_[3].targetPos.x = -ADJ_OFFX;
	RF_[3].targetPos.y = 0;


	LR_[4].duration = CLAW_STEP_DURAT;
	LR_[4].procID = CUBEPACE;
	LR_[4].targetPos.x = -ADJ_OFFX;
	LR_[4].targetPos.y = 0;

	LF_[5].duration = CLAW_STEP_DURAT;
	LF_[5].procID = CUBEPACE;
	LF_[5].targetPos.x = -ADJ_OFFX;
	LF_[5].targetPos.y = 0;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 6; j++)
		{
		            if(i == RF | i == RB)
                animation[i][j].targetPos.x += 9.41;
            else
                animation[i][j].targetPos.x -= 9.41;

			animation[i][j].targetPos.y = old[i].y;
			animation[i][j].targetPos.z = old[i].z;
		}
	}

	doMovement__tp(animation, 6);

    printf("end claw\n");
}















.targetPos.x = -ADJ_OFFX;
	RF_[3].targetPos.y = 0;


	LR_[4].duration = CLAW_STEP_DURAT;
	LR_[4].procID = CUBEPACE;
	LR_[4].targetPos.x = -ADJ_OFFX;
	LR_[4].targetPos.y = 0;

	LF_[5].duration = CLAW_STEP_DURAT;
	LF_[5].procID = CUBEPACE;
	LF_[5].targetPos.x = -ADJ_OFFX;
	LF_[5].targetPos.y = 0;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 6; j++)
		{
		            if(i == RF | i == RB)
                animation[i][j].targetPos.x += 9.41;
            else
                animation[i][j].targetPos.x -= 9.41;

			animation[i][j].targetPos.y = old[i].y;
			animation[i][j].targetPos.z = old[i].z;
		}
	}

	doMovement__tp(animation, 6);

    printf("end claw\n");
}















