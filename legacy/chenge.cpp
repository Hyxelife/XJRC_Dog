#include <math.h>
#include <stdio.h>
//#include "timer.h"
//axis: x-right y-front z-up 

#define PI		3.14159265359
#define NULL	0

char moveExit = 0;

typedef void(*MOTION_APPLY)(int id, struct Vector3 rotate);
typedef struct Vector3(*STEP_GENERATE)(struct Vector3,struct Vector3,double);

typedef unsigned char PROCID;

struct Step
{
	unsigned char duration;
    //STEP_GENERATE generateProc;
	PROCID procID;
    FeetMovement targetPos;
};

#define NONE			0
#define PACE			1
#define LINE			2	
#define CUBEPACE	3
#define LERP			4
#define QUALERP		5
#define PACEGAP		6


struct Vector3 idlePos[4];//LF  LB  RF  RB
double constLeanPos = 0;//only rotate in y axis


char key = 0;

struct Vector3 feetMovement[4];//calcualted Pos

struct Vector3 generateLiner(struct Vector3 initPos,struct Vector3 targetPos,double duration)
{
	struct Vector3 pos;
	pos.x = targetPos.x*duration+(1.0-duration)*initPos.x;
	pos.y = targetPos.y*duration+(1.0-duration)*initPos.y;
	pos.z = targetPos.z*duration+(1.0-duration)*initPos.z;	
	return pos;
}

#define STEP_OBS_HEIGHT		    50.0
#define STEP_HEIGHT				35.0
#define STEP_OBS_GAP			30.0
#define STEP_OBS_OFFY			30.0


struct Vector3 generatePaceForward(struct Vector3 initPos,struct Vector3 targetPos,double duration)
{
	struct Vector3 pos;
	double sigma = 2.0*PI*duration;
//if(duration >= 0.5)
//delay_ms(5000);
	pos.x = (targetPos.x - initPos.x)*((sigma - sin(sigma)) / (2.0*PI)) + initPos.x;
	pos.y = (targetPos.y - initPos.y)*((sigma - sin(sigma)) / (2.0*PI)) + initPos.y;
	pos.z = STEP_HEIGHT * ((1.0 - cos(sigma)) / 2.0)+initPos.z;
	return pos;
	
}

struct Vector3 generatePaceForwardWithGap(struct Vector3 initPos,struct Vector3 targetPos,double duration)
{
	struct Vector3 pos;
	double sigma = 0;
	if(duration <= STEP_OBS_GAP/STEP_OBS_HEIGHT/2.0)
	{
		sigma = duration / STEP_OBS_GAP/STEP_OBS_HEIGHT/2.0;
		pos.x = initPos.x;
		pos.y = initPos.y-sigma*STEP_OBS_OFFY;
		pos.z = initPos.z+sigma * STEP_OBS_GAP;
	}else
	{
		duration = (duration-STEP_OBS_GAP/STEP_OBS_HEIGHT/2.0)/(1.0-STEP_OBS_GAP/STEP_OBS_HEIGHT/2.0);
		sigma = 2.0*PI*duration;
	//if(duration >= 0.5)
	//delay_ms(5000);
		pos.x = (targetPos.x - initPos.x)*((sigma - sin(sigma)) / (2.0*PI)) + initPos.x;
		pos.y = (targetPos.y - initPos.y)*((sigma - sin(sigma)) / (2.0*PI)) + initPos.y-STEP_OBS_OFFY;
		pos.z = (STEP_OBS_HEIGHT-STEP_OBS_GAP) * ((1.0 - cos(sigma)) / 2.0)+targetPos.z+STEP_OBS_GAP*(1.0-duration);
	}
	return pos;
}


struct Vector3 generateTriStep(struct Vector3 initPos,struct Vector3 targetPos,double duration)
{

	static double topHeight = 80.0;
	struct Vector3 pos;
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
	return pos;
	
	
	/*
	static double topHeight = 85.0;
	struct Vector3 pos;
	if(duration <= 0.5)
	{
		pos.x = initPos.x;
		pos.y = initPos.y;
		pos.z = initPos.z+duration*topHeight*2.0;
	}else
	{
		pos.x = initPos.x*(2.0-duration*2.0)+targetPos.x*(duration*2.0-1.0);
		pos.y = initPos.y*(2.0-duration*2.0)+targetPos.y*(duration*2.0-1.0);
		pos.z = (initPos.z+topHeight)*(2.0-duration*2.0)+targetPos.z*(duration*2.0-1.0);		
	}
	return pos;
	*/
}

struct Vector3 generateQuadratic(struct Vector3 initPos,struct Vector3 targetPos,double duration)
{
	static double topHeight = 50.0;
	struct Vector3 pos;


	double dy = (targetPos.z-initPos.z)/2.0;
	double aVal = (dy-topHeight);

	pos.x = duration*targetPos.x+(1.0-duration)*initPos.x;
	pos.y = duration*targetPos.y+(1.0-duration)*initPos.y;

	pos.z = aVal*(duration-0.5)*(duration-0.5)*4.0+topHeight+dy+initPos.z;
	return pos;
}

double splineMatrix[5][5] = {
	{2.2500  , -0.7500 ,   0.2500 ,  -0.5000 ,  -0.1250},
	{-1.2500  ,  0.7500  , -0.2500  ,  0.5000 ,   0.1250},
	{0.7500  ,  0.7500  , -0.2500 ,  -0.5000 ,   0.1250},
	{-1.5000 ,   1.5000 ,  -0.5000  ,  1.0000 ,  -0.2500},
	{0.7500 ,  -1.2500 ,   0.7500 ,  -0.5000  ,  0.1250},
};

struct Vector2 splineCoef[8];//a1,b1,a2,c1,d1,b2,c2,d2
struct Vector2 splineTerms[5];//A,B,C,D,E
struct Vector3 generateSpline(struct Vector3 initPos,struct Vector3 targetPos,double duration)
{
	static int topHeight = 25.0;
	double deltaX = targetPos.x-initPos.x;
	double deltaY = targetPos.y-initPos.y;
	struct Vector3 pos;

	deltaX = sqrt(deltaX*deltaX+deltaY*deltaY)/2.0;
	deltaY = (targetPos.z-initPos.z)/2.0;
	splineCoef[0].x = -deltaX;
	splineCoef[0].y = -deltaY;
	splineCoef[1].x = 0;
	splineCoef[1].y = 1;
	splineCoef[2].x = 0;
	splineCoef[2].y = topHeight;
	
	for(int i = 0;i<5;i++)
	{
		splineCoef[i+3].x = 0;
		splineCoef[i+3].y = 0;
		for(int j = 0;j<5;j++)
		{
			splineCoef[i].x += splineMatrix[i][j]*splineTerms[j].x;
			splineCoef[i].y += splineMatrix[i][j]*splineTerms[j].y;
		}
	}
	double t = 0,x,y;
	if(duration < 0.5)
	{
		t = (duration)/0.5;
		x = splineCoef[0].x+splineCoef[1].x*t+splineCoef[3].x*t*t+splineCoef[4].x*t*t*t;
		y = splineCoef[0].y+splineCoef[1].y*t+splineCoef[3].y*t*t+splineCoef[4].y*t*t*t;		
	}else
	{
		t = (duration -0.5)/0.5;
		x = splineCoef[2].x+splineCoef[5].x*t+splineCoef[6].x*t*t+splineCoef[7].x*t*t*t;
		y = splineCoef[2].y+splineCoef[5].y*t+splineCoef[6].y*t*t+splineCoef[7].y*t*t*t;				
	}
	pos.x = initPos.x+(targetPos.x-initPos.x)/deltaX*x;
	pos.y = initPos.y+(targetPos.y-initPos.y)/deltaY*x;
	pos.z = y+deltaY+initPos.z;
	return pos;
}

struct Vector3 generateLerp(struct Vector3 initPos,struct Vector3 targetPos,double duration)
{
	double ctrlEnd = -5;
	double ctrlScale = 1.0-exp(ctrlEnd);
	struct Vector3 pos;
	
	duration = (1.0-exp(duration*ctrlEnd))/ctrlScale;
	pos.x = targetPos.x*duration+(1.0-duration)*initPos.x;
	pos.y = targetPos.y*duration+(1.0-duration)*initPos.y;
	pos.z = targetPos.z*duration+(1.0-duration)*initPos.z;
	return pos;
}

void doMovement(struct Step* animation[4],int stepCount)//index:LF  LB  RF  RB
{
	struct Step* indicator[4];
	FeetMovement initPos[4];
	int timer[4] = {0};
	int step = 0;
	for(int i = 0;i<4;i++)
	{
		indicator[i] = &animation[i][step];
		initPos[i] = feetMovement[i];
	}
	int tt = 0;
	while(step < stepCount && !moveExit)
	{
		tt ++;
		//USART_printf(USART1,"timer:%d\r\n",tt);
		//StartTick();		
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
					step++;
				if(step < stepCount)
					indicator[i] = &animation[i][step];
				else
					break;
			}			
			switch(indicator[i]->procID)
			{
				case NONE:break;
				case PACE:feetMovement[i] = generatePaceForward(initPos[i],indicator[i]->targetPos,(double)timer[i]/(double)indicator[i]->duration);break;
				case LINE:feetMovement[i] = generateLiner(initPos[i],indicator[i]->targetPos,(double)timer[i]/(double)indicator[i]->duration);break;
				case LERP:feetMovement[i] = generateLerp(initPos[i],indicator[i]->targetPos,(double)timer[i]/(double)indicator[i]->duration);break;
				case CUBEPACE:feetMovement[i] = generateTriStep(initPos[i],indicator[i]->targetPos,(double)timer[i]/(double)indicator[i]->duration);break;
				case QUALERP:feetMovement[i] = generateQuadratic(initPos[i],indicator[i]->targetPos,(double)timer[i]/(double)indicator[i]->duration);break;
				case PACEGAP:feetMovement[i] = generatePaceForwardWithGap(initPos[i],indicator[i]->targetPos,(double)timer[i]/(double)indicator[i]->duration);break;
			}					
				//if(i == 3)
				//	USART_printf(USART1,"orgm:%lf,%lf,%lf\n",feetMovement[3].x,feetMovement[3].y,feetMovement[3].z);
			//total <1ms
			timer[i]++;
		}
		//USART_printf(USART1,"time using=%dms\n",EndTick());		
		applyPosture();		
	}
	moveExit = 0;
	//USART_printf(USART1,"finished movement\n");
}


#define CLAW_MAXX			55
#define CLAW_OFFXR			45
#define CLAW_OFFXL			50
#define CLAW_OFFZ				5
#define CLAW_FUL_OFFZ		10
#define CLAW_STEP_DURAT		20
#define CLAW_LEAN_DURAT	 18
#define CLAW_OFFY					20


#define CLAWSTEP_YLEN		120
#define CLAWSTEP_XLEN		50.0

#define MIN(a,b)		a<b?a:b
#define MAX(a,b)		a>b?a:b


void ClawFwdOneStep(double lean, double stepSteep)
{
	struct Step LF[6];
	struct Step RF[6];
	struct Step LR[6];
	struct Step RR[6];
	struct Step* animation[4] = { LF,LR,RF,RR };
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			animation[i][j].duration = CLAW_STEP_DURAT;
			animation[i][j].procID = NULL;
			animation[i][j].targetPos.z = CLAW_FUL_OFFZ;
			if (i == 0 || i == 2)
				animation[i][j].targetPos.z += CLAW_OFFZ;
		}
	}

	LF[0].duration = CLAW_LEAN_DURAT;
	LF[0].procID = LINE;
	LF[0].targetPos.x = CLAW_OFFXR;
	LF[0].targetPos.x = MIN(LF[0].targetPos.x, CLAW_MAXX);
	LF[0].targetPos.y = feetMovement[0].y-idlePos[0].y;

	LR[0].duration = CLAW_LEAN_DURAT;
	LR[0].procID = LINE;
	LR[0].targetPos.x = + CLAW_OFFXR;
	LR[0].targetPos.x = MIN(LR[0].targetPos.x, CLAW_MAXX);
	LR[0].targetPos.y = feetMovement[1].y-idlePos[1].y;

	RF[0].duration = CLAW_LEAN_DURAT;
	RF[0].procID = LINE;
	RF[0].targetPos.x = +CLAW_OFFXR;
	RF[0].targetPos.y = feetMovement[2].y-idlePos[2].y;

	RR[0].duration = CLAW_LEAN_DURAT;
	RR[0].procID = LINE;
	RR[0].targetPos.x = +CLAW_OFFXR;
	RR[0].targetPos.y = feetMovement[3].y-idlePos[3].y;



	LF[1].duration = CLAW_STEP_DURAT;
	LF[1].procID = LINE;
	LF[1].targetPos.x = +CLAW_OFFXR;
	LF[1].targetPos.y = 0;

	LR[1].duration = CLAW_STEP_DURAT;
	LR[1].procID = LINE;
	LR[1].targetPos.x = +CLAW_OFFXR;
	LR[1].targetPos.y = 0;

	RF[1].duration = CLAW_STEP_DURAT;
	RF[1].procID = LINE;
	RF[1].targetPos.x = CLAW_OFFXR;
	RF[1].targetPos.x = MIN(RF[1].targetPos.x, CLAW_MAXX);
	RF[1].targetPos.y = CLAWSTEP_YLEN / -2.0;


	RR[1].duration = CLAW_STEP_DURAT;
	RR[1].procID = CUBEPACE;
	RR[1].targetPos.x = + CLAW_OFFXR;
	RR[1].targetPos.x = MIN(RR[1].targetPos.x, CLAW_MAXX);
	RR[1].targetPos.y = (CLAWSTEP_YLEN) / 2.0;


	RF[2].duration = CLAW_STEP_DURAT;
	RF[2].procID = CUBEPACE;
	RF[2].targetPos.x =  + CLAW_OFFXR;
	RF[2].targetPos.x = MIN(RF[2].targetPos.x, CLAW_MAXX);
	RF[2].targetPos.y =  CLAWSTEP_YLEN / 2.0;










	LF[3].duration = CLAW_LEAN_DURAT;
	LF[3].procID = LINE;
	LF[3].targetPos.x = -CLAW_OFFXL;
	LF[3].targetPos.y = 0;

	LR[3].duration = CLAW_LEAN_DURAT;
	LR[3].procID = LINE;
	LR[3].targetPos.x = -CLAW_OFFXL;
	LR[3].targetPos.y = 0;

	RR[3].duration = CLAW_LEAN_DURAT;
	RR[3].procID = LINE;
	RR[3].targetPos.x = - CLAW_OFFXL;
	RR[3].targetPos.x = MAX(RR[3].targetPos.x, -CLAW_MAXX);
	RR[3].targetPos.y = (CLAWSTEP_YLEN) / 2.0;

	RF[3].duration = CLAW_LEAN_DURAT;
	RF[3].procID = LINE;
	RF[3].targetPos.x = - CLAW_OFFXL;
	RF[3].targetPos.x = MAX(RF[3].targetPos.x, -CLAW_MAXX);
	RF[3].targetPos.y = (CLAWSTEP_YLEN) / 2.0;



	LF[4].duration = CLAW_STEP_DURAT;
	LF[4].procID = LINE;
	LF[4].targetPos.x = - CLAW_OFFXL;
	LF[4].targetPos.x = MAX(LF[4].targetPos.x, -CLAW_MAXX);
	LF[4].targetPos.y = (CLAWSTEP_YLEN) / -2.0;


	LR[4].duration = CLAW_STEP_DURAT;
	LR[4].procID = CUBEPACE;
	LR[4].targetPos.x = - CLAW_OFFXL;
	LR[4].targetPos.x = MAX(LR[4].targetPos.x, -CLAW_MAXX);
	LR[4].targetPos.y = (CLAWSTEP_YLEN) / 2.0;

	RR[4].duration = CLAW_STEP_DURAT;
	RR[4].procID = LINE;
	RR[4].targetPos.x = -CLAW_OFFXL;
	RR[4].targetPos.y = 0;

	RF[4].duration = CLAW_STEP_DURAT;
	RF[4].procID = LINE;
	RF[4].targetPos.x = -CLAW_OFFXL;
	RF[4].targetPos.y = 0;


	LF[4].duration = CLAW_STEP_DURAT;
	LF[4].procID = LINE;
	LF[4].targetPos.x = - CLAW_OFFXL;
	LF[4].targetPos.x = MAX(LF[4].targetPos.x, -CLAW_MAXX);
	LF[4].targetPos.y = (CLAWSTEP_YLEN) / -2.0;

	LF[5].duration = CLAW_STEP_DURAT;
	LF[5].procID = CUBEPACE;
	LF[5].targetPos.x =  - CLAW_OFFXL;
	LF[5].targetPos.x = MAX(LF[5].targetPos.x, -CLAW_MAXX);
	LF[5].targetPos.y = (CLAWSTEP_YLEN) / 2.0;


	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			animation[i][j].targetPos.x += idlePos[i].x;
			if(animation[i][j].targetPos.y >= 0 && (i == 0 ||i == 2))
				animation[i][j].targetPos.z += idlePos[i].z + sin(stepSteep) * animation[i][j].targetPos.y;
			else
				animation[i][j].targetPos.z += idlePos[i].z + sin(stepSteep/3.0) * animation[i][j].targetPos.y;
			animation[i][j].targetPos.y += idlePos[i].y + idlePos[0].z * tan(lean);
			if(i == 0 || i == 2)
				animation[i][j].targetPos.y += CLAW_OFFY;
			
		}
	}
	doMovement(animation, 6);
}



#define ADJ_DURAT			24.0
#define ADJ_OFFX			55.0
#define ADJ_STEP			50.0
void VerticalAdjOneStep(double vertical, double rotate)
{

	struct Step LF[6];
	struct Step RF[6];
	struct Step LR[6];
	struct Step RR[6];
	struct Step* animation[4] = { LF,LR,RF,RR };
	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 6; j++)
		{
			animation[i][j].duration = CLAW_STEP_DURAT;
			animation[i][j].procID = NULL;
			animation[i][j].targetPos.x = animation[i][j].targetPos.y = animation[i][j].targetPos.z = 0;

		}
	}



	LF[0].duration = CLAW_LEAN_DURAT;
	LF[0].procID = LINE;
	LF[0].targetPos.x = ADJ_OFFX;

	LR[0].duration = CLAW_LEAN_DURAT;
	LR[0].procID = LINE;
	LR[0].targetPos.x = +ADJ_OFFX;

	RF[0].duration = CLAW_LEAN_DURAT;
	RF[0].procID = LINE;
	RF[0].targetPos.x = +ADJ_OFFX;

	RR[0].duration = CLAW_LEAN_DURAT;
	RR[0].procID = LINE;
	RR[0].targetPos.x = +ADJ_OFFX;

	RR[1].duration = CLAW_STEP_DURAT;
	RR[1].procID = CUBEPACE;
	RR[1].targetPos.x = (cos(rotate) - 1.0) * (DOG_WIDTH / 2.0 + feetMovement[3].x - idlePos[3].x) - sin(rotate) * (-DOG_LENGTH / 2.0 + feetMovement[3].y - idlePos[3].y)
		+ ADJ_OFFX + ADJ_STEP / +2.0 * (vertical);
	RR[1].targetPos.y = (sin(rotate)) * (DOG_WIDTH / 2.0 + feetMovement[3].x - idlePos[3].x) + (cos(rotate) - 1.0) * (-DOG_LENGTH / 2.0 + feetMovement[3].y - idlePos[3].y);


	RF[2].duration = CLAW_STEP_DURAT;
	RF[2].procID = CUBEPACE;
	RF[2].targetPos.x = (cos(rotate) - 1.0) * (DOG_WIDTH / 2.0 + feetMovement[2].x - idlePos[2].x) - sin(rotate) * (DOG_LENGTH / 2.0 + feetMovement[2].y - idlePos[2].y) + ADJ_OFFX + ADJ_STEP / +2.0 * (vertical);
	RF[2].targetPos.y = (sin(rotate)) * (DOG_WIDTH / 2.0 + feetMovement[2].x - idlePos[2].x) + (cos(rotate) - 1) * (DOG_LENGTH / 2.0 + feetMovement[2].y - idlePos[2].y);


	LF[3].duration = CLAW_LEAN_DURAT;
	LF[3].procID = LINE;
	LF[3].targetPos.x = -((cos(rotate) - 1.0) * (-DOG_WIDTH / 2.0 + feetMovement[0].x - idlePos[0].x) - sin(rotate) * (DOG_LENGTH / 2.0 + feetMovement[0].y - idlePos[0].y)) - ADJ_OFFX + ADJ_STEP / -2.0 * (vertical);
	LF[3].targetPos.y = -((sin(rotate)) * (-DOG_WIDTH / 2.0 + feetMovement[0].x - idlePos[0].x) + (cos(rotate) - 1.0) * (DOG_LENGTH / 2.0 + feetMovement[0].y - idlePos[0].y));


	LR[3].duration = CLAW_LEAN_DURAT;
	LR[3].procID = LINE;
	LR[3].targetPos.x = -((cos(rotate) - 1.0) * (-DOG_WIDTH / 2.0 + feetMovement[1].x - idlePos[1].x) - sin(rotate) * (-DOG_LENGTH / 2.0 + feetMovement[1].y - idlePos[1].y)) - ADJ_OFFX + ADJ_STEP / -2.0 * (vertical);
	LR[3].targetPos.y = -((sin(rotate)) * (-DOG_WIDTH / 2.0 + feetMovement[1].x - idlePos[1].x) + (cos(rotate) - 1.0) * (-DOG_LENGTH / 2.0 + feetMovement[1].y - idlePos[1].y));

	RR[3].duration = CLAW_LEAN_DURAT;
	RR[3].procID = LINE;
	RR[3].targetPos.x = -ADJ_OFFX;
	RR[3].targetPos.y = 0;


	RF[3].duration = CLAW_LEAN_DURAT;
	RF[3].procID = LINE;
	RF[3].targetPos.x = -ADJ_OFFX;
	RF[3].targetPos.y = 0;


	LR[4].duration = CLAW_STEP_DURAT;
	LR[4].procID = CUBEPACE;
	LR[4].targetPos.x = -ADJ_OFFX;
	LR[4].targetPos.y = 0;

	LF[5].duration = CLAW_STEP_DURAT;
	LF[5].procID = CUBEPACE;
	LF[5].targetPos.x = -ADJ_OFFX;
	LF[5].targetPos.y = 0;

	for (int i = 0; i < 4; i++)
	{
		for (int j = 0; j < 6; j++)
		{

			animation[i][j].targetPos.x += idlePos[i].x;
			animation[i][j].targetPos.y += feetMovement[i].y;
			animation[i][j].targetPos.z = feetMovement[i].z;
		}
	}

	doMovement(animation, 6);

}
