#include "gait.h"
#include <math.h>
#include "usart.h"
#include "pwm.h"
#include "delay.h"
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
    struct Vector3 targetPos;
};

#define NONE			0
#define PACE			1
#define LINE			2	
#define CUBEPACE	3
#define LERP			4
#define QUALERP		5
#define PACEGAP		6



// struct Vector2 feetPhaseZeroPos[4];//LF  LB  RF  RB

// struct Vector2 feetPhaseEndPos[4];
// struct Vector2 feetPhaseHalfPos[4];

struct Vector3 idlePos[4];//LF  LB  RF  RB
double constLeanPos = 0;//only rotate in y axis


char key = 0;

struct Vector3 feetMovement[4];//calcualted Pos


// struct Vector3 generateFeetMovement(struct Vector3* idle, struct Vector2 phZPos, struct Vector2 phHPos, struct Vector2 phEPos, double height, double time, int left)
// {
// 	struct Vector3 pos;
// 	double scale = time / (double)STEP_DURATION;
// 	scale = scale - (int)scale;
// 	if (scale <= 0.5)
// 	{
// 		double sigma = 4.0*PI*scale;
// 		pos.x = (phHPos.x - phZPos.x)*((sigma - sin(sigma)) / (2.0*PI)) + phZPos.x;
// 		pos.y = (phHPos.y - phZPos.y)*((sigma - sin(sigma)) / (2.0*PI)) + phZPos.y;
// 		pos.z = height * ((1.0 - cos(sigma)) / 2.0);

// 	}
// 	else
// 	{
// 		scale = 2 * (scale - 0.5);
// 		pos.z = 0;
// 		pos.x = scale * phEPos.x + (1 - scale)*phHPos.x;
// 		pos.y = scale * phEPos.y + (1 - scale)*phHPos.y;
// 	}
// 	if (left == 1)
// 	{
// 		pos.x -= idle->x;
// 		pos.y += idle->y;
// 		pos.z += idle->z;

// 	}
// 	else
// 	{
// 		pos.x += idle->x;
// 		pos.y += idle->y;
// 		pos.z += idle->z;
// 	}
// 	return pos;
// }


struct Vector3 generateLiner(struct Vector3 initPos,struct Vector3 targetPos,double duration)
{
	struct Vector3 pos;
	pos.x = targetPos.x*duration+(1.0-duration)*initPos.x;
	pos.y = targetPos.y*duration+(1.0-duration)*initPos.y;
	pos.z = targetPos.z*duration+(1.0-duration)*initPos.z;	
	return pos;
}

#define STEP_OBS_HEIGHT		50.0
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


int movementSolveR(struct Vector3 feetPos, struct Vector3* rotate)
{
	double delta = (2.0*feetPos.x*TOPL)*(2.0*feetPos.x*TOPL)
		- 4.0*(feetPos.x*feetPos.x + feetPos.z*feetPos.z)*(TOPL*TOPL - feetPos.z*feetPos.z);
	if (delta < 0)
		return -1;
	delta = (sqrt(delta) + 2.0*feetPos.x*TOPL) / 2.0 / (feetPos.x*feetPos.x + feetPos.z*feetPos.z);
	if (delta > 1)
		delta = 1;
	rotate->x = acos(delta);
	if (feetPos.x < TOPL)
		rotate->x *= -1;
	struct Vector2 projPos = { feetPos.y,-TOPL * tan(rotate->x) + feetPos.z / cos(rotate->x) };
	double Lf = sqrt(projPos.x*projPos.x + projPos.y*projPos.y);
	rotate->y = acos(projPos.x / Lf);
	rotate->y = rotate->y + acos((MIDL*MIDL + Lf * Lf - FEETL * FEETL) / (2.0*MIDL*Lf));
	double angz = (MIDL * MIDL + FEETL * FEETL - Lf * Lf)/(2.0 * MIDL * FEETL);
	if(angz > 1)
		angz = 1;
	else if(angz <= -1)
		angz = -1;
	rotate->z = acos(angz);
	return 0;
}

int movementSolveL(struct Vector3 feetPos, struct Vector3* rotate)
{
	feetPos.x = -feetPos.x;
	double delta = (2.0*feetPos.x*TOPL)*(2.0*feetPos.x*TOPL)
		- 4.0*(feetPos.x*feetPos.x + feetPos.z*feetPos.z)*(TOPL*TOPL - feetPos.z*feetPos.z);
	if (delta < 0)
		return -1;
	delta = (sqrt(delta) + 2.0*feetPos.x*TOPL) / 2.0 / (feetPos.x*feetPos.x + feetPos.z*feetPos.z);
	if (delta > 1)
		delta = 1;
	rotate->x = acos(delta);
	if (feetPos.x < TOPL)
		rotate->x *= -1;
	struct Vector2 projPos = { feetPos.y,-TOPL * tan(rotate->x) + feetPos.z / cos(rotate->x) };
	double Lf = sqrt(projPos.x*projPos.x + projPos.y*projPos.y);
	rotate->y = acos(projPos.x / Lf);
	rotate->y = rotate->y + acos((MIDL*MIDL + Lf * Lf - FEETL * FEETL) / (2.0*MIDL*Lf));
	double angz = (MIDL * MIDL + FEETL * FEETL - Lf * Lf)/(2.0 * MIDL * FEETL);
	if(angz > 1)
		angz = 1;
	else if(angz <= -1)
		angz = -1;
	rotate->z = acos(angz);
	return 0;
}

void calcMotorAngle(int id,struct Vector3* rotate)//x:肩相对于水平向外的夹角，y:腿相对于水平前方的角度，z:脚相对与腿的角度
{
	rotate->y -= 135.0/180.0*PI;
	double phi = rotate->z - PI / 2.0;	
	double L = 2.0 * MID_MOTOR_AXIS * MID_MOTOR_AXIS * (1 - cos(phi)) + MID_FEET_JOINT * MID_FEET_JOINT - 2.0 * MID_MOTOR_AXIS * MID_FEET_JOINT * sin(phi);
	L = sqrt(L);

	//double L = MID_FEET_JOINT+MID_MOTOR_AXIS/tan(rotate->z);
	double angz = (MID_LEVER1*MID_LEVER1+L*L-MID_LEVER2*MID_LEVER2)/(2.0*MID_LEVER1*MID_LEVER2);
	if(angz >= 1)
		angz = 1;
	else if(angz <= -1)angz = -1;
	rotate->z = acos(angz);
	rotate->z -= PI/2;
	
	if(rotate->x <=-1.0471973)
	{	
		//printf("x lower!@id=%d with %lf\n",id,rotate->x);
		rotate->x = -1.0471973;
	}
	else if(rotate->x >= 1.0471973)
	{
		//printf("x upper!@id=%d with %lf\n",id,rotate->x);
		rotate->x = 1.0471973;
	}
	
	if(rotate->y >= 0.785398)
	{	
		//printf("y upper!@id=%d with %lf\n",id,rotate->y);
		rotate->y = 0.785398;
	}
	else if(rotate->y <= -2.094395)
	{
		//printf("y lower!@id=%d with %lf\n",id,rotate->y);
		rotate->y = -2.094395;
	}
	
	if (rotate->z <= -0.79)
	{
		//printf("z lower!@id=%d with %lf\n",id,rotate->z);
		rotate->z = -0.79;
	}
	else if(rotate->z >= 0.79)
	{
		//printf("z upper!@id=%d with %lf\n",id,rotate->z);
		rotate->z = 0.79;
	}
	
	switch(id)
	{
		case 1:
		{
			rotate->z *= -1;
			rotate->x += 137.4/180.0*PI;
			rotate->y += 142.0/180.0*PI;
			rotate->z += 131.9/180.0*PI;
		}break;
		case 2:
		{
			rotate->x *= -1;
			rotate->z *= -1;
			rotate->x += 125.4/180.0*PI;
			rotate->y += 140.8/180.0*PI;
			rotate->z += 156.2/180.0*PI;			
		}break;
		case 3:
		{
			
			rotate->x *= -1;
			rotate->y *= -1;
			rotate->x += 140.0/180.0*PI;
			rotate->y += 145.6/180.0*PI;
			rotate->z += 113.2/180.0*PI;
			//USART_printf(USART1,"angle:%f\n",rotate->z);
		}break;
		case 4:
		{
			rotate->y *= -1;
			rotate->x += 138.4/180.0*PI;
			rotate->y += 130.0/180.0*PI;
			rotate->z += 136.8/180.0*PI;
		}break;
	}
}


void projectionTransform(struct Vector3 *source,struct Vector3* dest)
{
	double z = source->z;
	dest->z = sin(-constLeanPos)*source->y+cos(constLeanPos)*z+source->y*sin(constLeanPos);	
	dest->y = cos(constLeanPos)*source->y+sin(constLeanPos)*z;
	dest->x = source->x;
}


void applyPosture()//12ms
{
	//for(int i = 0;i<4;i++)

	struct Vector3 rotate;
	struct Vector3 transformed;
	
	projectionTransform(&feetMovement[0],&transformed);
	//printf("asdsawdcafasdwfasdawddqeqwddawdasdadw\n",transformed.x,transformed.y,transformed.z);
	
	//USART_printf(USART1,"%f,%f,%f\n",transformed.x,transformed.y,transformed.z);
	
	movementSolveL(transformed, &rotate);//using 1ms

	calcMotorAngle(1,&rotate);//using < 1ms
	motor_control(1,1,rotate.x);//apply to motor
	motor_control(1,2,rotate.y);
	motor_control(1,3,rotate.z);

	projectionTransform(&feetMovement[1],&transformed);
	movementSolveL(transformed, &rotate);
	calcMotorAngle(2,&rotate);
	motor_control(2,1,rotate.x);//apply to motor
	motor_control(2,2,rotate.y);
	motor_control(2,3,rotate.z);

	projectionTransform(&feetMovement[2],&transformed);
//	static int i = 0;
//	i++;
	//printf("\nat %d ,loc:{%lf,%lf,%lf}\n",i,transformed.x,transformed.y,transformed.z);
	movementSolveR(transformed, &rotate);
	//printf("oang:%lf,%lf,%lf\n",rotate.x*180.0/PI,rotate.y*180.0/PI,rotate.z*180.0/PI);

	calcMotorAngle(3,&rotate);
	//printf("ang:%lf,%lf,%lf\n",rotate.x*180.0/PI,rotate.y*180.0/PI,rotate.z*180.0/PI);

	motor_control(3,1,rotate.x);//apply to motor
	motor_control(3,2,rotate.y);
	motor_control(3,3,rotate.z);


	projectionTransform(&feetMovement[3],&transformed);
	//USART_printf(USART1,"feet[3] = {%lf,%lf,%lf}\n",transformed.x,transformed.y,transformed.z);

	movementSolveR(transformed, &rotate);

	calcMotorAngle(4,&rotate);
	motor_control(4,1,rotate.x);//apply to motor
	motor_control(4,2,rotate.y);
	motor_control(4,3,rotate.z);
			//USART_printf(USART1,"time using=%dms\n",GetTickCount());
		delay_ms(APPLY_WAITTIME);

}


void doMovement(struct Step* animation[4],int stepCount)//index:LF  LB  RF  RB
{
	struct Step* indicator[4];
	struct Vector3 initPos[4];
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



void Initialize(struct Vector3 pos_RF)
{
	for(int i = 0;i<4;i++)
		idlePos[i] = pos_RF;
	idlePos[0].x *= -1;
	idlePos[1].x *= -1;
	for(int i = 0;i<4;i++)
		feetMovement[i] = idlePos[i];

	applyPosture();
}


void SetLeanPos(double lean)
{
	struct Step* animation[4];
	struct Step targetPos[4];
	struct Vector3 buffer[4];
	constLeanPos = lean;
	for(int i = 0;i<4;i++)
	{
		buffer[i] = feetMovement[i];
		targetPos[i].duration = 10;
		targetPos[i].procID = LERP;

		projectionTransform(&feetMovement[i],&targetPos[i].targetPos);
		animation[i] = &targetPos[i];
	}
	constLeanPos = 0;
	doMovement(animation,1);
	constLeanPos = lean;
	for(int i = 0;i<4;i++)
	{
		feetMovement[i] = buffer[i];
	}
}






















void StopAndStand()
{
	struct Step leftFront[2];
	struct Step rightFront[2];
	struct Step leftRear[2];
	struct Step rightRear[2];
	struct Step* animation[4] = {leftFront,leftRear,rightFront,rightRear};
	leftFront[0].duration = 12;
	leftFront[0].procID = PACE;
	leftFront[0].targetPos = idlePos[0];

	leftFront[1].duration = 12;
	leftFront[1].procID = NULL;

	leftRear[0].duration = 12;
	leftRear[0].procID = NULL;

	leftRear[1].duration = 12;
	leftRear[1].procID = PACE;
	leftRear[1].targetPos = idlePos[1];

	rightFront[0].duration = 12;
	rightFront[0].procID = NULL;

	rightFront[1].duration = 12;
	rightFront[1].procID = PACE;
	rightFront[1].targetPos = idlePos[2];

	rightRear[0].duration = 12;
	rightRear[0].procID = PACE;
	rightRear[0].targetPos = idlePos[3];

	rightRear[1].duration = 12;
	rightRear[1].procID = NULL;
	
	doMovement(animation,2);
}





#define CALIB_LF_X				4.0//LF&RR
#define CALIB_LF_Y				4.0
#define CALIB_LR_X				0.0//LF&RR
#define CALIB_LR_Y				0.0
#define CALIB_RF_X				0.0//RF&LR
#define CALIB_RF_Y				0.0
#define CALIB_RR_X				4.0//RF&LR
#define CALIB_RR_Y				4.0
#define MOVESTEP_LEN    80.0

#define FORWARDTIME_OBS		13
#define BACKTIME_OBS			13
void PaceOneStepObs(struct Vector2 move,double rotate)
{
	struct Step leftFront[2];
	struct Step rightFront[2];
	struct Step leftRear[2];
	struct Step rightRear[2];
	struct Step* animation[4] = {leftFront,leftRear,rightFront,rightRear};

	leftFront[0].duration = FORWARDTIME_OBS;
	leftFront[0].procID = PACEGAP;
	leftFront[0].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0+CALIB_LF_X) - sin(rotate)*(DOG_LENGTH / 2.0-CALIB_LF_Y)+move.x*MOVESTEP_LEN)/2.0-CALIB_LF_X;
	leftFront[0].targetPos.y = ((sin(rotate))*-(DOG_WIDTH / 2.0+CALIB_LF_X) + (cos(rotate) - 1.0)*(DOG_LENGTH / 2.0-CALIB_LF_Y)+move.y*MOVESTEP_LEN)/2.0-CALIB_LF_Y;
	leftFront[0].targetPos.z = 0;
	
	leftFront[1].duration = BACKTIME_OBS;
	leftFront[1].procID = LERP;
	leftFront[1].targetPos.x = ((cos(rotate) - 1.0)*-(DOG_WIDTH / 2.0+CALIB_LF_X) - sin(rotate)*(DOG_LENGTH / 2.0-CALIB_LF_Y)+move.x*MOVESTEP_LEN)/-2.0-CALIB_LF_X;
	leftFront[1].targetPos.y = ((sin(rotate))*-(DOG_WIDTH / 2.0+CALIB_LF_X) + (cos(rotate) - 1.0)*(DOG_LENGTH / 2.0-CALIB_LF_Y)+move.y*MOVESTEP_LEN)/-2.0-CALIB_LF_Y;
	leftFront[1].targetPos.z = 0;

	
	leftRear[0].duration = BACKTIME_OBS;
	leftRear[0].procID = LERP;
	leftRear[0].targetPos.x = ((cos(rotate) - 1.0)*-(DOG_WIDTH / 2.0-CALIB_LR_X) - sin(rotate)*-(DOG_LENGTH / 2.0+CALIB_LR_Y)+move.x*MOVESTEP_LEN)/-2.0+CALIB_LR_X;
	leftRear[0].targetPos.y = ((sin(rotate))*-(DOG_WIDTH / 2.0-CALIB_LR_X) + (cos(rotate) - 1.0)*-(DOG_LENGTH / 2.0+CALIB_LR_Y)+move.y*MOVESTEP_LEN)/-2.0-CALIB_LR_Y;
	leftRear[0].targetPos.z = 0;

	leftRear[1].duration = FORWARDTIME_OBS;
	leftRear[1].procID = PACEGAP;
	leftRear[1].targetPos.x = ((cos(rotate) - 1.0)*-(DOG_WIDTH / 2.0-CALIB_LR_X) - sin(rotate)*-(DOG_LENGTH / 2.0+CALIB_LR_Y)+move.x*MOVESTEP_LEN)/2.0+CALIB_LR_X;
	leftRear[1].targetPos.y = ((sin(rotate))*-(DOG_WIDTH / 2.0-CALIB_LR_X)+ (cos(rotate) - 1.0)*-(DOG_LENGTH / 2.0+CALIB_LR_Y)+move.y*MOVESTEP_LEN)/2.0-CALIB_LR_Y;
	leftRear[1].targetPos.z = 0;

	rightFront[0].duration = BACKTIME_OBS;
	rightFront[0].procID = LERP;
	rightFront[0].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0+CALIB_RF_X) - sin(rotate)*(DOG_LENGTH / 2.0-CALIB_RF_Y)+move.x*MOVESTEP_LEN)/-2.0+CALIB_RF_X;
	rightFront[0].targetPos.y = ((sin(rotate))*(DOG_WIDTH / 2.0+CALIB_RF_X) + (cos(rotate) - 1)*(DOG_LENGTH / 2.0-CALIB_RF_Y)+move.y*MOVESTEP_LEN)/-2.0-CALIB_RF_Y;
	rightFront[0].targetPos.z = 0;

	rightFront[1].duration = FORWARDTIME_OBS;
	rightFront[1].procID = PACEGAP;
	rightFront[1].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0+CALIB_RF_X) - sin(rotate)*(DOG_LENGTH / 2.0-CALIB_RF_Y)+move.x*MOVESTEP_LEN)/2.0+CALIB_RF_X;
	rightFront[1].targetPos.y = ((sin(rotate))*(DOG_WIDTH / 2.0+CALIB_RF_X) + (cos(rotate) - 1.0)*(DOG_LENGTH / 2.0-CALIB_RF_Y)+move.y*MOVESTEP_LEN)/2.0-CALIB_RF_Y;	
	rightFront[1].targetPos.z = 0;

	rightRear[0].duration = FORWARDTIME_OBS;
	rightRear[0].procID = PACEGAP;
	rightRear[0].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0-CALIB_RR_X) - sin(rotate)*-(DOG_LENGTH / 2.0+CALIB_RR_Y)+move.x*MOVESTEP_LEN)/2.0-CALIB_RR_X;
	rightRear[0].targetPos.y = ((sin(rotate))*(DOG_WIDTH / 2.0-CALIB_RR_X) + (cos(rotate) - 1.0)*-(DOG_LENGTH / 2.0+CALIB_RR_Y)+move.y*MOVESTEP_LEN)/2.0-CALIB_RR_Y;
	rightRear[0].targetPos.z = 0;

	rightRear[1].duration = BACKTIME_OBS;
	rightRear[1].procID = LERP;
	rightRear[1].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0-CALIB_RR_X) - sin(rotate)*-(DOG_LENGTH / 2.0+CALIB_RR_Y)+move.x*MOVESTEP_LEN)/-2.0-CALIB_RR_X;
	rightRear[1].targetPos.y = ((sin(rotate))*(DOG_WIDTH / 2.0-CALIB_RR_X) + (cos(rotate) - 1)*-(DOG_LENGTH / 2.0+CALIB_RR_Y)+move.y*MOVESTEP_LEN)/-2.0-CALIB_RR_Y;
	rightRear[1].targetPos.z = 0;
	
	for(int i = 0;i<4;i++)
	{
		//USART_printf(USART1,"idle feet[%d] = {%lf,%lf,%lf}\n",i,idlePos[i].x,idlePos[i].y,idlePos[i].z);
		
		for(int j = 0;j<2;j++)
		{
			animation[i][j].targetPos.x += idlePos[i].x;
			animation[i][j].targetPos.y += idlePos[i].y;
			animation[i][j].targetPos.z += idlePos[i].z;
			
		}
	}
	//StartTick();
	doMovement(animation,2);
	//USART_printf(USART1,"time using=%dms\n",EndTick());	
}


//#define SHRINK_Y		0
//#define SHRINK_X		0.25

//#define SLOW_LF_X				(0.0-SHRINK_X*DOG_WIDTH)//LF&RR
//#define SLOW_LF_Y				(16.0-SHRINK_Y*DOG_LENGTH)

//#define SLOW_LR_X				(5.0+SHRINK_X*DOG_WIDTH)//LF&RR
//#define SLOW_LR_Y				(20.0+SHRINK_Y*DOG_LENGTH)

//#define SLOW_RF_X				(0.0-SHRINK_X*DOG_WIDTH)//RF&LR
//#define SLOW_RF_Y				(20.0-SHRINK_Y*DOG_LENGTH)

//#define SLOW_RR_X				(5.0+SHRINK_X*DOG_WIDTH)//RF&LR
//#define SLOW_RR_Y				(16.0+SHRINK_Y*DOG_LENGTH)


#define SLOW_LF_X				8.0//LF&RR
#define SLOW_LF_Y				8.0
#define SLOW_LR_X				10.0//LF&RR
#define SLOW_LR_Y				10.0
#define SLOW_RF_X				10.0//RF&LR
#define SLOW_RF_Y				10.0
#define SLOW_RR_X				8.0//RF&LR
#define SLOW_RR_Y				8.0


#define SLOW_DURATION		12//17
#define SLOW_STEPLEN		80//60
void PaceOneStepSlow(struct Vector2 move,double rotate)
{
	struct Step leftFront[2];
	struct Step rightFront[2];
	struct Step leftRear[2];
	struct Step rightRear[2];
	struct Step* animation[4] = {leftFront,leftRear,rightFront,rightRear};

	leftFront[0].duration = SLOW_DURATION;
	leftFront[0].procID = PACE;
	leftFront[0].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0+SLOW_LF_X) - sin(rotate)*(DOG_LENGTH / 2.0-SLOW_LF_Y)+move.x*SLOW_STEPLEN)/2.0-SLOW_LF_X;
	leftFront[0].targetPos.y = ((sin(rotate))*-(DOG_WIDTH / 2.0+SLOW_LF_X) + (cos(rotate) - 1.0)*(DOG_LENGTH / 2.0-SLOW_LF_Y)+move.y*SLOW_STEPLEN)/2.0-SLOW_LF_Y;
	leftFront[0].targetPos.z = 0;
	
	leftFront[1].duration = SLOW_DURATION;
	leftFront[1].procID = LERP;
	leftFront[1].targetPos.x = ((cos(rotate) - 1.0)*-(DOG_WIDTH / 2.0+SLOW_LF_X) - sin(rotate)*(DOG_LENGTH / 2.0-SLOW_LF_Y)+move.x*SLOW_STEPLEN)/-2.0-SLOW_LF_X;
	leftFront[1].targetPos.y = ((sin(rotate))*-(DOG_WIDTH / 2.0+SLOW_LF_X) + (cos(rotate) - 1.0)*(DOG_LENGTH / 2.0-SLOW_LF_Y)+move.y*SLOW_STEPLEN)/-2.0-SLOW_LF_Y;
	leftFront[1].targetPos.z = 0;

	
	leftRear[0].duration = SLOW_DURATION;
	leftRear[0].procID = LERP;
	leftRear[0].targetPos.x = ((cos(rotate) - 1.0)*-(DOG_WIDTH / 2.0-SLOW_LR_X) - sin(rotate)*-(DOG_LENGTH / 2.0+SLOW_LR_Y)+move.x*SLOW_STEPLEN)/-2.0+SLOW_LR_X;
	leftRear[0].targetPos.y = ((sin(rotate))*-(DOG_WIDTH / 2.0-SLOW_LR_X) + (cos(rotate) - 1.0)*-(DOG_LENGTH / 2.0+SLOW_LR_Y)+move.y*SLOW_STEPLEN)/-2.0-SLOW_LR_Y;
	leftRear[0].targetPos.z = 0;

	leftRear[1].duration = SLOW_DURATION;
	leftRear[1].procID = PACE;
	leftRear[1].targetPos.x = ((cos(rotate) - 1.0)*-(DOG_WIDTH / 2.0-SLOW_LR_X) - sin(rotate)*-(DOG_LENGTH / 2.0+SLOW_LR_Y)+move.x*SLOW_STEPLEN)/2.0+SLOW_LR_X;
	leftRear[1].targetPos.y = ((sin(rotate))*-(DOG_WIDTH / 2.0-SLOW_LR_X)+ (cos(rotate) - 1.0)*-(DOG_LENGTH / 2.0+SLOW_LR_Y)+move.y*SLOW_STEPLEN)/2.0-SLOW_LR_Y;
	leftRear[1].targetPos.z = 0;

	rightFront[0].duration = SLOW_DURATION;
	rightFront[0].procID = LERP;
	rightFront[0].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0+SLOW_RF_X) - sin(rotate)*(DOG_LENGTH / 2.0-SLOW_RF_Y)+move.x*SLOW_STEPLEN)/-2.0+SLOW_RF_X;
	rightFront[0].targetPos.y = ((sin(rotate))*(DOG_WIDTH / 2.0+SLOW_RF_X) + (cos(rotate) - 1)*(DOG_LENGTH / 2.0-SLOW_RF_Y)+move.y*SLOW_STEPLEN)/-2.0-SLOW_RF_Y;
	rightFront[0].targetPos.z = 0;

	rightFront[1].duration = SLOW_DURATION;
	rightFront[1].procID = PACE;
	rightFront[1].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0+SLOW_RF_X) - sin(rotate)*(DOG_LENGTH / 2.0-SLOW_RF_Y)+move.x*SLOW_STEPLEN)/2.0+SLOW_RF_X;
	rightFront[1].targetPos.y = ((sin(rotate))*(DOG_WIDTH / 2.0+SLOW_RF_X) + (cos(rotate) - 1.0)*(DOG_LENGTH / 2.0-SLOW_RF_Y)+move.y*SLOW_STEPLEN)/2.0-SLOW_RF_Y;	
	rightFront[1].targetPos.z = 0;

	rightRear[0].duration = SLOW_DURATION;
	rightRear[0].procID = PACE;
	rightRear[0].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0-SLOW_RR_X) - sin(rotate)*-(DOG_LENGTH / 2.0+SLOW_RR_Y)+move.x*SLOW_STEPLEN)/2.0-SLOW_RR_X;
	rightRear[0].targetPos.y = ((sin(rotate))*(DOG_WIDTH / 2.0-SLOW_RR_X) + (cos(rotate) - 1.0)*-(DOG_LENGTH / 2.0+SLOW_RR_Y)+move.y*SLOW_STEPLEN)/2.0-SLOW_RR_Y;
	rightRear[0].targetPos.z = 0;

	rightRear[1].duration = SLOW_DURATION;
	rightRear[1].procID = LERP;
	rightRear[1].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0-SLOW_RR_X) - sin(rotate)*-(DOG_LENGTH / 2.0+SLOW_RR_Y)+move.x*SLOW_STEPLEN)/-2.0-SLOW_RR_X;
	rightRear[1].targetPos.y = ((sin(rotate))*(DOG_WIDTH / 2.0-SLOW_RR_X) + (cos(rotate) - 1)*-(DOG_LENGTH / 2.0+SLOW_RR_Y)+move.y*SLOW_STEPLEN)/-2.0-SLOW_RR_Y;
	rightRear[1].targetPos.z = 0;
	
	for(int i = 0;i<4;i++)
	{
		//USART_printf(USART1,"idle feet[%d] = {%lf,%lf,%lf}\n",i,idlePos[i].x,idlePos[i].y,idlePos[i].z);
		
		for(int j = 0;j<2;j++)
		{
			animation[i][j].targetPos.x += idlePos[i].x;
			animation[i][j].targetPos.y += idlePos[i].y;
			animation[i][j].targetPos.z += idlePos[i].z;
			
		}
	}
	//StartTick();
	doMovement(animation,2);
	//USART_printf(USART1,"time using=%dms\n",EndTick());	
}


//#define PACE_LF_X				8.0//LF&RR
//#define PACE_LF_Y				8.0
//#define PACE_LR_X				10.0//LF&RR
//#define PACE_LR_Y				10.0
//#define PACE_RF_X				10.0//RF&LR
//#define PACE_RF_Y				10.0
//#define PACE_RR_X				8.0//RF&LR
//#define PACE_RR_Y				8.0


#define PACE_LF_X				8.0//LF&RR
#define PACE_LF_Y				10.0
#define PACE_LR_X				10.0//LF&RR
#define PACE_LR_Y				12.0
#define PACE_RF_X				10.0//RF&LR
#define PACE_RF_Y				12.0
#define PACE_RR_X				8.0//RF&LR
#define PACE_RR_Y				10.0





#define PACE_STEPLEN		100//80

#define FORWARDTIME		10//12
#define BACKTIME			10
void PaceOneStep(struct Vector2 move,double rotate)
{
	struct Step leftFront[2];
	struct Step rightFront[2];
	struct Step leftRear[2];
	struct Step rightRear[2];
	struct Step* animation[4] = {leftFront,leftRear,rightFront,rightRear};

	leftFront[0].duration = FORWARDTIME;
	leftFront[0].procID = PACE;
	leftFront[0].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0+PACE_LF_X) - sin(rotate)*(DOG_LENGTH / 2.0-PACE_LF_Y)+move.x*PACE_STEPLEN)/2.0-PACE_LF_X;
	leftFront[0].targetPos.y = ((sin(rotate))*-(DOG_WIDTH / 2.0+PACE_LF_X) + (cos(rotate) - 1.0)*(DOG_LENGTH / 2.0-PACE_LF_Y)+move.y*PACE_STEPLEN)/2.0-PACE_LF_Y;
	leftFront[0].targetPos.z = 0;
	
	leftFront[1].duration = BACKTIME;
	leftFront[1].procID = LERP;
	leftFront[1].targetPos.x = ((cos(rotate) - 1.0)*-(DOG_WIDTH / 2.0+PACE_LF_X) - sin(rotate)*(DOG_LENGTH / 2.0-PACE_LF_Y)+move.x*PACE_STEPLEN)/-2.0-PACE_LF_X;
	leftFront[1].targetPos.y = ((sin(rotate))*-(DOG_WIDTH / 2.0+PACE_LF_X) + (cos(rotate) - 1.0)*(DOG_LENGTH / 2.0-PACE_LF_Y)+move.y*PACE_STEPLEN)/-2.0-PACE_LF_Y;
	leftFront[1].targetPos.z = 0;

	
	leftRear[0].duration = BACKTIME;
	leftRear[0].procID = LERP;
	leftRear[0].targetPos.x = ((cos(rotate) - 1.0)*-(DOG_WIDTH / 2.0-PACE_LR_X) - sin(rotate)*-(DOG_LENGTH / 2.0+PACE_LR_Y)+move.x*PACE_STEPLEN)/-2.0+PACE_LR_X;
	leftRear[0].targetPos.y = ((sin(rotate))*-(DOG_WIDTH / 2.0-PACE_LR_X) + (cos(rotate) - 1.0)*-(DOG_LENGTH / 2.0+PACE_LR_Y)+move.y*PACE_STEPLEN)/-2.0-PACE_LR_Y;
	leftRear[0].targetPos.z = 0;

	leftRear[1].duration = FORWARDTIME;
	leftRear[1].procID = PACE;
	leftRear[1].targetPos.x = ((cos(rotate) - 1.0)*-(DOG_WIDTH / 2.0-PACE_LR_X) - sin(rotate)*-(DOG_LENGTH / 2.0+PACE_LR_Y)+move.x*PACE_STEPLEN)/2.0+PACE_LR_X;
	leftRear[1].targetPos.y = ((sin(rotate))*-(DOG_WIDTH / 2.0-PACE_LR_X)+ (cos(rotate) - 1.0)*-(DOG_LENGTH / 2.0+PACE_LR_Y)+move.y*PACE_STEPLEN)/2.0-PACE_LR_Y;
	leftRear[1].targetPos.z = 0;

	rightFront[0].duration = BACKTIME;
	rightFront[0].procID = LERP;
	rightFront[0].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0+PACE_RF_X) - sin(rotate)*(DOG_LENGTH / 2.0-PACE_RF_Y)+move.x*PACE_STEPLEN)/-2.0+PACE_RF_X;
	rightFront[0].targetPos.y = ((sin(rotate))*(DOG_WIDTH / 2.0+PACE_RF_X) + (cos(rotate) - 1)*(DOG_LENGTH / 2.0-PACE_RF_Y)+move.y*PACE_STEPLEN)/-2.0-PACE_RF_Y;
	rightFront[0].targetPos.z = 0;

	rightFront[1].duration = FORWARDTIME;
	rightFront[1].procID = PACE;
	rightFront[1].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0+PACE_RF_X) - sin(rotate)*(DOG_LENGTH / 2.0-PACE_RF_Y)+move.x*PACE_STEPLEN)/2.0+PACE_RF_X;
	rightFront[1].targetPos.y = ((sin(rotate))*(DOG_WIDTH / 2.0+PACE_RF_X) + (cos(rotate) - 1.0)*(DOG_LENGTH / 2.0-PACE_RF_Y)+move.y*PACE_STEPLEN)/2.0-PACE_RF_Y;	
	rightFront[1].targetPos.z = 0;

	rightRear[0].duration = FORWARDTIME;
	rightRear[0].procID = PACE;
	rightRear[0].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0-PACE_RR_X) - sin(rotate)*-(DOG_LENGTH / 2.0+PACE_RR_Y)+move.x*PACE_STEPLEN)/2.0-PACE_RR_X;
	rightRear[0].targetPos.y = ((sin(rotate))*(DOG_WIDTH / 2.0-PACE_RR_X) + (cos(rotate) - 1.0)*-(DOG_LENGTH / 2.0+PACE_RR_Y)+move.y*PACE_STEPLEN)/2.0-PACE_RR_Y;
	rightRear[0].targetPos.z = 0;

	rightRear[1].duration = BACKTIME;
	rightRear[1].procID = LERP;
	rightRear[1].targetPos.x = ((cos(rotate) - 1.0)*(DOG_WIDTH / 2.0-PACE_RR_X) - sin(rotate)*-(DOG_LENGTH / 2.0+PACE_RR_Y)+move.x*PACE_STEPLEN)/-2.0-PACE_RR_X;
	rightRear[1].targetPos.y = ((sin(rotate))*(DOG_WIDTH / 2.0-PACE_RR_X) + (cos(rotate) - 1)*-(DOG_LENGTH / 2.0+PACE_RR_Y)+move.y*PACE_STEPLEN)/-2.0-PACE_RR_Y;
	rightRear[1].targetPos.z = 0;
	
	for(int i = 0;i<4;i++)
	{
		//USART_printf(USART1,"idle feet[%d] = {%lf,%lf,%lf}\n",i,idlePos[i].x,idlePos[i].y,idlePos[i].z);
		
		for(int j = 0;j<2;j++)
		{
			animation[i][j].targetPos.x += idlePos[i].x;
			animation[i][j].targetPos.y += idlePos[i].y;
			animation[i][j].targetPos.z += idlePos[i].z;
			
		}
	}
	//StartTick();
	doMovement(animation,2);
	//USART_printf(USART1,"time using=%dms\n",EndTick());	
}


/*
void ClawOneStep(struct Vector2 move,double rotate)
{
	struct Step leftFront[4];
	struct Step rightFront[4];
	struct Step leftRear[4];
	struct Step rightRear[4];
	struct Step* animation[4] = {leftFront,leftRear,rightFront,rightRear};

	leftFront[0].duration = 12;
	leftFront[0].procID = QUALERP;
	leftFront[0].targetPos.x = ((cos(rotate) - 1.0)*DOG_WIDTH / 2.0 - sin(rotate)*DOG_LENGTH / 2.0+move.x*MOVESTEP_LEN)/2.0;
	leftFront[0].targetPos.y = ((sin(rotate))*-DOG_WIDTH / 2.0 + (cos(rotate) - 1.0)*DOG_LENGTH / 2.0+move.y*MOVESTEP_LEN)/2.0;
	leftFront[0].targetPos.z = 0;
	
	leftFront[1].duration = 12;
	leftFront[1].procID = NULL;

	leftFront[2].duration = 12;
	leftFront[2].procID = NULL;

	leftFront[3].duration = 12;
	leftFront[3].procID = LERP;
	leftFront[3].targetPos.x = ((cos(rotate) - 1.0)*-DOG_WIDTH / 2.0 - sin(rotate)*DOG_LENGTH / 2.0+move.x*MOVESTEP_LEN)/-2.0;
	leftFront[3].targetPos.y = ((sin(rotate))*-DOG_WIDTH / 2.0 + (cos(rotate) - 1.0)*DOG_LENGTH / 2.0+move.y*MOVESTEP_LEN)/-2.0;
	leftFront[3].targetPos.z = 0;


	leftRear[0].duration = 12;
	leftRear[0].procID = LERP;
	leftRear[0].targetPos.x = 0;
	leftRear[0].targetPos.y = 0;
	leftRear[0].targetPos.z = 5;

	leftRear[1].duration = 12;
	leftRear[1].procID = NULL;

	leftRear[2].duration = 12;
	leftRear[2].procID = QUALERP;
	leftRear[2].targetPos.x = ((cos(rotate) - 1.0)*-DOG_WIDTH / 2.0 - sin(rotate)*-DOG_LENGTH / 2.0+move.x*MOVESTEP_LEN)/2.0;
	leftRear[2].targetPos.y = ((sin(rotate))*-DOG_WIDTH / 2.0 + (cos(rotate) - 1.0)*-DOG_LENGTH / 2.0+move.y*MOVESTEP_LEN)/2.0;
	leftRear[2].targetPos.z = 0;

	leftRear[3].duration = 12;
	leftRear[3].procID = LERP;
	leftRear[3].targetPos.x = ((cos(rotate) - 1.0)*-DOG_WIDTH / 2.0 - sin(rotate)*-DOG_LENGTH / 2.0+move.x*MOVESTEP_LEN)/-2.0;
	leftRear[3].targetPos.y = ((sin(rotate))*-DOG_WIDTH / 2.0 + (cos(rotate) - 1.0)*-DOG_LENGTH / 2.0+move.y*MOVESTEP_LEN)/-2.0;
	leftRear[3].targetPos.z = 0;


	rightFront[0].duration = 12;
	rightFront[0].procID = NULL;


	rightFront[1].duration = 12;
	rightFront[1].procID = QUALERP;
	rightFront[1].targetPos.x = ((cos(rotate) - 1.0)*DOG_WIDTH / 2.0 - sin(rotate)*DOG_LENGTH / 2.0+move.x*MOVESTEP_LEN)/2.0;
	rightFront[1].targetPos.y = ((sin(rotate))*DOG_WIDTH / 2.0 + (cos(rotate) - 1.0)*DOG_LENGTH / 2.0+move.y*MOVESTEP_LEN)/2.0;	
	rightFront[1].targetPos.z = 0;

	rightFront[2].duration = 12;
	rightFront[2].procID = NULL;

	rightFront[3].duration = 12;
	rightFront[3].procID = LERP;	
	rightFront[3].targetPos.x = ((cos(rotate) - 1.0)*DOG_WIDTH / 2.0 - sin(rotate)*DOG_LENGTH / 2+move.x*MOVESTEP_LEN)/-2.0;
	rightFront[3].targetPos.y = ((sin(rotate))*DOG_WIDTH / 2.0 + (cos(rotate) - 1)*DOG_LENGTH / 2+move.y*MOVESTEP_LEN)/-2.0;
	rightFront[3].targetPos.z = 0;

	rightRear[0].duration = 12;
	rightRear[0].procID = LERP;
	rightRear[0].targetPos.x = 0;
	rightRear[0].targetPos.y = 0;
	rightRear[0].targetPos.z = 5;

	rightRear[1].duration = 12;
	rightRear[1].procID = NULL;

	rightRear[2].duration = 12;
	rightRear[2].procID = NULL;	

	rightRear[3].duration = 12;
	rightRear[3].procID = QUALERP;	
	rightRear[3].targetPos.x = ((cos(rotate) - 1.0)*DOG_WIDTH / 2.0 - sin(rotate)*-DOG_LENGTH / 2.0+move.x*MOVESTEP_LEN)/-2.0;
	rightRear[3].targetPos.y = ((sin(rotate))*DOG_WIDTH / 2.0 + (cos(rotate) - 1)*-DOG_LENGTH / 2.0+move.y*MOVESTEP_LEN)/-2.0;
	rightRear[3].targetPos.z = 0;


	for(int i = 0;i<4;i++)
	{
		for(int j = 0;j<4;j++)
		{
			animation[i][j].targetPos.x += idlePos[i].x;
			animation[i][j].targetPos.y += idlePos[i].y;
			animation[i][j].targetPos.z = idlePos[i].z;					
		}
	}	
	doMovement(animation,4);
}
*/

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


/*
#define BLANK_GAP		5
void TriPaceOneStep(struct Vector2 move,double rotate)
{
	
	struct Step LF[6];
	struct Step rightFront[6];
	struct Step leftRear[6];
	struct Step rightRear[6];
	struct Step* animation[4] = {leftFront,leftRear,rightFront,rightRear};

	leftFront[0].duration = BLANK_GAP;
	leftFront[0].procID = NULL;	
	
	leftFront[1].duration = 12;
	leftFront[1].procID = QUALERP;
	leftFront[1].targetPos.x = ((cos(rotate) - 1.0)*DOG_WIDTH / 2.0 - sin(rotate)*DOG_LENGTH / 2.0+move.x*MOVESTEP_LEN)/2.0;
	leftFront[1].targetPos.y = ((sin(rotate))*-DOG_WIDTH / 2.0 + (cos(rotate) - 1.0)*DOG_LENGTH / 2.0+move.y*MOVESTEP_LEN)/2.0;
	
	
	leftFront[2].duration = BLANK_GAP;
	leftFront[2].procID = NULL;	
	
	leftFront[3].duration = 12;
	leftFront[3].procID = LERP;
	leftFront[3].targetPos.x = ((cos(rotate) - 1.0)*-DOG_WIDTH / 2.0 - sin(rotate)*DOG_LENGTH / 2.0+move.x*MOVESTEP_LEN)/-2.0;
	leftFront[3].targetPos.y = ((sin(rotate))*-DOG_WIDTH / 2.0 + (cos(rotate) - 1.0)*DOG_LENGTH / 2.0+move.y*MOVESTEP_LEN)/-2.0;

	
	leftRear[0].duration = BLANK_GAP;
	leftRear[0].procID = NULL;		
	
	leftRear[1].duration = 12;
	leftRear[1].procID = LERP;
	leftRear[1].targetPos.x = ((cos(rotate) - 1.0)*-DOG_WIDTH / 2.0 - sin(rotate)*-DOG_LENGTH / 2.0+move.x*MOVESTEP_LEN)/-2.0;
	leftRear[1].targetPos.y = ((sin(rotate))*-DOG_WIDTH / 2.0 + (cos(rotate) - 1.0)*-DOG_LENGTH / 2.0+move.y*MOVESTEP_LEN)/-2.0;

	leftRear[2].duration = BLANK_GAP;
	leftRear[2].procID = NULL;		
	
	leftRear[3].duration = 12;
	leftRear[3].procID = QUALERP;
	leftRear[3].targetPos.x = ((cos(rotate) - 1.0)*-DOG_WIDTH / 2.0 - sin(rotate)*-DOG_LENGTH / 2.0+move.x*MOVESTEP_LEN)/2.0;
	leftRear[3].targetPos.y = ((sin(rotate))*-DOG_WIDTH / 2.0 + (cos(rotate) - 1.0)*-DOG_LENGTH / 2.0+move.y*MOVESTEP_LEN)/2.0;

	rightFront[0].duration = BLANK_GAP;
	rightFront[0].procID = NULL;
	
	rightFront[1].duration = 12;
	rightFront[1].procID = LERP;
	rightFront[1].targetPos.x = ((cos(rotate) - 1.0)*DOG_WIDTH / 2.0 - sin(rotate)*DOG_LENGTH / 2+move.x*MOVESTEP_LEN)/-2.0;
	rightFront[1].targetPos.y = ((sin(rotate))*DOG_WIDTH / 2.0 + (cos(rotate) - 1)*DOG_LENGTH / 2+move.y*MOVESTEP_LEN)/-2.0;

	rightFront[2].duration = 12;
	rightFront[2].procID = QUALERP;
	rightFront[2].targetPos.x = ((cos(rotate) - 1.0)*DOG_WIDTH / 2.0 - sin(rotate)*DOG_LENGTH / 2.0+move.x*MOVESTEP_LEN)/2.0;
	rightFront[2].targetPos.y = ((sin(rotate))*DOG_WIDTH / 2.0 + (cos(rotate) - 1.0)*DOG_LENGTH / 2.0+move.y*MOVESTEP_LEN)/2.0;	

	rightFront[3].duration = 3;
	rightFront[3].procID = NULL;

	rightRear[0].duration = 12;
	rightRear[0].procID = QUALERP;
	rightRear[0].targetPos.x = ((cos(rotate) - 1.0)*DOG_WIDTH / 2.0 - sin(rotate)*-DOG_LENGTH / 2.0+move.x*MOVESTEP_LEN)/2.0;
	rightRear[0].targetPos.y = ((sin(rotate))*DOG_WIDTH / 2.0 + (cos(rotate) - 1.0)*-DOG_LENGTH / 2.0+move.y*MOVESTEP_LEN)/2.0;

	rightRear[1].duration = BLANK_GAP;
	rightRear[1].procID = NULL;

	rightRear[2].duration = BLANK_GAP;
	rightRear[2].procID = NULL;

	rightRear[3].duration = 12;
	rightRear[3].procID = LERP;
	rightRear[3].targetPos.x = ((cos(rotate) - 1.0)*DOG_WIDTH / 2.0 - sin(rotate)*-DOG_LENGTH / 2.0+move.x*MOVESTEP_LEN)/-2.0;
	rightRear[3].targetPos.y = ((sin(rotate))*DOG_WIDTH / 2.0 + (cos(rotate) - 1)*-DOG_LENGTH / 2.0+move.y*MOVESTEP_LEN)/-2.0;
	
	for(int i = 0;i<4;i++)
	{
		//USART_printf(USART1,"idle feet[%d] = {%lf,%lf,%lf}\n",i,idlePos[i].x,idlePos[i].y,idlePos[i].z);
		
		for(int j = 0;j<4;j++)
		{

			animation[i][j].targetPos.x += idlePos[i].x;
			animation[i][j].targetPos.y += idlePos[i].y;
			animation[i][j].targetPos.z = idlePos[i].z;			
		}
	}
	//StartTick();
	doMovement(animation,4);	
}
*/

void StopAndClaw(double lean)
{
	struct Step LF[3];
	struct Step RF[3];
	struct Step LR[3];
	struct Step RR[3];
	struct Step* animation[4] = {LF,LR,RF,RR};
	
	LF[0].duration = 20;
	LF[0].procID = PACE;
	LF[0].targetPos.x = 0;
	LF[0].targetPos.y = CLAWSTEP_YLEN/2.0;
	
	RR[0].duration = 20;
	RR[0].procID = PACE;
	RR[0].targetPos.x = 0;
	RR[0].targetPos.y = 0;
	
	LR[0].duration = 20;
	LR[0].procID = NULL;
	
	RF[0].duration = 20;
	RF[0].procID = NULL;
	
	
	LF[1].duration = 20;
	LF[1].procID = NULL;
	
	RR[1].duration = 20;
	RR[1].procID = NULL;	
	
	LR[1].duration = 20;
	LR[1].procID = PACE;
	LR[1].targetPos.x = 0;
	LR[1].targetPos.y = CLAWSTEP_YLEN/2.0;
	
	RF[1].duration = 20;
	RF[1].procID = PACE;
	RF[1].targetPos.x = 0;
	RF[1].targetPos.y = 0;

	LF[2].duration = 20;
	LF[2].procID = LINE;
	LF[2].targetPos.x = CLAW_OFFXR;
	LF[2].targetPos.y = CLAWSTEP_YLEN/2.0;
	
	RR[2].duration = 20;
	RR[2].procID = LINE;
	RR[2].targetPos.x = +CLAW_OFFXR;
	RR[2].targetPos.y = 0;	

	LR[2].duration = 20;
	LR[2].procID = LINE;
	LR[2].targetPos.x = CLAW_OFFXR;
	LR[2].targetPos.y = CLAWSTEP_YLEN/2.0;
	
	RF[2].duration = 20;
	RF[2].procID = LINE;
	RF[2].targetPos.x = +CLAW_OFFXR;
	RF[2].targetPos.y = 0;	
	
	for(int i = 0;i<4;i++)
	{
		for(int j = 0;j<3;j++)
		{
			animation[i][j].targetPos.x += idlePos[i].x;
			animation[i][j].targetPos.y += idlePos[i].y;
			animation[i][j].targetPos.z = idlePos[i].z;
			if(j == 2)
			{
				animation[i][j].targetPos.z += idlePos[i].z*(1.0/cos(lean/2.0)-1.0)+sin(lean/2.0)*animation[i][j].targetPos.y;							
				animation[i][j].targetPos.y += idlePos[i].y+idlePos[0].z*sin(lean);
			}
		}
	}	

	doMovement(animation,3);	
}

void StopImmediately()
{
	moveExit = 1;
}


