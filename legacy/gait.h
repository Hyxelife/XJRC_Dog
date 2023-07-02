// #include "usart.h"
#ifndef DOG_MOVE_H
#define DOG_MOVE_H

//Unit cm
//physics params
#define TOPL        48.1
#define MIDL        95.0
#define FEETL       95.0

#define MID_LEVER1			2.6
#define MID_LEVER2			6.6
#define MID_FEET_LEVER	2.4
#define MID_FEET_JOINT	6.4
#define MID_MOTOR_AXIS	3.0

#define DOG_WIDTH       94.0
#define DOG_LENGTH      246.4

//motion params

#define APPLY_WAITTIME	25




struct Vector2
{
    double x,y;
};

struct Vector3
{
    double x,y,z;
};



void Initialize(struct Vector3 pos_RF);
void SetLeanPos(double lean);//from joint
void StopAndStand();
void StopAndClaw(double lean);


//void BlendPaceOneStep(struct Vector2 move,double rotate,double lean);
void PaceOneStep(struct Vector2 move,double rotate);
void PaceOneStepObs(struct Vector2 move,double rotate);
void PaceOneStepSlow(struct Vector2 move,double rotate);

//void ClawOneStep(struct Vector2 move,double rotate);
void VerticalAdjOneStep(double vertical,double antiClock);
void ClawFwdOneStep(double lean,double stepSteep);
void StopImmediately();
//void TriPaceOneStep(struct Vector2 move,double rotate);


#endif
