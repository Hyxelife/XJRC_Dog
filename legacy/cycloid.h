#ifndef CYCLOID2_H_INCLUDED
#define CYCLOID2_H_INCLUDED
typedef struct
{
	int statenow;//
	int statepre;//上一个状态
	int trotstate[4];// 1 支撑 0 摆动
}legstate11;
typedef struct
{
    float reax;
    float reaz;
    float reay;
    float expx;
    float expz;
    float expy;
    float heightcontrol;
    float route_prex;
}Trotleg;
extern int dirstate[4];
extern legstate11 legstate1;
extern Trotleg  trotleg[4];
extern float trot_cg[4];
extern float movecontrol[3];
extern float deltaz[4];
void cycloid(int legnumber,float t,int legstate,float xs,float xf,float ges_z,float h,float y);
void square(int legnumber,float t,float *pos,float xs,float xf,float ges_z,float h,float y);
#endif // CYCLOID2_H_INCLUDED
