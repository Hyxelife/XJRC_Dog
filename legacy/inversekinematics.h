#ifndef INVERSEKINEMATICS_H
#define INVERSEKINEMATICS_H

#define L2 0.220//大腿长
#define L3 0.222//小腿长
#define L1 0.0988//髋到大腿 y方向偏移量

typedef struct {//存放位置x,y,z的结构体
    float z;
    float y;
    float x;
    int leg;
}position_xyz;
typedef struct {//存放位置腿三个角度的结构体
    float alfa;
    float beta;
    float gama;
    int leg;
}position_angel;

void inversekinematics(position_xyz *leg1,position_angel *leg2);
void motor_control(position_angel angel1,position_angel angel2,position_angel angel3,position_angel angel4);
void motor_control_high( position_angel angel1,position_angel angel2,position_angel angel3,position_angel angel4);

void motorcontrol(position_angel angel,int legnumber,float K_origin_pos,float D_origin_pos,float X_origin_pos);



#endif
