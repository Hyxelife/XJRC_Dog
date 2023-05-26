#include "all.h"
#define Ts  4
#define fai  0.5
float t;
float deta_t;
float mode[4];
Expect my_Exp_leg[4];
Real Real_leg[4];
Pre_real Pre_real_leg[4];
Pre_exp Pre_exp_leg[4];
struct uuu uuu[4];
struct real_angle real_angle[4];

void Init()
{
    deta_t = 0.05;
    int legnumber;
    for (legnumber = 0; legnumber < 4; legnumber++)
    {
        Real_leg[legnumber].real_x = 0;
        Real_leg[legnumber].real_y = 0;
        Real_leg[legnumber].real_z = -HHH;
    }


}

void stand_init()//站立初始化
{
    deta_t = 0.05;
    int legnumber;
    float zzz;//z坐标
    for (t = 0; t < 2; t += deta_t)
    {
        zzz = -HHH + HHH / 2 * t;
        for (legnumber = 0; legnumber < 4; legnumber++)
        {
            my_Exp_leg[legnumber].expx = 0;
            my_Exp_leg[legnumber].expy = 0;
            my_Exp_leg[legnumber].expz = zzz;

        }
        Readstate();
        vmc();
    }

}

void Walk()
{

    float x1 = 0;
    float x2 = 0;
    float x3 = 0;
    float x4 = 0;
    float z1 = 0;
    float z2 = 0;
    float z3 = 0;
    float z4 = 0;

    deta_t = 0.008;
    for (t = 0; t <= Ts; t += deta_t)
    {

        float sigma = 0;
        if (t <= Ts * fai)                                                    //前0.5个周期
        {

            mode[0] = 1;//摆动
            mode[1] = 0;//支撑
            mode[2] = 1;
            mode[3] = 0;
            float sigma = 2 * pi * t / (fai * Ts);
            float xep_b = (my_xf - my_xs) * ((sigma - sin(sigma)) / (2 * pi)) + my_xs;    //摆动项x坐标
            float xep_z = (my_xs - my_xf) * ((sigma - sin(sigma)) / (2 * pi)) + my_xf;    //支撑项
            float zep = hhh * (1 - cos(sigma)) / 2;  //摆动项z
            my_Exp_leg[0].expx = xep_b * r1;
            my_Exp_leg[1].expx = xep_z * r2;
            my_Exp_leg[2].expx = xep_b * r3;
            my_Exp_leg[3].expx = xep_z * r4;
            my_Exp_leg[0].expz = zep - HHH;
            my_Exp_leg[1].expz = -HHH;
            my_Exp_leg[2].expz = zep - HHH;
            my_Exp_leg[3].expz = -HHH;
            //y坐标暂时没用到
            my_Exp_leg[0].expy = 0;
            my_Exp_leg[1].expy = 0;
            my_Exp_leg[2].expy = 0;
            my_Exp_leg[3].expy = 0;
            ///printf("期望 x: %f   z:%f\n",my_Exp_leg[1].expx, my_Exp_leg[1].expz);

            Readstate();




        }
        else
        {
            mode[0] = 0;//支撑
            mode[1] = 1;//摆动
            mode[2] = 0;
            mode[3] = 1;
            sigma = 2 * pi * (t - fai * Ts) / (fai * Ts);

            float xep_b = (my_xf - my_xs) * ((sigma - sin(sigma)) / (2 * pi)) + my_xs;
            float xep_z = (my_xs - my_xf) * ((sigma - sin(sigma)) / (2 * pi)) + my_xf;
            float zep = hhh * (1 - cos(sigma)) / 2;
            my_Exp_leg[0].expx = xep_z * r1;                                                     //以下8行相当于是决定每个腿怎么转
            my_Exp_leg[1].expx = xep_b * r2;
            my_Exp_leg[2].expx = xep_z * r3;
            my_Exp_leg[3].expx = xep_b * r4;
            my_Exp_leg[0].expz = -HHH;
            my_Exp_leg[1].expz = zep - HHH;
            my_Exp_leg[2].expz = -HHH;
            my_Exp_leg[3].expz = zep - HHH;
            //y坐标暂时没用到
            my_Exp_leg[0].expy = 0;
            my_Exp_leg[1].expy = 0;
            my_Exp_leg[2].expy = 0;
            my_Exp_leg[3].expy = 0;
            Readstate();

        }
        vmc();
    }

}

void Readstate()
{
    int legnumber;    //腿的编号
    //read_angle();
    for (legnumber = 0; legnumber < 4; legnumber++)
    {

        //以下理解有待商榷
        Pre_real_leg[legnumber].pre_x = Real_leg[legnumber].real_x;
        Pre_real_leg[legnumber].pre_y = Real_leg[legnumber].real_y;
        Pre_real_leg[legnumber].pre_z = Real_leg[legnumber].real_z;

        /**********************************************************/
        //Real_leg[i]  里面的值是当前时刻反馈读取的

        forward_kinematics(legnumber);

        /**********************************************************/

        //printf("%f\n", Real_leg[i].real_z);
        //当前期望位置 - 上一时刻真实位置（now） or 当前期望位置 - 上一时刻期望位置 P67  ???
        Calculate_exp_Speed(Pre_exp_leg[legnumber].exp_x, my_Exp_leg[legnumber].expx,
            Pre_exp_leg[legnumber].exp_y, my_Exp_leg[legnumber].expy,
            Pre_exp_leg[legnumber].exp_z, my_Exp_leg[legnumber].expz, legnumber);

        Pre_exp_leg[legnumber].exp_x = my_Exp_leg[legnumber].expx;
        Pre_exp_leg[legnumber].exp_y = my_Exp_leg[legnumber].expy;
        Pre_exp_leg[legnumber].exp_z = my_Exp_leg[legnumber].expz;
        Calculate_real_Speed(Pre_real_leg[legnumber].pre_x, Real_leg[legnumber].real_x,
            Pre_real_leg[legnumber].pre_y, Real_leg[legnumber].real_y,
            Pre_real_leg[legnumber].pre_z, Real_leg[legnumber].real_z, legnumber);

        //该时刻真实位置 - 上一时刻真实位置

        if (legnumber == 0)
        {
            if (mode[0] == 0) { printf("支撑\n"); }
            else { printf("摆动\n"); }

            printf("z坐标期望位置：%f   实际位置：%f\n", uuu[legnumber].exp_shift.z, Real_leg[legnumber].real_z);
            printf("z坐标期望速度：%f   实际速度：%f\n", uuu[legnumber].exp_speed.z, uuu[legnumber].real_speed.z);
            printf("x坐标期望位置：%f   实际位置：%f\n", uuu[legnumber].exp_shift.x, Real_leg[legnumber].real_x);
            printf("x坐标期望速度：%f   实际速度：%f\n", uuu[legnumber].exp_speed.x, uuu[legnumber].real_speed.x);
            printf("z坐标实际位置：%f\n",Real_leg[legnumber].real_y);

            //  printf("fff\n");
        }    //这里没有计算y  所以y方向设为0

    }

}

//计算期望速度位移
void Calculate_exp_Speed(float x1, float x2, float y1, float y2, float z1, float z2, int legnumber)
{
    if (mode[legnumber] == 1)    //摆动
    {
        uuu[legnumber].exp_speed.x = 0;
        uuu[legnumber].exp_speed.y = 0;
        uuu[legnumber].exp_speed.z = 0;
        uuu[legnumber].exp_shift.x = my_Exp_leg[legnumber].expx;
        uuu[legnumber].exp_shift.y = my_Exp_leg[legnumber].expy;
        uuu[legnumber].exp_shift.z = my_Exp_leg[legnumber].expz;

    }
    else
    {
        uuu[legnumber].exp_speed.x = -3;
        uuu[legnumber].exp_speed.y = 0;
        uuu[legnumber].exp_speed.z = 0;
        uuu[legnumber].exp_shift.x = 0;
        uuu[legnumber].exp_shift.y = 0;
        uuu[legnumber].exp_shift.z = -HHH;
    }
}

//计算真实速度位移
void Calculate_real_Speed(float x1, float x2, float y1, float y2, float z1, float z2, int legnumber)
{

    uuu[legnumber].real_speed.x = (x2 - x1) / deta_t;
    uuu[legnumber].real_speed.y = (y2 - y1) / deta_t;
    uuu[legnumber].real_speed.z = (z2 - z1) / deta_t;
    uuu[legnumber].real_shift.x = Real_leg[legnumber].real_x;
    uuu[legnumber].real_shift.y = Real_leg[legnumber].real_y;
    uuu[legnumber].real_shift.z = Real_leg[legnumber].real_z;
    if (mode[legnumber] == 0)
    {
        uuu[legnumber].real_speed.x = -uuu[legnumber].real_speed.x;

    }

}

//正运动学求解
void forward_kinematics(int legnumber)
{
    read_angle();//注意这里读取了很多次
    get_position(legnumber);

}

//读取当前三个电机角度
void read_angle()
{//标定值+读取角度
    //real_angle[0].motor[2] = K1_origion_pos + a1.Pos;  //左前腿
    //real_angle[0].motor[0] = D1_origion_pos + a2.Pos;
    //real_angle[0].motor[1] = X1_origion_pos + a3.Pos;
    /*real_angle[1].motor[2] = K2_origion_pos + a4.Pos;  //右前
    real_angle[1].motor[0] = D2_origion_pos + a5.Pos;
    real_angle[1].motor[1] = X2_origion_pos + a6.Pos;
    real_angle[2].motor[2] = K3_origion_pos + a7.Pos;  //右后
    real_angle[2].motor[0] = D3_origion_pos + a8.Pos;
    real_angle[2].motor[1] = X3_origion_pos + a9.Pos;
    real_angle[3].motor[2] = K4_origion_pos + a10.Pos;  //左后
    real_angle[3].motor[0] = D4_origion_pos + a11.Pos;
    real_angle[3].motor[1] = X4_origion_pos + a12.Pos;*/
}


MOTOR_recv a2;
MOTOR_recv a3;


//正运动学求坐标
void get_position(int legnumber)
{


    float faai = real_angle[legnumber].motor[0];//大腿
    float theta = real_angle[legnumber].motor[1];//小腿
    float gamma = real_angle[legnumber].motor[2];//髋
    if (legnumber == 1)//右前腿
    {
        Real_leg[legnumber].real_x = l1 * sin(faai) + l2 * sin(faai + theta);
        Real_leg[legnumber].real_y = l3 * cos(gamma) - sin(gamma) * (l1 * cos(faai) + l2 * cos(faai + theta));
        Real_leg[legnumber].real_z = -l3 * sin(gamma) - cos(gamma) * (l1 * cos(faai) + l2 * cos(faai + theta));
        //printf(" x: %f  z:   %f",Real_leg[legnumber].real_x,Real_leg[legnumber].real_z);

    }
    else if (legnumber == 2)//右后腿
    {
        Real_leg[legnumber].real_x = l1 * sin(faai) + l2 * sin(faai + theta);
        Real_leg[legnumber].real_y = l3 * cos(gamma) + sin(gamma) * (l1 * cos(faai) + l2 * cos(faai + theta));
        Real_leg[legnumber].real_z = l3 * sin(gamma) - cos(gamma) * (l1 * cos(faai) + l2 * cos(faai + theta));
    }
    else if (legnumber == 0)//左前腿
    {
        Real_leg[legnumber].real_x = -l1 * sin(faai) - l2 * sin(faai + theta);
        Real_leg[legnumber].real_y = l3 * cos(gamma) + sin(gamma) * (l1 * cos(faai) + l2 * cos(faai + theta));
        Real_leg[legnumber].real_z = l3 * sin(gamma) - cos(gamma) * (l1 * cos(faai) + l2 * cos(faai + theta));
    }
    else//左后腿
    {
        Real_leg[legnumber].real_x = -l1 * sin(faai) - l2 * sin(faai + theta);
        Real_leg[legnumber].real_y = l3 * cos(gamma) - sin(gamma) * (l1 * cos(faai) + l2 * cos(faai + theta));
        Real_leg[legnumber].real_z = -l3 * sin(gamma) - cos(gamma) * (l1 * cos(faai) + l2 * cos(faai + theta));
    }
 if(Real_leg[0].real_x>=0.15||Real_leg[0].real_x<=-0.15)
    {
    while(1)
    {
    printf("position_x error\n");
    a2=torque_control(1,1,0);
    a3=torque_control(1,2,0);
    }
    }
if(Real_leg[0].real_z>=-0.15||Real_leg[0].real_x<=-0.4)
    {
    while(1)
    {
    printf("position_z error\n");
    a2=torque_control(1,1,0);
    a3=torque_control(1,2,0);
    }
    }

}
