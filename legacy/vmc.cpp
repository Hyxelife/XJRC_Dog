#include"all.h"
Jacobian Jacobian[4];
moment_force leg_moment_force[4];
force leg_force[4];
K KKK[4];
B BBB[4];
extern MOTOR_recv a2;
extern MOTOR_recv a3;
int T_warning0;
int T_warning1;
//using:set_torque(a1,a2,a3)---a1 a2 a3 return value    MOTOR_recv& a1,MOTOR_recv&a2,MOTOR_recv&a3
void set_torque()
{
    //MOTOR_recv a1=torque_control(1,0,leg_moment_force[0].force_2*0);//左前腿 髋
if(leg_moment_force[0].force_0>=3)
    {
    T_warning0++;
    a2=torque_control(1,1,0);
    a3=torque_control(1,2,0);
    printf("small problem");
    while(T_warning0>2)
    {
    printf("error\n");
    a2=torque_control(1,1,0);
    a3=torque_control(1,2,0);
    }
    }
else if(leg_moment_force[0].force_1>=3)
    {
    T_warning1++;
    a2=torque_control(1,1,0);
    a3=torque_control(1,2,0);
    printf("small problem");
    while(T_warning1>2)
    {
    printf("error\n");
    a2=torque_control(1,1,0);
    a3=torque_control(1,2,0);
    }
    }
else
    {
    a2=torque_control(1,1,leg_moment_force[0].force_0*0);
    a3=torque_control(1,2,leg_moment_force[0].force_1*0);
    /*MOTOR_recv a4=torque_control(2,0,leg_moment_force[1].force_2*0)
    MOTOR_recv a5=torque_control(2,1,leg_moment_force[1].force_0*0)
    MOTOR_recv a6=torque_control(2,2,leg_moment_force[1].force_1*0)
    MOTOR_recv a7=torque_control(3,0,leg_moment_force[2].force_2*0)
    MOTOR_recv a8=torque_control(3,1,leg_moment_force[2].force_0*0)
    MOTOR_recv a9=torque_control(3,2,leg_moment_force[2].force_1*0)
    MOTOR_recv a10=torque_control(4,0,leg_moment_force[3].force_2*0)
    MOTOR_recv a11=torque_control(4,1,leg_moment_force[3].force_0*0)
    MOTOR_recv a12=torque_control(4,2,leg_moment_force[3].force_1*0)*/
    real_angle[0].motor[2]=0;
    //real_angle[0].motor[2] = (-K2_origin_pos + a1.Pos)/9.1;  //左前腿
    real_angle[0].motor[0] = (-D2_origin_pos + a2.Pos)/9.1;
    real_angle[0].motor[1] = (-X2_origin_pos + a3.Pos)/9.1;
    printf("大腿力矩:%f,小腿力矩：%f\n",leg_moment_force[0].force_0,leg_moment_force[0].force_1);
    //printf("大腿：%f 小腿：%f 髋：%f \n",real_angle[0].motor[0],real_angle[0].motor[1],real_angle[0].motor[2]);
}


usleep(80000);

}

void vmc()
{

    int legnumber = 0;
    for (legnumber = 0; legnumber < 4; legnumber++)
    {
        if (mode[legnumber] == 1)
        {
            KB_init(legnumber);
            vmc_force(legnumber);
            Get_Jacobian(legnumber);
            vmc_moment_force(legnumber);
        }
        else
        {
            KB_init(legnumber);
            vmc_force(legnumber);
            vmc_f1orce_correct(legnumber);
            Get_Jacobian(legnumber);
            vmc_moment_force(legnumber);
            vmc_force_motion_correct(legnumber);

        }


    }
set_torque();

}

void vmc_force(int legnumber)
{
    leg_force[legnumber].force_x = KKK[legnumber].x*(uuu[legnumber].exp_shift.x - uuu[legnumber].real_shift.x)+
                                   BBB[legnumber].x * (uuu[legnumber].exp_speed.x - uuu[legnumber].real_speed.x);
    leg_force[legnumber].force_y = KKK[legnumber].y * (uuu[legnumber].exp_shift.y - uuu[legnumber].real_shift.y) +
                                   BBB[legnumber].y * (uuu[legnumber].exp_speed.y - uuu[legnumber].real_speed.y);
    leg_force[legnumber].force_z = KKK[legnumber].z * (uuu[legnumber].exp_shift.z - uuu[legnumber].real_shift.z) +
                                   BBB[legnumber].z * (uuu[legnumber].exp_speed.z - uuu[legnumber].real_speed.z);



    //以下看上去挺合理的
    if(legnumber==0)
    {
    printf("理论方向:%f\n",uuu[legnumber].exp_shift.z - uuu[legnumber].real_shift.z);
    printf("xxleg:%d  %f\n", legnumber, leg_force[legnumber].force_x);
    //printf("yyleg:%d  %f\n", legnumber, leg_force[legnumber].force_y);
    printf("zzleg:%d  %f\n", legnumber, leg_force[legnumber].force_z);
    }//printf("角度:%d  %f\n", legnumber, atan(leg_force[legnumber].force_x/leg_force[legnumber].force_z)/3.14*180);
}

//阻尼系数和弹簧系数初始化  需要手动输入
void KB_init(int legnumber)
{
    if (mode[legnumber] == 0)   //支撑项
    {
        if (legnumber == 1 || legnumber == 0)//前腿
        {
            KKK[legnumber].x = 0;        // 支撑项KX 为0
            KKK[legnumber].y = 0;        // 0
            KKK[legnumber].z = -0.5;        // 负的
            BBB[legnumber].x = -2;        // 负的
            BBB[legnumber].y = 0;        // 0
            BBB[legnumber].z = -2;        // 负的
        }
        else
        {
            KKK[legnumber].x = -350*0;
            KKK[legnumber].y = 0;
            KKK[legnumber].z = -690*0;
            BBB[legnumber].x = -10*0;
            BBB[legnumber].y = 1*0;
            BBB[legnumber].z = -55*0;
        }
    }
    else
    {
        KKK[legnumber].x = 0.5;   //应该是正的
        KKK[legnumber].y = 0;   //0
        KKK[legnumber].z = 0.5;   //正的
        BBB[legnumber].x = 2;   //正的
        BBB[legnumber].y = 0;   //0
        BBB[legnumber].z = 2; //正的
    }
}

void Get_Jacobian(int legnumber)
{
    read_angle();//调用了一次读取  不调用应该也可
    float faai = real_angle[legnumber].motor[0];
    float theta = real_angle[legnumber].motor[1];
    float gamma = real_angle[legnumber].motor[2];//方便书写
    int a1 = 1;
    int a2 = 1;
    int a3 = 1;
    if (legnumber == 0)
    {
        Jacobian[legnumber].faai[0] = -l1 * cos(faai) - l2 * cos(faai + theta);
        Jacobian[legnumber].theta[0] = -l2 * cos(faai + theta);
        Jacobian[legnumber].gamma[0] = 0;
        Jacobian[legnumber].faai[1] = (-1) * sin(gamma) * (l1 * sin(faai) + l2 * sin(faai + theta));
        Jacobian[legnumber].theta[1] = (-1) * l2 * sin(gamma) * sin(faai + theta);
        Jacobian[legnumber].gamma[1] = cos(gamma) * (l1 * cos(faai) + l2 * cos(faai + theta)) - l3 * sin(gamma);
        Jacobian[legnumber].faai[2] = cos(gamma) * (l1 * sin(faai) + l2 * sin(faai + theta));//cos
        Jacobian[legnumber].theta[2] = l2 * cos(gamma) * sin(faai + theta);
        Jacobian[legnumber].gamma[2] = sin(gamma) * (l1 * cos(faai) + l2 * cos(faai + theta)) + l3 * cos(gamma);
    }
    else if (legnumber == 1)
    {
        Jacobian[legnumber].faai[0] = l1 * cos(faai) + l2 * cos(faai + theta);
        Jacobian[legnumber].theta[0] = l2 * cos(faai + theta);
        Jacobian[legnumber].gamma[0] = 0;
        Jacobian[legnumber].faai[1] = sin(gamma) * (l1 * sin(faai) + l2 * sin(faai + theta));
        Jacobian[legnumber].theta[1] = l2 * sin(gamma) * sin(faai + theta);
        Jacobian[legnumber].gamma[1] = (-1) * cos(gamma) * (l1 * sin(faai) + l2 * sin(faai + theta)) - l3 * sin(gamma);
        Jacobian[legnumber].faai[2] = cos(gamma) * (l1 * sin(faai) + l2 * sin(faai + theta));
        Jacobian[legnumber].theta[2] = l2 * cos(gamma) * sin(faai + theta);
        Jacobian[legnumber].gamma[2] = (sin(gamma)) * (l1 * cos(faai) + l2 * cos(faai + theta)) - l3 * cos(gamma);
    }

    else if (legnumber == 2)
    {
        Jacobian[legnumber].faai[0] = l1 * cos(faai) + l2 * cos(faai + theta);
        Jacobian[legnumber].theta[0] = l2 * cos(faai + theta);
        Jacobian[legnumber].gamma[0] = 0;
        Jacobian[legnumber].faai[1] = -sin(gamma) * (l1 * sin(faai) + l2 * sin(faai + theta));
        Jacobian[legnumber].theta[1] = -l2 * sin(gamma) * sin(faai + theta);
        Jacobian[legnumber].gamma[1] = cos(gamma) * (l1 * cos(faai) + l2 * cos(faai + theta)) - l3 * sin(gamma);
        Jacobian[legnumber].faai[2] = cos(gamma) * (l1 * sin(faai) + l2 * sin(faai + theta));
        Jacobian[legnumber].theta[2] = l2 * cos(gamma) * sin(faai + theta);
        Jacobian[legnumber].gamma[2] = (sin(gamma)) * (l1 * cos(faai) + l2 * cos(faai + theta)) + l3 * cos(gamma);
    }
    else
    {
        Jacobian[legnumber].faai[0] = -l1 * cos(faai) - l2 * cos(faai + theta);
        Jacobian[legnumber].theta[0] = -l2 * cos(faai + theta);
        Jacobian[legnumber].gamma[0] = 0;
        Jacobian[legnumber].faai[1] = sin(gamma) * (l1 * sin(faai) + l2 * sin(faai + theta));
        Jacobian[legnumber].theta[1] = l2 * sin(gamma) * sin(faai + theta);
        Jacobian[legnumber].gamma[1] = cos(gamma) * (l1 * cos(faai) + l2 * cos(faai + theta)) - l3 * sin(gamma);
        Jacobian[legnumber].faai[2] = cos(gamma) * (l1 * sin(faai) + l2 * sin(faai + theta));
        Jacobian[legnumber].theta[2] = l2 * cos(gamma) * sin(faai + theta);
        Jacobian[legnumber].gamma[2] = sin(gamma) * (l1 * cos(faai) + l2 * cos(faai + theta)) - l3 * cos(gamma);
    }



}

void vmc_moment_force(int legnumber)
{
    int j = 0;
    //矩阵乘法求力矩
    leg_moment_force[legnumber].force_0 = Jacobian[legnumber].faai[0] * leg_force[legnumber].force_x+
                                          Jacobian[legnumber].faai[1] * leg_force[legnumber].force_y+
                                          Jacobian[legnumber].faai[2] * leg_force[legnumber].force_z;

    leg_moment_force[legnumber].force_1 = Jacobian[legnumber].theta[0] * leg_force[legnumber].force_x +
                                          Jacobian[legnumber].theta[1] * leg_force[legnumber].force_y +
                                          Jacobian[legnumber].theta[2] * leg_force[legnumber].force_z;

    leg_moment_force[legnumber].force_2 = Jacobian[legnumber].gamma[0] * leg_force[legnumber].force_x +
                                          Jacobian[legnumber].gamma[1] * leg_force[legnumber].force_y +
                                          Jacobian[legnumber].gamma[2] * leg_force[legnumber].force_z;

}
float deta_force_motion ;
float gamma_force_motion ;

//横转控制
void vmc_force_correct(int legnumber)
{

    imu();
    if(legnumber==2||legnumber==3)
    {
        gamma_force_motion = gamma_force_motion *(-1);
    }
    leg_force[legnumber].force_y += gamma_force_motion;

}

//偏航控制
void vmc_force_motion_correct(int legnumber)
{
    leg_moment_force[legnumber].force_0 = -leg_moment_force[legnumber].force_0
                                          - deta_force_motion;
    leg_moment_force[legnumber].force_1 *= (-1);
    leg_moment_force[legnumber].force_2 *= (-1);
}
