
#include "stdio.h"
#include "all.h"

void Up_oneK(int number,float v,float posK)
{
    MOTOR_recv motor_r=Zero_control(number,K_Joint);
    if (checkposition(motor_r.Pos,K1))
    {
        if(motor_r.Pos<posK)
        {
            while(motor_r.Pos<posK)
            {
                motor_r=velocity_control(number,K_Joint,v);
            }
            if (checkposition(motor_r.Pos,posK))
            {
                motor_r=postion_control(number,K_Joint,posK);
                param_allprint(motor_r);
                printf("FINALoneK****************:%f\n",posK);
            }
            else printf("position_control Error!");
        }
        else if(motor_r.Pos>posK || motor_r.Pos==posK)
        {
            while(motor_r.Pos>posK)
            {
                motor_r=velocity_control(number,K_Joint,-v);
            }
            if (checkposition(motor_r.Pos,posK))
            {
                motor_r=postion_control(number,K_Joint,posK);
                param_allprint(motor_r);
                printf("FINALoneK****************:%f\n",posK);
            }
            else printf("position_control Error!");
        }
    }
}


void Up_oneD(int number,float v,float posD)
{
    MOTOR_recv motor_r=Zero_control(number,D_Joint);
    if (checkposition(motor_r.Pos,D1))
    {
        if(motor_r.Pos<posD)
        {
            while(motor_r.Pos<posD)
            {
                motor_r=velocity_control(number,D_Joint,v);
            }
            if (checkposition(motor_r.Pos,posD))
            {
                motor_r=postion_control(number,D_Joint,posD);
                printf("******1D*******\n");
                param_allprint(motor_r);
                printf("FINALoneD****************:%f\n",posD);
            }
            else printf("position_control Error!");
        }
        else if(motor_r.Pos>posD || motor_r.Pos==posD)
        {
            while(motor_r.Pos>posD)
            {
                motor_r=velocity_control(number,D_Joint,-v);
            }
            if (checkposition(motor_r.Pos,posD))
            {
                motor_r=postion_control(number,D_Joint,posD);
                printf("******1D*******\n");
                param_allprint(motor_r);
                printf("FINALoneD****************:%f\n",posD);
            }
            else printf("position_control Error!");
        }
    }
    else printf("D1_Zero_point error!");
}


void Up_oneX(int number,float v,float posX)
{
    MOTOR_recv motor_r=Zero_control(number,X_Joint);
    if (checkposition(motor_r.Pos,X1))
    {
        if(motor_r.Pos>posX)
        {
            while(motor_r.Pos>posX)
            {
                motor_r=velocity_control(number,X_Joint,v);
            }
            if (checkposition(motor_r.Pos,posX))
            {
                motor_r=postion_control(number,X_Joint,posX);
                printf("******1X*******\n");
                param_allprint(motor_r);
                printf("FINALoneX****************:%f\n",posX);
            }
            else printf("position_control Error!");
        }
        else if(motor_r.Pos<posX || motor_r.Pos==posX)
        {
            while(motor_r.Pos<posX)
            {
                motor_r=velocity_control(number,X_Joint,-v);
            }
            if (checkposition(motor_r.Pos,posX))
            {
                motor_r=postion_control(number,X_Joint,posX);
                printf("******1X*******\n");
                param_allprint(motor_r);
                printf("FINALoneX****************:%f\n",posX);
            }
            else printf("position_control Error!");
        }
    }
    else printf("X1_Zero_point error!");
}


void Up_twoK(int number,float v,float posK)
{
    MOTOR_recv motor_r=Zero_control(number,K_Joint);
    if (checkposition(motor_r.Pos,K2))
    {
        if(motor_r.Pos>posK)
        {
            while(motor_r.Pos>posK)
            {
                motor_r=velocity_control(number,K_Joint,v);
            }
            if (checkposition(motor_r.Pos,posK))
            {
                motor_r=postion_control(number,K_Joint,posK);
                printf("******2K*******\n");
                printf("FINALtwoK****************:%f\n",posK);
            }
            else printf("position_control Error!");
        }
        else if(motor_r.Pos<posK || motor_r.Pos==posK)
        {
            while(motor_r.Pos<posK)
            {
                motor_r=velocity_control(number,K_Joint,-v);
            }
            if (checkposition(motor_r.Pos,posK))
            {
                motor_r=postion_control(number,K_Joint,posK);
                printf("******2K*******\n");
                printf("FINALtwoK****************:%f\n",posK);
            }
            else printf("position_control Error!");
        }
    }
    else printf("K2_Zero_point error!");
}


void Up_twoD(int number,float v,float posD)
{
    MOTOR_recv motor_r=Zero_control(number,D_Joint);
    if (checkposition(motor_r.Pos,D2))
    {
        if(motor_r.Pos>posD)
        {
            while(motor_r.Pos>posD)
            {
                motor_r=velocity_control(number,D_Joint,v);
            }
            if (checkposition(motor_r.Pos,posD))
            {
                motor_r=postion_control(number,D_Joint,posD);
                printf("******2D*******\n");
                param_allprint(motor_r);
                printf("FINALtwoD****************:%f\n",posD);
            }
            else printf("position_control Error!");
        }
        else if(motor_r.Pos<posD || motor_r.Pos==posD)
        {
            while(motor_r.Pos<posD)
            {
                motor_r=velocity_control(number,D_Joint,-v);
            }
            if (checkposition(motor_r.Pos,posD))
            {
                motor_r=postion_control(number,D_Joint,posD);
                printf("******2D*******\n");
                param_allprint(motor_r);
                printf("FINALtwoD****************:%f\n",posD);
            }
            else printf("position_control Error!");
        }
    }
    else printf("D2_Zero_point error!");
}


void Up_twoX(int number,float v,float posX)
{
    MOTOR_recv motor_r=Zero_control(number,X_Joint);
    printf("***********%f",motor_r.Pos);
    if (checkposition(motor_r.Pos,X2))
    {
        if(motor_r.Pos<posX)
        {
            while(motor_r.Pos<posX)
            {
                motor_r=velocity_control(number,X_Joint,v);
            }
            if (checkposition(motor_r.Pos,posX))
            {
                motor_r=postion_control(number,X_Joint,posX);
                param_allprint(motor_r);
                printf("FINALtwoX****************:%f\n",posX);
            }
            else printf("position_control Error!");
        }
        else if(motor_r.Pos>posX || motor_r.Pos==posX)
        {
            while(motor_r.Pos>posX)
            {
                motor_r=velocity_control(number,X_Joint,-v);
            }
            if (checkposition(motor_r.Pos,posX))
            {
                motor_r=postion_control(number,X_Joint,posX);
                param_allprint(motor_r);
                printf("FINALtwoX****************:%f\n",posX);
            }
            else printf("position_control Error!");
        }
    }
    else printf("X2_Zero_point error!");
}


void Up_threeK(int number,float v,float posK)
{
    MOTOR_recv motor_r=Zero_control(number,K_Joint);
    if (checkposition(motor_r.Pos,K3))
    {
        if(motor_r.Pos<posK)
        {
            while(motor_r.Pos<posK)
            {
                motor_r=velocity_control(number,K_Joint,v);
            }
            if (checkposition(motor_r.Pos,posK))
            {
                motor_r=postion_control(number,K_Joint,posK);
                param_allprint(motor_r);
                printf("FINALthreeK****************:%f\n",posK);
            }
            else printf("position_control Error!");
        }
        else if(motor_r.Pos>posK || motor_r.Pos==posK)
        {
            while(motor_r.Pos>posK)
            {
                motor_r=velocity_control(number,K_Joint,-v);
            }
            if (checkposition(motor_r.Pos,posK))
            {
                motor_r=postion_control(number,K_Joint,posK);
                param_allprint(motor_r);
                printf("FINALthreeK****************:%f\n",posK);
            }
            else printf("position_control Error!");
        }
    }
    else printf("K3_Zero_point error!");
}


void Up_threeD(int number,float v,float posD)
{
    MOTOR_recv motor_r=Zero_control(number,D_Joint);
    if (checkposition(motor_r.Pos,D3))
    {
        if(motor_r.Pos>posD)
        {
            while(motor_r.Pos>posD)
            {
                motor_r=velocity_control(number,D_Joint,v);
            }
            if (checkposition(motor_r.Pos,posD))
            {
                motor_r=postion_control(number,D_Joint,posD);
                param_allprint(motor_r);
                printf("FINALthreeD****************:%f\n",posD);
            }
            else printf("position_control Error!");
        }
        else if(motor_r.Pos<posD || motor_r.Pos==posD)
        {
            while(motor_r.Pos<posD)
            {
                motor_r=velocity_control(number,D_Joint,-v);
            }
            if (checkposition(motor_r.Pos,posD))
            {
                motor_r=postion_control(number,D_Joint,posD);
                param_allprint(motor_r);
                printf("FINALthreeD****************:%f\n",posD);
            }
            else printf("position_control Error!");
        }
    }
    else printf("D3_Zero_point error!");
}


void Up_threeX(int number,float v,float posX)
{
    MOTOR_recv motor_r=Zero_control(number,X_Joint);
    if (checkposition(motor_r.Pos,X3))
    {
        if(motor_r.Pos<posX)
        {
            while(motor_r.Pos<posX)
            {
                motor_r=velocity_control(number,X_Joint,v);
            }
            if (checkposition(motor_r.Pos,posX))
            {
                motor_r=postion_control(number,X_Joint,posX);
                param_allprint(motor_r);
                printf("FINALthreeX****************:%f\n",posX);
            }
            else printf("position_control Error!");
        }
        else if(motor_r.Pos>posX || motor_r.Pos==posX)
        {
            while(motor_r.Pos>posX)
            {
                motor_r=velocity_control(number,X_Joint,-v);
            }
            if (checkposition(motor_r.Pos,posX))
            {
                motor_r=postion_control(number,X_Joint,posX);
                param_allprint(motor_r);
                printf("FINALthreeX****************:%f\n",posX);
            }
            else printf("position_control Error!");
        }
    }
    else printf("X3_Zero_point error!");
}


void Up_fourK(int number,float v,float posK)
{
    MOTOR_recv motor_r=Zero_control(number,K_Joint);
    if (checkposition(motor_r.Pos,K4))
    {
        if(motor_r.Pos>posK)
        {
            while(motor_r.Pos>posK)
            {
                motor_r=velocity_control(number,K_Joint,v);
            }
            if (checkposition(motor_r.Pos,posK))
            {
                motor_r=postion_control(number,K_Joint,posK);
                param_allprint(motor_r);
                printf("FINALfourK****************:%f\n",posK);
            }
            else printf("position_control Error!");
        }
        else if(motor_r.Pos<posK || motor_r.Pos==posK)
        {
            while(motor_r.Pos<posK)
            {
                motor_r=velocity_control(number,K_Joint,-v);
            }
            if (checkposition(motor_r.Pos,posK))
            {
                motor_r=postion_control(number,K_Joint,posK);
                param_allprint(motor_r);
                printf("FINALfourK****************:%f\n",posK);
            }
            else printf("position_control Error!");
        }
    }
    else printf("K4_Zero_point error!");
}


void Up_fourD(int number,float v,float posD)
{
    MOTOR_recv motor_r=Zero_control(number,D_Joint);
    if (checkposition(motor_r.Pos,D4))
    {
        if(motor_r.Pos<posD)
        {
            while(motor_r.Pos<posD)
            {
                motor_r=velocity_control(number,D_Joint,v);
            }
            if (checkposition(motor_r.Pos,posD))
            {
                motor_r=postion_control(number,D_Joint,posD);
                param_allprint(motor_r);
                printf("FINALfourD****************:%f\n",posD);
            }
            else printf("position_control Error!");
        }
        else if (motor_r.Pos>posD || motor_r.Pos==posD)
        {
            while(motor_r.Pos>posD)
            {
                motor_r=velocity_control(number,D_Joint,-v);
            }
            if (checkposition(motor_r.Pos,posD))
            {
                motor_r=postion_control(number,D_Joint,posD);
                param_allprint(motor_r);
                printf("FINALfourD****************:%f\n",posD);
            }
            else printf("position_control Error!");
        }
    }
    else printf("D4_Zero_point error!");
}


void Up_fourX(int number,float v,float posX)
{
    MOTOR_recv motor_r=Zero_control(number,X_Joint);
    if (checkposition(motor_r.Pos,X4))
    {
        if(motor_r.Pos>posX)
        {
            while(motor_r.Pos>posX)
            {
                motor_r=velocity_control(number,X_Joint,v);
            }
            if (checkposition(motor_r.Pos,posX))
            {
                motor_r=postion_control(number,X_Joint,posX);
                param_allprint(motor_r);
                printf("FINALfourX****************:%f\n",posX);
            }
            else printf("position_control Error!");
        }
        else if(motor_r.Pos<posX || motor_r.Pos==posX)
        {
            while(motor_r.Pos<posX)
            {
                motor_r=velocity_control(number,X_Joint,-v);
            }
            if (checkposition(motor_r.Pos,posX))
            {
                motor_r=postion_control(number,X_Joint,posX);
                param_allprint(motor_r);
                printf("FINALfourX****************:%f\n",posX);
            }
            else printf("position_control Error!");
        }
    }
    else printf("X4_Zero_point error!");
}
float Y=0.0988;
float GES_Z=-0.30;
void execute_one(void)
{
    position_xyz leg_pos;
    position_angel leg_angle;

    leg_pos.x=trot_cg[0];
    leg_pos.y=-Y;
    leg_pos.z=GES_Z;
    leg_pos.leg=1;

    inversekinematics(&leg_pos,&leg_angle);

    float posK1=K1_origin_pos+(-1)*leg_angle.gama*9.1;
    float posD1=D1_origin_pos+leg_angle.alfa*9.1;
    float posX1=X1_origin_pos+leg_angle.beta*9.1;


printf("%f %f %f\n",leg_pos.x,leg_pos.y,leg_pos.z);
  printf("%f %f %f\n",leg_angle.alfa,leg_angle.beta,leg_angle.gama);
//    printf("%f %f %f\n",posK1,posD1,posX1);


    float v1=1;
    Up_oneK(leg_pos.leg,v1,posK1);

    v1=v1;
   Up_oneD(leg_pos.leg,v1,posD1);

    v1=-v1;
    Up_oneX(leg_pos.leg,v1,posX1);

}
void execute_two(void)
{
    float v2=-0.3;

    position_xyz leg_pos;
    position_angel leg_angle;

    leg_pos.x=trot_cg[1];
    leg_pos.y=Y;
    leg_pos.z=GES_Z;
    leg_pos.leg=2;

    inversekinematics(&leg_pos,&leg_angle);

    float posK2=K2_origin_pos+leg_angle.gama*9.1;
    float posD2=D2_origin_pos+leg_angle.alfa*9.1;
    float posX2=X2_origin_pos+leg_angle.beta*9.1;
printf("%f %f %f\n",leg_pos.x,leg_pos.y,leg_pos.z);
    printf("%f %f %f\n",leg_angle.alfa,leg_angle.beta,leg_angle.gama);
//    printf("%f %f %f\n",posK2,posD2,posX2);


    Up_twoK(leg_pos.leg,v2,posK2);

    v2=v2;
    Up_twoD(leg_pos.leg,v2,posD2);

    v2=-v2;
    Up_twoX(leg_pos.leg,v2,posX2);


}
void execute_three(void)
{
    float v3=1;

    position_xyz leg_pos;
    position_angel leg_angle;

    leg_pos.x=trot_cg[2];
    leg_pos.y=Y;
    leg_pos.z=GES_Z;
    leg_pos.leg=3;

    inversekinematics(&leg_pos,&leg_angle);
    float posK3=K3_origin_pos+(-1)*leg_angle.gama*9.1;
    float posD3=D3_origin_pos+leg_angle.alfa*9.1;
    float posX3=X3_origin_pos+leg_angle.beta*9.1;

printf("%f %f %f\n",leg_pos.x,leg_pos.y,leg_pos.z);
printf("%f %f %f\n",leg_angle.alfa,leg_angle.beta,leg_angle.gama);
//    printf("%f %f %f\n",posK3,posD3,posX3);

    Up_threeK(leg_pos.leg,v3,posK3);
    v3=-v3;
    Up_threeD(leg_pos.leg,v3,posD3);
    v3=-v3;
    Up_threeX(leg_pos.leg,v3,posX3);

}
void execute_four(void)
{
    float v4=-1;

    position_xyz leg_pos;
    position_angel leg_angle;

    leg_pos.x=trot_cg[3];
    leg_pos.y=-Y;
    leg_pos.z=GES_Z;
    leg_pos.leg=4;

    inversekinematics(&leg_pos,&leg_angle);

    float posK4=K4_origin_pos+leg_angle.gama*9.1;
    float posD4=D4_origin_pos+leg_angle.alfa*9.1;
    float posX4=X4_origin_pos+leg_angle.beta*9.1;

printf("%f %f %f\n",leg_pos.x,leg_pos.y,leg_pos.z);
   printf("%f %f %f\n",leg_angle.alfa,leg_angle.beta,leg_angle.gama);
//    printf("%f %f %f\n",posK4,posD4,posX4);




    Up_fourK(leg_pos.leg,v4,posK4);
    v4=-v4;
    Up_fourD(leg_pos.leg,v4,posD4);
    v4=-v4;
    Up_fourX(leg_pos.leg,v4,posX4);


}

void standingup(int leg_number)
{
    if(leg_number==1) execute_one();
    if(leg_number==2) execute_two();
    if(leg_number==3) execute_three();
    if(leg_number==4) execute_four();
}

void *standingup1(void *arg)//处理陀螺仪数据
{
    standingup(1);
    return NULL;
}
void *standingup2(void *arg)//处理陀螺仪数据
{
    standingup(2);
    return NULL;
}
void *standingup3(void *arg)//处理陀螺仪数据
{
     standingup(3);
    return NULL;
}
void *standingup4(void *arg)//处理陀螺仪数据
{
     standingup(4);
   return NULL;
}
void SU(int mode)
{
    if(mode==1)
    {
        standingup(1);
        standingup(2);
        standingup(3);
        standingup(4);
    }
    else
    {
        pthread_t t1;
        pthread_t t2;
        pthread_t t3;
        pthread_t t4;
        pthread_create(&t1,NULL,standingup1,NULL);
        pthread_create(&t2,NULL,standingup2,NULL);
        pthread_create(&t3,NULL,standingup3,NULL);
        pthread_create(&t4,NULL,standingup4,NULL);
        pthread_join(t1,NULL);
        pthread_join(t2,NULL);
        pthread_join(t3,NULL);
        pthread_join(t4,NULL);
    }
}

