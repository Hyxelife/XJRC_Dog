#include "all.h"

int dirstate[4]={0};//每条腿前进方向，1为前，-1为后
legstate11 legstate1;//机体状态的结构体
Trotleg  trotleg[4];//每条腿位置数组
float trot_cg[4];//摆线偏移
float movecontrol[3];
float deltaz[4];//
void cycloid(int legnumber,float t,int legstate,float xs,float xf,float ges_z,float h,float y)//xf==1/2步长
{
    float a,f,s;
    a=2*pi*t;
    f=t-(sin(2*a))/(4*pi);
        if(legnumber==0 || legnumber==3)
    {
        trotleg[legnumber].expy=-y;
    }
    else if (legnumber==1 || legnumber==2)
    {
        trotleg[legnumber].expy=y;
    }
    if (legstate==0)//摆动相
    {
 //       s=dirstate[legnumber]*(xf+dirstate[legnumber]*trot_cg[legnumber]-dirstate[legnumber]*xs);

        trotleg[legnumber].expx=xs+dirstate[legnumber]*(xf+dirstate[legnumber]*trot_cg[legnumber]-dirstate[legnumber]*xs)*(a-sin(a))/(2*pi);//wait
//        trotleg[legnumber].expx=xs+6*s*(pow(t,5))-15*s*(pow(t,4))+10*s*(pow(t,3));
//        trotleg[legnumber].expz=ges_z-768*h*(pow(t,8))+3072*h*(pow(t,7))-4868*h*(pow(t,6))+3840*h*(pow(t,5))-1536*h*(pow(t,4))+256*h*(pow(t,3));

       trotleg[legnumber].expz=0.5*h*(1-cos(a))+ges_z;
//        if(t<=0.5)
//        {
//            trotleg[legnumber].expz=2*f*h+ges_z;
//        }
//        else
//        {
//            trotleg[legnumber].expz=2*h*(1-f)+ges_z;
//        }

    }
    else if(legstate==1)
    {
        trotleg[legnumber].expx=xs-dirstate[legnumber]*t*(xf-dirstate[legnumber]*trot_cg[legnumber]+dirstate[legnumber]*xs);
//        s=dirstate[legnumber]*(xf-dirstate[legnumber]*trot_cg[legnumber]+dirstate[legnumber]*xs);
//
//        trotleg[legnumber].expx=xs-6*s*(pow(t,5))-15*s*(pow(t,4))+10*s*(pow(t,3));
        trotleg[legnumber].expz=ges_z;
    }

}



//void square(int legnumber,float t,float *pos,float xs,float xf,float ges_z,float h,float y)
//{
//    float ts;
//    if(legnumber==1 || legnumber==4)
//    {
//        pos[1]=-y;
//    }
//    else if (legnumber==2 || legnumber==3)
//    {
//        pos[1]=y;
//    }
//    if(t<=0.25*Ts/2)
//    {
//        if(legnumber==1||legnumber==3)
//        {
//            pos[0]=xs;
//            pos[2]=ges_z+(t/(Ts/8))*h;
//        }
//        else
//        {
//            pos[0]=xf+(xs-xf)/(Ts/2)*t;
//            pos[2]=ges_z;
//        }
//    }
//    else if(t>0.25*Ts/2&&t<=0.75*Ts/2)
//    {
//        if(legnumber==1||legnumber==3)
//        {
//            pos[0]=xs+(xf-xs)/(Ts/4)*(t-Ts/8);
//            pos[2]=ges_z+h;
//        }
//        else
//        {
//            pos[0]=xf+(xs-xf)/(Ts/2)*t;
//            pos[2]=ges_z;
//        }
//    }
//    else if(t>0.75*Ts/2&&t<=Ts/2)
//    {
//        if(legnumber==1||legnumber==3)
//        {
//            pos[0]=xf;
//            pos[2]=ges_z+h+(-h)/(Ts/8)*(t-3*Ts/8);
//        }
//        else
//        {
//            pos[0]=xf+(xs-xf)/(Ts/2)*t;
//            pos[2]=ges_z;
//        }
//    }//1 3摆动
//    else if(t>Ts/2&&t<=1.25*Ts/2)
//    {
//        if(legnumber==2||legnumber==4)
//        {
//            pos[0]=xs;
//            pos[2]=ges_z+h/(Ts/8)*(t-Ts/2);
//        }
//        else
//        {
//            pos[0]=xf+(xs-xf)/(Ts/2)*(t-Ts/2);
//            pos[2]=ges_z;
//        }
//    }
//    else if(t>1.25*Ts/2&&t<=1.75*Ts/2)
//    {
//        if(legnumber==2||legnumber==4)
//        {
//            pos[0]=xs+(xf-xs)/(Ts/4)*(t-5*Ts/8);
//            pos[2]=ges_z+h;
//        }
//        else
//        {
//            pos[0]=xf+(xs-xf)/(Ts/2)*(t-Ts/2);
//            pos[2]=ges_z;
//        }
//    }
//    else if(t>1.75*Ts/2&&t<=Ts)
//    {
//        if(legnumber==2||legnumber==4)
//        {
//            pos[0]=xf;
//            pos[2]=ges_z+h+(-h)/(Ts/8)*(t-7*Ts/8);
//        }
//        else
//        {
//            pos[0]=xf+(xs-xf)/(Ts/2)*(t-Ts/2);
//            pos[2]=ges_z;
//        }
//    }
//}


//void y_z_cycloid(int legnumber,float t,float *pos,float xs,float xf,float ys,float yf,float ges_z,float h,float x)
//
//{
//    float sigma;
//    float zep_baixian;
//    float yep[2];//0是左摆，1是右摆
//    float zep[2];//0是支撑相，1是摆动相
//
//
//    pos[0]=(xs+xf)/2;//加了半步走之后可以用
//
//
////    if(legnumber==1 || legnumber==3)
////    {
////        pos[0]=xs;
////    }
////    else if (legnumber==2 || legnumber==4)
////    {
////        pos[0]=xf;
////    }
//
//    if(t<=Ts*faai)
//    {
//        sigma=2*pi*t/(faai*Ts);
//        zep_baixian=h*(1-cos(sigma))/2;
//        zep[0]=ges_z;
//        zep[1]=ges_z+zep_baixian;
//        xep[0]=(xs-xf)*(sigma-sin(sigma))/(2*pi)+xf;
//        xep[1]=(xf-xs)*(sigma-sin(sigma))/(2*pi)+xs;
//
//        if(direction==1)//前进
//        {
//            if(legnumber==1 || legnumber==3)
//            {
//                pos[2]=zep[1];
//                pos[0]=xep[1];
//            }
//            else if(legnumber==2 || legnumber==4)
//            {
//                pos[2]=zep[0];
//                pos[0]=xep[0];
//            }
//        }
//        else if(direction==2)//后退
//        {
//            if(legnumber==1 || legnumber==3)
//            {
//                pos[2]=zep[1];
//                pos[0]=xep[0];
//            }
//            else if(legnumber==2 || legnumber==4)
//            {
//                pos[2]=zep[0];
//                pos[0]=xep[1];
//            }
//        }
//        else if(direction==3)//左转
//        {
//            if(legnumber==1)
//            {
//                pos[2]=zep[1];
//                pos[0]=xep[1];
//            }
//            else if(legnumber==2)
//            {
//                pos[2]=zep[0];
//                pos[0]=xep[1];
//            }
//            else if(legnumber==3)
//            {
//                pos[2]=zep[1];
//                pos[0]=xep[0];
//            }
//            else if(legnumber==4)
//            {
//                pos[2]=zep[0];
//                pos[0]=xep[0];
//            }
//        }
//        else if(direction==4)//右转
//        {
//            if(legnumber==1)
//            {
//                pos[2]=zep[1];
//                pos[0]=xep[0];
//            }
//            else if(legnumber==2)
//            {
//                pos[2]=zep[0];
//                pos[0]=xep[0];
//            }
//            else if(legnumber==3)
//            {
//                pos[2]=zep[1];
//                pos[0]=xep[1];
//            }
//            else if(legnumber==4)
//            {
//                pos[2]=zep[0];
//                pos[0]=xep[1];
//            }
//        }
//
//
//    }
//
//    else if(t>Ts*faai && t<Ts)
//    {
//        sigma=2*pi*(t-Ts*faai)/(faai*Ts);
//        zep_baixian=h*(1-cos(sigma))/2;
//        zep[0]=ges_z;
//        zep[1]=ges_z+zep_baixian;
//        xep[0]=(xs-xf)*(sigma-sin(sigma))/(2*pi)+xf;
//        xep[1]=(xf-xs)*(sigma-sin(sigma))/(2*pi)+xs;
//
//
//        if(direction==1)//前进
//        {
//            if(legnumber==1 || legnumber==3)
//            {
//                pos[2]=zep[0];
//                pos[0]=xep[0];
//            }
//            else if(legnumber==2 || legnumber==4)
//            {
//                pos[2]=zep[1];
//                pos[0]=xep[1];
//            }
//        }
//        else if(direction==2)//后退
//        {
//            if(legnumber==1 || legnumber==3)
//            {
//                pos[2]=zep[0];
//                pos[0]=xep[1];
//            }
//            else if(legnumber==2 || legnumber==4)
//            {
//                pos[2]=zep[1];
//                pos[0]=xep[0];
//            }
//        }
//        else if(direction==3)//左转
//        {
//            if(legnumber==1)
//            {
//                pos[2]=zep[0];
//                pos[0]=xep[0];
//            }
//            else if(legnumber==2)
//            {
//                pos[2]=zep[1];
//                pos[0]=xep[0];
//            }
//            else if(legnumber==3)
//            {
//                pos[2]=zep[0];
//                pos[0]=xep[1];
//            }
//            else if(legnumber==4)
//            {
//                pos[2]=zep[1];
//                pos[0]=xep[1];
//            }
//        }
//        else if(direction==4)//右转
//        {
//            if(legnumber==1)
//            {
//                pos[2]=zep[0];
//                pos[0]=xep[1];
//            }
//            else if(legnumber==2)
//            {
//                pos[2]=zep[1];
//                pos[0]=xep[1];
//            }
//            else if(legnumber==3)
//            {
//                pos[2]=zep[0];
//                pos[0]=xep[0];
//            }
//            else if(legnumber==4)
//            {
//                pos[2]=zep[1];
//                pos[0]=xep[0];
//            }
//        }
//    }
//
//}
