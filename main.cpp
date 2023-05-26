#include <iostream>
#include <stdio.h>
#include <termios.h>
#include "Controller.h"
using namespace std;

#define DEG(deg)    (deg)/180.0f*3.141592653589f

bool sysQuit = false;
bool arrowCtrl = false;
bool autoCtrl = false;
float ctrl_x,ctrl_y,ctrl_r;
bool run = false;



int getch_(void)
{
     struct termios tm, tm_old;
     int fd = 0, ch;

     if (tcgetattr(fd, &tm) < 0) {//保存现在的终端设置
          return -1;
     }

     tm_old = tm;
     cfmakeraw(&tm);//更改终端设置为原始模式，该模式下所有的输入数据以字节为单位被处理
     if (tcsetattr(fd, TCSANOW, &tm) < 0) {//设置上更改之后的设置
          return -1;
     }

     ch = getchar();
     if (tcsetattr(fd, TCSANOW, &tm_old) < 0) {//更改设置为最初的样子
          return -1;
     }

     return ch;
}

void * console(void * arg)
{
    char cmd;
    while(!sysQuit)
    {
        if(arrowCtrl)
        {
            cmd = getch_();
            switch(cmd)
            {
                case 'q':ctrl_r = 1;run = true;break;
                case 'e':ctrl_r = -1;run = true;break;
                case 'w':ctrl_y = 1;run = true;break;
                case 'a':ctrl_x = -1;run = true;break;
                case 's':ctrl_y = -1;run = true;break;
                case 'd':ctrl_x = 1;run = true;break;
                case 'p':arrowCtrl = false;printf("[console]:mannual control quit\n");break;
            }
        }else if(autoCtrl)
        {
            cmd = getchar();
            switch(cmd)
            {
                case 'Q':
                case 'q':autoCtrl = false;break;
            }
        }else
        {
            scanf("%c",&cmd);
            switch(cmd)
            {
                case 'H':
                case 'h':
                {
                    printf("**********************************\n");
                    printf("*           help page            *\n");
                    printf("**********************************\n");
                    printf("[h/H] help page\n");
                    printf("[m/M] mannually control\n");
                    printf("[x/X] automatically control\n");
                    printf("[b/B] quit sustem\n");
                    printf("when system is mannually controlling:\n");
                    printf("\t[w/a/s/d] moving control\n");
                    printf("\t[q/e] rotating control\n");
                    printf("\t[p] quit control\n");
                    printf("when system is automatically controlling:\n");
                    printf("\t[q] quit\n");
                }break;
                case 'm':
                case 'M':autoCtrl = false;arrowCtrl = true;printf("[console]:entering mannual control.\n");break;
                case 'x':
                case 'X':autoCtrl = true;arrowCtrl = false;break;
                case 'B':
                case 'b':sysQuit = true;break;
            }
        }
    }
}

#define DECAY(num,decay)    if(num>0)\
                            {num-=decay;if(num < 0)num=0;}\
                            else\
                            {num += decay;if(num>0)num = 0;}


int main()
{
    LegStructure::RegisterStructure(LegStructure(9.41f, 25.0f, 25.0f));
    LegMotors::SetMotorScalar(9.1f);
    THREAD prop = thread_create(console,NULL,prop);


    Controller controller(
        {"/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3"},
        {
         AxisMovement(4.60809,3.0603,4.72927),
        AxisMovement(5.37086,4.23993,0.693744),
        AxisMovement(1.0109,3.30344,1.13822),
        AxisMovement(2.29829,0.757788,0.122719),

        },
        {
            //AxisMovement(0,DEG(90),DEG(90)),
            AxisMovement(DEG(23.66),DEG(180-21.45),DEG(18.95)),
            AxisMovement(DEG(26.66),DEG(180-21.45),DEG(19.95)),
            AxisMovement(DEG(23.66),DEG(180-21.45),DEG(18.95)),
            AxisMovement(DEG(23.66),DEG(180-21.45),DEG(18.95)),
        },
        {
            {-1,-1,-1},
            {1,1,1},
            {-1,1,1},
            {1,-1,-1},
        },
        LegController::VMCParam()
    );
    cout<<"[main]:system ready!"<<endl;
    PacePlanner& planner = controller.GetPacePlanner();
    planner.SetCurveHeight(5.0);
    planner.SetDogHeight(30.0f);
    planner.SetDogOffsetX(9.41f);
    planner.SetGait(Gait::Pace(0.6f,0.08f),0);
    controller.EnableVMC(false);
    cout<<"[main]:params ready!starting up ..."<<endl;
    controller.Start(3.0f);

    cout<<"[main]:finish start up procedure!"<<endl;
    controller.Update(0,0,0);
    cout<<"[main]:start loop"<<endl;

    bool continue_ = false;

    clock_t lastTime = clock(),curTime = 0;
    float decay_ = 0;
    const float ctrlDecay = 0.5;
    const float zeroThres = 0.05;

    float set_x = ctrl_x,set_y = ctrl_y,set_r = ctrl_r;
    float upd_x = set_x,upd_y =set_y,upd_r = set_r;
    while(!sysQuit)
    {
    //controller.Update(0,1,0);

        if(continue_)
        {

            if(controller.Update(upd_x,upd_y,upd_r))
            {

                if(upd_x != 0 || upd_y != 0 || upd_r != 0)printf("[main]:%.2f,%.2f,%.2f\n",upd_x,upd_y,upd_r);
                upd_x = set_x;
                upd_y = set_y;
                upd_r = set_r;

                if(upd_x == 0 && upd_y == 0 && upd_r == 0){continue_ = false;run = false;}

                if(fabsf(upd_x) < zeroThres)upd_x = 0;
                if(fabsf(upd_y) < zeroThres)upd_y = 0;
                if(fabsf(upd_r) < zeroThres)upd_r = 0;
            }
        }else
        {
            continue_ = run;
            //printf("[main]:idlding...\n");
        }
        if(ctrl_x != 0 || ctrl_y != 0 || ctrl_r != 0)
        {
        set_x = ctrl_x;
        ctrl_x = 0;
        set_y = ctrl_y;
        ctrl_y = 0;
        set_r = ctrl_r;
        ctrl_r = 0;
        }
        //printf("[main]:controll:%.1f,%.1f,%.1f\n",set_x,set_y,set_r);
        curTime = clock();
        decay_ = curTime-lastTime;
        lastTime = curTime;
        decay_ /= (float)CLOCKS_PER_SEC;
        decay_ *= ctrlDecay;
        DECAY(set_x,decay_);
        DECAY(set_y,decay_);
        DECAY(set_r,decay_);
        //cout<<endl;
        //usleep(100000);


    }
    controller.Exit();
    return 0;
}
