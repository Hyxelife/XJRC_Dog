#include <iostream>
#include <stdio.h>

#include "Controller.h"
using namespace std;

#define DEG(deg)    (deg)/180.0f*3.141592653589f

bool sysQuit = false;
bool arrowCtrl = false;
bool autoCtrl = false;
float ctrl_x,ctrl_y,ctrl_r;
bool run = false;





void * console(void * arg)
{

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
