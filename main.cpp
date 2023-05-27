#include <iostream>
#include <stdio.h>
#include "Console.h"
#include "Controller.h"
using namespace std;

#define DEG(deg)    (deg)/180.0f*3.141592653589f

int main()
{
    LegStructure::RegisterStructure(LegStructure(9.41f, 25.0f, 25.0f));
    LegMotors::SetMotorScalar(9.1f);
    Console con(0.5,5,1e-2);
    con.Start();


    Controller controller(
        {"/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3"},
        {
         AxisMovement(3.8411,3.13738,4.77453),
        AxisMovement(4.77261,4.14214,0.573327),
        AxisMovement(1.99303,3.10824,1.07877),
        AxisMovement(2.54756,0.799973,0.440254) ,

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

    float x,y,r;
    bool sysQuit = false;
    while(!sysQuit)
    {
        con.Update(false);
        sysQuit = con.IsRequestExit();
        if(con.IsDogStop()){controller.ClearTime();continue;}
        if(con.IsAutoControl())
        {
            //todo:add auto ctrl
        }else
        {
            con.UpdateMannualParams(x,y,r);
            //cout<<"[main]"<<x<<","<<y<<","<<","<<endl;
            if(controller.Update(x,y,r))
                con.Update(true);
        }

    }
    printf("[main]:system quitting...\n");
    controller.Exit();
    printf("[main]Controller quit! Press any key to stop console...\n");
    con.Exit();
    printf("[main]:system quit!\n");
    return 0;
}
