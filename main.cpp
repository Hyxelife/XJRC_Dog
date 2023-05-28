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



    Controller controller(
        {"/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3"},
        {
         AxisMovement(4.91949,3.08408,4.74078),
        AxisMovement(5.24776,4.15786,0.671885),
        AxisMovement(1.21645,3.32031,1.11789),
        AxisMovement(1.90483,0.584065,0.265763) ,

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
    planner.SetCurveHeight(9.0);
    planner.SetDogHeight(30.0f);
    planner.SetDogOffsetX(9.41f);
    planner.SetGait(Gait::Pace(0.6f,0.08f),0);
    controller.EnableVMC(false);
    cout<<"[main]:params ready!starting up ..."<<endl;
    controller.Start(3.0f);

    cout<<"[main]:finish start up procedure!"<<endl;
    controller.Update(0,0,0);
    cout<<"[main]:start loop"<<endl;
    con.Start();
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
