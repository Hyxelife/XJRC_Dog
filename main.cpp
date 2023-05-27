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
    Console con(1,1,1e-3);
    con.Start();

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

    float x,y,r;
    bool sysQuit = false;
    while(!sysQuit)
    {
        con.Update(false);
        if(con.IsDogStop())continue;
        if(con.IsAutoControl())
        {
            //todo:add auto ctrl
        }else
        {
            con.UpdateMannualParams(x,y,r);
            if(controller.Update(x,y,r))
                con.Update(true);
        }
        sysQuit = con.IsRequestExit();
    }
    controller.Exit();
    con.Exit();
    printf("[main]:system quit!\n");
    return 0;
}
