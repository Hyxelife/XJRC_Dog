#include <iostream>
#include <stdio.h>
#include "Console.h"
#include "Controller.h"
using namespace std;

#define DEG(deg)    (deg)/180.0f*3.141592653589f
//printf
int main()
{
    LegStructure::RegisterStructure(LegStructure(9.41f, 25.0f, 25.0f));
    LegMotors::SetMotorScalar(9.1f);
    Console con("/dev/input/event4");

    Controller controller(
        {"/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3"},
        {
        AxisMovement(5.07212,2.40644,4.74922),
        AxisMovement(5.24738,0.298743,0.719439),
        AxisMovement(1.08951,1.55239,1.21645),
        AxisMovement(1.7986,1.53974,0.101243),

        },

        {
            //AxisMovement(DEG(90),DEG(90),DEG(90)),
            AxisMovement(DEG(23.66),DEG(180-21.45),DEG(18.95)),
            AxisMovement(DEG(26.66),DEG(180-21.45),DEG(19.95)),
            AxisMovement(DEG(20.66),DEG(180-21.45),DEG(18.95)),
            AxisMovement(DEG(23.66),DEG(180-21.45),DEG(18.95)),
        },
        {
            {-1,-1,-1},
            {1,1,1},
            {-1,1,1},
            {1,-1,-1},
        },
        LegController::VMCParam(),
        Controller::CtrlInitParam(0.5,5,9,9,0.3,0.01),
        Controller::MechParam(41,15+9.41f+9.41f)
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

    cout<<"[main]:start loop"<<endl;
    con.Start();
    Console::ConsoleRequest req;
    Console::ConsoleStatus status;
    //while(1);
    bool sysQuit = false;
    while(!sysQuit)
    {
        con.GetConsoleRequest(req);
        con.GetConsoleStatus(status);
        controller.Update(req.x,req.y,req.r,req.reqHop,true);
        //printf("update\n");
        if(req.reqStop)
        {
            controller.StopMoving();
            //printf("req stop\n");
            }
        con.UpdateEvent(controller.IsStop());
        sysQuit = status.quit;

    }
    printf("[main]:system quitting...\n");
    controller.Exit();
    printf("[main]Controller quit!\n");
    con.Exit();
    printf("[main]:system quit!\n");
    return 0;
}
