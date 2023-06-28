#include <iostream>
#include <stdio.h>
#include "Console.h"
#include "Controller.h"
#include "AutoControl.h"
#include "Connector.h"
#include "Debug.h"
using namespace std;

#define DEG(deg)    (deg)/180.0f*3.141592653589f
//printf
int main()
{
    Debug::Initialize("./log/record.txt","./log/log.txt",NULL);//"./log/pipe.txt"
    LegStructure::RegisterStructure(LegStructure(9.41f, 25.0f, 25.0f));
    LegMotors::SetMotorScalar(9.1f);
    AutoCtrl autoCtrl;
    Console con("/dev/input/event5",&autoCtrl);

    Connector connector(Connector::speed,&autoCtrl);
    Controller controller(
        {"/dev/ttyUSB0","/dev/ttyUSB1","/dev/ttyUSB2","/dev/ttyUSB3"},
        {
AxisMovement(4.7891,1.04273,4.89494),
AxisMovement(4.64452,3.32453,2.8559),
AxisMovement(1.46572,5.39886,0.995556),
AxisMovement(2.16484,1.46879,0.449074),





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
        Controller::MechParam(15+9.41f+9.41f,41)
    );

    OUT("[main]:system ready!\n");




    PacePlanner& planner = controller.GetPacePlanner();
    planner.SetCurveHeight(9.0);
    planner.SetDogHeight(30.0f);
    planner.SetDogOffsetX(9.41f);
    planner.SetGait(Gait::Pace(0.6f,0.08f),0);
    controller.EnableVMC(false);
    OUT("[main]:params ready!starting up ...\n");

    controller.Start(3.0f);
    OUT("[main]:finish start up procedure!\n");

    OUT("[main]:start loop\n");
    con.Start();
    Console::ConsoleRequest req;
    Console::ConsoleStatus status;
    AutoCtrl::AutoCtrlParam autoPar;
    //while(1);
    bool sysQuit = false;
    bool connectStart = false;
    while(!sysQuit)
    {
        /////////////test area//////////////////////
        //controller.Update(0,0,0,true);
        //continue;
        /////////////end test area//////////////////

    //OUT("loop\n");
        con.GetConsoleRequest(req);
        con.GetConsoleStatus(status);
        if(status.auto_ || status.test)
        {
            //TODO
            if(status.auto_)
            {
                if(!connectStart)
                {
                    connector.Start();
                    connectStart = true;
                }
            }
            if(autoCtrl.IsEmpty())
                controller.StopMoving();
            else if(controller.IsStop())
                controller.StartMoving();
            autoCtrl.GetAutoCtrlParam(autoPar);
            if(controller.Update(autoPar.x,autoPar.y,autoPar.r,autoPar.hop,true))
                autoCtrl.UpdateStep();
        }else
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
    OUT("[main]:system quitting...\n");
    controller.Exit();
    OUT("[main]:Controller quit!\n");
    printf("111\n");
    connector.Exit();
    OUT("[Auto]:Connector quit!\n");
    printf("222\n");
    con.Exit();
    OUT("[Console]:Console quit!\n");
    printf("333\n");
    OUT("[main]:system quit!\n");
    Debug::Exit();
    return 0;
}
