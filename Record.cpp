#include "AutoControl.h"

void Record(AutoCtrl* pCtrl)
{
    AutoCtrl::Action act;
    act.action = AutoCtrl::sback;
    float init = pCtrl->GetIMUSensor()->GetIMUData().yaw;
    act.r = init;
    act.actionCnt = 12;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::hold;
    act.actionCnt = 3;
    pCtrl->AddAction(act);

    init -= 45;
    act.action = AutoCtrl::autoRotateTo;
    act.r = init;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::straight;
    act.r = init;
    act.actionCnt = 20;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::hold;
    act.actionCnt = 3;
    pCtrl->AddAction(act);

    init += 90;
    act.action = AutoCtrl::autoRotateTo;
    act.r = init;
    pCtrl->AddAction(act);

    //return ;
    act.action = AutoCtrl::straight;
    act.r = init;
    act.actionCnt = 20;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::hold;
    act.actionCnt = 3;
    pCtrl->AddAction(act);

    init += 90;
    act.action = AutoCtrl::autoRotateTo;
    act.r = init;
    pCtrl->AddAction(act);

    act.action = AutoCtrl::straight;
    act.r = init;
    act.actionCnt = 20;
    pCtrl->AddAction(act);



}
