#pragma once
#include "fcntl.h"
#include "MultiThread.h"
#include <queue>
#include "Controller.h"


class AutoCtrl
{
    public:
    enum ActionType
    {
        run,
        back,
        stop,
        turnR,
        turnL,
        rotateR,
        rotateL,
        climb,
        moveR,
        moveL,
        hop,
        hopToBalance,
        balanceRestore,
    };
    struct Action
    {
        int actionCnt;
        ActionType action;
    };
public:

    struct AutoCtrlParam
    {
        float x,y,r;
        bool hop;
        float angle;
        Controller::HopType hopType;
        bool stop;
    };

public:
    AutoCtrl();
    ~AutoCtrl();
    void UpdateStep();
    void GetAutoCtrlParam(AutoCtrlParam& param){param = m_param;}
    void AddAction(Action action);
    void AddAction(ActionType type,int count);
    void AddActions(std::vector<Action> actions);
    void ClearActions();
    bool IsEmpty();
protected:
    MUTEX m_mutex;
    AutoCtrlParam m_param;
    std::queue<Action> m_actions;
};
