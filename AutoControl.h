#pragma once
#include "fcntl.h"
#include "MultiThread.h"
#include <queue>


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
        moveR,
        moveL,
        hop,
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
