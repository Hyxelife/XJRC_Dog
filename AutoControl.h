#pragma once
#include "fcntl.h"
#include "MultiThread.h"
#include <queue>
#include "Controller.h"
#include "Sensor.h"


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
        stepAndSpan,
        stepAndRestore,
        record,
        autoRotateTo,
        autoRotateWith,
        ClawDown,
        clawForward,
        clawRight,
        clawLeft,
    };
    struct Action
    {
        int actionCnt;
        ActionType action;
        float x,y,r;
    };
public:

    struct AutoCtrlParam
    {
        float x,y,r;
        bool hop;
        Controller::HopType hopType;
        bool stop;
    };

public:
    AutoCtrl(IMU* pIMU);
    ~AutoCtrl();
    void UpdateStep();
    void GetAutoCtrlParam(AutoCtrlParam& param){param = m_param;}
    void AddAction(Action action);
    void AddAction(ActionType type,int count);
    void AddRecord(float x,float y,float r);
    void AddActions(std::vector<Action> actions);
    void ClearActions();
    bool IsEmpty();
protected:
    MUTEX m_mutex;
    AutoCtrlParam m_param;
    std::queue<Action> m_actions;
    bool m_startRotate;
    float m_threshold;
    float m_targetAngle;
    bool m_meetThres;
    IMU *m_pIMU;
};
