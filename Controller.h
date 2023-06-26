#pragma once
#include "eigen-3.4.0/Eigen/Eigen"
#include "PacePlanner.h"
#include "Sensor.h"
#include "LegController.h"

class Controller
{
    public:
    struct MechParam
    {
        MechParam(float dogWidth,float dogLength):
        dogWidth(dogWidth),dogLength(dogLength){}
        float dogWidth,dogLength;
    };
    struct CtrlInitParam
    {
        CtrlInitParam(
        float kp,float kw,
        float maxVelFw,
        float maxVelVt,
        float maxVelRt,
        float movingThreshold):
        kp(kp),kw(kw),
        maxVelFw(maxVelFw),
        maxVelRt(maxVelRt),
        maxVelVt(maxVelVt),
        movingThreshold(movingThreshold){}
        float kp,kw;
        float maxVelFw,maxVelVt,maxVelRt;
        float movingThreshold;
    };
public:
    Controller(
        std::vector<std::string> serialName,
        std::vector<AxisMovement> motorAngle,
        std::vector<AxisMovement> realAngle,
        std::vector<std::vector<float>> motorSign,
        LegController::VMCParam param,
        CtrlInitParam initParam,
        MechParam mcParam);
    ~Controller();

    void EnableSmoothCtrl(bool enable);
    bool Update(float velX,float velY,float velYaw,bool hop,bool restrictHop = false);
    void EnableVMC(bool enable);
    void Start(float startUpTime);
    void Exit();
    bool IsMoving(){return m_moving;}
    bool IsStop(){return m_stop;}
    void StopMoving();
    void StartMoving();
    PacePlanner& GetPacePlanner();
    protected:
    void _doHop();
    void _updateVel(float x,float y,float r,float deltaTime);

protected:
    bool m_bVMCCtrl;
    LegController* m_pControllers[4];
    PacePlanner m_planner;
    clock_t m_time;
    std::vector<FeetMovement> m_pos;
    std::vector<bool> m_touchStatus;

    //bool m_usingGlobalRecord;

    bool m_smthCtrl;
    float m_maxVel[3];
    float m_outVel[3];
    float m_hisVel[3];
    float m_incVel[3];
    float m_kp,m_kw;
    bool m_needStop;
    bool m_needHop;
    bool m_stop;
    bool m_moving;
    float m_movingThres;
};
