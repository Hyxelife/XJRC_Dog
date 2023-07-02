#pragma once
#include "eigen-3.4.0/Eigen/Eigen"
#include "PacePlanner.h"
#include "Sensor.h"
#include "LegController.h"

class Controller
{

    struct Step__tp
    {
        float duration;
        unsigned char procID;
        FeetMovement targetPos;
    };
    public:
    enum HopType
    {
        HopForward,
        //TestMotor,
        StepAndSpan,
        LerpToRestore,
        StepToRestore,
        Claw,
        StepAndClaw,
        ClawLeft,
        ClawRight,

    };
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
    bool Update(float velX,float velY,float velYaw,bool hop,HopType type,bool restrictHop = false);
    void EnableVMC(bool enable);
    void Start(float startUpTime);
    void Exit();
    bool IsMoving(){return m_moving;}
    bool IsStop(){return m_stop;}
    void StopMoving();
    void StartMoving();
    PacePlanner& GetPacePlanner();
    void GetCurrentVelocity(float &x,float &y,float &r);
    protected:
    void _doHop(HopType type);
    void _updateVel(float x,float y,float r,float deltaTime);

    //Hops
    void _hopForward();
    void _stepToChange(float offsetX,float offsetY);
    void _lerpRestore();
    void _claw(float x,float y);
    void lerp_(FeetMovement new_[4],float time);
    void doMovement__tp(Step__tp* animation[4],int stepCount);
    void ClawFwdOneStep__tp();
    void VerticalAdjOneStep__tp(double vertical);

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
    HopType m_hopType;
    bool m_stop;
    bool m_moving;
    float m_movingThres;
    float m_leanAngle;
};
