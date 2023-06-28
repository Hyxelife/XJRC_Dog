#pragma once

#include "Gait.h"
#include "Kinematics.h"

class PacePlanner
{
public:
    enum PaceType
    {
        cycCurve,
    };
    void Reset();
    PacePlanner(float maxStepFwd,float maxStepVrt,float maxRotation,float dogWidth,float dogLength);
    void SetPaceType(PaceType type);
    void SetGait(Gait gait,float transitionTime);

    void SetVelocity(float forward,float vertical,float rotation);
    void SetDogHeight(float height);
    void SetDogOffsetX(float offset);
    void SetCurveHeight(float height);
    bool Update(float deltaTime,std::vector<FeetMovement>& move,std::vector<bool>& touch);
    void EnableVMC(bool vmcEnable);
    void SetClimbAngle(float angle);
protected:
    float _cycloidCurve(float time,float height);

protected:
    float m_maxStepForward;
    float m_maxStepVertical;
    float m_maxRotation;

    float m_dogDirX;
    float m_dogDirY;
    float m_dogRadius;
    float m_dogHeight;
    float m_curveHeight;
    float m_offsetX;

    using vec2 = std::pair<float,float>;
    vec2 m_offset[4];
    vec2 m_lastOffset[4];
    bool m_legSwinging[4];
    PaceType m_type;
    float m_totalTime;
    float m_gaitLerpTime;

    Gait m_gait;
    Gait m_lastGait;
    bool m_bMVCCtrl;
    float m_centerX,m_centerY;
    float m_climbAngle;
};
