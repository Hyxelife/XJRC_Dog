#include "PacePlanner.h"
#include <iostream>
using namespace std;

PacePlanner::PacePlanner(float maxStepFwd,float maxStepVrt,float maxRotation,float dogWidth,float dogLength)
    :m_maxStepForward(maxStepFwd),
    m_maxStepVertical(maxStepVrt),
    m_maxRotation(maxRotation),
    m_gait({0,0,0,0},{1,1,1,1},1),
    m_lastGait({0,0,0,0},{1,1,1,1},1)
{
    m_dogRadius = sqrtf(dogWidth*dogWidth+dogLength*dogLength);
    m_dogDirX =  dogWidth/m_dogRadius;
    m_dogDirY = dogLength/m_dogRadius;
    m_dogRadius /= 2.0f;
    for(int i = 0;i<4;++i)
    {
        m_offset[i] = m_lastOffset[i] = {0.0f,0.0f};
        m_legSwinging[i] = false;
    }
}

float PacePlanner::_cycloidCurve(float time,float height)
{
    const float pi_2 = 3.141592653589f*2.0f;
    float phi = time*pi_2;
    return 0.5f*height*(1.0f-cos(phi));
}

void PacePlanner::SetPaceType(PaceType type)
{
    m_type = type;
}
void PacePlanner::SetGait(Gait gait,float transitionTime)
{
    m_lastGait = m_gait;
    m_gait = gait;
    m_gaitLerpTime = transitionTime;
}
void PacePlanner::SetDogOffsetX(float offset)
{
    m_offsetX = offset;
}

void PacePlanner::Reset()
{
    m_totalTime = 0;
    for (int i = 0; i < 4; ++i)
        m_offset[i] = m_lastOffset[i] = { 0,0 };
}

void PacePlanner::SetVelocity(float forward,float vertical,float rotation)
{
    if(m_bMVCCtrl)return;
    if (forward > 1.0f)forward = 1.0f;
    else if (forward < -1.0f)forward = -1.0f;

    if (vertical > 1.0f)vertical = 1.0f;
    else if (vertical < -1.0f)vertical = -1.0f;

    if (rotation > 1.0f)rotation = 1.0f;
    else if (rotation < -1.0f)rotation = -1.0f;


    float stepFwd = forward * m_maxStepForward;
    float stepVert = vertical * m_maxStepVertical;
    float stepRot = rotation * m_maxRotation * m_dogRadius;


    m_offset[RF] = { stepVert - stepRot * m_dogDirY,stepFwd + stepRot * m_dogDirX };
    m_offset[LF] = { stepVert - stepRot * m_dogDirY,stepFwd - stepRot * m_dogDirX };
    m_offset[LB] = { stepVert + stepRot * m_dogDirY,stepFwd - stepRot * m_dogDirX };
    m_offset[RB] = { stepVert + stepRot * m_dogDirY,stepFwd + stepRot * m_dogDirX };

    //for(int i = 0;i<4;++i)
     //   std::cout<<"applay motor:"<<i<<"pos:"<<m_offset[i].first<<","<<m_offset[i].second<<endl;

}

bool PacePlanner::Update(float deltaTime,std::vector<FeetMovement>& move,std::vector<bool>& touch)
{
float centerY = -5;
    bool loopOver = false;
    m_totalTime += deltaTime;
    //cout<<"totalTime:"<<m_totalTime<<endl;
    if(m_totalTime >= m_gait.GetTotalDuration())
    {
        cout<<"1s"<<endl;
        loopOver = true;
        m_totalTime -= m_gait.GetTotalDuration();
    }
    std::vector<float> legTime = m_gait.GetMovingStatus(m_totalTime);

    float sign = 1;
    if(m_bMVCCtrl)
    {

        for(int i = 0;i<4;++i)
        {
            if (i == LF || i == LB)sign = -1;
            else sign = 1;
            move[i].x = move[i].y=0;
            move[i].x += sign * m_offsetX;
            if (legTime[i] <= 1.0f)
            {
                move[i].z = -m_dogHeight + _cycloidCurve(legTime[i], m_curveHeight);
                touch[i] = false;
            }
            else
            {
                move[i].z = -m_dogHeight;
                touch[i] = true;
            }
        }
    }else
    {

        for(int i = 0;i<4;++i)
        {
            if (i == LF || i == LB)sign = -1;
            else sign = 1;
            if(legTime[i]<=1.0f)
            {
                if(!m_legSwinging[i])
                {
                    m_legSwinging[i] = true;
                    m_lastOffset[i] = m_offset[i];
                }
                move[i].x = sign*m_offsetX+(1.0f-legTime[i])*-m_lastOffset[i].first+legTime[i]*m_offset[i].first;
                move[i].y = (1.0f-legTime[i])*-m_lastOffset[i].second+legTime[i]*m_offset[i].second+centerY;
                move[i].z = -m_dogHeight+_cycloidCurve(legTime[i],m_curveHeight);
                m_legSwinging[i] = true;
                touch[i] = false;
            }else{
                if(m_legSwinging[i])
                {
                    m_legSwinging[i] = false;
                    m_lastOffset[i] = m_offset[i];
                }
                move[i].x = sign * m_offsetX+(2.0f-legTime[i])*m_lastOffset[i].first-(legTime[i]-1.0f)*m_offset[i].first;
                move[i].y = (2.0f-legTime[i])*m_lastOffset[i].second-(legTime[i]-1.0f)*m_offset[i].second+centerY;
                move[i].z = -m_dogHeight;
                touch[i] = true;
            }
        }
    }
    return loopOver;
}

void PacePlanner::EnableVMC(bool vmcEnable)
{
    m_bMVCCtrl = vmcEnable;
}

void PacePlanner::SetDogHeight(float height)
{
    m_dogHeight = height;
}
void PacePlanner::SetCurveHeight(float height)
{
    m_curveHeight = height;
}
