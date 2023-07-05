#include "AutoControl.h"

AutoCtrl::AutoCtrl(IMU* pIMU)
{
    m_param.hop = false;
    m_param.r = m_param.x = m_param.y = 0;
    m_startAuto = false;
    m_startRotate = false;
    m_meetThres = false;
    m_threshold = 2.0;
    m_kp = 0.05;
    m_kp_str = 0.05;
    m_lastDiff = 0;
    m_pIMU = pIMU;
    mutex_create(m_mutex);
}

AutoCtrl::~AutoCtrl()
{
    mutex_destroy(m_mutex);
}
#include <stdio.h>
void AutoCtrl::UpdateStep()
{

    mutex_lock(m_mutex);
    if(m_actions.empty())
    {
        m_param.hop = false;
        m_param.r = 0;
        m_param.x = 0;
        m_param.y = 0;
    }else
    {
    printf("[Auto]:update steps!\n");
        ActionType type = m_actions.front().action;
        if(type == autoRotateTo || type == autoRotateWith)
        {
            float yaw = m_pIMU->GetIMUData().yaw;
            float delta = yaw-m_targetAngle;

            if(fabsf(delta) >= 180.0)
            {
                if(delta > 0)delta -= 360.0;
                else delta += 360.0;
            }
            if(!m_startRotate)
            {
                m_startRotate = true;
                m_meetThres = false;
                if(type == autoRotateTo)m_targetAngle = m_actions.front().r;
                else m_targetAngle = m_pIMU->GetIMUData().yaw+m_actions.front().r;
                if(m_targetAngle>180)m_targetAngle-= 360.0;
                else if(m_targetAngle < -180)m_targetAngle+=360.0;
                m_lastDiff = delta;
            }


            float p = -m_kp* delta;
            if(p >= 1)p = 1;
            if(p<=-1)p = -1;
            m_param.hop = false;
            m_param.x = m_param.y = 0;

            if(!m_meetThres)
                m_param.r = p;
            else
                m_param.r = 0.3*p;
            printf("[Auto] r=%.3f,delta=%.3f,over=%d\n",m_param.r,delta,m_meetThres);
            if(delta * m_lastDiff <= 0)
                m_meetThres = true;

            if(fabsf(yaw-m_targetAngle) <= m_threshold)
            {
                if(m_meetThres)
                {
                    printf("[AutoRotate]ok!\n");
                    m_actions.pop();
                    m_startRotate = false;
                    m_meetThres = false;
                    m_param.r = 0;
                }
            }
            m_lastDiff = delta;
            printf("[AutoRotate]:cur:%.3f,tar:%.3f\n",yaw,m_targetAngle);

        }else
        {
            switch(type)
            {
                case run:{m_param.hop = false;m_param.r = m_param.x = 0;m_param.y = 1;}break;
                case back:{m_param.hop = false;m_param.r = m_param.x = 0;m_param.y = -1;}break;
                case stop:{m_param.hop = false;m_param.r = m_param.x = m_param.y = 0;}break;
                case turnR:{m_param.hop = false;m_param.r = -0.5;m_param.x = 0;m_param.y = 0.8;}break;
                case turnL:{m_param.hop = false;m_param.r = 0.5;m_param.x = 0;m_param.y = 0.8;}break;
                case rotateR:{m_param.hop = false;m_param.r = -0.5;m_param.x = m_param.y = 0;}break;
                case rotateL:{m_param.hop = false;m_param.r = 0.5;m_param.x = m_param.y = 0;}break;
                case moveR:{m_param.hop = false;m_param.r = m_param.y = 0;m_param.x = 1;}break;
                case moveL:{m_param.hop = false;m_param.r = m_param.y = 0;m_param.x = -1;}break;
                case hop:{m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::HopForward;}break;
                case stepAndSpan:{m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::StepAndSpan;}break;
                case stepAndRestore:{m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::StepToRestore;}break;
                case clawForward:{m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::Claw;}break;
                case clawRight:{m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::ClawRight;}break;
                case clawLeft:{m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::ClawLeft;}break;
                case hold:{m_param.hop = false;m_param.r = m_param.x = m_param.y = 0;}break;
                case record:{
                    m_param.hop = false;
                    Action &act = m_actions.front();
                    m_param.x = act.x;
                    m_param.y = act.y;
                    m_param.r = act.r;
                }break;
                case straight:
                {
                    Action &act = m_actions.front();
                    if(!m_startAuto)
                    {
                        while(fabsf(act.r) >= 180.0)
                        {
                            if(act.r > 0)act.r -= 360.0;
                            else act.r += 360.0;
                        }  
                        m_startAuto = true;
                        m_targetAngle = act.r;
                    }
                    float delta = m_pIMU->GetIMUData.yaw-m_targetAngle;
                    if(delta > 180.0)act.r -= 360.0;
                    else if(delta < -180.0)delta += 360.0;
                    m_param.hop = false;
                    m_param.x = 0;
                    m_param.y = 1;
                    m_param.r = m_kp_str*delta;
                }
            }
            switch(type)
            {
                case hop:
                case stepAndSpan:
                case stepAndRestore:
                case clawForward:
                case clawRight:
                case clawLeft:m_actions.pop();break;
                default:
                {
                    m_actions.front().actionCnt--;
                    if(m_actions.front().actionCnt <= 0)
                    {
                        m_startAuto = false;
                        m_actions.pop();
                    }
                }break;
            }
        }

    }
    mutex_unlock(m_mutex);
}

void AutoCtrl::AddRecord(float x,float y,float r)
{
    mutex_lock(m_mutex);
    Action act;
    act.action = record;
    act.actionCnt = 1;
    act.x = x;
    act.y = y;
    act.r = r;
    m_actions.push(act);
    mutex_unlock(m_mutex);
}

void AutoCtrl::AddAction(Action action)
{
    mutex_lock(m_mutex);
    m_actions.push(action);
    mutex_unlock(m_mutex);
}

void AutoCtrl::AddAction(ActionType type,int count)
{
    mutex_lock(m_mutex);
    Action act;
    act.action = type;
    act.actionCnt = count;
    m_actions.push(act);
    mutex_unlock(m_mutex);
}

void AutoCtrl::AddActions(std::vector<Action> actions)
{
    mutex_lock(m_mutex);
    for(int i = 0;i<actions.size();++i)
        this->m_actions.push(actions[i]);
    mutex_unlock(m_mutex);
}

void AutoCtrl::ClearActions()
{
    mutex_lock(m_mutex);
    while(!m_actions.empty())m_actions.pop();
    m_param.hop = false;
    m_param.r = 0;
    m_param.x = 0;
    m_param.y = 0;
    mutex_unlock(m_mutex);
}

bool AutoCtrl::IsEmpty()
{
mutex_lock(m_mutex);
    bool empty = m_actions.empty();
    mutex_unlock(m_mutex);
    return empty;
}
