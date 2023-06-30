#include "AutoControl.h"

AutoCtrl::AutoCtrl()
{
    m_param.hop = false;
    m_param.r = m_param.x = m_param.y = 0;
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
            case hopToBalance:{m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::HopAndLean;}break;
            case balanceRestore:{m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;m_param.hopType = Controller::Restore;}break;
            case climb:{m_param.hop = false;m_param.r = m_param.x = 0;m_param.y = 1;}break;
            case record:{
                m_param.hop = false;
                Action &act = m_actions.front();
                m_param.x = act.x;
                m_param.y = act.y;
                m_param.r = act.r;
            }break;
        }
        switch(type)
        {
            case hop:
            case hopToBalance:
            case balanceRestore:m_actions.pop();break;
            default:
            {
                m_actions.front().actionCnt--;
                if(m_actions.front().actionCnt <= 0)m_actions.pop();
            }break;
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
