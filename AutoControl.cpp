#include "AutoControl.h"

AutoCtrl::AutoCtrl()
{
    m_param.hop = false;
    m_param.r = m_param.x = m_param.y = 0;
}

void AutoCtrl::UpdateStep()
{
    if(m_actions.empty())
    {
        m_param.hop = false;
        m_param.r = 0;
        m_param.x = 0;
        m_param.y = 0;
    }else
    {
        ActionType type = m_actions.front().action;
        switch(type)
        {
            case run:{m_param.hop = false;m_param.r = m_param.x = 0;m_param.y = 1;}break;
            case back:{m_param.hop = false;m_param.r = m_param.x = 0;m_param.y = -1;}break;
            case stop:{m_param.hop = false;m_param.r = m_param.x = m_param.y = 0;}break;
            case turnR:{m_param.hop = false;m_param.r = -1;m_param.x = 0;m_param.y = 0.5;}break;
            case turnL:{m_param.hop = false;m_param.r = 1;m_param.x = 0;m_param.y = 0.5;}break;
            case rotateR:{m_param.hop = false;m_param.r = 1;m_param.x = m_param.y = 0;}break;
            case rotateL:{m_param.hop = false;m_param.r = 1;m_param.x = m_param.y = 0;}break;
            case moveR:{m_param.hop = false;m_param.r = m_param.y = 0;m_param.x = 1;}break;
            case moveL:{m_param.hop = false;m_param.r = m_param.y = 0;m_param.x = -1;}break;
            case hop:{m_param.hop = true;m_param.r = m_param.x = m_param.y = 0;}break;
        }
        m_actions.front().actionCnt--;
        if(m_actions.front().actionCnt <= 0)m_actions.pop();
    }
}

void AutoCtrl::AddAction(Action action)
{
    m_actions.push(action);
}

void AutoCtrl::AddActions(std::vector<Action> actions)
{
    for(int i = 0;i<actions.size();++i)
        this->m_actions.push(actions[i]);
}

void AutoCtrl::ClearActions()
{
    while(!m_actions.empty())m_actions.pop();
    m_param.hop = false;
    m_param.r = 0;
    m_param.x = 0;
    m_param.y = 0;
}

bool AutoCtrl::IsEmpty()
{
    return m_actions.empty();
}