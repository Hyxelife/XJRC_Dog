#include "AutoControl.h"

#define MSG_NULL        -1

#define MSG_WAKE        0
#define MSG_TURN        1
#define MSG_ADJ_LEFT    2
#define MSG_ADJ_RIGHT   3
#define MSG_STOP        4
#define MSG_HOP         5

#define STA_STOP            0
#define STA_SPD_START_RUN   1
#define STA_SPD_TURN        2
#define STA_SPD_FINAL       3


#define STA_OBS_START       1
#define STA_OBS_BRIDGE      2
#define STA_OBS_BALANCE     3
#define STA_OBS_STAIR           4
#define STA_OBS_FINAL           5    
#define STA_OBS_TURN            6   






void* AutoCtrl::_threadFunc(void* p)
{
    AutoCtrl* pThis = (AutoCtrl*) p;
    
    //TODO: out layer [communication]
    

    return 0;
}




AutoCtrl::AutoCtrl(GameType type)
:m_gameType(type)
{
    m_sysQuit = true;
    m_process.status = 0;
    m_process.stepCnt = 0;
}   

void AutoCtrl::UpdateStep()
{
    m_process.stepCnt++;
    if(m_gameType == speed)
    {
        switch(m_process.status)
        {
            case STA_SPD_START_RUN:
            case STA_SPD_TURN:
            case STA_SPD_FINAL:
        }
    }else if(m_gameType == obstacle)
    {
        switch(m_process.status)
        {
            case STA_OBS_START:
            case STA_OBS_BRIDGE:
            case STA_OBS_TURN:
            case STA_OBS_BALANCE:
            case STA_OBS_STAIR:
            case STA_OBS_FINAL:
        }
    }
}

void AutoCtrl::_speedCmp(int msgId)
{

}

void AutoCtrl::_obstCmp(int msgId)
{

}

void AutoCtrl::_sendMsg(int msgId)
{

}

void AutoCtrl::Start()
{
    if(!m_sysQuit)return;
    m_sysQuit = false;
    thread_create(AutoCtrl::_threadFunc,this,m_threadDesc);
}

void AutoCtrl::Exit()
{
    m_sysQuit = true;
    thread_join(m_threadDesc);
    thread_destroy(m_threadDesc);
}