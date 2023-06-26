#include "fcntl.h"
#include "MultiThread.h"


class AutoCtrl
{
    struct AutoProcess
    {
        int stepCnt;
        int status;  
    };
public:
    enum GameType
    {
        speed,
        obstacle
    };
    struct AutoCtrlParam
    {
        float x,float y,float r;
        bool hop;
    };
    protected:
    static void* _threadFunc(void* p);

    void _speedCmp(int msgId);
    void _obstCmp(int msgId);
    void _sendMsg(int msgId);
public:
    void Start();
    void Exit();
    AutoCtrl(GameType type);
    void UpdateStep();
    void GetAutoCtrlParam(AutoCtrlParam& param);

    protected:
    GameType m_gameType;
    AutoCtrlParam m_param;
    THREAD m_threadDesc;
    bool m_sysQuit;
    AutoProcess m_process;
};