#include "MultiThread.h"
#include <time.h>
#include <termios.h>

class Console
{
    protected:
    int getch_();
    void* consoleFunc(void* arg);
    public:
    Console(float kp,float ki);
    void Start();
    void Exit();

    bool IsMannualControl(){return m_mannual}
    bool IsDogIdle(){return m_idling;}
    bool IsAutoControl(){return m_auto;}
    bool IsRequestExit(){return m_needQuit;}

    void GetMannualParams(float& x,float& y,float& r);
    void UpdateControlParams();
    

    protected:
    THREAD m_threadDesc;
    MUTEX m_mutexDesc;
    clock_t m_lastUpdateTime;
    float m_ctrl_x,m_ctrl_y,m_ctrl_r;
    float m_out_x,m_out_y,m_out_r;
    bool m_mannual;
    bool m_auto;
    float m_kp,m_ki;
    bool m_sysQuit;
    bool m_idling;
    float m_his_x,m_his_y,m_his_r;
    bool m_needQuit;
};