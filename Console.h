#include "MultiThread.h"
#include <time.h>
#include <termios.h>

class Console
{
    protected:
    static int getch_();
    static void* consoleFunc(void* arg);

    void console_();
    void outstatus();
    public:
    Console(float kp,float kw,float zeroThres);
    ~Console();
    void Start();
    void Exit();

    bool IsMannualControl(){return m_mannual;}
    bool IsDogStop(){return m_stop;}
    bool IsAutoControl(){return m_auto;}
    bool IsRequestExit(){return m_quit;}

    void UpdateMannualParams(float& x,float& y,float& r);
    void Update(bool stepOver);


    protected:
    THREAD m_threadDesc;
    MUTEX m_mutexDesc;
    clock_t m_lastUpdateTime;
    float m_ctrl_x,m_ctrl_y,m_ctrl_r;
    float m_ctrlTime;
    float m_out_x,m_out_y,m_out_r;
    float m_inc_x,m_inc_y,m_inc_r;
    int m_need_mannual,m_need_auto,m_need_stop,m_need_quit;
    bool m_mannual,m_auto,m_stop,m_quit;

    float m_kp,m_kw;
    bool m_threadQuit;
    float m_zeroThres;
    float m_his_x,m_his_y,m_his_r;
};
