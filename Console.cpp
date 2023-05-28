#include "Console.h"
#include <stdio.h>
#include <math.h>


int Console::getch_(void)
{
     struct termios tm, tm_old;
     int fd = 0, ch;

     if (tcgetattr(fd, &tm) < 0) {//保存现在的终端设置
          return -1;
     }

     tm_old = tm;
     cfmakeraw(&tm);//更改终端设置为原始模式，该模式下所有的输入数据以字节为单位被处理
     if (tcsetattr(fd, TCSANOW, &tm) < 0) {//设置上更改之后的设置
          return -1;
     }

     ch = getchar();
     if (tcsetattr(fd, TCSANOW, &tm_old) < 0) {//更改设置为最初的样子
          return -1;
     }

     return ch;
}

void Console::outstatus()
{
    printf("n_mannual:%d,mannual:%d\nn_auto:%d,auto:%d\nn_stop:%d,stop:%d\nn_quit:%d,quit:%d\nctrl[%.3f,%.3f,%.3f]\nout[%.3f%.3f%.3f]\nerr_his[%.3f,%.3f,%.3f]\n\
    inc[%.3f,%.3f,%.3f]"
    ,m_need_mannual,m_mannual,m_need_auto,m_auto,m_need_stop,m_stop,m_need_quit,m_quit,m_ctrl_x,m_ctrl_y,m_ctrl_r,m_out_x,m_out_y,m_out_r,
    m_his_x,m_his_y,m_his_r,m_inc_x,m_inc_y,m_inc_r);
}

void* Console::consoleFunc(void* arg)
{
    Console* pThis = (Console*)arg;
    printf("[Console]:start up!\n");
    pThis->console_();
}

#define CTRL_TIME   0.5f

void Console::console_()
{
    char cmd;
    while(!m_threadQuit)
    {
        if(m_mannual)
        {
            cmd = getch_();
            printf("get:%c\n",cmd);
            outstatus();
            if(m_need_mannual == 0 || m_need_stop == 1)continue;
            mutex_lock(m_mutexDesc);

            switch(cmd)
            {
                case 'q':m_ctrl_r = 1;m_ctrlTime = CTRL_TIME;m_need_stop = 0;break;
                case 'e':m_ctrl_r = -1;m_ctrlTime = CTRL_TIME;m_need_stop = 0;break;
                case 'w':m_ctrl_y = 1;m_ctrlTime = CTRL_TIME;m_need_stop = 0;break;
                case 'a':m_ctrl_x = -1;m_ctrlTime = CTRL_TIME;m_need_stop = 0;break;
                case 's':m_ctrl_y = -1;m_ctrlTime = CTRL_TIME;m_need_stop = 0;break;
                case 'd':m_ctrl_x = 1;m_ctrlTime = CTRL_TIME;m_need_stop = 0;break;
                case 'f':m_need_stop = 1;break;
                case 'x':outstatus();break;
                case 'p':m_need_mannual = 0;m_ctrl_x = m_ctrl_y = m_ctrl_r = 0;break;
            }
            mutex_unlock(m_mutexDesc);
        }else if(m_auto)
        {
            scanf("%c",&cmd);
            if(m_need_auto == 0)continue;
            switch(cmd)
            {
                case 'Q':
                case 'q':m_need_auto = 0;break;
            }
        }else
        {
            scanf("%c",&cmd);
            switch(cmd)
            {
                case 'H':
                case 'h':
                {
                    printf("**********************************\n");
                    printf("*           help page            *\n");
                    printf("**********************************\n");
                    printf("[h/H] help page\n");
                    printf("[m/M] mannually control\n");
                    printf("[a/A] automatically control\n");
                    printf("[x/X] quit system\n");
                    printf("[b/B] abort system\n");
                    printf("when system is mannually controlling:\n");
                    printf("\t[w/a/s/d] moving control\n");
                    printf("\t[q/e] rotating control\n");
                    printf("\t[p] quit control\n");
                    printf("when system is automatically controlling:\n");
                    printf("\t[q] quit\n");
                }break;
                case 'm':
                case 'M':m_need_mannual = 1;m_need_stop = 0;break;
                case 'a':
                case 'A':m_need_auto = 1;break;
                case 'q':
                case 'Q':m_need_quit = 1;break;
                case 'b':
                case 'B':m_quit = true;m_stop = true;break;
            }
        }
        while(!m_quit && (m_need_mannual != -1 || m_need_auto != -1 || m_need_quit != -1)){printf("blocking...\n");outstatus();}
    }
    printf("[Console]:system quit!\n");
}

Console::Console(float kp,float kw,float zerosThres)
:m_kp(kp),m_kw(kw)
{
    m_lastUpdateTime = -1;
    m_auto = false;
    m_mannual = false;
    m_stop = true;
    m_quit = false;

    m_need_mannual = m_need_auto = m_need_stop = m_need_quit = -1;

    m_threadQuit = true;
    m_zeroThres = zerosThres;
    m_ctrl_x = m_ctrl_y = m_ctrl_r =
    m_out_x = m_out_y = m_out_r =
    m_his_r = m_his_x  =m_his_y =
    m_inc_x = m_inc_y = m_inc_r =
    m_out_r = m_out_x = m_out_y = 0;
    mutex_create(m_mutexDesc);
}

void Console::Start()
{
    if(!m_threadQuit)return;
    m_threadQuit = false;
    thread_create(Console::consoleFunc,this,m_threadDesc);

}

void Console::Exit()
{
    m_threadQuit = true;
    thread_join(m_threadDesc);
    thread_destroy(m_threadDesc);
}

Console::~Console()
{
    mutex_destroy(m_mutexDesc);
}

void Console::UpdateMannualParams(float& x,float& y,float& r)
{
    if(!m_mannual){m_lastUpdateTime = -1;return;}
    if(m_stop){m_lastUpdateTime = -1;return;}
    if(m_lastUpdateTime == -1){m_lastUpdateTime = clock();return;}
    clock_t curTime = clock();
    float dt = curTime-m_lastUpdateTime;
    m_lastUpdateTime = curTime;
    dt /= (float)CLOCKS_PER_SEC;
    mutex_lock(m_mutexDesc);
    float err_x = m_ctrl_x - m_out_x,err_y = m_ctrl_y-m_out_y,err_r = m_ctrl_r-m_out_r;
    if(m_ctrlTime <= 0)
    {m_ctrl_x = 0;m_ctrl_y = 0;m_ctrl_r = 0;m_ctrlTime = 0;}
    m_ctrlTime-=dt;
    mutex_unlock(m_mutexDesc);

    m_inc_x += m_kp*err_x*dt+m_kw*(err_x-m_his_x);
    m_inc_y += m_kp*err_y*dt+m_kw*(err_y-m_his_y);
    m_inc_r += m_kp*err_r*dt+m_kw*(err_r-m_his_r);

    m_out_x += m_inc_x*dt;
    //printf("%.3f\n",dt*1000);
    m_out_y += m_inc_y*dt;
    m_out_r += m_inc_r*dt;
    m_his_x = err_x;
    m_his_y = err_y;
    m_his_r = err_r;

    x = m_out_x;
    y = m_out_y;
    r = m_out_r;
}

void Console::Update(bool stepOver)
{
    if(m_stop)
    {
        m_lastUpdateTime = -1;
        if(m_need_stop == 0)
        {
            m_stop = false;
            printf("[Console]:moving...\n");
        }
        m_need_stop = -1;

        if(m_need_quit == 1)
        {
            m_quit =true;
            printf("[Console]:quitting...\n");
            m_need_quit = -1;
        }
        if(m_need_mannual != -1)
        {
            m_mannual = m_need_mannual;
            if(m_mannual)
                printf("[Console]:enter mannual mode!\n");
            else
                printf("[Console]:exit mannual mode!\n");
            m_need_mannual = -1;
        }
        if(m_need_auto == 1)
        {
            m_auto = true;
            printf("[Console]:enter auto mode!\n");
            m_need_auto = -1;
        }

    }else if(stepOver)
    {
        if(fabsf(m_out_x) <= m_zeroThres && fabsf(m_out_y) <= m_zeroThres&& fabsf(m_out_r) <= m_zeroThres)
        {
            if(m_need_mannual == 0)
            {
                m_mannual = false;
                m_stop = true;
                printf("[Console]:exit mannual mode!\n");
                m_need_mannual = -1;
            }
            if(m_need_auto == 0)
            {
                m_auto = false;
                m_stop = true;
                printf("[Console]:exit auto mode!\n");
                m_need_auto = -1;
            }
            if(m_need_quit == 1)
            {
                m_stop = true;
                m_quit = true;
                printf("[Console]:quitting...\n");
            }
            if(m_need_stop == 1)
            {
                m_stop = true;
                printf("[Console]:stopped!\n");
                m_need_stop = -1;
            }
        }
    }

}
