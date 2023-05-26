#include "Console.h"


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

void* Console::consoleFunc(void* arg)
{
    Console* pThis = (Console*)arg;
    char cmd;
    while(!pThis->m_sysQuit)
    {
        if(pThis->m_mannual)
        {
            cmd = getch_();
            switch(cmd)
            {
                case 'q':m_ctrl_r = 1;m_idling = false;break;
                case 'e':m_ctrl_r = -1;m_idling = false;break;
                case 'w':m_ctrl_y = 1;m_idling = false;break;
                case 'a':m_ctrl_x = -1;m_idling = false;break;
                case 's':m_ctrl_y = -1;m_idling = false;break;
                case 'd':m_ctrl_x = 1;m_idling = false;break;
                case 'f':m_idling = true;break;
                case 'p':m_mannual = false;printf("[console]:mannual control quit\n");break;
            }
        }else if(pThis->m_auto)
        {
            scanf("%c",&cmd);
            switch(cmd)
            {
                case 'Q':
                case 'q':m_auto = false;break;
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
                    printf("[x/X] quit sustem\n");
                    printf("when system is mannually controlling:\n");
                    printf("\t[w/a/s/d] moving control\n");
                    printf("\t[q/e] rotating control\n");
                    printf("\t[p] quit control\n");
                    printf("when system is automatically controlling:\n");
                    printf("\t[q] quit\n");
                }break;
                case 'm':
                case 'M':m_auto = false;m_mannual = true;printf("[console]:entering mannual control.\n");break;
                case 'a':
                case 'A':m_auto = true;m_mannual = false;break;
                case 'X':
                case 'x':m_needQuit = true;break;
            }
        }
        
    }
}

Console::Console(float kp,float ki)
:m_kp(kp),m_ki(ki)
{
    m_lastUpdateTime = -1;
    m_auto = false;
    m_mannual = false;
    m_idling = true;
    m_sysQuit = true;
    mutex_create(m_mutexDesc);
}

void Console::Start()
{
    if(m_sysQuit)return;
    m_sysQuit = false;
    thread_create(Console::consoleFunc,this,m_threadDesc);
}

void Console::Exit()
{
    m_sysQuit = true;
    thread_join(m_threadDesc);
    thread_destroy(m_threadDesc);
}

Console::~Console()
{
    mutex_destroy(&m_mutexDesc);
}

void Console::GetMannualParams(float& x,float& y,float& r)
{
    x = m_out_x;
    y = m_out_y;
    r = m_out_r;
}

void Console::UpdateControlParams()
{
    if(!m_mannual)return;
    if(m_lastUpdateTime == -1){m_lastUpdateTime = clock();return;}
    clock_t curTime = clock();
    float dt = curTime-m_lastUpdateTime;
    m_lastUpdateTime = curTime;
    dt /= (float)CLOCKS_PER_SEC;
    mutex_lock(m_mutexDesc);
    float err_x = m_ctrl_x - m_out_x,err_y = m_ctrl_y-m_out_y,err_r = m_ctrl_r-m_out_r;
    m_ctrl_x = 0;m_ctrl_y = 0;m_ctrl_r = 0;
    mutex_unlock(m_mutexDesc);
    float inc_x = m_kp*(err_x-m_his_x)+dt*m_ki*(err_x),
          inc_y = m_kp*(err_y-m_his_y)+dt*m_ki*(err_y),
          inc_r = m_kp*(err_x-m_his_r)+dt*m_ki*(err_r);


    m_out_x += inc_x;
    m_out_y += inc_y;
    m_out_r += inc_r;
    m_his_x = err_x;
    m_his_y = err_y;
    m_his_r = err_r;
}