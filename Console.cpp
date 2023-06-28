#include "Console.h"
#include <stdio.h>
#include <math.h>
#include <linux/input.h>
#include <unistd.h>
#include <fcntl.h>

////////////key masks///////////////
#define MASK_A          0x01
#define MASK_S          0x02
#define MASK_D          0x04
#define MASK_Q          0x08
#define MASK_W          0x10
#define MASK_E          0x20
#define MASK_QUIT       0x40
#define MASK_HOP        0x80
#define MASK_STAND      0x100
#define MASK_HOPDOWN    0x200
#define MASK_HOPTEST    0x400

void Console::_updateKeyEvents()
{
    input_event key_info;
    //m_event.read((char*)&key_info,sizeof(key_info));
    if(read(m_eventFd,&key_info,sizeof(input_event)) > 0)
    {
        //printf("event\n");
        if(key_info.type != EV_KEY)return;
        unsigned int mask = 0;
        switch(key_info.code)
        {
            case KEY_A:mask = MASK_A;break;
            case KEY_S:mask = MASK_S;break;
            case KEY_D:mask = MASK_D;break;
            case KEY_Q:mask = MASK_Q;break;
            case KEY_W:mask = MASK_W;break;
            case KEY_E:mask = MASK_E;break;
            case KEY_SPACE:mask = MASK_HOP;break;
            case KEY_P:mask = MASK_QUIT;break;
            case KEY_F:mask = MASK_STAND;break;
            case KEY_J:mask = MASK_HOPTEST;break;
        }
        if(!mask)return;
        if(key_info.value == 0)
            m_keyMask &= ~mask;
        else if(key_info.value == 1 || key_info.value == 2)
        {
            //printf("key pressed!\n");
            mask = mask|m_keyMask;
            //printf("%d,%d\n",mask&MASK_HOP,m_keyMask & MASK_HOP);
            if((mask & MASK_HOP) && (m_keyMask & MASK_HOP )== 0)
                mask |= MASK_HOPDOWN;
            else mask &= ~MASK_HOPDOWN;
            m_keyMask = mask;
        }
    }
}

//void Console::outstatus()
//{
    //printf("n_mannual:%d,mannual:%d\nn_auto:%d,auto:%d\nn_stop:%d,stop:%d\nn_quit:%d,quit:%d\nctrl[%.3f,%.3f,%.3f]\nout[%.3f%.3f%.3f]\nerr_his[%.3f,%.3f,%.3f]\n\
    inc[%.3f,%.3f,%.3f]"
    //,m_need_mannual,m_mannual,m_need_auto,m_auto,m_need_stop,m_stop,m_need_quit,m_quit,m_ctrl_x,m_ctrl_y,m_ctrl_r,m_out_x,m_out_y,m_out_r,
    //m_his_x,m_his_y,m_his_r,m_inc_x,m_inc_y,m_inc_r);
//}

void* Console::consoleFunc(void* arg)
{
    Console* pThis = (Console*)arg;
    printf("[Console]:start up!\n");
    pThis->_console();
}

void Console::_console()
{
    char cmd;
    while(!m_threadQuit)
    {
        if(m_status.mannaul)
        {
            //printf("mannual\n");
            if(!m_expStatus.mannaul)
            {
                if(!m_prop)
                printf("waiting to exit mannual mode...\n"),m_prop = true;
                continue;
            }
            _updateKeyEvents();
            if(m_keyMask & MASK_A && m_keyMask & MASK_D)
                m_request.x = 0;
            else if(m_keyMask & MASK_D) m_request.x = 1;
            else if(m_keyMask & MASK_A) m_request.x = -1;
            else m_request.x = 0;

            if(m_keyMask & MASK_W && m_keyMask & MASK_S)
                m_request.y = 0;
            else if(m_keyMask & MASK_W) m_request.y = 1;
            else if(m_keyMask & MASK_S) m_request.y = -1;
            else m_request.y = 0;

            if(m_keyMask & MASK_E && m_keyMask & MASK_Q)
                m_request.r = 0;
            else if(m_keyMask & MASK_E) m_request.r = -1;
            else if(m_keyMask & MASK_Q) m_request.r = 1;
            else m_request.r = 0;

            if(m_keyMask & MASK_HOPDOWN)
            {
                m_request.reqHop = true;
                m_request.hopType = Controller::HopForward;
                //printf("Request Hop\n");
                m_keyMask &= ~MASK_HOPDOWN;
            }
            if(m_keyMask & MASK_HOPTEST)
            {
                m_request.reqHop = true;
                m_request.hopType = Controller::TestMotor;
            }

            if(m_keyMask & MASK_STAND)
                m_request.reqStop = true;

            if(m_keyMask & MASK_QUIT)
            {
                m_request.reqStop = true;
                m_expStatus.mannaul = false;
                m_expStatus.auto_ = false;
                m_expStatus.quit = false;
                m_keyMask = 0;
            }
            //printf("\nmask:%d\n",m_keyMask);

        }else if(m_status.auto_)
        {
            if(!m_expStatus.auto_)
            {
                if(!m_prop)
                printf("waiting to exit auto mode...\n"),m_prop = true;
                continue;
            }
            scanf("%c",&cmd);
            switch(cmd)
            {
                case 'Q':
                case 'q':
                {
                    m_request.reqStop = true;
                    m_expStatus.auto_ = false;

                }break;
            }
        }else if(m_status.test)
        {
            if(!m_expStatus.test)
            {
                if(!m_prop)
                printf("waiting to exit test mode...\n"),m_prop = true;
                continue;
            }
            scanf("%c",&cmd);
            printf("test recv : %c\n",cmd);
            switch(cmd)
            {
                case 'g':
                {
                    m_pCtrl->AddAction(AutoCtrl::run,10);
                    m_pCtrl->AddAction(AutoCtrl::rotateL,4);
                    printf("test added!\n");
                }break;
                case 'h':
                {
                    m_pCtrl->AddAction(AutoCtrl::hopToBalance,1);
                    m_pCtrl->AddAction(AutoCtrl::climb,10);
                    m_pCtrl->AddAction(AutoCtrl::balanceRestore,1);
                }break;
                case 'q':
                case 'Q':
                {
                    m_pCtrl->ClearActions();
                    m_request.reqStop = true;
                    m_expStatus.test = false;
                    printf("test removed!\n");

                }break;
            }
        }else
        {
            if(m_expStatus.auto_ || m_expStatus.mannaul || m_expStatus.test)
            {
                if(!m_prop)
                    printf("waiting to change mode...\n"),m_prop = true;
                continue;
            }
            if(m_expStatus.quit)
            {
                //printf("waiting to quit...\n");
                continue;
            }
            scanf("%c",&cmd);
            printf("recv:%c\n",cmd);
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
                    printf("[v/V] automatically control\n");
                    printf("[t/T] automatic test\n");
                    printf("[x/X] quit system\n");
                    printf("when system is mannually controlling:\n");
                    printf("\t[w/a/s/d] moving control\n");
                    printf("\t[q/e] rotating control\n");
                    printf("\t[Space] hop\n");
                    printf("\t[p] quit control\n");
                    printf("when system is automatically controlling:\n");
                    printf("\t[q] quit\n");
                    printf("when system is in automatic test:\n");
                    printf("\t[s{cnt}}] start up and move {cnt} steps\n");
                    printf("\t[r{cnt}] rotate {cnt} loops\n");
                    printf("\t[m{cnt1},{cnt2},{cnt3}] start up ,move {cnt1} steps, turn {cnt2} loops, run{cnt3} steps\n");
                    printf("\t[k{cnt1}] hop and move {cnt} steps\n");
                    printf("\t[h] hop once\n");
                    printf("\t[q] quit\n");

                }break;
                case 'm':
                case 'M':
                {
                    m_request.reqStop = true;
                    m_expStatus.mannaul = true;
                }break;
                case 'v':
                case 'V':
                {
                    m_expStatus.auto_ = true;
                    m_request.reqStop = true;
                }break;
                case 'x':
                case 'X':
                {
                    m_request.reqStop = true;
                    m_expStatus.quit = true;
                }break;
                case 't':
                case 'T':
                {
                    m_request.reqStop = true;
                    m_expStatus.test = true;
                }
            }
        }
    }
    printf("[Console]:system quit!\n");
}

Console::Console(const char* strKeyEvent,AutoCtrl* pCtrl)
{
    m_eventFd = open(strKeyEvent,O_RDONLY,0777);
    if(m_eventFd < 0)
    {
        printf("[Console]:cannot open event device\n");
        return;
    }
    m_pCtrl = pCtrl;
    //m_event.open(strKeyEvent);


    m_threadQuit = true;
    m_threadDesc = 0;
    m_request.r = m_request.x = m_request.y = 0;
    m_request.reqHop = m_request.reqStop = false;
    m_status.auto_ = m_status.mannaul =m_status.quit = m_status.test= false;
    m_expStatus.auto_ = m_expStatus.mannaul =m_expStatus.quit= m_expStatus.test= false;
    m_keyMask = 0;
    m_prop = false;
}

void Console::Start()
{
    if(!m_threadQuit)return;
    m_threadQuit = false;
    thread_create(Console::consoleFunc,this,m_threadDesc);
}

void Console::Exit()
{
    if(m_threadQuit)return;
    m_threadQuit = true;
    thread_join(m_threadDesc);
    thread_destroy(m_threadDesc);
}

Console::~Console()
{
}

void Console::UpdateEvent(bool ctrlStop)
{
    if(ctrlStop)
    {
        //printf("stop\n");
        if(m_status.auto_ != m_expStatus.auto_ ||
         m_status.mannaul != m_expStatus.mannaul ||
          m_status.quit != m_expStatus.quit ||
          m_status.test != m_expStatus.test)
        {
        m_status = m_expStatus;
        printf("[Console]:mode changed!\n");
        printf("mannaul:%d,auto:%d,test:%d,quit:%d\n",m_status.mannaul,m_status.auto_,m_status.test,m_status.quit);
        m_prop = false;
        m_keyMask = 0;
        fflush(stdin);
        }
    }
}
