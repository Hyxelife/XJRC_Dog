#include "Console.h"
#include <stdio.h>
#include <math.h>
#include <linux/input.h>
#include <unistd.h>
#include <fcntl.h>
#include "Debug.h"
#include "Record.h"

////////////key masks///////////////
#define MASK_A          0x01
#define MASK_S          0x02
#define MASK_D          0x04
#define MASK_Q          0x08
#define MASK_W          0x10
#define MASK_E          0x20
#define MASK_QUIT       0x40
#define MASK_STAND      0x80
#define MASK_RECORD     0x100
#define MASK_SHOWRECORD 0x200

#define MASK_HOP            0x800
#define MASK_HOPFRONT       0x1000
#define MASK_STEPANDSPAN    0x2000
#define MASK_CLAW           0x4000
#define MASK_CLAWLEFT       0x8000
#define MASK_CLAWRIGHT      0x10000
#define MASK_RESTORE        0x20000
#define MASK_LERPRES        0x40000



void Console::_updateKeyEvents()
{
    input_event key_info;
    unsigned int mask = 0;
    //m_event.read((char*)&key_info,sizeof(key_info));
    if(read(m_eventFd,&key_info,sizeof(input_event)) > 0)
    {
        //printf("event\n");
        if(key_info.type != EV_KEY)return;

        switch(key_info.code)
        {
            case KEY_A:mask = MASK_A;break;
            case KEY_S:mask = MASK_S;break;
            case KEY_D:mask = MASK_D;break;
            case KEY_Q:mask = MASK_Q;break;
            case KEY_W:mask = MASK_W;break;
            case KEY_E:mask = MASK_E;break;
            case KEY_SPACE:mask = MASK_HOP|MASK_HOPFRONT;break;
            case KEY_P:mask = MASK_QUIT;break;
            case KEY_F:mask = MASK_STAND;break;
            case KEY_2:mask = MASK_HOP|MASK_CLAW;break;
            case KEY_1:mask = MASK_HOP|MASK_CLAWLEFT;break;
            case KEY_3:mask = MASK_HOP|MASK_CLAWRIGHT;break;
            case KEY_9:mask = MASK_HOP|MASK_STEPANDSPAN;break;
            case KEY_0:mask = MASK_HOP|MASK_RESTORE;break;
            case KEY_4:mask = MASK_HOP|MASK_LERPRES;break;
            case KEY_U:mask = MASK_RECORD;break;
            case KEY_I:mask = MASK_SHOWRECORD;break;

        }
        if(!mask)return;
        if(key_info.value == 0)
            m_keyMask &= ~mask;
        else if(key_info.value == 1 || key_info.value == 2)
            m_keyMask |= mask;
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

int __parseNum(char* &ptr)
{
    int num = 0;
    for(;*ptr >= '0' && *ptr <= '9';++ptr)
    {
        num *= 10;
        num += *ptr-'0';
    }
    return num;
}

float __parseFloat(char* &ptr)
{
    int num = 0,dot = -1,sign = 1;
    float result = 0;
    for(;(*ptr >= '0' && *ptr <= '9') || *ptr == '-' || *ptr == '.';++ptr)
    {
        if(*ptr == '-')
        {
            if(num)return 0;
            sign = -1;
        }else if(*ptr == '.')
        {
            if(dot != -1)return 0;
            dot = 0;
        }else
        {
            if(dot != -1)dot++;
            num *= 10;
            num += *ptr -'0';
        }
    }
    result = num;
    for(int i = 0;i<dot;++i)result /= 10.0;
    return result;
}

char *__doParse(char* ptr,AutoCtrl::Action &action)
{
    if(ptr == NULL)return NULL;
    if(*ptr == '\0')return NULL;
    char cmd = *ptr;
    if(cmd == 't')
    {
        ptr++;
        action.r = __parseFloat(ptr);
        action.action = AutoCtrl::autoRotateTo;
        return ptr+1;
    }else if(cmd == 'r')
    {
        ptr++;
        action.r = __parseFloat(ptr);
        action.action = AutoCtrl::autoRotateWith;
        return ptr+1;
    }
    ptr = ptr+1;
    int num = __parseNum(ptr);
    if(*ptr != ';')return NULL;
    ptr++;
    action.actionCnt = num;
    switch(cmd)
    {
        case 'w':action.action = AutoCtrl::run;break;
        case 'a':action.action = AutoCtrl::moveL;break;
        case 's':action.action = AutoCtrl::back;break;
        case 'd':action.action = AutoCtrl::moveR;break;
        case 'q':action.action = AutoCtrl::rotateL;break;
        case 'e':action.action = AutoCtrl::rotateR;break;
        case 'z':action.action = AutoCtrl::turnL;break;
        case 'c':action.action = AutoCtrl::turnR;break;
    }
    return ptr;
}

char buffer[1024];



void Console::_console()
{
    char cmd;
    while(!m_threadQuit)
    {
        usleep(1000);
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

            if(m_keyMask & MASK_HOP)
            {
                if(!m_hopBanned)
                {
                    m_request.reqHop = true;
                    if(m_keyMask & MASK_HOPFRONT)
                        m_request.hopType = Controller::HopForward;
                    else if(m_keyMask & MASK_STEPANDSPAN)
                        m_request.hopType = Controller::StepAndSpan;
                    else if(m_keyMask & MASK_CLAW)
                        m_request.hopType = Controller::Claw;
                    else if(m_keyMask & MASK_CLAWLEFT)
                        m_request.hopType = Controller::ClawLeft;
                    else if(m_keyMask & MASK_CLAWRIGHT)
                        m_request.hopType = Controller::ClawRight;
                    else if(m_keyMask & MASK_RESTORE)
                        m_request.hopType = Controller::StepToRestore;
                    else if(m_keyMask & MASK_LERPRES)
                        m_request.hopType = Controller::LerpToRestore;
                    else m_request.reqHop = false;
                }
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
            if(m_keyMask & MASK_RECORD)
            {
                if(!m_recording)
                {
                    printf("\n[Console] start recording...\n");
                    m_recording = true;
                    m_record.clear();
                }
            }
            if(m_keyMask & MASK_SHOWRECORD)
            {
                if(m_recording)
                {
                    m_recording = false;
                    OUT("\n[Console]: showing the record result:\n");
                    for(int i = 0;i<m_record.size();++i)
                        OUT("pCtrl->AddRecord(%.3f,%.3f,%.3f);\n",m_record[i].x,m_record[i].y,m_record[i].r);
                    OUT("\n[Console]: end of the record\n");
                    m_request.reqStop = true;
                }
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
                case 'r':
                case 'R':
                {
                    printf("[Console] using records ...\n");
                    Record(m_pCtrl);

                }break;
                case 'x':
                case 'X':
                {
                    printf("[Console] Abort!\n");
                    m_pCtrl->ClearActions();
                    m_request.reqStop = true;
                }break;
                case 'Q':
                case 'q':
                {
                    m_pCtrl->ClearActions();
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
            scanf("%s",buffer);
            printf("test recv : %s\n",buffer);
            if(buffer[0] == 'x' || buffer[0] == 'X')
            {
                m_pCtrl->ClearActions();
                m_request.reqStop = true;
                m_expStatus.test = false;
                continue;
            }
            if(buffer[0] == 'p' || buffer[0] == 'P')
            {
                m_pCtrl->ClearActions();
                m_request.reqStop = true;
                continue;
            }

            char* ptr = buffer;
            std::vector<AutoCtrl::Action> actions;
            AutoCtrl::Action action;
            ptr = __doParse(ptr,action);
            while(ptr)
            {
                actions.push_back(action);
                ptr = __doParse(ptr,action);
            }
            printf("command check: (total command %d)\n",actions.size());
            for(int i = 0;i<actions.size();++i)
            {
                switch(actions[i].action)
                {
                    case AutoCtrl::run:printf("run ");break;
                    case AutoCtrl::back:printf("back ");break;
                    case AutoCtrl::moveL:printf("moveL ");break;
                    case AutoCtrl::moveR:printf("moveR ");break;
                    case AutoCtrl::rotateL:printf("rotateL ");break;
                    case AutoCtrl::rotateR:printf("rotateR ");break;
                    case AutoCtrl::turnL:printf("turnL ");break;
                    case AutoCtrl::turnR:printf("turnR ");break;
                }
                printf("%d times;\n",actions[i].actionCnt);
            }
            bool ok = false;
            while(true)
            {
                printf("continue? y/n:\n");
                char cmd;
                scanf("%c",&cmd);
                if(cmd == 'n')
                {
                    printf("aborted!\n");
                    break;
                }else if(cmd == 'y')
                {
                    printf("executing...\n");
                    ok = true;
                    break;
                }
            }
            if(ok)
                m_pCtrl->AddActions(actions);
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
                    printf("\t[Space] hop forward\n");

                    printf("\t[1] claw left\n");
                    printf("\t[2] claw forward\n");
                    printf("\t[3] claw right\n");
                    printf("\t[4] lerp to restore posture\n");

                    printf("\t[9] span the legs\n");
                    printf("\t[0] step to restore posture\n");


                    printf("\t[p] quit control\n");
                    printf("when system is automatically controlling:\n");
                    printf("\t[q] quit\n");
                    printf("when system is in automatic test:\n");
                    printf("\t[x/X] quit\n");
                    printf("\t[p/P] stop\n");
                    printf("\t[{cmd<number>;}+] make test command\n");
                    printf("\tbasic command:\n");
                    printf("\t[w<num>] move forward <num> steps\n");
                    printf("\t[a<num>] move left <num> steps\n");
                    printf("\t[s<num>] move backward <num> steps\n");
                    printf("\t[d<num>] move right <num> steps\n");
                    printf("\t[q<num>] rotate anticlockwise <num> steps\n");
                    printf("\t[e<num>] rotate clockwise <num> steps\n");
                    printf("\t[z<num>] turn left <num> steps\n");
                    printf("\t[c<num>] turn right <num> steps\n");
                    printf("\t[t<num>] rotate to <num> degree\n");
                    printf("\t[r<num>] rotate with <num> degree\n");
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

Console::Console(const char* strKeyEvent,AutoCtrl* pCtrl,Controller* pCtrller)
{

    m_pCtrl = pCtrl;
    m_pCtrller = pCtrller;
    //m_event.open(strKeyEvent);


    m_threadQuit = true;
    m_threadDesc = 0;
    m_request.r = m_request.x = m_request.y = 0;
    m_request.reqHop = m_request.reqStop = false;
    m_status.auto_ = m_status.mannaul =m_status.quit = m_status.test= false;
    m_expStatus.auto_ = m_expStatus.mannaul =m_expStatus.quit= m_expStatus.test= false;
    m_keyMask = 0;
    m_prop = false;
    m_hopBanned = false;
    m_eventFd = open(strKeyEvent,O_RDONLY,0777);
    if(m_eventFd < 0)
        printf("[Console]:cannot open event device\n");
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

void Console::UpdateEvent(bool ctrlStop,bool stepOver)
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
    if(stepOver)
    {
        if(m_recording && !ctrlStop)
        {
            AutoCtrl::Action action;
            m_pCtrller->GetCurrentVelocity(action.x,action.y,action.r);
            m_record.push_back(action);
        }

    }
}

void Console::GetConsoleRequest(Console::ConsoleRequest &request)
{
    m_hopBanned = false;
    if(m_request.reqHop)m_hopBanned = true;
    request = m_request;
    m_request.reqHop = m_request.reqStop = false;
}
