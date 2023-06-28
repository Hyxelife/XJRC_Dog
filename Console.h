#include "MultiThread.h"
#include <fstream>
#include "AutoControl.h"
#include "Controller.h"

class Console
{
    public:
    struct ConsoleStatus
    {
        bool auto_;
        bool mannaul;
        bool quit;
        bool test;
    };
    struct ConsoleRequest
    {
        bool reqStop;
        bool reqHop;
        Controller::HopType hopType;
        float x,y,r;
    };
    struct TestItem
    {

    };
    protected:
    void _updateKeyEvents();

    static void* consoleFunc(void* arg);

    void _console();
    void _outstatus();
    public:
    Console(const char* strKeyEvent,AutoCtrl* pCtrl);
    ~Console();
    void Start();
    void Exit();

    void UpdateEvent(bool ctrlStop);

    void GetConsoleRequest(ConsoleRequest& request){request = m_request;m_request.reqHop = m_request.reqStop = false;}
    void GetConsoleStatus(ConsoleStatus& status){status = m_status;}

    protected:
    unsigned int m_keyMask;
    int m_eventFd;
    THREAD m_threadDesc;
    ConsoleStatus m_status;
    ConsoleStatus m_expStatus;
    ConsoleRequest m_request;
    bool m_prop;
    bool m_threadQuit;
    std::ifstream m_event;
    AutoCtrl* m_pCtrl;
};
