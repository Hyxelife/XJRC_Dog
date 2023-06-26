#include "MultiThread.h"
#include <fstream>

class Console
{
    public:
    struct ConsoleStatus
    {
        bool auto_;
        bool mannaul;
        bool quit;
    };
    struct ConsoleRequest
    {
        bool reqStop;
        bool reqHop;
        float x,y,r;
    };
    protected:
    void _updateKeyEvents();

    static void* consoleFunc(void* arg);

    void _console();
    void _outstatus();
    public:
    Console(const char* strKeyEvent);
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
};
