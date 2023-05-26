#include "check.h"
#include "math.h"
#include "stdio.h"
#include "unitreeMotor.h"
#include "motor_msg.h"  // 电机通信协议
#include "SerialPort.h"

bool checkposition(float a,float b)
{
    if(abs(a-b)<=3) return 1;
    else
    {
        while(1)
        {
            printf("Zero_Point_Error***************************************\n");
            printf("%f\n",a);
            printf("%f\n",b);
            usleep(10000);
        }
        return 0;
    }
}

bool trot_check(float a,float b)
{
    if(abs(a-b)<=3) return 1;
    else
    {
        printf("trot_check_Error");
        return 0;
    }
}
