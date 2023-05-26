#ifndef ROUTE_H_INCLUDED
#define ROUTE_H_INCLUDED

#define r1 1
#define r2 1
#define r3 1
#define r4 1
#define l1 0.220      //大腿长度
#define l2 0.222     // 小腿长度
#define l3 0.0988
#define my_xs -0.03     //摆线起始点坐标
#define my_xf 0.03      // 结束点
#define hhh 0.04      //抬腿高度8
#define HHH 0.3      //站立时身体离地距离
#define WWidth 0.2  //身体宽度
#define Real_read  //读取模式

//存放期待坐标值的结构体
void stand_init();
void Walk();
void Readstate();
void Calculate_exp_Speed(float x1, float x2, float y1, float y2, float z1, float z2,int i);
void Calculate_real_Speed(float x1, float x2, float y1, float y2, float z1, float z2, int i);
void forward_kinematics(int legnumber);
void read_angle();
void get_position(int legnumber);
void Init();
//存放期待坐标值的结构体
struct Expect
{
    float expx;
    float expy;
    float expz;

};
//存放读取实际值的结构体
struct Real
{
    float real_x;
    float real_y;
    float real_z;

};
//实际上一时刻位置
struct Pre_real
{
    float pre_x;
    float pre_y;
    float pre_z;

};
//期待上一时刻位置
struct Pre_exp
{
    float exp_x;
    float exp_y;
    float exp_z;

};
//论文中用字母u表示，故起名。。
struct uuu
{
    struct uu0
    {
        float x;
        float y;
        float z;
    }exp_shift;//期望位移
    struct uu1
    {
        float x;
        float y;
        float z;
    }exp_speed;//期望速度
    struct uu2
    {
        float x;
        float y;
        float z;
    }real_shift;//实际位移
    struct uu3
    {
        float x;
        float y;
        float z;
    }real_speed;//实际速度
};
//储存读取角度的数组
//控制大腿是0  控制小腿是1   髋部是2
struct real_angle
{
    float motor[3];

};
extern Expect my_Exp_leg[4];
extern Real Real_leg[4];
extern Pre_real Pre_real_leg[4];
extern Pre_exp Pre_exp_leg[4];
extern uuu uuu[4];
extern real_angle real_angle[4];
extern float mode[4];
extern MOTOR_recv a2;
extern MOTOR_recv a3;
#endif // ROUTE_H_INCLUDED
