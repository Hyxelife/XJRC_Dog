#ifndef VMC_H_INCLUDED
#define VMC_H_INCLUDED

void vmc();
void KB_init(int legnumber);
void vmc_force(int legnumber);
void Get_Jacobian(int legnumber);
void vmc_moment_force(int legnumber);
void vmc_force_correct(int legnumber);
void vmc_force_motion_correct(int legnumber);
struct B//阻尼系数
{
    float x ;
    float y ;
    float z ;
};
struct K//弹簧系数
{
    float x;
    float y;
    float z;
};
struct force//存放输出力的结构体
{
    float force_x;
    float force_y;
    float force_z;

};
struct moment_force//存放输出力矩的结构体
{
    float force_0;
    float force_1;
    float force_2;

};
struct Jacobian
{
    float faai [3];  //对应0号电机
    float theta[3];  //对应1号电机
    float gamma[3];  //对应2号电机

};


#endif // VMC_H_INCLUDED
