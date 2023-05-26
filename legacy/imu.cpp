#include"all.h"
float Beta_velocity;
float Beta_acceleration;
float Gamma_velocity;
float Gamma_acceleration;
extern float deta_force_motion ;
extern float gamma_force_motion ;
void imu()
{
	imu_read();
	float M_Beta = K_Beta * (Beta_d - Beta_velocity) +
		B_Beta * (Beta_point_d - Beta_acceleration);
	deta_force_motion = M_Beta / 2;
	float M_Gamma = K_Gamma * (Gamma_d - Gamma_velocity) +
		B_Gamma * (Gamma_point_d - Gamma_acceleration);
	gamma_force_motion = M_Gamma / WWidth;

}

void imu_read()
{
	//读取横滚角速度和角加速度
	Beta_velocity = 0;
	Beta_acceleration = 0;
	//读取偏航角
	Gamma_velocity = 0;
	Gamma_acceleration = 0;

}

