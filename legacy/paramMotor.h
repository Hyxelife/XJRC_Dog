#ifndef PARAMMOTOR_H
#define PARAMMOTOR_H

#include "SerialPort.h"
#include "unitreeMotor.h"


void param_allprint(MOTOR_recv motor_r);

#define pi acos(-1)

#define  Leg1 1
#define Leg2 2
#define Leg3 3
#define Leg4 4



#define K_Joint 0
#define D_Joint 1
#define X_Joint 2

//定义第一条腿标定位置
#define K1 1.4059//1.42047//1.19766//6.009383678436279
#define D1 5.79309//5.51046//3.78396//4.50531//-0.37505918741226196
#define X1 1.6555//1.78441//3.16039//5.303367614746094

//定义第二条腿标定位置
#define K2 1.62564//1.6195//3.894019365310669
#define D2 5.61362//5.61783//1.3256004571914673
#define X2 3.37131//3.47102//1.96964//4.228044509887695

//定义第三条腿标定位置
#define K3 4.90683//5.08593//4.90185
#define D3 2.56252//2.48889//4.0524
#define X3 1.94816//1.9286//2.7512//4.55095

//定义第四条腿标定位置
#define K4 4.8658//4.79025//1.2302554845809937
#define D4 5.96643//6.08378//3.77015//4.08999//2.315155228797728
#define X4 2.07241//2.03943//3.34945//3.87868//5.689548015594482

#define K1_origin_pos 15.7001465738//15.7147165738//20.303630252269837
#define D1_origin_pos 20.0873365738//19.8047065738//18.799973
#define X1_origin_pos -12.6387465738//-12.5098365738//-11.133856574//-20.58033643122785


#define K2_origin_pos -12.6686065738//-12.6747465738//-10.400227208522889
#define D2_origin_pos -8.6806265738//-8.6764165738//-12.96864611664209
#define X2_origin_pos 17.6655565738//17.7652665738//16.2638865738//30.111748555861638

#define K3_origin_pos 19.2010765738//19.3801765738//19.1960965738//14.109032706015544
#define D3_origin_pos -11.7317265738//-11.8053565738//-10.241846574//-9.046093086810973
#define X3_origin_pos 16.2424065738//16.2228465738//17.0454465738//18.8451965738//15.5099265738//19.7855265738//30.097175458632634

#define K4_origin_pos -9.4284465738//-9.5039965738//-13.063991089252564
#define D4_origin_pos 20.2606765738//20.3816265738//18.0643965738//18.3841
#define X4_origin_pos -12.2218365738//-12.2548165738//-10.944796574//-10.970106574//-22.0050





#endif  // PARAMMOTOR_H


//
//class Leg{
//public:
//    SerialPort legport;
//    K_Joint=0;
//    D_Joint=1;
//    X_Joint=2;
//}
//
//class ONE_LEG:Leg{
//private:
//    SerialPort legport1("/dev/ttyUSB0");
//    K1=6.009383678436279;
//    D1=-0.37505918741226196++-
//    X1=5.303367614746094;
//
//}
//
//class TWO_LEG:Leg{
//private:
//    SerialPort legport1("/dev/ttyUSB0");
//    K2=3.894019365310669;
//    D2=1.3756004571914673;
//    X2=4.228044509887695;
//
//};
//
//class THREE_LEG:Leg{
//private:
//    SerialPort legport1("/dev/ttyUSB0");
//    K3=-0.18521386781801397;
//    D3=5.248153487022584;
//    X3=4.213471412658691;
//
//};
//
//class FOUR_LEG:Leg{
//private:
//    SerialPort legport1("/dev/ttyUSB0");
//    K4=1.2302554845809937;
//    D4=2.315155228797728;
//    X4=5.689548015594482;
//
//};

//typedef struct
//{
//
//
//}ONELEG
//
//
//typedef struct
//{
//
//}TWOLEG
//
//typedef struct
//{
//
//}THREELEG
//
//typedef struct
//{
//
//}FOURLEG
