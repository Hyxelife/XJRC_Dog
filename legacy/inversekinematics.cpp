#include "math.h"
#include "stdio.h"
#include "inversekinematics.h"
//逆运动学
//输入x,y,z结构体指针leg1，三个角度的结构体指针leg2
//对结构体进行修改，得到想要的三个角度的结构体
void inversekinematics(position_xyz *leg1,position_angel *leg2)
{
    leg2->alfa=acos((L2+((*leg1).x*(*leg1).x+(*leg1).y*(*leg1).y+(*leg1).z*(*leg1).z-L3*L3-L1*L1-L2*L2)/(2*L2))/(sqrt((*leg1).x*(*leg1).x+(*leg1).y*(*leg1).y+(*leg1).z*(*leg1).z-L1*L1)))-atan((*leg1).x/sqrt((*leg1).y*(*leg1).y+(*leg1).z*(*leg1).z-L1*L1));
    leg2->beta=-acos(((*leg1).x*(*leg1).x+(*leg1).y*(*leg1).y+(*leg1).z*(*leg1).z-L3*L3-L1*L1-L2*L2)/(2*L2*L3));
    if((*leg1).leg==1||(*leg1).leg ==4) {
        leg2->alfa=-leg2->alfa;
        leg2->beta=-leg2->beta;
        leg2->gama=atan((*leg1).y/(*leg1).z)-atan(L1/sqrt((*leg1).y*(*leg1).y+(*leg1).z*(*leg1).z-L1*L1));
    }
    else if((*leg1).leg==2||(*leg1).leg ==3){
        leg2->gama=-atan((*leg1).y/(*leg1).z)-atan(L1/sqrt((*leg1).y*(*leg1).y+(*leg1).z*(*leg1).z-L1*L1));
    }
    else{
        leg2->gama=0;
        while(1){
            printf("逆运动学错误");
        }
    }

}



//void inversekinematics(position_xyz *leg1,position_angel *leg2)
//{
//    leg2->alfa=
//    acos((L1+((*leg1).x*(*leg1).x+(*leg1).y*(*leg1).y+(*leg1).z*(*leg1).z-L3*L3-L1*L1-L2*L2)/(2*L1))/
//         (sqrt((*leg1).x*(*leg1).x+(*leg1).y*(*leg1).y+(*leg1).z*(*leg1).z-L3*L3)))-
//          atan((*leg1).x/sqrt((*leg1).y*(*leg1).y+(*leg1).z*(*leg1).z-L3*L3));
//    leg2->beta=-acos(((*leg1).x*(*leg1).x+(*leg1).y*(*leg1).y+(*leg1).z*(*leg1).z-L3*L3-L1*L1-L2*L2)/(2*L2*L1));
//    if((*leg1).leg==1||(*leg1).leg ==4) {
//        leg2->alfa=-leg2->alfa;
//        leg2->beta=-leg2->beta;
//        leg2->gama=atan((*leg1).y/(*leg1).z)-atan(L3/sqrt((*leg1).y*(*leg1).y+(*leg1).z*(*leg1).z-L3*L3));
//    }
//    else if((*leg1).leg==2||(*leg1).leg ==3){
//        leg2->gama=-atan((*leg1).y/(*leg1).z)+atan(L3/sqrt((*leg1).y*(*leg1).y+(*leg1).z*(*leg1).z-L3*L3));
//    }
//    else{
//        leg2->gama=0;
//        while(1){
//            printf("逆运动学错误");
//        }
//    }
//
//}
