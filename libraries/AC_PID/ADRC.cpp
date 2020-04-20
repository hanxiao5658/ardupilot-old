
#include "ADRC.h"
#include <cmath>
//#include <stdlib.h>
#include "AC_PID.h"
//#include <iostream>
//#include <fstream>
//using namespace std;
//typedef signed short int16_t ;
//ofstream ADRCfile("ADRCdata.txt"); //创建ADRCfile
Fhan_Data ADRCROLL;
Fhan_Data ADRCPITCH;
Fhan_Data ADRCYAW;
Fhan_Data ADRCdata;




//int cc=0;

//Fhan_Data ADRC_Roll_Controller;
const float ADRC_Unit[3][16]=
{
/*TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO           扰动补偿     非线性组合*/
/*  r     h      N                  beta_01   beta_02    beta_03     b0     beta_0  beta_1     beta_2     N1     C    alpha1  alpha2  zeta  b*/
 {300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   2.0,      0.0010,    5,    5,    0.8,   1.5,    50,    0},
 {300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   2.0,      0.0010,    5,    5,    0.8,   1.5,    50,    0},
 {300000 ,0.005 , 3,               300,      4000,      10000,     0.001,    0.002,   1.2,      0.0005,    5,    5,    0.8,   1.5,    50,    0},
};


float Constrain_Float(float amt, float low, float high)
{
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

int Sign_ADRC(float Input)//判断符号1E-6 为很小的一个数
{
    int output=0;
    if(Input>1E-6) output=1;
    else if(Input<-1E-6) output=-1;
    else output=0;
    return output;
}

int Fsg_ADRC(float x,float d)
{
  int output=0;
  output=(Sign_ADRC(x+d)-Sign_ADRC(x-d))/2;
  return output;
}

//原点附近有连线性段的连续幂次函数
float Fal_ADRC(float e,float alpha,float zeta)
{
    float s=0;
    float fal_output=0;
    s=(Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
    fal_output=e*s/(powf(zeta,1-alpha))+powf(abs(e),alpha)*Sign_ADRC(e)*(1-s);
    return fal_output;
}

/************临时计算用fhan********************/
float adrc_fhan(float v1, float v2, float r0, float h0)
{
	float d = h0 * h0 * r0;
	float a0 = h0 * v2;
	float y = v1 + a0;
	float a1 = sqrtf(d*(d + 8.0f*fabsf(y)));
	float a2 = a0 + Sign_ADRC(y)*(a1-d)*0.5f;
	float sy = (Sign_ADRC(y+d) - Sign_ADRC(y-d))*0.5f;
	float a = (a0 + y - a2)*sy + a2;
	float sa = (Sign_ADRC(a+d) - Sign_ADRC(a-d))*0.5f;
	
	return -r0*(a/d - Sign_ADRC(a))*sa - r0*Sign_ADRC(a);
}


float Fhan_ADRC(float x1_delta , float x2 ,float r , float h)//安排ADRC过度过程
{
  float d = 0,a0 = 0,y = 0,a1 = 0,a2 = 0,a = 0;
  float h0 = h;//用h0替代h，解决最速跟踪微分器速度超调问题
  d = r * h0 * h0;//d=rh^2;
  a0 = h0 * x2;//a0=h*x2
  y = x1_delta + a0;//y=x1+a0
  a1 = sqrtf(d * ( d + 8 * abs(y) ) );//a1=sqrt(d*(d+8*ABS(y))])
  a2 = a0 + Sign_ADRC(y) * ( a1 - d ) / 2;//a2=a0+sign(y)*(a1-d)/2;
  a =(a0+y) * Fsg_ADRC(y,d) + a2 * ( 1 - Fsg_ADRC(y,d));

  //得到最速微分加速度跟踪量
  return - r * (a/d) * Fsg_ADRC(a,d) - r * Sign_ADRC(a) * ( 1 - Fsg_ADRC(a,d) );
 
}


//ADRC最速跟踪微分器TD，改进的算法fhan
void TD_ADRC(Fhan_Data *fhan_Input,float expect_ADRC)//安排ADRC过度过程
{
  
  fhan_Input->fh =Fhan_ADRC( fhan_Input->x1 - expect_ADRC , fhan_Input->x2, fhan_Input->r, fhan_Input->h);
  fhan_Input->x1 += fhan_Input->h*fhan_Input->x2;//跟新最速跟踪状态量x1
  fhan_Input->x2 += fhan_Input->h*fhan_Input->fh;//跟新最速跟踪状态量微分x2
  
 }



/************扩张状态观测器********************/
//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) 
//beta 选取的不对会造成側反
void ESO_ADRC(Fhan_Data *fhan_Input)
{

  fhan_Input->e = fhan_Input->z1 - fhan_Input->y ;//状态误差
   
 
 /*2阶 LESO */
 float LESO_w0 = 50 ;
 fhan_Input->z1 += fhan_Input->h * ( fhan_Input->z2 - ( 2 * LESO_w0 ) * fhan_Input->e + fhan_Input->b0 * fhan_Input->u );
 fhan_Input->z2 += fhan_Input->h * ( - ( LESO_w0 * LESO_w0 ) * fhan_Input->e);
                                    

}






void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
{
  /*********第一种组合形式*********/
  //fhan_Input->u0=fhan_Input->beta_1*fhan_Input->e1+fhan_Input->beta_2*fhan_Input->e2+(fhan_Input->beta_0*fhan_Input->e0);

  /*********第二种组合形式*********/
  float temp_e2=0;
  temp_e2=Constrain_Float(fhan_Input->e2,-3000,3000);
  fhan_Input->u0=fhan_Input->beta_1*Fal_ADRC(fhan_Input->e1,fhan_Input->alpha1,fhan_Input->zeta)
                +fhan_Input->beta_2*Fal_ADRC(temp_e2,fhan_Input->alpha2,fhan_Input->zeta);

}


void ADRC_Control(Fhan_Data *fhan_Input , float expect_ADRC , float feedback_ADRC)
{

float ADRC_error = expect_ADRC - feedback_ADRC ;
 

/*自抗扰控制器第1步    TD*/
 TD_ADRC(fhan_Input , ADRC_error );

/*自抗扰控制器第2步*/

/****ESO****/

 fhan_Input->y=feedback_ADRC;      

 ESO_ADRC(fhan_Input); 

/*自抗扰控制器第3步*/  

/********状态误差反馈率***/

fhan_Input->e1 = fhan_Input->x1 ;
fhan_Input->e2 = fhan_Input->x2 ;

Nolinear_Conbination_ADRC(fhan_Input);

 
  /**********扰动补偿*******/
  fhan_Input->u=fhan_Input->u0-fhan_Input->k*fhan_Input->z2/fhan_Input->b0;
  

  
 /**********输出数据*******/
  fhan_Input->u=Constrain_Float(fhan_Input->u,-1,1); 
    
  
}

