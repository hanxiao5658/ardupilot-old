
#include "POSADRC.h"
#include <cmath>
#include "AC_PID.h"

POS_Fhan_Data ADRC_POS_X;
POS_Fhan_Data ADRC_POS_Y;
POS_Fhan_Data ADRC_POS_Z;
POS_Fhan_Data ADRC_POS_XY_TEST_1;
POS_Fhan_Data ADRC_POS_XY_TEST_2;
POS_Fhan_Data ADRC_POS_XY_TEST_3;
POS_Fhan_Data ADRC_POS_Z_TEST_1;
POS_Fhan_Data ADRC_POS_Z_TEST_2;
POS_Fhan_Data ADRC_POS_Z_TEST_3;

float Constrain_Float_POS(float amt, float low, float high)
{
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

int Sign_ADRC_POS(float Input)//判断符号1E-6 为很小的一个数
{
    int output=0;
    if(Input>1E-6) output=1;
    else if(Input<-1E-6) output=-1;
    else output=0;
    return output;
}

int Fsg_ADRC_POS(float x,float d)
{
  int output=0;
  output=(Sign_ADRC_POS(x+d)-Sign_ADRC_POS(x-d))/2;
  return output;
}

//原点附近有连线性段的连续幂次函数
float Fal_ADRC_POS(float e,float alpha,float zeta)
{
    float s=0;
    float fal_output=0;
    s=(Sign_ADRC_POS(e+zeta)-Sign_ADRC_POS(e-zeta))/2;
    fal_output=e*s/(powf(zeta,1-alpha))+powf(abs(e),alpha)*Sign_ADRC_POS(e)*(1-s);
    return fal_output;
}


float Fhan_ADRC_POS(float x1_delta , float x2 ,float r , float h)//安排ADRC过度过程
{
  float d = 0,a0 = 0,y = 0,a1 = 0,a2 = 0,a = 0;
  float h0 = h;//用h0替代h，解决最速跟踪微分器速度超调问题
  d = r * h0 * h0;//d=rh^2;
  a0 = h0 * x2;//a0=h*x2
  y = x1_delta + a0;//y=x1+a0
  a1 = sqrtf(d * ( d + 8 * abs(y) ) );//a1=sqrt(d*(d+8*ABS(y))])
  a2 = a0 + Sign_ADRC_POS(y) * ( a1 - d ) / 2;//a2=a0+sign(y)*(a1-d)/2;
  a =(a0+y) * Fsg_ADRC_POS(y,d) + a2 * ( 1 - Fsg_ADRC_POS(y,d));

  //得到最速微分加速度跟踪量
  return - r * (a/d) * Fsg_ADRC_POS(a,d) - r * Sign_ADRC_POS(a) * ( 1 - Fsg_ADRC_POS(a,d) );
 
}

//ADRC最速跟踪微分器TD，改进的算法fhan
/************ TD ********************/
void TD_ADRC_POS(POS_Fhan_Data *fhan_Input,float expect_ADRC)//安排ADRC过度过程
{

  fhan_Input->fh = Fhan_ADRC_POS( fhan_Input->x1 - expect_ADRC , fhan_Input->x2, fhan_Input->r, fhan_Input->h);
  fhan_Input->x1 += fhan_Input->h * fhan_Input->x2;//跟新最速跟踪状态量x1,用于快速跟踪expect-ADRC 输出来看看
  fhan_Input->x2 += fhan_Input->h * fhan_Input->fh;//跟新最速跟踪状态量微分x2
  
 }

/************扩张状态观测器********************/
void ESO_ADRC_POS(POS_Fhan_Data *fhan_Input)
{

 fhan_Input->e = fhan_Input->z1 - fhan_Input->y; //状态误差

 /*2阶 LESO */ 
 float LESO_w0 = 50;

 fhan_Input->z1 += fhan_Input->h * ( fhan_Input->z2 - ( 2 * LESO_w0 ) * fhan_Input->e + fhan_Input->b0 * ( fhan_Input->u ));
                                      
 fhan_Input->z2 += fhan_Input->h * ( - ( LESO_w0 * LESO_w0 ) * fhan_Input->e);

 if(fhan_Input->constrain_flag == 1)
 {
   fhan_Input->z2 = Constrain_Float_POS(fhan_Input->z2 , -1 , 1 );
 } 
                     
}

void ESO_POS(POS_Fhan_Data *fhan_Input, float PD_signal, float feedback_signal ,float w0 )
{

  fhan_Input->e = fhan_Input->z1 - feedback_signal ;//状态误差
   
 
 /*2阶 LESO */
 float LESO_w0 = w0 ;
 fhan_Input->z1 += fhan_Input->h * ( fhan_Input->z2 - ( 2 * LESO_w0 ) * fhan_Input->e + fhan_Input->b0 * PD_signal );
 fhan_Input->z2 += fhan_Input->h * ( - ( LESO_w0 * LESO_w0 ) * fhan_Input->e);
                                    

}


void first_order_ESO_POS(POS_Fhan_Data *fhan_Input, float final_signal, float feedback_signal ,float w0 )
{

  fhan_Input->e = fhan_Input->z1 - feedback_signal ;//状态误差
  
 
 /*1阶 LESO */
 float LESO_w0 = w0 ;
 fhan_Input->z2 += fhan_Input->h * LESO_w0 * ( feedback_signal- fhan_Input->b0 * final_signal - fhan_Input->z2) ;
                                   

}

//LSEF is just a special PD control
/************LSEF********************/
void linear_Conbination_ADRC_POS(POS_Fhan_Data *fhan_Input)
{

  float temp_e2 = Constrain_Float_POS(fhan_Input->e2,-300000,300000);

  fhan_Input->u0=fhan_Input->kp * fhan_Input->e1 + fhan_Input->kd * temp_e2 ; 

}




void ADRC_Control_POS(POS_Fhan_Data *fhan_Input,float expect_ADRC,float feedback_ADRC)
{

ADRC_POS_X.constrain_flag = 1;
ADRC_POS_Y.constrain_flag = 1;
ADRC_POS_Z.constrain_flag = 1;

ADRC_POS_Z.kp = 2;
ADRC_POS_Z.kd = 0.1;

ADRC_POS_X.kp = 2;
ADRC_POS_X.kd = 0.05;

ADRC_POS_Y.kp = 2;
ADRC_POS_Y.kd = 0.05;

ADRC_POS_X.b0 = 50;
ADRC_POS_Y.b0 = 50;

ADRC_POS_X.k = 1;
ADRC_POS_Y.k = 1;

float ADRC_POS_error = expect_ADRC - feedback_ADRC ;

/*自抗扰控制器第1步    TD*/
 TD_ADRC_POS(fhan_Input , ADRC_POS_error);


/****自抗扰控制器第2步 ESO****/

 fhan_Input->y = feedback_ADRC ; 
 ESO_ADRC_POS(fhan_Input); 

/*自抗扰控制器第3步 状态误差反馈率*/  
     
 fhan_Input->e1=fhan_Input->x1;//状态偏差项
 fhan_Input->e2=fhan_Input->x2;//状态微分项，
 linear_Conbination_ADRC_POS(fhan_Input);
  
/**********2阶扰动补偿*******/ 
 //fhan_Input->u = ( ( fhan_Input->u0 - fhan_Input->k * fhan_Input->z2 / fhan_Input->b0 )  * fhan_Input->gain_compenseter ) + fhan_Input->bais_compenseter;
 // u = PD * gain -z2  
 // in Z control gain is 0.001   
 // in xy control gain is ekfNavVelGainScaler 
 fhan_Input->u = (fhan_Input->u0 * fhan_Input->gain_compenseter) - (fhan_Input->k * (fhan_Input->z2 / fhan_Input->b0));


}

