
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


/*---constrain funtion make sure output is not too big or too small--------*/
float Constrain_Float(float amt, float low, float high)
{
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}
/*----used in transcient profile generator, no need to tune--------------*/
int Sign_ADRC(float Input)
{
    int output=0;
    if(Input>1E-6) output=1;
    else if(Input<-1E-6) output=-1;
    else output=0;
    return output;
}
/*----used in transcient profile generator, no need to tune--------------*/
int Fsg_ADRC(float x,float d)
{
  int output=0;
  output=(Sign_ADRC(x+d)-Sign_ADRC(x-d))/2;
  return output;
}

/*------ Nolinear function used in NLSEF ---------*/
float Fal_ADRC(float e,float alpha,float zeta)
{
    float s=0;
    float fal_output=0;
    s=(Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
    fal_output=e*s/(powf(zeta,1-alpha))+powf(abs(e),alpha)*Sign_ADRC(e)*(1-s);
    return fal_output;
}

/*------fhan funtion used in transcient profile generator, no need to tune---------------*/
float Fhan_ADRC(float x1_delta , float x2 ,float r , float h)
{
  float d = 0,a0 = 0,y = 0,a1 = 0,a2 = 0,a = 0;
  float h0 = h;
  d = r * h0 * h0;//d=rh^2;
  a0 = h0 * x2;//a0=h*x2
  y = x1_delta + a0;//y=x1+a0
  a1 = sqrtf(d * ( d + 8 * abs(y) ) );//a1=sqrt(d*(d+8*ABS(y))])
  a2 = a0 + Sign_ADRC(y) * ( a1 - d ) / 2;//a2=a0+sign(y)*(a1-d)/2;
  a =(a0+y) * Fsg_ADRC(y,d) + a2 * ( 1 - Fsg_ADRC(y,d));
  
  return - r * (a/d) * Fsg_ADRC(a,d) - r * Sign_ADRC(a) * ( 1 - Fsg_ADRC(a,d) );
 
}


/*------transcient profile generator, tunning parameters are h, r---------------*/
void TD_ADRC(Fhan_Data *fhan_Input,float expect_ADRC)//安排ADRC过度过程
{
  
  fhan_Input->fh =Fhan_ADRC( fhan_Input->x1 - expect_ADRC , fhan_Input->x2, fhan_Input->r, fhan_Input->h);
  fhan_Input->x1 += fhan_Input->h*fhan_Input->x2;//跟新最速跟踪状态量x1
  fhan_Input->x2 += fhan_Input->h*fhan_Input->fh;//跟新最速跟踪状态量微分x2
  
 }



/*--------------ESO,tunning parameters are w0 and b0 -------------*/
void ESO(Fhan_Data *fhan_Input, float final_signal, float feedback_signal, float w0)
{

fhan_Input->e = fhan_Input->z1 - feedback_signal ;//状态误差

 /*2阶 LESO */
 float LESO_w0 = w0 ;
 fhan_Input->z1 += fhan_Input->h * ( fhan_Input->z2 - ( 2 * LESO_w0 ) * fhan_Input->e + fhan_Input->b0 * final_signal );
 fhan_Input->z2 += fhan_Input->h * ( - ( LESO_w0 * LESO_w0 ) * fhan_Input->e);
                                    

}






void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
{
// just a special kind of PD control 
// tunning parameter is beta_1  beta_2 (like Kp Kd in PD control)

  float temp_e2 = 0;
  temp_e2=Constrain_Float(fhan_Input->e2,-3000,3000);
  
  // special P control. also used for logging
  fhan_Input->ADRC_P_signal = fhan_Input->beta_1*Fal_ADRC(fhan_Input->e1,fhan_Input->alpha1,fhan_Input->zeta); 

  // special D control. also used for logging
  fhan_Input->ADRC_D_signal = fhan_Input->beta_2*Fal_ADRC(temp_e2,fhan_Input->alpha2,fhan_Input->zeta); 

  //final PD control signal
  fhan_Input->u0=fhan_Input->ADRC_P_signal + fhan_Input->ADRC_D_signal;  
                
  
  

}


void ADRC_Control(Fhan_Data *fhan_Input , float expect_ADRC , float feedback_ADRC)
{

float ADRC_error = expect_ADRC - feedback_ADRC ;

/*ADRC step 1    TD*/
 TD_ADRC(fhan_Input , ADRC_error );


/*ADRC step 2    ESO*/

 fhan_Input->y=feedback_ADRC;      

 ESO(fhan_Input,fhan_Input->u,feedback_ADRC, fhan_Input->w0); 

/*ADRC step 3 NLSEF*/  

fhan_Input->e1 = fhan_Input->x1 ;
fhan_Input->e2 = fhan_Input->x2 ;

Nolinear_Conbination_ADRC(fhan_Input);

 
/*--------- compensate for disturbance ---------*/
fhan_Input->u=fhan_Input->u0-fhan_Input->k*fhan_Input->z2/fhan_Input->b0;

  
/*-------output control signal --------*/
fhan_Input->u=Constrain_Float(fhan_Input->u,-1,1); 

/*----------for logging-----------*/
fhan_Input->ADRC_final_signal = fhan_Input->u;    
  
}

