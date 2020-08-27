#define _ADRC_H_

//typedef unsigned short uint16;
//typedef signed short int16;
///////// uint16 may mean unsigned int
typedef struct
{
/*---transcient profile generator parameter---*/
float x1=0.0f;//transcient signal x1 used to track input signal
float x2=0.0f;//dx1/dt 
float r=1e6;  //tunning parameter 
float h=0.0005;//h is integral step, must smaller than 0.0025 (because main loop run this program in 400hz)
float N0=15;
float h0;
float fh;//

/*---ESO---*/
float z1;
float z2; // disturbance result
float e;
float y;  //feedback signal
float b0= 20;   //tunning parameter
float w0 = 50;  //tunning parameter

/*--- NLSEF paramter ---*/
float e1; // P error
float e2; // D error
float u0; // PD control signal
float u;  // final control signal with disturbance compensation

float beta_1=0.2;     //tunning parameter
float beta_2=0.001; //tunning parameter 

float alpha1=0.8f;  //u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)  0.1 loaded0.25
float alpha2=1.5f;  //0<alpha1<1<alpha2?????????????  0.05   loaded0.5
float zeta=50 ;     //can be tunned if necessary, too big will act like linear function

float ADRC_P_signal = 0 ;   // P control signal, also used for logging 
float ADRC_D_signal = 0 ;   // D control signal, also used for logging
float ADRC_final_signal = 0;// only used for logging 

//////////////////////////////////////////////////
float k=1;//z3的系数   0.0015 0.5
float target_signal = 0;
float actual_velocity = 0;
float target_velocity = 0;
float _i=0;
int ADRC_flag = 0 ;
float ADRC_ESO_error1 = 0.0;
float ADRC_ESO_error2 = 0.0;
float ADRC_ESO_z1_error = 0.0;
float ADRC_ESO_z2_error = 0.0;



}Fhan_Data;



void ADRC_Init(Fhan_Data *fhan_Input1,Fhan_Data *fhan_Input2);
void ADRC_Init1(Fhan_Data *fhan_Input1);
void TD_ADRC(Fhan_Data *fhan_Input,float expect_ADRC);
void ADRC_Control(Fhan_Data *fhan_Input,float expect_ADRC,float feedback);
float Constrain_Float(float amt, float low, float high);
int Sign_ADRC(float Input);
int Fsg_ADRC(float x,float d);
float adrc_fhan(float v1, float v2, float r0, float h0);
float Fal_ADRC(float e,float alpha,float zeta);
void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input);
float Fhan_ADRC(float x1_delta , float x2 ,float r , float h);
//extern Fhan_Data ADRC_Pitch_Controller,ADRC_Roll_Controller;
void ESO(Fhan_Data *fhan_Input, float PD_signal, float feedback_signal, float w0);
void linear_Conbination_ADRC(Fhan_Data *fhan_Input);



