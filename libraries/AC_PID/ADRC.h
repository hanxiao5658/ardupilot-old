#define _ADRC_H_

//typedef unsigned short uint16;
//typedef signed short int16;
///////// uint16 may mean unsigned int
typedef struct
{
/*****安排过度过程*******/
float x1=0.0f;//跟踪微分期状态量
float x2=0.0f;//跟踪微分期状态量微分项
float r=1e15;//时间尺度  1e15
float h=0.0005f;//ADRC系统积分时间 0.0005
float N0=15;//跟踪微分器解决速度超调h0=N*h 15

float h0;
float fh;//最速微分加速度跟踪量
/*****扩张状态观测器*******/
/******已系统输出y和输入u来跟踪估计系统状态和扰动*****/
float z1;
float z2;
float z3;//根据控制对象输入与输出，提取的扰动信息
float e;//系统状态误差
float y;//系统输出量
float fe;
float fe1;
float beta_01=1000.0f;//300
float beta_02=30000.0f;//4000
float beta_03=10000.0f; //beta01/02/03 为观测器参数 分别为3w，3w*w，w*w*w

/**********系统状态误差反馈率*********/
float e0;//状态误差积分项
float e1;//状态偏差
float e2;//状态量微分项
float u0;//非线性组合系统输出
float u;//带扰动补偿后的输出
float tempu;//位置输出
float b0=500;//扰动补偿

/*********第一种组合形式*********/
/*********第二种组合形式*********/

float beta_1=2;//非线性组合参数 2
float beta_2=0.001;//u0=beta_1*e1+beta_2*e2+(beta_0*e0);  0.001  

float alpha1=0.8f;//u0=beta_1*fal(e1,alpha1,zeta)+beta_2*fal(e2,alpha2,zeta)  0.1 loaded0.25
float alpha2=1.5f;//0<alpha1<1<alpha2?????????????  0.05   loaded0.5
float zeta=50 ;//线性段的区间长度 50 10 15 9 13

/*********第三种组合形式*********/
//float h1;//u0=-fhan(e1,e2,r,h1);
//uint16 N1;//跟踪微分器解决速度超调h0=N*h
/*********第四种组合形式*********/



//////////////////////////////////////////////////
float k=1;//z3的系数   0.0015 0.5
float _i=0;
int ADRC_flag = 0 ;
//float attitudedata=0;////////////////////////////////在pos control.cpp里传递高度 全部用这个就行
/*/float attitudedata_temp=0;
float PIDoutput=0;
float target_roll_angle=0.0f;//在attitudecontrol.cpp里246左右传递角度数据
float target_pitch_angle=0.0f;
float target_yaw_angle=0.0f;

float actual_pitch_angle=0.0f;
float actual_yaw_angle=0.0f;
float alphaz3=0;
float zetaz3=0;
*//////////////////////////////////////////////////
float actual_angle=0.0f;

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
void ESO_ADRC(Fhan_Data *fhan_Input);
void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input);
float Fhan_ADRC(float x1_delta , float x2 ,float r , float h);
void ESO(Fhan_Data *fhan_Input,float raw_control_signal, float feedback_signal );


