#define _ADRC_H_


typedef struct
{

/*****安排过度过程*******/
float x1=0.0f;//跟踪微分期状态量
float x2=0.0f;//跟踪微分期状态量微分项
float r=1e15;//时间尺度  30000.0f 30000000000.0f  1e6 越小偏差越小？
float h=0.0005f;//ADRC系统积分时间POS中更小 0.001 0.0001f
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

/**********系统状态误差反馈率*********/
float e0;//状态误差积分项
float e1;//状态偏差
float e2;//状态量微分项
float u0;//非线性组合系统输出
float u;//带扰动补偿后的输出
float b0=10;//扰动补偿
float w0=1.0;

float kp = 0;
float kd = 0;
//float P_signal =0;
//float D_signal =0;



//////////////////////////////////////////////////
int constrain_flag = 0 ;
//int ADRC_flag = 0 ;
//float gravity_compenseter = 0 ;
//float bais_compenseter = 0 ;
float gain_compenseter = 1 ;
float currant_altitude = 0;
float k=1 ;//z2的系数   
float PD=0;
float ADRC_P_signal;
float ADRC_D_signal;
float ADRC_final_signal;
/////////////////////////////////////////////////
float actual_angle=0.0f;

}POS_Fhan_Data;




void TD_ADRC_POS(POS_Fhan_Data *fhan_Input,float expect_ADRC);
void ADRC_Control_POS(POS_Fhan_Data *fhan_Input,float expect_ADRC,float feedback);
float Constrain_Float_POS(float amt, float low, float high);
int Sign_ADRC_POS(float Input);
int Fsg_ADRC_POS(float x,float d);
float Fal_ADRC_POS(float e,float alpha,float zeta);
void ESO_ADRC_POS(POS_Fhan_Data *fhan_Input);
float Fhan_ADRC_POS(float x1_delta , float x2 ,float r , float h);
void linear_Conbination_ADRC_POS(POS_Fhan_Data *fhan_Input);
void ESO_POS(POS_Fhan_Data *fhan_Input, float PD_signal, float feedback_signal ,float w0 );
void first_order_ESO_POS(POS_Fhan_Data *fhan_Input, float final_signal, float feedback_signal ,float w0 );


