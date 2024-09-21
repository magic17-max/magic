#include "stm32f10x.h"                  // Device header
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Encoder.h"
#include "Motor.h"
#include "MPU6050.h"
#include "IMU.h"

int a;

int PWM_out;

//左右编码器
int encoder_left,encoder_right;

//MPU6050
int16_t AX, AY, AZ, GX, GY, GZ;

//闭环控制中间变量
int Vertical_out,Velocity_out;
int Turn_out,target_AX;
int target_speed=0,target_turn,moto1,moto2;

//平衡时角度值偏移量
float Med_Angle=10;

float Vertical_Kp=-1,Vertical_Kd=0.28;
float Velocity_Kp=0.4,Velocity_Ki=0.002;

/*****************  
直立环
PD:Kp*Ek+Kd*Ek_D
输入：期望角度，真实角度，角速度
******************/
int Vertical(float target_AX,float R_AX,float R_GY) 
{
  int pwm;
  
  pwm = Vertical_Kp*(R_AX-target_AX)+Vertical_Kd*R_GY;
  
  return pwm;
} 

/*****************  
速度环:PI
输入：期望速度，左编码器，右编码器
******************/
int Velocity(int target_v,int encoder_left,int encoder_right)
{
	static int Err_lowout_last,Encoder_S;
	static float a=0.7;
	int Err,Err_lowout,temp;
	//1.计算偏差值
	Err=(encoder_left+encoder_right)-target_v;
	//2.低通滤波
	Err_lowout=(1-a)*Err+a*Err_lowout_last;
	Err_lowout_last=Err_lowout;
	//3.积分
	Encoder_S+=Err_lowout;
	//4.积分限幅
	Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
	//5.速度环计算
	temp=Velocity_Kp*Err_lowout+Velocity_Ki*Encoder_S;
	return temp;
}

void limit(void)
{
	if(PWM_out>100) PWM_out=90;
	if(PWM_out<-100) PWM_out=-90;
}

void Control(void)		//每隔10ms调用一次
{
	//1.读取编码器和陀螺仪的值
	encoder_left=Encoder_Get_left();
	encoder_right=Encoder_Get_right();
	
	//2.将数据传入到pid控制器中，计算输出结果，左右电机转速值
	Velocity_out=Velocity(target_speed,encoder_left,encoder_right);
	Vertical_out=Vertical(Velocity_out+Med_Angle,imu_Angle.Pitch,GY);
	PWM_out=Vertical_out;
	limit();
	Motor_SetSpeed_1(PWM_out);
	Motor_SetSpeed_2(PWM_out);
}

int main(void)
{
	OLED_Init();
	Timer_Init();
	Encoder_Init();
	Motor_Init();
	MPU6050_Init();
	
	OLED_ShowString(1, 1, "PWM:");
	OLED_ShowString(3, 1, "left:");
	OLED_ShowString(4, 1, "right:");
	
	while (1)
	{
		OLED_ShowSignedNum(1, 7, PWM_out, 5);
	
//		OLED_ShowSignedNum(3, 7, encoder_left, 5);
//		OLED_ShowSignedNum(4, 7, encoder_right, 5);
		
		OLED_ShowSignedNum(2, 1, imu_Angle.Pitch, 5);
		OLED_ShowSignedNum(3, 1, imu_Angle.Roll, 5);
		OLED_ShowSignedNum(4, 1, imu_Angle.Yaw, 5);
	}
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		IMU_getEuleranAngles();
		Control();
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}
