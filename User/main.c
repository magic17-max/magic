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

//���ұ�����
int encoder_left,encoder_right;

//MPU6050
int16_t AX, AY, AZ, GX, GY, GZ;

//�ջ������м����
int Vertical_out,Velocity_out;
int Turn_out,target_AX;
int target_speed=0,target_turn,moto1,moto2;

//ƽ��ʱ�Ƕ�ֵƫ����
float Med_Angle=10;

float Vertical_Kp=-1,Vertical_Kd=0.28;
float Velocity_Kp=0.4,Velocity_Ki=0.002;

/*****************  
ֱ����
PD:Kp*Ek+Kd*Ek_D
���룺�����Ƕȣ���ʵ�Ƕȣ����ٶ�
******************/
int Vertical(float target_AX,float R_AX,float R_GY) 
{
  int pwm;
  
  pwm = Vertical_Kp*(R_AX-target_AX)+Vertical_Kd*R_GY;
  
  return pwm;
} 

/*****************  
�ٶȻ�:PI
���룺�����ٶȣ�����������ұ�����
******************/
int Velocity(int target_v,int encoder_left,int encoder_right)
{
	static int Err_lowout_last,Encoder_S;
	static float a=0.7;
	int Err,Err_lowout,temp;
	//1.����ƫ��ֵ
	Err=(encoder_left+encoder_right)-target_v;
	//2.��ͨ�˲�
	Err_lowout=(1-a)*Err+a*Err_lowout_last;
	Err_lowout_last=Err_lowout;
	//3.����
	Encoder_S+=Err_lowout;
	//4.�����޷�
	Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
	//5.�ٶȻ�����
	temp=Velocity_Kp*Err_lowout+Velocity_Ki*Encoder_S;
	return temp;
}

void limit(void)
{
	if(PWM_out>100) PWM_out=90;
	if(PWM_out<-100) PWM_out=-90;
}

void Control(void)		//ÿ��10ms����һ��
{
	//1.��ȡ�������������ǵ�ֵ
	encoder_left=Encoder_Get_left();
	encoder_right=Encoder_Get_right();
	
	//2.�����ݴ��뵽pid�������У����������������ҵ��ת��ֵ
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
