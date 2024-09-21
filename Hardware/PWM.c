#include "stm32f10x.h"                  // Device header

void PWM_Init(void)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);  
	GPIO_PinRemapConfig(GPIO_Remap_TIM4, ENABLE); 
 

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	TIM_TimeBaseStructure.TIM_Period = 50-1; 
	TIM_TimeBaseStructure.TIM_Prescaler =7200-1; 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; 
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); 
	

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
//CH1	
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);  
	TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);  
//CH2
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);  
	TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Enable);  
	
//CH3
//	TIM_OC3Init(TIM4, &TIM_OCInitStructure);  
//	TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);  
//CH4
//	TIM_OC4Init(TIM4, &TIM_OCInitStructure);  
//	TIM_OC4PreloadConfig(TIM4, TIM_OCPreload_Enable);  
	
	TIM_Cmd(TIM4, ENABLE);  
}

void PWM_SetCompare1(uint16_t Compare1)
{
	TIM_SetCompare1(TIM4, Compare1);
}

void PWM_SetCompare2(uint16_t Compare2)                            
{
	TIM_SetCompare2(TIM4,Compare2);
}
