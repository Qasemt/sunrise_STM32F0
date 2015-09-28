//------------------------------------------------------------------------------------------------------------------------//
// Part of this code was inspired by L.Latorre's work
// Author : L.Latorre , L.Bonicel
//------------------------------------------------------------------------------------------------------------------------//

#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "delay.h"
#include "USART_Init.h"
#include "GPIO.h"
#include "Alarm.h"

#include <stdio.h>


//------------------------------------------------------------------------------------------------------------------------//


//------------------------------------------------------------------------------------------------------------------------//


//------------------------------------------------------------------------------------------------------------------------//
uint16_t dummy;

volatile uint8_t alarm = 0;
volatile uint8_t setAlarm = 0;

//------------------------------------------------------------------------------------------------------------------------//
int main(void)
 {
	SysTick_Init(1000);

	Led_Init();
	GPIOC_Init();
#ifdef DEBUG_USB
	USART2_Init();
#else
	BT_Init();
#endif

	dummy = printf("USART Initialized !\r\n");

	Clock_Init();
	Button_Init();

	TIM1_Init();

	dummy = printf("Timer Initialized !\r\n");

	GPIO_SetBits( GPIOC , GPIO_Pin_7 );

	Set_PWM( MAX_PWM , 0 , 0 );
	delay_nms( 500 );
	Set_PWM( 0 , MAX_PWM , 0 );
	delay_nms( 500 );
	Set_PWM( 0 , 0 , MAX_PWM );
	delay_nms( 500 );
	Set_PWM( MAX_PWM , MAX_PWM , MAX_PWM );
	delay_nms( 2000 );
	Set_PWM( 0 , 0 , 0 );

	GPIO_ResetBits( GPIOC , GPIO_Pin_7 );

	Led_StartUp_Flash();

	IWDG_WriteAccessCmd (IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler (IWDG_Prescaler_32);
	IWDG_SetReload(500);
	IWDG_ReloadCounter();

	IWDG_Enable();

    while(1)
    {


    	if( alarm )
    	{
    		WakeUpSequence();
    		alarm = 0;
    	}
    	else if( setAlarm )
    	{
    		RTC_TimeRegulate();
    		setAlarm = 0;
    	}

		delay_nms(200);
		IWDG_ReloadCounter();
    }
}


//------------------------------------------------------------------------------------------------------------------------//
void RTC_IRQHandler(void)
{
	if(RTC_GetITStatus(RTC_IT_ALRA) != RESET)
	{
		RTC_ClearITPendingBit(RTC_IT_ALRA);
		EXTI_ClearITPendingBit(EXTI_Line17);
		alarm = 1;
	}
}


//------------------------------------------------------------------------------------------------------------------------//
void EXTI4_15_IRQHandler(void)
{
  if(EXTI_GetITStatus(EXTI_Line13) != RESET)
  {
    EXTI_ClearITPendingBit(EXTI_Line13);
    setAlarm = 1;
  }
}
