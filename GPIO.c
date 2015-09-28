#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "GPIO.h"


//------------------------------------------------------------------------------------------------------------------------//
void TIM1_Init( void )
{
	GPIO_InitTypeDef 				GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  		TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  			TIM_OCInitStructure;

	// TIM1 clock enable

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

	// GPIOA clock enable

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	// GPIOA Configuration: TIM1 CH1 (PA8), CH2 (PA9), CH3 (PA10)

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_OD;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Connect TIM1 pins to AF2

	GPIO_PinAFConfig(GPIOA , GPIO_PinSource8 , GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA , GPIO_PinSource9 , GPIO_AF_2);
	GPIO_PinAFConfig(GPIOA , GPIO_PinSource10, GPIO_AF_2);

	// Time base configuration : 1ms = 1kHz

	TIM_TimeBaseStructure.TIM_Prescaler = 0;		// Clock 48 MHz
	TIM_TimeBaseStructure.TIM_Period = (1 << 16) - 1;	// Period is 65535 tick = 65535/48e6 s = 732.4Hz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

	// PWM1 Mode configuration: Channel 1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// PWM1 Mode configuration: Channel 2

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	// PWM1 Mode configuration: Channel 3

	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;

	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);


	// TIM1 enable counter
	TIM_Cmd(TIM1, ENABLE);
	/* TIM1 Main Output Enable */
	TIM_CtrlPWMOutputs(TIM1, ENABLE);
}


//------------------------------------------------------------------------------------------------------------------------//
void Led_Init( void )
{
	// Create struct for all GPIO parameters
	GPIO_InitTypeDef GPIO_InitStructure;
	// Initialize clock for GPIO port A
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	// Fill Struct with parameters : PA_5 , Output , 50MHz speed , No pull up resistor
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}


//------------------------------------------------------------------------------------------------------------------------//
void GPIOC_Init( void )
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

	/* Configure PC10 and PC11 in output push-pull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_ResetBits(GPIOC , GPIO_Pin_7);
}


//------------------------------------------------------------------------------------------------------------------------//
void Button_Init(void) {
    /* Set variables used */
    GPIO_InitTypeDef GPIO_InitStruct;
    EXTI_InitTypeDef EXTI_InitStruct;
    NVIC_InitTypeDef NVIC_InitStruct;

    /* Enable clock for GPIOC */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
    /* Enable clock for SYSCFG */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

    /* Set pin as input */
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* Tell system that you will use PC13 for EXTI_Line13 */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOC, EXTI_PinSource13);

    /* PD0 is connected to EXTI_Line13 */
    EXTI_InitStruct.EXTI_Line = EXTI_Line13;
    /* Enable interrupt */
    EXTI_InitStruct.EXTI_LineCmd = ENABLE;
    /* Interrupt mode */
    EXTI_InitStruct.EXTI_Mode = EXTI_Mode_Interrupt;
    /* Triggers on rising and falling edge */
    EXTI_InitStruct.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    /* Add to EXTI */
    EXTI_Init(&EXTI_InitStruct);

    /* Add IRQ vector to NVIC */
    /* PD0 is connected to EXTI_Line0, which has EXTI0_IRQn vector */
    NVIC_InitStruct.NVIC_IRQChannel = EXTI4_15_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    /* Add to NVIC */
    NVIC_Init(&NVIC_InitStruct);
}


//------------------------------------------------------------------------------------------------------------------------//
void Led_StartUp_Flash( void )
{
	if (RCC_GetFlagStatus (RCC_FLAG_IWDGRST)) // Reset from the watchdog
	{
		Flash_Led(GPIOA, GPIO_Pin_5, 2, 500);
		RCC_ClearFlag();
	}

	else // Normal Reset
	{
		Flash_Led(GPIOA, GPIO_Pin_5, 5, 100);
		delay_nms(100);
	}
}


//------------------------------------------------------------------------------------------------------------------------//
void Flash_Led( GPIO_TypeDef* GPIOx,
		        uint16_t GPIO_Pin,
		        uint8_t number_of_flash,
		        uint16_t delay_between_flash )
{
	int counter_flash = 0;

	for (counter_flash = 0 ; counter_flash < number_of_flash ; counter_flash++)
	{
		GPIO_SetBits(GPIOx, GPIO_Pin);
		delay_nms(delay_between_flash);
		GPIO_ResetBits(GPIOx, GPIO_Pin);
		delay_nms(delay_between_flash);
	}
}


//------------------------------------------------------------------------------------------------------------------------//
void GPIO_ToggleBits( GPIO_TypeDef* GPIOx , uint16_t GPIO_Pin )
{
	uint8_t current_state = GPIO_ReadOutputDataBit( GPIOx , GPIO_Pin );

	if (current_state == (uint8_t)Bit_SET)
	{
		GPIO_ResetBits(GPIOx, GPIO_Pin);
	}
	else
	{
		GPIO_SetBits(GPIOx, GPIO_Pin);
	}
}


//------------------------------------------------------------------------------------------------------------------------//
void Set_PWM( uint16_t red , uint16_t green , uint16_t blue )
{
	TIM_OCInitTypeDef  			TIM_OCInitStructure;

	// PWM1 Mode configuration: Channel 1
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse = red;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel2 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = green;

	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);

	/* PWM1 Mode configuration: Channel3 */
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = blue;

	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
}
