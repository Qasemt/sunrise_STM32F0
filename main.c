//------------------------------------------------------------------------------------------------------------------------//
// Part of this code was inspired by L.Latorre's work
// Author : L.Latorre , L.Bonicel
//------------------------------------------------------------------------------------------------------------------------//

#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "delay.h"

#include <stdio.h>


//------------------------------------------------------------------------------------------------------------------------//
#define BUFFER_SIZE 3
#define MIN_ADC 18
#define MAX_ADC 4082
#define MAX_PWM ((1<<16) - 1)


//------------------------------------------------------------------------------------------------------------------------//
void TIM2_IRQHandler( void );
void Led_Init( void );
void USART2_Init( void );
void TIM1_Init( void );
void TIM2_Init( void );
void ADC_DMA_Init(void);
void Led_StartUp_Flash( void );
void Flash_Led( GPIO_TypeDef* GPIOx,
		        uint16_t GPIO_Pin,
		        uint8_t number_of_flash,
		        uint16_t delay_between_flash );
void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
uint16_t Map_ADC ( uint16_t val );
void Get_ADC (	uint16_t *g_pot_red,
				uint16_t *g_pot_green,
				uint16_t *g_pot_blue );
void Set_PWM( uint16_t red , uint16_t green , uint16_t blue );


//------------------------------------------------------------------------------------------------------------------------//
uint16_t a_pot_value[BUFFER_SIZE];
uint16_t dummy;

volatile uint8_t process = 0;


//------------------------------------------------------------------------------------------------------------------------//
int main(void)
 {
	SysTick_Init(1000);

	Led_Init();
	USART2_Init();

	dummy = printf("USART Initialized !\r\n");

	TIM1_Init();
	TIM2_Init();

	dummy = printf("Timer Initialized !\r\n");

	Set_PWM(MAX_PWM,0,0);
	delay_nms(500);
	Set_PWM(0,MAX_PWM,0);
	delay_nms(500);
	Set_PWM(0,0,MAX_PWM);
	delay_nms(500);
	Set_PWM(MAX_PWM,MAX_PWM,MAX_PWM);
	delay_nms(2000);
	Set_PWM(0,0,0);

	ADC_DMA_Init();
	ADC_StartOfConversion (ADC1);
	dummy = printf("ADC Initialized !\r\n");

	Led_StartUp_Flash();

	IWDG_WriteAccessCmd (IWDG_WriteAccess_Enable);
	IWDG_SetPrescaler (IWDG_Prescaler_32);
	IWDG_SetReload(500);
	IWDG_ReloadCounter();

	IWDG_Enable();

	uint16_t i = 0;

    while(1)
    {
    	while(process)
		{
    		if(i >= (MAX_PWM - 1000))
    		{
    			i = 0;
    		}
    		else
    		{
    			i += 1000;
    		}
    		uint16_t g_pot_red   = i;
			uint16_t g_pot_green = i;
			uint16_t g_pot_blue  = i;

    		// Get_ADC( &g_pot_red , &g_pot_green , &g_pot_blue );

    		Set_PWM( g_pot_red , g_pot_green , g_pot_blue );

    		dummy = printf("PWM values : %d , %d , %d\r\n" , g_pot_red , g_pot_green , g_pot_blue );
    		delay_nms(200);

			process = 0;
			IWDG_ReloadCounter();
		}
    }
}


//------------------------------------------------------------------------------------------------------------------------//
void TIM2_IRQHandler( void )
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		process = 1;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
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
void USART2_Init( void )
{
	GPIO_InitTypeDef 	GPIO_InitStructure;
	USART_InitTypeDef 	USART_InitStructure;

	// Start GPIOA Clock

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	// Start USART2 Clock

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

	// GPIO configuration for USART2 (TX on PA2)

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// Connect the TX-RX pins (PA2 - PA3) to USART alternative function

	GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_1);

	// Setup the properties of USART2

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx;
	USART_Init(USART2, &USART_InitStructure);

	// Enable USART2

	USART_Cmd(USART2, ENABLE);

	delay_nms(1000);
}


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
void TIM2_Init( void )
{
	NVIC_InitTypeDef 			NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef		TIM_TimeBaseStructure;

	// Enable the TIM2 global Interrupt

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// TIM2 clock enable

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

	// Time base configuration

	TIM_TimeBaseStructure.TIM_Period = 10000 - 1; 							// TS in µs
	TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / 1000000) - 1;	// 48 MHz Clock down to 1 MHz
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);

	// TIM IT enable

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	// TIM2 enable counter

	TIM_Cmd(TIM2, ENABLE);
}


//------------------------------------------------------------------------------------------------------------------------//
void ADC_DMA_Init( void )
{
	ADC_InitTypeDef       ADC_InitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;

	// Enable ADCx, DMA and GPIO clocks

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	ADC_DeInit(ADC1);
	ADC_StructInit( &ADC_InitStructure );

	// Configure ADC1 Channels as analog input

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;		// Channels 0|1|4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	// DMA1 Channel 1 configuration

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&a_pot_value;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = BUFFER_SIZE;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	// DMA1 Channel 1 enable
	DMA_Cmd(DMA1_Channel1, ENABLE);

	// Enable ADC1 DMA
	ADC_DMACmd(ADC1, ENABLE);

	// Enable DMA request after last transfer (Single-ADC mode)
	ADC_DMARequestModeConfig(ADC1, ADC_DMAMode_Circular);

	// ADC1 Init
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init(ADC1, &ADC_InitStructure);

	// ADC1 regular channels 1, 3 configuration

	ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_28_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_1, ADC_SampleTime_28_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_28_5Cycles);

	ADC_GetCalibrationFactor(ADC1);

	/* Enable the ADC peripheral */
	ADC_Cmd(ADC1, ENABLE);

	/* Wait the ADRDY flag */
	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_ADRDY));

	/* ADC1 regular Software Start Conv */
	ADC_StartOfConversion(ADC1);
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
void Get_ADC ( uint16_t *g_pot_red , uint16_t *g_pot_green , uint16_t *g_pot_blue )
{
	*g_pot_red    = Map_ADC(a_pot_value[0]);
   	*g_pot_green  = Map_ADC(a_pot_value[1]);
    *g_pot_blue   = Map_ADC(a_pot_value[2]);
}


//------------------------------------------------------------------------------------------------------------------------//
uint16_t Map_ADC ( uint16_t val )
{
	return (uint16_t)(((uint32_t)MAX_PWM * ((uint32_t)val - (uint32_t)MIN_ADC) / ((uint32_t)MAX_ADC - (uint32_t)MIN_ADC)));
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
