#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "delay.h"

//------------------------------------------------------------------------------------------------------------------------//
#define BUFFER_SIZE 3

//------------------------------------------------------------------------------------------------------------------------//
void TIM2_IRQHandler( void );
void Led_Init( void );
void USART2_Init( void );
void Led_Flash_StartUp( void );
void TIM2_Init( void );
void ADC_DMA_Init(void);
void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

//------------------------------------------------------------------------------------------------------------------------//
uint16_t a_pot_value[BUFFER_SIZE];
volatile uint8_t process = 0;
volatile uint8_t counter = 0;


//------------------------------------------------------------------------------------------------------------------------//
int main(void)
 {
	SysTick_Init(1000);

	Led_Init();
	USART2_Init();
	Led_Flash_StartUp();

	// Init TIMERS

	TIM2_Init();			// Controller Sampling period

	// Init ADC

	ADC_DMA_Init();
	ADC_StartOfConversion(ADC1);

    while(1)
    {
    	while(process)
		{
    		uint16_t g_pot_red    = a_pot_value[0];
    		uint16_t g_pot_green  = a_pot_value[1];
    		uint16_t g_pot_blue   = a_pot_value[2];

    		if (g_pot_blue > 50 || g_pot_red > 50 || g_pot_green > 50 )
    		{
    			GPIO_SetBits(GPIOA, GPIO_Pin_5);
    		}
    		else
    		{
    			GPIO_ResetBits(GPIOA, GPIO_Pin_5);
    		}

			process = 0;
			//IWDG_ReloadCounter();
		}
    }
}


//------------------------------------------------------------------------------------------------------------------------//
void TIM2_IRQHandler( void )
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)
	{
		process = 1;
		counter++;
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

	// Fill Struct with parameters : PA_5 , Output , 50 MHz speed , No pull up resistor
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
void Led_Flash_StartUp( void )
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	delay_nms(5);
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	delay_nms(500);
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	delay_nms(500);
	GPIO_SetBits(GPIOA, GPIO_Pin_5);
	delay_nms(500);
	GPIO_ResetBits(GPIOA, GPIO_Pin_5);
	delay_nms(500);
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

	TIM_TimeBaseStructure.TIM_Period = 1000000 - 1; 							// TS in µs
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

	// Configure ADC1 Channels as analog input

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_4;		// Channels 0|1|4
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	// DMA1 Channel 1 configuration

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&a_pot_value[0];
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

	ADC_DeInit(ADC1);

	// ADC1 Init
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC4;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
	ADC_Init(ADC1, &ADC_InitStructure);

	// ADC1 regular channels 1, 3 configuration

	ADC_ChannelConfig(ADC1, ADC_Channel_0, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_1, ADC_SampleTime_55_5Cycles);
	ADC_ChannelConfig(ADC1, ADC_Channel_4, ADC_SampleTime_55_5Cycles);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
}


//------------------------------------------------------------------------------------------------------------------------//
void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
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
