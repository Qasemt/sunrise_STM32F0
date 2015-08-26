#include "stm32f0xx.h"
#include "stm32f0xx_conf.h"
#include "delay.h"

void Led_Init( void );
void Led_Flash_StartUp( void );
void USART2_Init( void );
void ADC_DMA_Init(void);

uint8_t dummy;


//------------------------------------------------------------------------------------------------------------------------//
int main(void)
 {
	SysTick_Init(1000);

	Led_Init();
	USART2_Init();
	ADC_DMA_Init();
	Led_Flash_StartUp();

    while(1)
    {
    	static uint8_t counter = 0;

    	counter++;

    	delay_nms(1000);
    }
}

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

void ADC_DMA_Init(void)
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

	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)0x40012400;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&ADC_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 3;
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

	// ADC Common Init

	ADC_DeInit(ADC1);

	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_20Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);

	// ADC1 Init

	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T1_CC1;
	ADC_InitStructure.ADC_NbrOfConversion = 3;
	ADC_Init(ADC1, &ADC_InitStructure);

	// ADC1 regular channels 1, 3 configuration

	ADC_RegularChannelConfig(ADC1, ADC_Channel_4,  1, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_5,  2, ADC_SampleTime_28Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_6,  3, ADC_SampleTime_28Cycles);

	// Enable DMA request after last transfer (Single-ADC mode)

	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	// Enable ADC1 DMA

	ADC_DMACmd(ADC1, ENABLE);

	// Enable ADC1
	ADC_Cmd(ADC1, ENABLE);
	}
