#ifndef __GPIO_H
#define __GPIO_H

void TIM1_Init( void );
void Led_Init( void );
void GPIOC_Init( void );
void Button_Init( void );
void Led_StartUp_Flash( void );
void Flash_Led( GPIO_TypeDef* GPIOx,
		        uint16_t GPIO_Pin,
		        uint8_t number_of_flash,
		        uint16_t delay_between_flash );
void GPIO_ToggleBits(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
void Set_PWM( uint16_t red , uint16_t green , uint16_t blue );

#endif //__GPIO_H
