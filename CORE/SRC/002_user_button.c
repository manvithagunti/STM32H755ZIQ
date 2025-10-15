/*
 * 002_user_button.c
 *
 *  Created on: Oct 2, 2025
 *      Author: manvitha.g
 */


#include "stm32h7xx.h"

#define HIGH 1
#define LOW  0
#define BUTTON_PRESSED HIGH


void delay(void)
{
	for(volatile uint32_t i=0;i<500000 ; i++);
}


int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;

	//THIS IS LED GPIO CONFIG
	GpioLed.pGPIOx = GPIOB;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_0;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&GpioLed);


	//THIS IS BTN GPIO CONFIG
	GpioBtn.pGPIOx = GPIOC;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//OP TYPE NOT APPLICABLE NOW AS THE MODE IS NOT OUTPUT
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;//NO PUPD BECAUSE EXTERNAL PULL DOWN IS THERE. CHECK SCHEMATICS OF B1. PAGE 3

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&GpioBtn);


	while(1)
	{
		uint8_t btn = GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13);  // read into variable

		if(btn == BUTTON_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_0);
		}



	}
	return 0;
}
