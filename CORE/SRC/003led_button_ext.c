/*
 * 003led_button_ext.c
 *
 *  Created on: Oct 3, 2025
 *      Author: manvitha.g
 */

#include "stm32h7xx.h"

#define HIGH 1
#define LOW  0
#define BUTTON_PRESSED LOW


void delay(void)
{
	for(uint32_t i=0;i< 500000/2 ; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed, GpioBtn;

	//THIS IS LED GPIO CONFIG
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_8;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&GpioLed);


	//THIS IS BTN GPIO CONFIG
	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//OP TYPE NOT APPLICABLE NOW AS THE MODE IS NOT OUTPUT
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;//NO PUPD BECAUSE EXTERNAL PULL DOWN IS THERE. CHECK SCHEMATICS OF B1. PAGE 3

	GPIO_PeriClockControl(GPIOB, ENABLE);

	GPIO_Init(&GpioBtn);


	while(1){
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12) == BUTTON_PRESSED)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_8);
		}


	}
	return 0;
}

