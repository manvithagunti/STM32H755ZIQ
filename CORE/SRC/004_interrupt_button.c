/*
 * 002_user_button.c
 *
 *  Created on: Oct 2, 2025
 *      Author: manvitha.g
 */

#include "string.h"
#include "stm32h7xx.h"

#define HIGH 1
#define LOW  0
#define BUTTON_PRESSED LOW


void delay(void)
{
	for(volatile uint32_t i=0;i<500000/2 ; i++);
}


int main(void)
{

	GPIO_Handle_t GpioLed, GpioBtn;
	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GpioBtn,0,sizeof(GpioBtn));

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
	GpioBtn.pGPIOx = GPIOD;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_13;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	//OP TYPE NOT APPLICABLE NOW AS THE MODE IS NOT OUTPUT
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;//NO PUPD BECAUSE EXTERNAL PULL DOWN IS THERE. CHECK SCHEMATICS OF B1. PAGE 3

	GPIO_PeriClockControl(GPIOD, ENABLE);

	GPIO_Init(&GpioBtn);

	//IRQ CONFIGURATIONS
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10,NVIC_IRQ_PRI15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

    while(1);
}

void EXTI15_10_IRQHandler(void)
{
	delay();//200ms  . wait till button de-bouncing gets over
	GPIO_IRQHandling(GPIO_PIN_NO_13);//clear the pending event from exti line
	GPIO_ToggleOutputPin(GPIOB, GPIO_PIN_NO_0);
}
