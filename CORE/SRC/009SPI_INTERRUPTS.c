/*
 * 006_spi_tx_testing.c
 *
 *  Created on: Oct 13, 2025
 *      Author: manvitha.g
 */
#include "stm32h7xx.h"
#include <string.h>

SPI_Handle_t SPI4handle;

void SPI4_IRQHandler(void){

	SPI_IRQHandling(&SPI4handle);
}

void delay(void)
{
	for(uint32_t i=0;i< 500000/2 ; i++);
}


//PB15-SPI2_MOSI
//PB13-SPI2_SCK
//PB12-SPI2_NSS

void SPI4_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOE;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode =5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_LOW;

	GPIOE_PCLK_EN();

	//CONFIGURING SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_2;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);


	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);

	//mco2 cnfg
//	GPIOA_PCLK_EN();
//	GPIO_Handle_t MCOPins={0};
//	MCOPins.pGPIOx = GPIOA;
//	MCOPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
//	MCOPins.GPIO_PinConfig.GPIO_PinAltFunMode =0;
//	MCOPins.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
//	MCOPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
//	MCOPins.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_HIGH;
//	MCOPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_8;
//	GPIO_Init(&MCOPins);

}


void SPI4_Inits(void)
{
    //PLL2 CONFIGURATION
	RCC->PLLCFGR |= (0x7<<4);//PLL2RGE AND VCOSEL AND FRACEN ENABLED
	RCC->PLL2DIVR &= ~(0X1FF <<0);// CLEARING DIVN2
	RCC->PLL2DIVR |= (0X4F <<0);//DIVN2 SETTING
	RCC->PLL2DIVR  |= (0x13 <<16);//DIVQ2 SETTING

	RCC->CR |= (1<<26);//PLLON

	while(!(RCC->CR & (1<<27)));//PLL2RDY FLAG.....WAITING FOR THE FLAG

	RCC->D2CCIP1R |= (0X1<<16);	//CK SELECTION- SELECTING PLL2_Q


	SPI4handle.pSPIx = SPI4;

	SPI_PeriClockControl(SPI4, ENABLE);

	SPI4_GPIOInits();
	SPI4handle.SPIConfig.SPI_BusConfig= SPI_BUS_CONFIG_SIMPLEX_TXONLY;
	SPI4handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	SPI4handle.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV8;//GENERATES SERIAL CLOCK OF 2MHz
	SPI4handle.SPIConfig.SPI_DFF= SPI_DFF_8BITS;
	SPI4handle.SPIConfig.SPI_CPOL= SPI_CPOL_LOW;
	SPI4handle.SPIConfig.SPI_CPHA= SPI_CPHA_LOW;
	SPI4handle.SPIConfig.SPI_SSM= SPI_SSM_HW;//HARDWARE SLAVE MANAGEMENT ENABLED FOR NSS PIN

	SPI_Init(&SPI4handle);
	//APB2 CLOCK HAS BEEN ENABLED IN THE SPI_INIT IMPLEMENTATION
}

void GPIO_ButtonInit(void)
{
	    GPIO_Handle_t GpioBtn;
	    //THIS IS BTN GPIO CONFIG
		GpioBtn.pGPIOx = GPIOB;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_12;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		//OP TYPE NOT APPLICABLE NOW AS THE MODE IS NOT OUTPUT
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;//NO PUPD BECAUSE EXTERNAL PULL DOWN IS THERE. CHECK SCHEMATICS OF B1. PAGE 3

		GPIO_Init(&GpioBtn);

}

int main(void)
{


    GPIO_ButtonInit();

	SPI4_Inits();

    SPI_IRQInterruptConfig(IRQ_NO_SPI4, ENABLE);


	while(1)
	{

	}


	    return 0;

}
