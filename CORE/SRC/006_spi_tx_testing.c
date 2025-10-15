/*
 * 006_spi_tx_testing.c
 *
 *  Created on: Oct 13, 2025
 *      Author: manvitha.g
 */
#include "stm32h7xx.h"
#include <string.h>


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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_14;
	//GPIOInits(&SPIPins);

	//NSS
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_12;
	//GPIOInits(&SPIPins);
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


	SPI_Handle_t SPI4handle;
	SPI4handle.pSPIx = SPI4;

	SPI_PeriClockControl(SPI4, ENABLE);

	SPI4_GPIOInits();
	SPI4handle.SPIConfig.SPI_BusConfig= SPI_BUS_CONFIG_SIMPLEX_TXONLY;
	SPI4handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
	//CHECK THIS AGAIN
	SPI4handle.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV2;//GENERATES SERIAL CLOCK OF 8MHz
	SPI4handle.SPIConfig.SPI_DFF= SPI_DFF_8BITS;
	SPI4handle.SPIConfig.SPI_CPOL= SPI_CPOL_LOW;
	SPI4handle.SPIConfig.SPI_CPHA= SPI_CPHA_LOW;
	SPI4handle.SPIConfig.SPI_SSM= SPI_SSM_SW;//SOFTWARE SLAVE MANAGEMENT ENABLED FOR NSS PIN

	SPI_Init(&SPI4handle);
	//APB2 CLOCK HAS BEEN ENABLED IN THE SPI_INIT IMPLEMENTATION
}
int main(void)
{
	char user_data[]= "Hello World";



	//HSI DIVISION BY 8
	//RCC->CR |=(0x3<<3);
	//while(!(RCC->CR & (1<<5)));


	//mco2 config
	//RCC->CFGR |=(0x0<<29);

	SPI4_Inits();

	//THIS FUNCTION IS USED TO INITIlize gpio PINS TO BEHAVE AS SPI2 PINS
		//SPI4_GPIOInits();

	// Clear previous FTHLV bits
	//SPI4->CFG1 &= ~(0x7 << 5);   // Bits 8:5 = FTHLV

	// Set FTHLV = 1 data frame (TX FIFO threshold)
	//SPI4->CR2 |= (0x0 << 5);    // 0b000 = 1 frame


	//enable the SPI4 peripheral
	//SPI_PeripheralControl(SPI4,ENABLE);

	SPI_SendData(SPI4, (uint8_t*)user_data,strlen(user_data));

	while(1);

	return 0;
}
