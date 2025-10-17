/*
 * 006_spi_tx_testing.c
 *
 *  Created on: Oct 13, 2025
 *      Author: manvitha.g
 */
#include "stm32h7xx.h"
#include <string.h>


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
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_14;
	//GPIOInit(&SPIPins);

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


	SPI_Handle_t SPI4handle;
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
		GpioBtn.pGPIOx = GPIOC;
		GpioBtn.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_13;
		GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
		GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
		//OP TYPE NOT APPLICABLE NOW AS THE MODE IS NOT OUTPUT
		GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;//NO PUPD BECAUSE EXTERNAL PULL DOWN IS THERE. CHECK SCHEMATICS OF B1. PAGE 3

		GPIO_Init(&GpioBtn);

}
int main(void)
{
	char user_data[]= "Hello World";

    GPIO_ButtonInit();

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

	//ENABLING THE SSOE BIT
	SPI_SSOEConfig(SPI4,ENABLE);

	while(1)
	{

	    while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

	    //TO AVOID BUTTON DEBOUNCING
	    delay();

        //ENABLING THE spi4 peripheral
	    SPI_PeripheralControl(SPI4,ENABLE);

	    //first send length information
	    uint8_t dataLen= strlen(user_data);
	    SPI_SendData(SPI4,&dataLen,1);

	    //Sending the data
	    SPI_SendData(SPI4, (uint8_t*)user_data,strlen(user_data));


	    //CONFIRM THAT SPI ISNT BUSY
	   // while (!(SPI4->SR & SPI_SR_TXC_FLAG));   // Wait for Transmission Complete


	    //DISABLE THE SPI
	   // SPI_PeripheralControl(SPI4,DISABLE);
	}


	    return 0;

}
