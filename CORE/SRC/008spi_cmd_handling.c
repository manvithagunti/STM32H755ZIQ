/*
 * 006_spi_tx_testing.c
 *
 *  Created on: Oct 13, 2025
 *      Author: manvitha.g
 */
#include "stm32h7xx.h"
#include <string.h>

//COMMAND CODES
//theses are the command codes that the slave recognises
#define COMMAND_LED_CTRL               0x50
#define COMMAND_SENSOR_READ            0x51
#define COMMAND_LED_READ               0x52
#define COMMAND_PRINT                  0x53
#define COMMAND_ID_READ                0x54

#define LED_ON          1
#define LED_OFF         0

//ARDUINO ANALOG PINS
#define ANALOG_PIN0     0
#define ANALOG_PIN1     1
#define ANALOG_PIN2     2
#define ANALOG_PIN3     3
#define ANALOG_PIN4     4
#define ANALOG_PIN5     5
//arduino led

#define LED_PIN         8

void delay(void)
{
	for(uint32_t i=0;i< 500000/2 ; i++);
}


//PE6-SPI4_MOSI
//PE2-SPI4_SCK
//PE4-SPI4_NSS
//PE5-SPI4_MISO


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
	SPI4handle.SPIConfig.SPI_BusConfig= SPI_BUS_CONFIG_FD;
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

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	if(ackbyte == 0xF5)
	{
		//ack
		return 1;
	}
	return 0;
}
int main(void)
{
	uint8_t dummy_write = 0xff;
	uint8_t dummy_read;

    GPIO_ButtonInit();

	SPI4_Inits();


	//ENABLING THE SSOE BIT
	SPI_SSOEConfig(SPI4,ENABLE);

	while(1)
	{

	    while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

	    //TO AVOID BUTTON DEBOUNCING
	    delay();



	    //1. CMD_LED_CTRL     <pin no(1)>    <value(1)>

	    uint8_t commandcode = COMMAND_LED_CTRL;
	    uint8_t ackbyte;
	    uint8_t args[2];

	    //SEND COMMAND
	    SPI_SendData(SPI4, &commandcode,1);

//	    //do dummy read
//	    SPI_ReceiveData(SPI4,&dummy_read,1);

//	    //send some dummy bits(1byte) to fetch the response from the slave
//	    SPI_SendData(SPI4, &dummy_write,1);

	    //READ THE ACK BYTE RECEIVED
	    SPI_ReceiveData(SPI4, &ackbyte,1);

	    if(SPI_VerifyResponse(ackbyte) )
	    {
	    	//send arguments
	    	args[0]= LED_PIN;
	    	args[1]= LED_ON;
	    	SPI_SendData(SPI4,args,2);
	    	//SPI_ReceiveData(SPI4,&dummy_read,1);
	    }


	    //2. CMD_SENSOR_READ    <analog pin number (1)>

	    //wait till button is pressed
	    while(! GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) );

	    //TO AVOID BUTTON DEBOUNCING
	    delay();

	    commandcode= COMMAND_SENSOR_READ;

	    //SEND COMMAND
	   	SPI_SendData(SPI4, &commandcode,1);

	   	//do dummy read
	    SPI_ReceiveData(SPI4,&dummy_read,1);

	    //insert some delay so that slave can be ready with the data
	    delay();


	    //send some dummy bits(1byte) to fetch the response from the slave
	    SPI_SendData(SPI4, &dummy_write,1);

	    //READ THE ACK BYTE RECEIVED
	    SPI_ReceiveData(SPI4, &ackbyte,1);

	    if(SPI_VerifyResponse(ackbyte) )
	    {
	       args[0]= ANALOG_PIN0;
	       //send arguments
	       SPI_SendData(SPI4,args,1);//sending one byte
	       //SPI_ReceiveData(SPI4,&dummy_read,1);
	       //do dummy read
	       SPI_ReceiveData(SPI4,&dummy_read,1);

	       //delay
	       delay();

	       //send some dummy bits(1byte) fetch the response from the slave
	       SPI_SendData(SPI4,&dummy_write,1);

	       uint8_t analog_read;
	       SPI_ReceiveData(SPI4,&analog_read,1);

	    }



	    //CONFIRM THAT SPI ISNT BUSY
	    while (!(SPI4->SR & SPI_SR_TXC_FLAG));   // Wait for Transmission Complete


	    //DISABLE THE SPI
	    SPI_PeripheralControl(SPI4,DISABLE);
	}


	    return 0;

}
