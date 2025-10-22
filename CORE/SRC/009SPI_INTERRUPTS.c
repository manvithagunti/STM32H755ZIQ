/*
 * 009spi_message_rcv_it.c
 *
 *  Created on: Oct 20, 2025
 *      Author: manvitha.g
 */
//DOWNLOAD THIS CODE ONTO STM
//003SPISLAVEUARTREADOVERSPI.INO-  THIS CODE ONTO ARDUINO
//RESET BOTH BOARDS
//ENABLE SWV ITM DATA CONSOLE
//OPEN ARDUINO IDE SERIAL MONITOR
//TYPE ANYTHING AND SEND MESSAGE- IN SERIAL MONITOR, TOOL LINE ENDING MUST BE SET TO CARRIAGE RETURN


#include<stdio.h>
#include<string.h>
#include "stm32h7xx.h"


SPI_Handle_t SPI4handle;

#define MAX_LEN 500

char RcvBuff[MAX_LEN];

volatile char ReadByte;


volatile uint8_t rcvStop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}

/*
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 -> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * ALT function mode : 5
 */

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




void Slave_GPIO_InterruptPinInit(void)
{
	GPIO_Handle_t spiIntPin;

		//THIS IS LED GPIO CONFIG
	spiIntPin.pGPIOx = GPIOD;
	spiIntPin.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_6;
	spiIntPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	spiIntPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_LOW;
	//spiIntPin.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	spiIntPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

		GPIO_PeriClockControl(GPIOD, ENABLE);

		GPIO_Init(&spiIntPin);

		GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
		GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);
}


int main(void)
{
	uint8_t dummy= 0xff;

	Slave_GPIO_InterruptPinInit();

	//this function is used to initialize teh GPIO pins to behave as SPI4 pins
	SPI4_GPIOInits();

	//this function is used to initialize teh SPI PERIPHERAL PARAMETERS
	SPI4_Inits();




	/*
	 * making SSOE 1 does NSS output enable
	 * the NSS pin is automatically managed by hardware
	 * i.e when SPE=1, NSS will be pulled to low
	 * and NSS pin will be high when SPE=0
	 */
	SPI_SSOEConfig(SPI4, ENABLE);

	SPI_IRQInterruptConfig(IRQ_NO_SPI4,ENABLE);

		while(1){

			rcvStop = 0;

			while(!dataAvailable); //wait till data available interrupt from transmitter device(slave)

			GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,DISABLE);

			//enable the SPI2 peripheral
			SPI_PeripheralControl(SPI4,ENABLE);


			while(!rcvStop)
			{
				/* fetch the data from the SPI peripheral byte by byte in interrupt mode */
				while ( SPI_SendDataIT(&SPI4handle,&dummy,1) == SPI_BUSY_IN_TX);
				while ( SPI_ReceiveDataIT(&SPI4handle,&ReadByte,1) == SPI_BUSY_IN_RX );
			}


			// confirm SPI is not busy
			while( SPI_GetFlagStatus(SPI4,SPI_BUSY_FLAG) );

			//Disable the SPI2 peripheral
			SPI_PeripheralControl(SPI4,DISABLE);

			printf("Rcvd data = %s\n",RcvBuff);

			dataAvailable = 0;

			GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);
		}

			return 0;

}


/* Runs when a data byte is received from the peripheral over SPI*/
void SPI4_IRQHandler(void)
{

	SPI_IRQHandling(&SPI4handle);
}



void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{
	static uint32_t i = 0;
	/* In the RX complete event , copy data in to rcv buffer . '\0' indicates end of message(rcvStop = 1) */
	if(AppEv == SPI_EVENT_RX_CMPLT)
	{
				RcvBuff[i++] = ReadByte;
				if(ReadByte == '\0' || ( i == MAX_LEN)){
					rcvStop = 1;
					RcvBuff[i-1] = '\0';
					i = 0;
				}
	}

}

/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;
}
