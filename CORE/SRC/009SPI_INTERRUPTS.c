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

//extern void initialise_monitor_handles(void);


SPI_Handle_t SPI1handle;

#define MAX_LEN 100

char RcvBuff[MAX_LEN]={1,1,11,1,1,1,1,1,1,1,11,1,1,1,1,1,1,1};

uint8_t ReadByte;
uint32_t i = 0,count=0,ready=0;

volatile uint8_t rcvStop = 0;

/*This flag will be set in the interrupt handler of the Arduino interrupt GPIO */
volatile uint8_t dataAvailable = 0;

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}


//PA7-SPI1_MOSI
//PA5-SPI1_SCK
//PA4-SPI1_NSS
//PA6-SPI1_MISO

void SPI1_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode =5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType= GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed= GPIO_SPEED_LOW;

	GPIOA_PCLK_EN();

	//CONFIGURING SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_5;
	GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_7;
	GPIO_Init(&SPIPins);


	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_6;
	GPIO_Init(&SPIPins);

	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber =GPIO_PIN_NO_4;
	GPIO_Init(&SPIPins);

}



	void SPI1_Inits(void)
	{
	    //PLL2 CONFIGURATION
		RCC->PLLCFGR |= (0x7<<4);//PLL2RGE AND VCOSEL AND FRACEN ENABLED
		RCC->PLL2DIVR &= ~(0X1FF <<0);// CLEARING DIVN2
		RCC->PLL2DIVR |= (0X4F <<0);//DIVN2 SETTING
		RCC->PLL2DIVR  |= (0x13 <<12);//DIVP2 SETTING

		RCC->CR |= (1<<26);//PLLON

		while(!(RCC->CR & (1<<27)));//PLL2RDY FLAG.....WAITING FOR THE FLAG

		RCC->D2CCIP1R |= (0X1<<12);	//CK SELECTION- SELECTING PLL2_P



		SPI1handle.pSPIx = SPI1;

		SPI_PeriClockControl(SPI1, ENABLE);

		SPI1_GPIOInits();
		SPI1handle.SPIConfig.SPI_BusConfig= SPI_BUS_CONFIG_SIMPLEX_RXONLY;
		SPI1handle.SPIConfig.SPI_DeviceMode=SPI_DEVICE_MODE_MASTER;
		SPI1handle.SPIConfig.SPI_SclkSpeed=SPI_SCLK_SPEED_DIV8;//GENERATES SERIAL CLOCK OF 2MHz
		SPI1handle.SPIConfig.SPI_DFF= SPI_DFF_8BITS;
		SPI1handle.SPIConfig.SPI_CPOL= SPI_CPOL_LOW;
		SPI1handle.SPIConfig.SPI_CPHA= SPI_CPHA_LOW;
		SPI1handle.SPIConfig.SPI_SSM= SPI_SSM_HW;//HARDWARE SLAVE MANAGEMENT ENABLED FOR NSS PIN

		SPI_Init(&SPI1handle);
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



    uint8_t dummy = 0xFF;

    Slave_GPIO_InterruptPinInit();
    SPI1_GPIOInits();
    SPI1_Inits();
    SPI_IRQInterruptConfig(IRQ_NO_SPI1, ENABLE);

    while(1)
    {
        rcvStop = 0;

        // Wait for Arduino to notify data ready
        while(!dataAvailable);

        GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, DISABLE);

//        SPI_ReceiveDataIT(&SPI1handle, RcvBuff,20);


        // Wait until full message received
        while(!rcvStop){
            // Start SPI receive for first byte (ISR handles rest)
            SPI_ReceiveDataIT(&SPI1handle, RcvBuff+i,1);
            while(!ready);
            ready=0;

            delay();
        }

        GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
        dataAvailable = 0;

        //initialise_monitor_handles();
        printf("Received message: %s\n", RcvBuff);
    }
}

/* Runs when a data byte is received from the peripheral over SPI*/
void SPI1_IRQHandler(void)
{
	SPI_IRQHandling(&SPI1handle);
    if(++i==20){
    	rcvStop = 1;
    }

}





void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{


    if(AppEv == SPI_EVENT_RX_1BYTE) // single byte received
    {
     RcvBuff[i++] = ReadByte;

        // End of message detection
        if(ReadByte == '\0' || i == MAX_LEN)
        {
            rcvStop = 1;
//            RcvBuff[i-1] = '\0'; // terminate string
            i = 0;

            // Close SPI reception
            SPI_CloseReception(pSPIHandle);
        }
    }
}


/* Slave data available interrupt handler */
void EXTI9_5_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_6);
	dataAvailable = 1;

}
