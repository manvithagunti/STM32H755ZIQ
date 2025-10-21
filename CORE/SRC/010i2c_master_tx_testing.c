/*
 * 010i2c_master_tx_testing.c
 *
 *  Created on: Oct 21, 2025
 *      Author: manvitha.g
 */

#include<stdio.h>
#include<string.h>
#include "stm32h7xx.h"
#include "i2c_driver.h"   // Your I2C_Init/I2C_MasterSendData APIs
#include "gpio_driver.h"  // Your GPIO_Init APIs

void delay(void)
{
	for(volatile uint32_t i = 0; i < 500000/2; i++);
}

// Handle for I2C1
I2C_Handle_t I2C1Handle;

// GPIO for LED
GPIO_Handle_t GpioLed;

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4; // AF4 for I2C1
	I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	// SCL
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
	GPIO_Init(&I2CPins);

	// SDA
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
	GPIO_Init(&I2CPins);
}

void I2C1_Inits(void)
{
	I2C1Handle.pI2Cx = I2C1;

	// I2C configuration
	I2C1Handle.I2C_Config.I2C_DeviceAddress   = 0x61;       // STM32 as slave address (7-bit)
	I2C1Handle.I2C_Config.I2C_AddressMode     = I2C_ADDRMODE_7BIT;
	I2C1Handle.I2C_Config.I2C_AnalogFilter    = I2C_ANALOGFILTER_ENABLE;
	I2C1Handle.I2C_Config.I2C_DigitalFilter   = 0;
	I2C1Handle.I2C_Config.I2C_ClockStretching = I2C_STRETCH_ENABLE;
	I2C1Handle.I2C_Config.I2C_GeneralCall     = I2C_GENCALL_DISABLE;
	I2C1Handle.I2C_Config.I2C_DualAddressMode = I2C_DUALADDR_DISABLE;
	I2C1Handle.I2C_Config.I2C_Timing          = 0x10707DBC; // Example TIMINGR for 100kHz @ 64MHz

	// Initialize I2C
	I2C_Init(&I2C1Handle);
}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t GpioBtn;

	// Button configuration
	GpioBtn.pGPIOx = GPIOB;
	GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU; // Pull-up

	GPIO_Init(&GpioBtn);

	// LED configuration
	GpioLed.pGPIOx = GPIOB;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioLed);
}

int main(void)
{
	// Initialize GPIOs and I2C
	I2C1_GPIOInits();
	I2C1_Inits();
	GPIO_ButtonInit();

	uint8_t data[] = "Hello World"; // Data to send
	uint8_t SlaveAddr = 0x68;          // Arduino I2C slave address

	while(1)
	{
		// Wait for button press (active high)
		if(GPIO_ReadFromInputPin(GPIOB, GPIO_PIN_NO_12))
		{
			delay(); // simple debounce

			// Send data to Arduino
			I2C_MasterSendData(&I2C1Handle, SlaveAddr, data, strlen((char*)data));

			// Toggle LED to indicate successful transmission
			GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
		}
	}
}
