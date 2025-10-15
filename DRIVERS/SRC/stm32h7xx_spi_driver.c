/*
 * stm32h7xx_spi_driver.c
 *
 *  Created on: Oct 9, 2025
 *      Author: manvitha.g
 */
#include "stm32h7xx_spi_driver.h"
#include "stm32h7xx.h"

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi ==ENABLE)
	   {
		   if(pSPIx==SPI1){
			   SPI1_PCLK_EN();
		   }else if(pSPIx==SPI2){
			   SPI2_PCLK_EN();
		   }else if(pSPIx==SPI3){
			   SPI3_PCLK_EN();
		   }else if(pSPIx==SPI4){
			   SPI4_PCLK_EN();
		   }else if(pSPIx==SPI5){
			   SPI5_PCLK_EN();
		   }else if(pSPIx==SPI6){
			   SPI6_PCLK_EN();
		   }
	   }
	   else{
		   if(pSPIx==SPI1){
		  	   SPI1_PCLK_DI();
		   }else if(pSPIx==SPI2){
		  	   SPI2_PCLK_DI();
		   }else if(pSPIx==SPI3){
		  	   SPI3_PCLK_DI();
		   }else if(pSPIx==SPI4){
		  	   SPI4_PCLK_DI();
		   }else if(pSPIx==SPI5){
		  	   SPI5_PCLK_DI();
		   }else if(pSPIx==SPI6){
		  	   SPI6_PCLK_DI();
		   }
	   }
}



void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    //PERIPHERAL CLOCK ENABLE

	//SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	//DISABLE the SPE IN SPI_CR1 reg. THIS IS DONE AS ALL THE FURTHER CONFIGURATIONS REQUIRE THE SPE TO BE DISABLED
	pSPIHandle->pSPIx->CR1 &= ~(1<< SPI_CR1_SPE_POS);

    //FIRST LETS CONFIGURE THE SPI_CR1 REGISTER
	uint32_t tempreg =0;


	//CFG2 REGISTER
	if(pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_SW)
	{
		//ssm=1,ssi=1
		//FIRST SET SSI BIT AND THEN SET SSM BIT. CHANGING TO THIS ORDER LED TO THE REDUCTION OF CTSIZE
		pSPIHandle->pSPIx->CR1  |= (1<<SPI_CR1_SSI_POS);//setting the SSI bit=1
		pSPIHandle->pSPIx->CFG2 |= (1<<SPI_CFG2_SSM_POS);//setting the SSM bit=1

	}else
	{
		//ssm=0,ssoe=1
		pSPIHandle->pSPIx->CFG2 &= ~(1<<SPI_CFG2_SSM_POS);//setting the SSM bit=0
		pSPIHandle->pSPIx->CFG2 |= (1<<SPI_CFG2_SSOE_POS);//setting the SSOE bit=1
	}

//IDEAL REGISTER VALUES FROM CUBE MX CODE
//	pSPIHandle->pSPIx->CFG1 = 0x70007UL;
//	pSPIHandle->pSPIx->CFG2 = 0x44420000UL;
	//1. configure the device mode
	pSPIHandle->pSPIx->CFG2 |= pSPIHandle->SPIConfig.SPI_DeviceMode << 22 ;// MASTER BIT IS LOCATED IN SPI_CFG2 REG

	tempreg = pSPIHandle->pSPIx->CFG2;

	//CFG2 REGISTER
	// CONFIGURE THE BUS CONFIG
	//we need not clear the bits before setting because we are anyway using a tempreg.
	tempreg |= (pSPIHandle->SPIConfig.SPI_BusConfig << SPI_CFG2_COMM_POS);//you can use hard coded values like (15) instead of SPI_CFG2_COMM_POS. but it doesnt make the code readable
	//setting the CPHA
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPHA << SPI_CFG2_CPHA_POS);
	//setting the CPOL
	tempreg |= (pSPIHandle->SPIConfig.SPI_CPOL << SPI_CFG2_CPOL_POS);
	//configure AFCNTR later if necessary

	pSPIHandle->pSPIx->CFG2 =tempreg;// loading the contents of the temp register into the SPI_CFG2 reg


	tempreg = 0;// clearing the contents of the tempreg to use it for further configurations

	//CFG1 REGISTER
	//setting the baud rate
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CFG1_MBR_POS);//this refers to the SPI_CFG1 reg
	//tempreg |= (0x3 << 5);//fthlv value

	tempreg |= (pSPIHandle->SPIConfig.SPI_DFF << SPI_CFG1_DSIZE_POS);//SPI_CFG1

    pSPIHandle->pSPIx->CFG1 =tempreg;


    //CONFIGURING THE AFCNTR BIT IN SPI_CFG2. BIT 31
   // pSPIHandle->pSPIx->CFG2 |= (1 << SPI_CFG2_AFCNTR_POS);

    //CONFIGURING THE IOLOCK BIT IN SPI_CR1 BIT 16
  //  pSPIHandle->pSPIx->CR1 |= (1<<SPI_CR1_IOLOCK_POS);




	//INITIALISING THE SPE
	//pSPIHandle->pSPIx->CR1 |= (1<< SPI_CR1_SPE_POS);
	//i have enabled it in the main.c
}


void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx==SPI1){
    SPI1_REG_RESET();//macro for this is defined in the MCU specific header file
	}else if(pSPIx==SPI2){
	SPI2_REG_RESET();
	}else if(pSPIx==SPI3){
	SPI3_REG_RESET();
	}else if(pSPIx==SPI4){
	SPI4_REG_RESET();
	}else if(pSPIx==SPI5){
	SPI5_REG_RESET();
	}else if(pSPIx==SPI6){
	SPI6_REG_RESET();
	}
}


uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx,uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}

	return FLAG_RESET;
}

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len)
{

	//SETTING TSIZE=len
	pSPIx->CR2 |=(Len <<0);


	// 1. Enable SPI peripheral if not already enabled
	if (!(pSPIx->CR1 & SPI_CR1_SPE))
	    pSPIx->CR1 |= SPI_CR1_SPE;//SPI_CR1_SPE



	// 2. Start the communication
	pSPIx->CR1 |= SPI_CR1_CSTART;


    while(Len>0)
    {
    	//1.wait until TXP is set
    	//while( !(pSPIx->SR & (1<<1) ) );
    	while(SPI_GetFlagStatus(pSPIx, SPI_SR_TXP_FLAG) == FLAG_RESET );

    	uint8_t dsize = ((pSPIx->CFG1 >> SPI_CFG1_DSIZE_POS) & 0x1F) + 1;//EXTRACTING THE DSIZE VALUE.

//    	uint8_t fthvl = ;
    	//2. CHECK THE DFF BIT IN CFG1
    	if(dsize == 16)
    	{

    		// 16bit DFF
    		//1. load the data into the TXDR
    		*((uint16_t*)&(pSPIx->TXDR)) = *((uint16_t*)pTxBuffer);
    		Len--;
    		Len--;//cause we sent out 2 bytes of data
    		pTxBuffer +=2;

    	}else if(dsize == 8)
    	{
    		*((uint8_t*)&(pSPIx->TXDR)) = *pTxBuffer;
    		Len--;
    		pTxBuffer++;
    	}

    }

    //CHECKING FOR TXC FLAG
    while(!(pSPIx->SR & (1<<12)));

    //.CLEARING /DISBALING THE SPE (SPI PERIPH)
    pSPIx->CR1 &= ~(1<< SPI_CR1_SPE_POS);


}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE_POS);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE_POS);
	}
}

void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
{

}

void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{

}

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{

}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}


