/*
 * stm32h7xx_spi_driver.c
 *
 *  Created on: Oct 9, 2025
 *      Author: manvitha.g
 */
#include "stm32h7xx_spi_driver.h"
#include "stm32h7xx.h"




static void spi_txp_interrupt_handle(SPI_Handle_t *pSPIHandle);//static keyword because they are private functions
static void spi_rxp_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

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
	pSPIx->CR2 &= ~(0xff <<0);
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


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi ==ENABLE)
	{
		pSPIx->CFG2 |=(1<<SPI_CFG2_SSOE_POS);
	}else
	{
		pSPIx->CFG2 &= ~(1<<SPI_CFG2_SSOE_POS);
	}
}


void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len)
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
	        //1.wait until RXP is set
	    	//while( !(pSPIx->SR & (1<<1) ) );
	    	while(SPI_GetFlagStatus(pSPIx, SPI_SR_RXP_FLAG) == FLAG_RESET );

	    	uint8_t dsize = ((pSPIx->CFG1 >> SPI_CFG1_DSIZE_POS) & 0x1F) + 1;//EXTRACTING THE DSIZE VALUE.

	        //uint8_t fthvl = ;
	    	//2. CHECK THE DFF BIT IN CFG1
	    	if(dsize == 16)
	    	{

	    		// 16bit DFF
	    		//1. load the data from RXD into the buffer
	    	    *((uint16_t*)pRxBuffer)=*((uint16_t*)&(pSPIx->RXDR));
	    		Len--;
	    		Len--;//cause we sent out 2 bytes of data
	    		pRxBuffer +=2;

	    	}else if(dsize == 8)
	    	{
	    		*pRxBuffer=*((uint8_t*)&(pSPIx->RXDR));
	    		Len--;
	    		pRxBuffer++;
	    	}
	}
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
{
	//this section deals with the configuration of NVIC registers
	    // It deals with the processor side not the peripheral side. (refer to the EXTI block diagram)
	   if(EnorDi == ENABLE)
	   {
		   if(IRQNumber <= 31)
		   {
			   //program ISER0 register
			   *NVIC_ISER0 |= (1<< IRQNumber );

		   }else if(IRQNumber > 31 && IRQNumber < 64)// 32 to 63
		   {
			   //program ISER1 register
			   *NVIC_ISER1 |= (1<< (IRQNumber % 32) );

		   }else if(IRQNumber >= 64 && IRQNumber < 96)//64 to 95
		   {
			   //program ISER2 register
			   *NVIC_ISER2 |= (1<< (IRQNumber % 64) );

		   }else if(IRQNumber >= 96 && IRQNumber < 128)//64 to 95
	  	   {
	  		 //program ISER3 register
			   *NVIC_ISER3 |= (1<< (IRQNumber % 96) );

	  	   }else if(IRQNumber >= 128 && IRQNumber < 160)//64 to 95
	  	   {
	  		 //program ISER4 register
	  		 *NVIC_ISER4 |= (1<< (IRQNumber % 128) );

	  	   }

	   }else
	   {
		   if(IRQNumber <= 31)
		  	   {
		  		   //program ICER0 register
			   *NVIC_ICER0 |= (1<< IRQNumber );

		  	   }else if(IRQNumber > 31 && IRQNumber < 64)// 32 to 63
		  	   {
		  		   //program ICER1 rgister
		  		 *NVIC_ICER1 |= (1<< (IRQNumber % 32) );

		  	   }else if(IRQNumber >= 64 && IRQNumber < 96)//64 to 95
		  	   {
		  		   //program ICER2 register
		  		 *NVIC_ICER2 |= (1<< (IRQNumber % 64) );

		  	   }else if(IRQNumber >= 96 && IRQNumber < 128)//64 to 95
		  	   {
		  		 //program ICER3 register
		  		 *NVIC_ICER3 |= (1<< (IRQNumber % 96) );

		  	   }else if(IRQNumber >= 128 && IRQNumber < 160)//64 to 95
		  	   {
		  		 //program ICER4 register
		  		 *NVIC_ICER4 |= (1<< (IRQNumber % 128) );
		  	   }
	   }
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
	 //1. first lets find out the IPR register
		uint8_t iprx= IRQNumber / 4;
		uint8_t iprx_section = IRQNumber % 4;
		uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED );
		*(NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );

}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
    uint8_t temp1,temp2;
    //first lets check for TXP
    temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_TXP_POS);
    temp2 = pHandle->pSPIx->IER & (1<<SPI_IER_TXPIE_POS);

    if(temp1 &&temp2)
    {
    	//handle TXP
    	//this is a helper function. this will not be exposeed to the user application
    	spi_txp_interrupt_handle(pHandle);
    }

    //check for RXPIE
        temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_RXP_POS);
        temp2 = pHandle->pSPIx->IER & (1<<SPI_IER_RXPIE_POS);

        if(temp1 &&temp2)
            {
            	//handle TXP
            	//this is a helper function. this will not be exposeed to the user application
            	spi_rxp_interrupt_handle(pHandle);
            }


        //CHECK FOR OVR FLAG
        temp1 = pHandle->pSPIx->SR & (1<<SPI_SR_OVR_POS);
        temp2 = pHandle->pSPIx->IER & (1<<SPI_IER_RXPIE_POS);
        //THIS IS FOR THE ERRIE FLAG BIT .
        //BUT THE H755 DOESNT HAVE THIS BIT FLAG

        if(temp1 &&temp2)
        {
             //handle TXP
             //this is a helper function. this will not be exposed to the user application
             spi_ovr_err_interrupt_handle(pHandle);
        }

        //checking for CRC and MODF is not required in this course
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pTxBuffer, uint32_t Len)
{

	//FOR DEBUGGING, INCLUDE LOADING THE TSIZE INTO CR2

	uint8_t state= pSPIHandle->TxState;
	if(state!= SPI_BUSY_IN_TX)
	{


	       //1. Save the TX buffer address and len information in some global variables
	       pSPIHandle->pTxBuffer =pTxBuffer;
	       pSPIHandle->TxLen = Len;

	       //2. Mark the SPI state as busy in transmission so that
	       //no other code can take over the same SPI peripheral until transmission is over
	       pSPIHandle->TxState =  SPI_BUSY_IN_TX;


	       //3. ENABLE the TXPIE control bit to get interrupt whenever TXE flag is set in SR.

	       pSPIHandle->pSPIx->IER |= (1<<SPI_IER_TXPIE_POS);


	       // ENABLE SPE FIRST  AND THEN CSTART


	       //4. data transmission will be handled by the  ISR code(will implement later)


	}

	return state;
}


uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle,uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state= pSPIHandle->RxState;
		if(state!= SPI_BUSY_IN_RX)
		{


		       //1. Save the TX buffer address and len information in some global variables
		       pSPIHandle->pRxBuffer =pRxBuffer;
		       pSPIHandle->RxLen = Len;

		       //2. Mark the SPI state as busy in transmission so that
		       //no other code can take over the same SPI peripheral until transmission is over
		       pSPIHandle->RxState =  SPI_BUSY_IN_RX;


		       //3. ENABLE the TXPIE control bit to get interrupt whenever TXE flag is set in SR.

		       pSPIHandle->pSPIx->IER |= (1<<SPI_IER_RXPIE_POS);
		       //4. data transmission will be handled by the  ISR code(will implement later)

		}

		return state;
}



//SOME HELPER FUNCTION IMPLEMENTATIONS
//user application should not call these helper functions. these are private functions

static void spi_txp_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t dsize = ((pSPIHandle->pSPIx->CFG1 >> SPI_CFG1_DSIZE_POS) & 0x1F) + 1;//EXTRACTING THE DSIZE VALUE.

	//    	uint8_t fthvl = ;
	    	//2. CHECK THE DFF BIT IN CFG1
	    	if(dsize == 16)
	    	{

	    		// 16bit DFF
	    		//1. load the data into the TXDR
	    		*((uint16_t*)&(pSPIHandle->pSPIx->TXDR)) = *((uint16_t*)pSPIHandle->pTxBuffer);
	    		pSPIHandle->TxLen--;
	    		pSPIHandle->TxLen--;//cause we sent out 2 bytes of data
	    		pSPIHandle->pTxBuffer +=2;

	    	}else if(dsize == 8)
	    	{
	    		*((uint8_t*)&(pSPIHandle->pSPIx->TXDR)) = *pSPIHandle->pTxBuffer;
	    		pSPIHandle->TxLen--;
	    		pSPIHandle->pTxBuffer++;
	    	}

	    	if(! pSPIHandle->TxLen)
	    	{
	    		//TXLen is zero so close the spi transmission
	    		//and inform the application that TX is over
	    		//THIS PREVENTS INTERRUPTS FROM SETTING UP OF TXP FLAG
	    		SPI_CloseTransmission(pSPIHandle);
	    		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	    	}
}
static void spi_rxp_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t dsize = ((pSPIHandle->pSPIx->CFG1 >> SPI_CFG1_DSIZE_POS) & 0x1F) + 1;//EXTRACTING THE DSIZE VALUE.

		//    	uint8_t fthvl = ;
		    	//2. CHECK THE DFF BIT IN CFG1
		    	if(dsize == 16)
		    	{

		    		// 16bit DFF
		    		//1. load the data into the TXDR
		    		*((uint16_t*)pSPIHandle->pRxBuffer)= *((uint16_t*)&(pSPIHandle->pSPIx->RXDR));
		    		pSPIHandle->RxLen--;
		    		pSPIHandle->RxLen--;//cause we sent out 2 bytes of data
		    		pSPIHandle->pRxBuffer +=2;

		    	}else if(dsize == 8)
		    	{
		    		*pSPIHandle->pRxBuffer= *((uint8_t*)&(pSPIHandle->pSPIx->RXDR));
		    		pSPIHandle->RxLen--;
		    		pSPIHandle->pRxBuffer++;
		    	}

		    	if(! pSPIHandle->RxLen)
		    	{
		    		//TXLen is zero so close the spi transmission
		    		//and inform the application that TX is over
		    		//THIS PREVENTS INTERRUPTS FROM SETTING UP OF RXP FLAG
		    		SPI_CloseReception(pSPIHandle);
		    		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
		    	}
}
static void spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	//uint8_t temp;
    //clear the OVR flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
	   //CLEARING THE OVR BIT
		pSPIHandle->pSPIx->IER &= ~(1<<6);//CLEARING THE OVRIE BIT TO DISABLE THE OVR INTERRUPT
	}
	//and inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);


}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t AppEv)
{
         //this is a weak implementation adn the application will re-write this implenentation
		 //if application does not implement this function, this call back will be called
}


void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->IER &= ~(1<<SPI_IER_TXPIE_POS);
    pSPIHandle->pTxBuffer = NULL;//NULL IS DEFINED IN stddef.h
	pSPIHandle->TxLen=0;
	pSPIHandle->TxState= SPI_READY;
}
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->IER &= ~(1<<SPI_IER_RXPIE_POS);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen=0;
	pSPIHandle->RxState= SPI_READY;
}


void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	pSPIx->IER &= ~(1<<6);
	//CLEARING THE OVRIE BIT TO DISABLE THE OVR INTERRUPT

}

