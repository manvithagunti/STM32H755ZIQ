/*
 * stm32h7xx_i2c_driver.c
 *
 *  Created on: Oct 21, 2025
 *      Author: manvitha.g
 */



/*
 *Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0;

    // 1. Enable the I2C peripheral clock
    if(pI2CHandle->pI2Cx == I2C1)
        RCC->APB1LENR |= RCC_APB1LENR_I2C1EN;
    else if(pI2CHandle->pI2Cx == I2C2)
        RCC->APB1LENR |= RCC_APB1LENR_I2C2EN;
    else if(pI2CHandle->pI2Cx == I2C3)
        RCC->APB1LENR |= RCC_APB1LENR_I2C3EN;
    else if(pI2CHandle->pI2Cx == I2C4)
        RCC->APB4ENR |= RCC_APB4ENR_I2C4EN;

    // 2. Disable the peripheral before configuration
    pI2CHandle->pI2Cx->CR1 &= ~I2C_CR1_PE;

    // 3. Configure analog filter
    if(pI2CHandle->I2C_Config.I2C_AnalogFilter == I2C_ANALOGFILTER_DISABLE)
        pI2CHandle->pI2Cx->CR1 |= I2C_CR1_ANFOFF;
    else
        pI2CHandle->pI2Cx->CR1 &= ~I2C_CR1_ANFOFF;

    // 4. Configure digital filter (0–15)
    tempreg = pI2CHandle->pI2Cx->CR1;
    tempreg &= ~I2C_CR1_DNF;
    tempreg |= (pI2CHandle->I2C_Config.I2C_DigitalFilter << I2C_CR1_DNF_Pos);
    pI2CHandle->pI2Cx->CR1 = tempreg;

    // 5. Clock stretching configuration
    if(pI2CHandle->I2C_Config.I2C_ClockStretching == I2C_STRETCH_DISABLE)
        pI2CHandle->pI2Cx->CR1 |= I2C_CR1_NOSTRETCH;
    else
        pI2CHandle->pI2Cx->CR1 &= ~I2C_CR1_NOSTRETCH;

    // 6. General call enable
    if(pI2CHandle->I2C_Config.I2C_GeneralCall == I2C_GENCALL_ENABLE)
        pI2CHandle->pI2Cx->CR1 |= I2C_CR1_GCEN;
    else
        pI2CHandle->pI2Cx->CR1 &= ~I2C_CR1_GCEN;

    // 7. Configure addressing mode and own address
    tempreg = 0;
    if(pI2CHandle->I2C_Config.I2C_AddressMode == I2C_ADDRMODE_10BIT)
        tempreg |= I2C_OAR1_OA1MODE; // 10-bit
    tempreg |= (pI2CHandle->I2C_Config.I2C_DeviceAddress << 1);
    tempreg |= I2C_OAR1_OA1EN;
    pI2CHandle->pI2Cx->OAR1 = tempreg;

    // 8. Configure dual addressing if enabled
    if(pI2CHandle->I2C_Config.I2C_DualAddressMode == I2C_DUALADDR_ENABLE)
    {
        tempreg = 0;
        tempreg |= (pI2CHandle->I2C_Config.I2C_SecondAddress << 1);
        tempreg |= I2C_OAR2_OA2EN;
        pI2CHandle->pI2Cx->OAR2 = tempreg;
    }

    // 9. Configure timing register (TIMINGR)
    pI2CHandle->pI2Cx->TIMINGR = pI2CHandle->I2C_Config.I2C_Timing;

    // 10. Enable the peripheral
    pI2CHandle->pI2Cx->CR1 |= I2C_CR1_PE;
}



void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{

	    if(pI2Cx==I2C1){
		I2C1_REG_RESET() ;//macro for this is defined in the MCU specific header file
		}else if(pI2Cx==I2C2){
		I2C2_REG_RESET() ;
		}else if(pI2Cx==I2C3){
		I2C3_REG_RESET() ;
		}else if(pI2Cx==I2C4){
		I2C4_REG_RESET() ;
		}

}


void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
	if(EnorDi ==ENABLE)
	   {
		   if(pI2Cx==I2C1){
			   I2C1_PCLK_EN();
		   }else if(pI2Cx==I2C2){
			   I2C2_PCLK_EN();
		   }else if(pI2Cx==I2C3){
			   I2C3_PCLK_EN();
		   }else if(pI2Cx==I2C4){
			   I2C4_PCLK_EN();
		   }
	   }
	   else{
		   if(pI2Cx==I2C1){
			   I2C1_PCLK_DI();
		   }else if(pI2Cx==I2C2){
			   I2C2_PCLK_DI();
		   }else if(pI2Cx==I2C3){
			   I2C3_PCLK_DI();
		   }else if(pI2Cx==I2C4){
			   I2C4_PCLK_DI();
		   }
	   }
}

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE_POS);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE_POS);
	}
}




void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t *pTxBuffer, uint32_t Len)
{
    //Set peripheral buffer and length
    pI2CHandle->pTxBuffer = pTxBuffer;
    pI2CHandle->TxLen = Len;

    I2C_TypeDef *I2Cx = pI2CHandle->pI2Cx;

    //Configure NBYTES and slave address
    I2Cx->CR2 = 0; // Clear CR2 first
    I2Cx->CR2 |= (SlaveAddr << 1);      // 7-bit slave address
    I2Cx->CR2 |= (Len << I2C_CR2_NBYTES_Pos); // Number of bytes to send
    I2Cx->CR2 &= ~I2C_CR2_RELOAD;       // RELOAD = 0, single transfer
    I2Cx->CR2 |= I2C_CR2_AUTOEND;       // AUTOEND = 1 → automatic STOP
    I2Cx->CR2 |= I2C_CR2_START;         // Generate START condition

    // Send bytes
    while (pI2CHandle->TxLen > 0)
    {
        // Wait until TXIS (Transmit interrupt status) is set
        while (!(I2Cx->ISR & I2C_ISR_TXIS));

        // Write data to TXDR
        I2Cx->TXDR = *(pI2CHandle->pTxBuffer++);
        pI2CHandle->TxLen--;
    }

    // Wait until STOPF is set
    while (!(I2Cx->ISR & I2C_ISR_STOPF));

    // Clear STOPF by writing to ICR
    I2Cx->ICR |= I2C_ICR_STOPCF;
}


void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t *pRxBuffer, uint32_t Len)
{
    // 1. Assign buffer and length to handle
    pI2CHandle->pRxBuffer = pRxBuffer;
    pI2CHandle->RxLen = Len;

    I2C_TypeDef *I2Cx = pI2CHandle->pI2Cx;

    // 2. Configure CR2: Slave address, NBYTES, AUTOEND, START
    I2Cx->CR2 = 0; // Clear previous settings
    I2Cx->CR2 |= (SlaveAddr << 1);                  // 7-bit slave address
    I2Cx->CR2 |= (Len << I2C_CR2_NBYTES_Pos);      // Number of bytes to receive
    I2Cx->CR2 &= ~I2C_CR2_RELOAD;                  // RELOAD = 0 for single transfer
    I2Cx->CR2 |= I2C_CR2_AUTOEND;                  // AUTOEND = 1 automatic STOP
    I2Cx->CR2 |= I2C_CR2_RD_WRN;                   // Set RD_WRN = 1  Receive mode
    I2Cx->CR2 |= I2C_CR2_START;                    // Generate START condition

    // 3. Receive data
    while (pI2CHandle->RxLen > 0)
    {
        // Wait until RXNE (Receive buffer not empty)
        while (!(I2Cx->ISR & I2C_ISR_RXNE));

        // Read data from RXDR
        *(pI2CHandle->pRxBuffer++) = I2Cx->RXDR;
        pI2CHandle->RxLen--;
    }

    // 4. Wait until STOPF is set
    while (!(I2Cx->ISR & I2C_ISR_STOPF));

    // 5. Clear STOPF by writing to ICR
    I2Cx->ICR |= I2C_ICR_STOPCF;
}



uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle,
                             uint8_t SlaveAddr,
                             uint8_t *pTxBuffer,
                             uint32_t Len)
{
    uint32_t tempreg = 0;

    if(pI2CHandle->TxRxState != I2C_READY)
        return pI2CHandle->TxRxState;

    pI2CHandle->pTxBuffer = pTxBuffer;
    pI2CHandle->TxLen = Len;
    pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
    pI2CHandle->DevAddr = SlaveAddr;

    // 1. Set START + slave address + write direction
    tempreg = (SlaveAddr << 1) & 0xFE;  // LSB = 0 for write
    pI2CHandle->pI2Cx->CR2 = 0;
    pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES_Pos);
    pI2CHandle->pI2Cx->CR2 |= tempreg;
    pI2CHandle->pI2Cx->CR2 |= I2C_CR2_START;

    // 2. Enable I2C interrupts
    pI2CHandle->pI2Cx->CR1 |= (I2C_CR1_TXIE | I2C_CR1_TCIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE);

    return I2C_READY;
}



uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle,
                                uint8_t SlaveAddr,
                                uint8_t *pRxBuffer,
                                uint32_t Len)
{
    uint32_t tempreg = 0;

    if(pI2CHandle->TxRxState != I2C_READY)
        return pI2CHandle->TxRxState;

    pI2CHandle->pRxBuffer = pRxBuffer;
    pI2CHandle->RxLen = Len;
    pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
    pI2CHandle->DevAddr = SlaveAddr;

    // 1. Set START + slave address + read direction
    tempreg = ((SlaveAddr << 1) | 0x01); // LSB = 1 for read
    pI2CHandle->pI2Cx->CR2 = 0;
    pI2CHandle->pI2Cx->CR2 |= (Len << I2C_CR2_NBYTES_Pos);
    pI2CHandle->pI2Cx->CR2 |= tempreg;
    pI2CHandle->pI2Cx->CR2 |= I2C_CR2_START;

    // 2. Enable interrupts
    pI2CHandle->pI2Cx->CR1 |= (I2C_CR1_RXIE | I2C_CR1_STOPIE | I2C_CR1_NACKIE);

    return I2C_READY;
}





void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    uint32_t sr1 = pI2CHandle->pI2Cx->ISR;
    uint32_t cr1 = pI2CHandle->pI2Cx->CR1;

    // TX Interrupt
    if((sr1 & I2C_ISR_TXIS) && (cr1 & I2C_CR1_TXIE))
    {
        pI2CHandle->pI2Cx->TXDR = *(pI2CHandle->pTxBuffer);
        pI2CHandle->pTxBuffer++;
        pI2CHandle->TxLen--;

        if(pI2CHandle->TxLen == 0)
        {
            // Disable TX interrupt
            pI2CHandle->pI2Cx->CR1 &= ~I2C_CR1_TXIE;
            // Generate STOP automatically
            pI2CHandle->pI2Cx->CR2 |= I2C_CR2_STOP;
            pI2CHandle->TxRxState = I2C_READY;
            pI2CHandle->I2C_AppEventCallback(pI2CHandle, I2C_EV_TX_COMPLETE);
        }
    }

    // RX Interrupt
    if((sr1 & I2C_ISR_RXNE) && (cr1 & I2C_CR1_RXIE))
    {
        *(pI2CHandle->pRxBuffer) = pI2CHandle->pI2Cx->RXDR;
        pI2CHandle->pRxBuffer++;
        pI2CHandle->RxLen--;

        if(pI2CHandle->RxLen == 0)
        {
            // Disable RX interrupt
            pI2CHandle->pI2Cx->CR1 &= ~I2C_CR1_RXIE;
            // Generate STOP
            pI2CHandle->pI2Cx->CR2 |= I2C_CR2_STOP;
            pI2CHandle->TxRxState = I2C_READY;
            pI2CHandle->I2C_AppEventCallback(pI2CHandle, I2C_EV_RX_COMPLETE);
        }
    }

    //STOP detection
    if((sr1 & I2C_ISR_STOPF) && (cr1 & I2C_CR1_STOPIE))
    {
        // Clear STOP flag
        pI2CHandle->pI2Cx->ICR |= I2C_ICR_STOPCF;
        pI2CHandle->I2C_AppEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    //  NACK detection
    if((sr1 & I2C_ISR_NACKF) && (cr1 & I2C_CR1_NACKIE))
    {
        pI2CHandle->pI2Cx->ICR |= I2C_ICR_NACKCF;
        pI2CHandle->TxRxState = I2C_READY;
        pI2CHandle->I2C_AppEventCallback(pI2CHandle, I2C_EV_NACK);
    }
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t event)
{
    switch(event)
    {
        case I2C_EV_TX_COMPLETE:
            printf("Transmission Complete\r\n");
            break;
        case I2C_EV_RX_COMPLETE:
            printf("Reception Complete\r\n");
            break;
        case I2C_EV_STOP:
            printf("STOP detected\r\n");
            break;
        case I2C_EV_NACK:
            printf("NACK received\r\n");
            break;
    }
}
