/*
 * stm32h7xx_gpio_driver.c
 *
 *  Created on: Sep 26, 2025
 *      Author: manvitha.g
 */

#include "stm32h7xx_gpio_driver.h"

/*
 *Peripheral clock setup
 */
/*********************************************************************************************************
 * @fn        -GPIO PERI CLOCK CONTROL
 *
 * @brief     -THIS FUNCTION ENABLES OR DISBALES PERIPHERAL CLOCK FRO THE GIVEN GPIO PORT
 *
 * @param[in] - BASE ADDRESS OF THE GPIO PERIPHERAL
 * @param[in] -ENABLE ADN DISABLE MACROS
 * @param[in] -
 *
 * @return    -NONE
 *
 * @note      -NONE
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
   if(EnorDi ==ENABLE)
   {
	   if(pGPIOx==GPIOA){
		   GPIOA_PCLK_EN();
	   }else if(pGPIOx==GPIOB){
		   GPIOB_PCLK_EN();
	   }else if(pGPIOx==GPIOC){
		   GPIOC_PCLK_EN();
	   }else if(pGPIOx==GPIOD){
		   GPIOD_PCLK_EN();
	   }else if(pGPIOx==GPIOE){
		   GPIOE_PCLK_EN();
	   }else if(pGPIOx==GPIOF){
		   GPIOF_PCLK_EN();
	   }else if(pGPIOx==GPIOG){
		   GPIOG_PCLK_EN();
	   }else if(pGPIOx==GPIOH){
		   GPIOH_PCLK_EN();
	   }else if(pGPIOx==GPIOI){
		   GPIOI_PCLK_EN();
	   }else if(pGPIOx==GPIOJ){
		   GPIOJ_PCLK_EN();
	   }else if(pGPIOx==GPIOK){
		   GPIOK_PCLK_EN();
	   }
   }
   else{
	        if(pGPIOx==GPIOA){
	   		   GPIOA_PCLK_DI();
	   	   }else if(pGPIOx==GPIOB){
	   		   GPIOB_PCLK_DI();
	   	   }else if(pGPIOx==GPIOC){
	   		   GPIOC_PCLK_DI();
	   	   }else if(pGPIOx==GPIOD){
	   		   GPIOD_PCLK_DI();
	   	   }else if(pGPIOx==GPIOE){
	   		   GPIOE_PCLK_DI();
	   	   }else if(pGPIOx==GPIOF){
	   		   GPIOF_PCLK_DI();
	   	   }else if(pGPIOx==GPIOG){
	   		   GPIOG_PCLK_DI();
	   	   }else if(pGPIOx==GPIOH){
	   		   GPIOH_PCLK_DI();
	   	   }else if(pGPIOx==GPIOI){
	   		   GPIOI_PCLK_DI();
	   	   }else if(pGPIOx==GPIOJ){
	   		   GPIOJ_PCLK_DI();
	   	   }else if(pGPIOx==GPIOK){
	   		   GPIOK_PCLK_DI();
	   	   }
   }

}


/*
 *data init and de init
 */
/*********************************************************************************************************
 * @fn        -
 *
 * @brief     -THIS FUNCTION ENABLES OR DISBALES PERIPHERAL CLOCK FRO THE GIVEN GPIO PORT
 *
 * @param[in] - BASE ADDRESS OF THE GPIO PERIPHERAL
 * @param[in] -ENAVLE ADN DISABLE MACROS
 * @param[in] -
 *
 * @return    -NONE
 *
 * @note      -NONE
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{


    uint32_t temp=0;//temp register

    //enable the peripheral clock
    GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure the mode of the gpio pin

	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		//the non interrupt mode
		temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber ) );
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );//clearing
		pGPIOHandle->pGPIOx->MODER |= temp;//setting

	}else
	{
        //INTERRUPT MODE
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << 2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );//clearing

		if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_FT)
		{
			//1. CONFIGURE THE FTSR
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//CLEAR THE CORRESPONDING RTSR BIT
			EXTI->RTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT)
		{
			//1. CONFIGURE THE RTSR
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//CLEAR THE CORRESPONDING RTSR BIT
			EXTI->FTSR1 &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RFT)
		{
			//1. CONFIGURE BOTH FTSR AND RTSR
			EXTI->RTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//CLEAR THE CORRESPONDING RTSR BIT
			EXTI->FTSR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. CONFIGURE THE GPIO PORT SLECTION IN SYSCFG_EXTICR
		uint8_t temp1 =pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		uint8_t temp2 =pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4 ;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle ->pGPIOx);//THIS MACRO "GPIO_BASEADDR_TO_CODE" will be implemnted later
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		//3. ENABLE INTERRUPT DELIVERY USING INTERRUPT MASK REGISTER(IMR)
		EXTI->C1IMR1 |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);



	}
	temp=0;

	//2. configure the speed
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );//clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;//setting

	temp=0;

	//3. configure the pupd settings
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );//clearing
	pGPIOHandle->pGPIOx->PUPDR |=temp;	//setting

	temp=0;

	//4. configure the optype
	temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) ;
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );//clearing
	pGPIOHandle->pGPIOx->OTYPER |=temp;//setting

    temp=0;

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_ALTFN)
	{
		//configure the alternate function registers
		uint8_t temp1, temp2;

		temp1= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2= pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~( 0xF << (4 * temp2) );//clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2) );

	}

}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
           if(pGPIOx==GPIOA){
			   GPIOA_REG_RESET();//macro for this is defined in the MCU specific header file
		   }else if(pGPIOx==GPIOB){
			   GPIOB_REG_RESET();
		   }else if(pGPIOx==GPIOC){
			   GPIOC_REG_RESET();
		   }else if(pGPIOx==GPIOD){
			   GPIOD_REG_RESET();
		   }else if(pGPIOx==GPIOE){
			   GPIOE_REG_RESET();
		   }else if(pGPIOx==GPIOF){
			   GPIOF_REG_RESET();
		   }else if(pGPIOx==GPIOG){
			   GPIOG_REG_RESET();
		   }else if(pGPIOx==GPIOH){
			   GPIOH_REG_RESET();
		   }else if(pGPIOx==GPIOI){
			   GPIOI_REG_RESET();
		   }else if(pGPIOx==GPIOJ){
			   GPIOJ_REG_RESET();
		   }else if(pGPIOx==GPIOK){
			   GPIOK_REG_RESET();
		   }
}

/*
 *data read and write
 */
/*********************************************************************************************************
 * @fn        -GPIO- READ FROM INPUT PIN
 *
 * @brief     -THIS FUNCTION ENABLES OR DISBALES PERIPHERAL CLOCK FRO THE GIVEN GPIO PORT
 *
 * @param[in] - BASE ADDRESS OF THE GPIO PERIPHERAL
 * @param[in] -ENAVLE ADN DISABLE MACROS
 * @param[in] -
 *
 * @return    -NONE
 *
 * @note      -NONE
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
uint8_t value;
value=(uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001) ;
return value;
}

/*********************************************************************************************************
 * @fn        -GPIO READ FROM INPUT PORT
 *
 * @brief     -THIS FUNCTION ENABLES OR DISBALES PERIPHERAL CLOCK FRO THE GIVEN GPIO PORT
 *
 * @param[in] - BASE ADDRESS OF THE GPIO PERIPHERAL
 * @param[in] -ENAVLE ADN DISABLE MACROS
 * @param[in] -
 *
 * @return    -NONE
 *
 * @note      -NONE
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
uint16_t value;
value=(uint16_t )pGPIOx->IDR;
return value;
}

/*********************************************************************************************************
 * @fn        -GPIO WRITE TO OUTPUT PIN
 *
 * @brief     -THIS FUNCTION ENABLES OR DISBALES PERIPHERAL CLOCK FRO THE GIVEN GPIO PORT
 *
 * @param[in] - BASE ADDRESS OF THE GPIO PERIPHERAL
 * @param[in] -ENAVLE ADN DISABLE MACROS
 * @param[in] -
 *
 * @return    -NONE
 *
 * @note      -NONE
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
   if(Value == GPIO_PIN_SET)
   {
	   //write 1 to the output data register at the bit field corresponding pin number
	   pGPIOx->ODR |= (1 << PinNumber);
   }else
   {
	   //write 0
	   pGPIOx->ODR &= ~(1 << PinNumber);
   }
}

/*********************************************************************************************************
 * @fn        -GPIO WRITE TO OUTPUT PORT
 *
 * @brief     -THIS FUNCTION ENABLES OR DISBALES PERIPHERAL CLOCK FRO THE GIVEN GPIO PORT
 *
 * @param[in] - BASE ADDRESS OF THE GPIO PERIPHERAL
 * @param[in] -ENAVLE ADN DISABLE MACROS
 * @param[in] -
 *
 * @return    -NONE
 *
 * @note      -NONE
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value )
{
    pGPIOx->ODR =Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
    pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 *IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi)
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

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority)
{
    //1. first lets find out the IPR register
	uint8_t iprx= IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED );
	*(NVIC_PR_BASE_ADDR + iprx ) |= ( IRQPriority << shift_amount );

}




void GPIO_IRQHandling(uint8_t PinNumber)
{
    //clear the exti pr register corresponding to the pin number
	if(EXTI->C1PR1 & (1<<PinNumber))
	{
		//clear
		EXTI->C1PR1 |= (1<<PinNumber);
	}
}
