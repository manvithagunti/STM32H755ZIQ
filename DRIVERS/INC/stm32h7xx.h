/*
 * stm32h7xx.h
 *
 *  Created on: Sep 24, 2025
 *      Author: manvitha.g
 */

#ifndef INC_STM32H7XX_H_
#define INC_STM32H7XX_H_

#include<stdint.h>

#define __vo volatile

/*
 * *****************PROCESSOR SPECIFIC DETAILS*******************************/
/*
 * ARM CORTEX M7 PROCESSOR NVIC ISERx REGIDTER ADDRESSES
 */
#define NVIC_ISER0            (  (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1            (  (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2            (  (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3            (  (__vo uint32_t*)0xE000E10C )
#define NVIC_ISER4            (  (__vo uint32_t*)0xE000E110 )

/*
 * ARM CORTEX M7 PROCESSOR NVIC ICERx REGISTER ADDRESSES
 */
#define NVIC_ICER0            (  (__vo uint32_t*)0XE000E180 )
#define NVIC_ICER1            (  (__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2            (  (__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3            (  (__vo uint32_t*)0xE000E18C )
#define NVIC_ICER4            (  (__vo uint32_t*)0xE000E190 )


/*
 * ARM CORTEX M7 PROCESSOR PRIORITY  REGIDTER ADDRESS CALCULATION
 */
#define NVIC_PR_BASE_ADDR          (  (__vo uint32_t*)0xE000E400 )

#define NO_PR_BITS_IMPLEMENTED       4  //this value is 3 in other microcontrollers like TI

/*
base addresses of sram and flash memories
*/

#define FLASH_BASEADDR               0x08000000U   //present in the flash memory organization
#define SRAM1_BASEADDR               0x30000000U   //taken from memory map and default evice memory attributes table
#define SRAM2_BASEADDR               0x30020000U   //can be found by adding the size of SRAM1 size to it's base address
#define ROM_BASEADDR                 0x1FF00000U  //base address of system memory
#define SRAM                         SRAM1_BASEADDR

/* AHBx and APBx bus peripheral base addresses */

#define PERIPH_BASE                  0x40000000U //check from the register boundary addresses
#define APB1PERIPH_BASE              PERIPH_BASE //APB1 starts at PERIPHERAL BASE ADDRRESS itself
#define APB2PERIPH_BASE              0x40010000U //APB is the advanced peripheral bus which is used for low speed communication
#define APB3PERIPH_BASE              0x50000000U
#define APB4PERIPH_BASE              0x58000000U
#define AHB1PERIPH_BASE              0x40020000U //AHB is the advanced high performance bus
#define AHB2PERIPH_BASE              0x48020000U //AHB connects to GPIOs
#define AHB3PERIPH_BASE              0x51000000U
#define AHB4PERIPH_BASE              0x58020000U


/* base addresses of all peripherals hanging on the AHB4 bus */

#define GPIOA_BASEADDR               0x58020000U
#define GPIOB_BASEADDR               0x58020400U
#define GPIOC_BASEADDR               0x58020800U
#define GPIOD_BASEADDR               0x58020C00U
#define GPIOE_BASEADDR               0x58021000U
#define GPIOF_BASEADDR               0x58021400U
#define GPIOG_BASEADDR               0x58021800U
#define GPIOH_BASEADDR               0x58021C00U
#define GPIOI_BASEADDR               0x58022000U
#define GPIOJ_BASEADDR               0x58022400U
#define GPIOK_BASEADDR               0x58022800U


#define RCC_BASEADDR                 0x58024400U

/* base addresses of peripherals hanging on APB1 bus */

#define I2C1_BASEADDR                0x40005400U
#define I2C2_BASEADDR                0x40005800U
#define I2C3_BASEADDR                0x40005C00U

#define SPI2_BASEADDR                0x40003800U
#define SPI3_BASEADDR                0x40003C00U

#define USART2_BASEADDR              0x40004400U
#define USART3_BASEADDR              0x40004800U

#define UART4_BASEADDR               0x40004C00U
#define UART5_BASEADDR               0x40005000U
#define UART7_BASEADDR               0x40007800U
#define UART8_BASEADDR               0x40007C00U


/* base addresses of peripherals hanging on APB2 bus */

#define USART6_BASEADDR              0x40011400U
#define USART1_BASEADDR              0x40011000U

#define SPI1_BASEADDR                0x40013000U
#define SPI4_BASEADDR                0x40013400U
#define SPI5_BASEADDR                0x40015000U


/* base addresses of peripherals hanging on APB4 bus */

#define SPI6_BASEADDR                0x58001400U
#define I2C4_BASEADDR                0x58001C00U
#define EXTI_BASEADDR                0x58000000U
#define SYSCFG_BASEADDR              0x58000400U



/***************** PERIPHERAL REGISTER DEFINITION ADDRESS *******************/

typedef struct
{
   __vo uint32_t MODER;       //GPIO port mode register
   __vo uint32_t OTYPER;      //GPIO port output type register
   __vo uint32_t OSPEEDR;      //GPIO port output speed register
   __vo uint32_t PUPDR;        //GPIO port pull-up/pull-down register
   __vo uint32_t IDR;          //GPIO port input data register
   __vo uint32_t ODR;          //GPIO port output data register
   __vo uint32_t BSRR;          //GPIO port bit set/reset register
   __vo uint32_t LCKR;          //GPIO port configuration lock register
   __vo uint32_t AFR[2];        //AFR[0]: GPIO alternate function low register;  AFR[1]: GPIO alternate function high register

}GPIO_RegDef_t;


/*
 *  peripheral definitions (peripheral base addresses typecasted to xxx_RegDef_t)
 */
#define GPIOA         ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB         ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC         ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD         ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE         ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF         ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG         ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH         ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI         ((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ         ((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK         ((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define RCC           ((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI          ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG        ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1          ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2          ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3          ((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4          ((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5          ((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6          ((SPI_RegDef_t*)SPI6_BASEADDR)



/*
 *  clock enable macros for GPIOx peripherals
  */
#define GPIOA_PCLK_EN()      (RCC ->C1_AHB4ENR |= (1<<0) )//changed it from RCC ->AHB4ENR  to RCC ->C1_AHB4ENR as C1 corresponds to M7 on 3.10.2025
#define GPIOB_PCLK_EN()      (RCC ->C1_AHB4ENR |= (1<<1) )
#define GPIOC_PCLK_EN()      (RCC ->C1_AHB4ENR |= (1<<2) )
#define GPIOD_PCLK_EN()      (RCC ->C1_AHB4ENR |= (1<<3) )
#define GPIOE_PCLK_EN()      (RCC ->C1_AHB4ENR |= (1<<4) )
#define GPIOF_PCLK_EN()      (RCC ->C1_AHB4ENR |= (1<<5) )
#define GPIOG_PCLK_EN()      (RCC ->C1_AHB4ENR |= (1<<6) )
#define GPIOH_PCLK_EN()      (RCC ->C1_AHB4ENR |= (1<<7) )
#define GPIOI_PCLK_EN()      (RCC ->C1_AHB4ENR |= (1<<8) )
#define GPIOJ_PCLK_EN()      (RCC ->C1_AHB4ENR |= (1<<9) )
#define GPIOK_PCLK_EN()      (RCC ->C1_AHB4ENR |= (1<<10))



/* clock enable macros for I2Cx peripherals */
#define I2C1_PCLK_EN()      (RCC ->C1_APB1LENR |= (1<<21))
#define I2C2_PCLK_EN()      (RCC ->C1_APB1LENR |= (1<<22))
#define I2C3_PCLK_EN()      (RCC ->C1_APB1LENR |= (1<<23))
#define I2C4_PCLK_EN()      (RCC ->C1_APB4ENR  |= (1<<7))


/* clock enable macros for SPIx peripherals */
#define SPI1_PCLK_EN()      (RCC ->C1_APB2ENR  |= (1<<12))
#define SPI2_PCLK_EN()      (RCC ->C1_APB1LENR  |= (1<<14))
#define SPI3_PCLK_EN()      (RCC ->C1_APB1LENR  |= (1<<15))
#define SPI4_PCLK_EN()      (RCC ->C1_APB2ENR  |= (1<<13))
#define SPI5_PCLK_EN()      (RCC ->C1_APB2ENR  |= (1<<20))
#define SPI6_PCLK_EN()      (RCC ->C1_APB4ENR  |= (1<<5))


/* clock enable macros for USARTx peripherals */
#define USART1_PCLK_EN()      (RCC ->C1_APB2ENR  |= (1<<4))
#define USART2_PCLK_EN()      (RCC ->C1_APB1ENR  |= (1<<17))
#define USART3_PCLK_EN()      (RCC ->C1_APB1ENR  |= (1<<18))
#define USART6_PCLK_EN()      (RCC ->C1_APB2ENR  |= (1<<5))



/* clock enable macros for SYSCFG peripherals */
#define SYSCFG_PCLK_EN()      (RCC ->C1_APB4ENR  |= (1<<1))



/* clock disable macros for GPIOx peripherals */
#define GPIOA_PCLK_DI()      (RCC ->C1_AHB4ENR &= ~(1<<0) )
#define GPIOB_PCLK_DI()      (RCC ->C1_AHB4ENR &= ~(1<<1) )
#define GPIOC_PCLK_DI()      (RCC ->C1_AHB4ENR &= ~(1<<2) )
#define GPIOD_PCLK_DI()      (RCC ->C1_AHB4ENR &= ~(1<<3) )
#define GPIOE_PCLK_DI()      (RCC ->C1_AHB4ENR &= ~(1<<4) )
#define GPIOF_PCLK_DI()      (RCC ->C1_AHB4ENR &= ~(1<<5) )
#define GPIOG_PCLK_DI()      (RCC ->C1_AHB4ENR &= ~(1<<6) )
#define GPIOH_PCLK_DI()      (RCC ->C1_AHB4ENR &= ~(1<<7) )
#define GPIOI_PCLK_DI()      (RCC ->C1_AHB4ENR &= ~(1<<8) )
#define GPIOJ_PCLK_DI()      (RCC ->C1_AHB4ENR &= ~(1<<9) )
#define GPIOK_PCLK_DI()      (RCC ->C1_AHB4ENR &= ~(1<<10))


/* clock disable macros for I2Cx peripherals */
#define I2C1_PCLK_DI()      (RCC ->C1_APB1LENR &= ~(1<<21))
#define I2C2_PCLK_DI()      (RCC ->C1_APB1LENR &= ~(1<<22))
#define I2C3_PCLK_DI()      (RCC ->C1_APB1LENR &= ~(1<<23))
#define I2C4_PCLK_DI()      (RCC ->C1_APB4ENR  &= ~(1<<7))



/* clock disable macros for SPIx peripherals */
#define SPI1_PCLK_DI()      (RCC ->C1_APB2ENR  &= ~(1<<12))
#define SPI2_PCLK_DI()      (RCC ->C1_APB1LENR &= ~(1<<14))
#define SPI3_PCLK_DI()      (RCC ->C1_APB1LENR &= ~(1<<15))
#define SPI4_PCLK_DI()      (RCC ->C1_APB2ENR  &= ~(1<<13))
#define SPI5_PCLK_DI()      (RCC ->C1_APB2ENR  &= ~(1<<20))
#define SPI6_PCLK_DI()      (RCC ->C1_APB4ENR  &= ~(1<<5))


/* clock disable macros for USARTx peripherals */
#define USART1_PCLK_DI()      (RCC ->C1_APB2ENR  &= ~(1<<4))
#define USART2_PCLK_DI()      (RCC ->C1_APB1ENR  &= ~(1<<17))
#define USART3_PCLK_DI()      (RCC ->C1_APB1ENR  &= ~(1<<18))
#define USART6_PCLK_DI()      (RCC ->C1_APB2ENR  &= ~(1<<5))


/* clock disable macros for SYSCFG peripherals */
#define SYSCFG_PCLK_DI()      (RCC ->C1_APB2ENR  &= ~(1<<1))



/*
 *macros to reset SPI peripherals
 */
#define SPI1_REG_RESET()        do{ (RCC ->APB2RSTR  |= (1<<12)); (RCC ->APB2RSTR  &= ~(1<<12)); } while(0)
#define SPI2_REG_RESET()        do{ (RCC ->APB1LRSTR |= (1<<14)); (RCC ->APB1LRSTR &= ~(1<<14)); } while(0)
#define SPI3_REG_RESET()        do{ (RCC ->APB1LRSTR |= (1<<15)); (RCC ->APB1LRSTR &= ~(1<<15)); } while(0)
#define SPI4_REG_RESET()        do{ (RCC ->APB2RSTR  |= (1<<13)); (RCC ->APB2RSTR  &= ~(1<<13)); } while(0)
#define SPI5_REG_RESET()        do{ (RCC ->APB2RSTR  |= (1<<20)); (RCC ->APB2RSTR  &= ~(1<<20)); } while(0)
#define SPI6_REG_RESET()        do{ (RCC ->APB4RSTR  |= (1<<5 )); (RCC ->APB4RSTR  &= ~(1<<5 )); } while(0)


/*
 *macros to reset GPIO peripherals
 */

#define GPIOA_REG_RESET()        do{ (RCC ->AHB4RSTR  |= (1<<0)); (RCC ->AHB4RSTR  &= ~(1<<0)); } while(0)
#define GPIOB_REG_RESET()        do{ (RCC ->AHB4RSTR  |= (1<<1)); (RCC ->AHB4RSTR  &= ~(1<<1)); } while(0)
#define GPIOC_REG_RESET()        do{ (RCC ->AHB4RSTR  |= (1<<2)); (RCC ->AHB4RSTR  &= ~(1<<2)); } while(0)
#define GPIOD_REG_RESET()        do{ (RCC ->AHB4RSTR  |= (1<<3)); (RCC ->AHB4RSTR  &= ~(1<<3)); } while(0)
#define GPIOE_REG_RESET()        do{ (RCC ->AHB4RSTR  |= (1<<4)); (RCC ->AHB4RSTR  &= ~(1<<4)); } while(0)
#define GPIOF_REG_RESET()        do{ (RCC ->AHB4RSTR  |= (1<<5)); (RCC ->AHB4RSTR  &= ~(1<<5)); } while(0)
#define GPIOG_REG_RESET()        do{ (RCC ->AHB4RSTR  |= (1<<6)); (RCC ->AHB4RSTR  &= ~(1<<6)); } while(0)
#define GPIOH_REG_RESET()        do{ (RCC ->AHB4RSTR  |= (1<<7)); (RCC ->AHB4RSTR  &= ~(1<<7)); } while(0)
#define GPIOI_REG_RESET()        do{ (RCC ->AHB4RSTR  |= (1<<8)); (RCC ->AHB4RSTR  &= ~(1<<8)); } while(0)
#define GPIOJ_REG_RESET()        do{ (RCC ->AHB4RSTR  |= (1<<9)); (RCC ->AHB4RSTR  &= ~(1<<9)); } while(0)
#define GPIOK_REG_RESET()        do{ (RCC ->AHB4RSTR  |= (1<<10));(RCC ->AHB4RSTR  &= ~(1<<10));} while(0)

/*
 * returns port code for given GPIOx base address
 */

//using ternary operator
#define GPIO_BASEADDR_TO_CODE(x)   ( (x==GPIOA)?0:\
		                             (x==GPIOB)?1:\
		                             (x==GPIOC)?2:\
		                             (x==GPIOD)?3:\
		                             (x==GPIOE)?4:\
		                             (x==GPIOF)?5:\
		                             (x==GPIOG)?6:\
		                             (x==GPIOH)?7:\
		                             (x==GPIOI)?8:\
		                             (x==GPIOJ)?9:\
		                             (x==GPIOK)?10:0 )


// we can also use function for the above macro


/*
 * IRQ NUMBER - THESE MACROS WILL CHANGE WITH MCU
 */
#define IRQ_NO_EXTI0      6
#define IRQ_NO_EXTI1      7
#define IRQ_NO_EXTI2      8
#define IRQ_NO_EXTI3      9
#define IRQ_NO_EXTI4      10
#define IRQ_NO_EXTI9_5    23
#define IRQ_NO_EXTI15_10  40


/*
 * MACROS FOR ALL THE POSSSIBLE PRIORITY LEVELS
 */
#define NVIC_IRQ_PRI0              0
#define NVIC_IRQ_PRI15             15





/*
 * some generic macros
 */
#define ENABLE         1
#define DISABLE        0
#define SET            ENABLE
#define RESET          DISABLE
#define GPIO_PIN_SET   SET
#define GPIO_PIN_RESET  RESET
#define FLAG_RESET      RESET
#define FLAG_SET        SET








typedef struct
{
	__vo uint32_t CR;       //GPIO port mode register
	__vo uint32_t HSICFGR;       //GPIO port mode register
	__vo uint32_t CRRCR;       //GPIO port mode register
	__vo uint32_t CSICFGR;       //GPIO port mode register
	__vo uint32_t CFGR;       //GPIO port mode register
	uint32_t RESERVED0;       //GPIO port mode register
	__vo uint32_t D1CFGR;       //GPIO port mode register
	__vo uint32_t D2CFGR;       //GPIO port mode register
	__vo uint32_t D3CFGR;       //GPIO port mode register
	uint32_t RESERVED1;
	__vo uint32_t PLLCKSELR;       //GPIO port mode register
	__vo uint32_t PLLCFGR;       //GPIO port mode register
	__vo uint32_t PLL1DIVR;       //GPIO port mode register
	__vo uint32_t PLL1FRACR;       //GPIO port mode register
	__vo uint32_t PLL2DIVR;       //GPIO port mode register
	__vo uint32_t PLL2FRACR;       //GPIO port mode register
	__vo uint32_t PLL3DIVR;       //GPIO port mode register
	__vo uint32_t PLL3FRACR;       //GPIO port mode register
	uint32_t RESERVED2;
	__vo uint32_t D1CCIPR;       //GPIO port mode register
	__vo uint32_t D2CCIP1R;       //GPIO port mode register
	__vo uint32_t D2CCIP2R;       //GPIO port mode register
	__vo uint32_t D3CCIPR;       //GPIO port mode register
	uint32_t RESERVED3;
	__vo uint32_t CIER;       //GPIO port mode register
	__vo uint32_t CIFR;       //GPIO port mode register
	__vo uint32_t CICR;       //GPIO port mode register
	uint32_t RESERVED4;
	__vo uint32_t BDCR;       //GPIO port mode register
	__vo uint32_t CSR;       //GPIO port mode register
	uint32_t RESERVED5;
	__vo uint32_t AHB3RSTR;       //GPIO port mode register
	__vo uint32_t AHB1RSTR;       //GPIO port mode register
	__vo uint32_t AHB2RSTR;       //GPIO port mode register
	__vo uint32_t AHB4RSTR;       //GPIO port mode register
	__vo uint32_t APB3RSTR;       //GPIO port mode register
	__vo uint32_t APB1LRSTR;       //GPIO port mode register
	__vo uint32_t APB1HRSTR;       //GPIO port mode register
	__vo uint32_t APB2RSTR;       //GPIO port mode register
	__vo uint32_t APB4RSTR;       //GPIO port mode register
	__vo uint32_t GCR;       //GPIO port mode register
	uint32_t RESERVED6;
	__vo uint32_t D3AMR;       //GPIO port mode register
	uint32_t RESERVED7[9];
	__vo uint32_t RSR;       //GPIO port mode register
	__vo uint32_t AHB3ENR;       //GPIO port mode register
	__vo uint32_t AHB1ENR;       //GPIO port mode register
	__vo uint32_t AHB2ENR;       //GPIO port mode register
	__vo uint32_t AHB4ENR;       //GPIO port mode register
	__vo uint32_t APB3ENR;       //GPIO port mode register
	__vo uint32_t APB1LENR;       //GPIO port mode register
	__vo uint32_t APB1HENR;       //GPIO port mode register
	__vo uint32_t APB2ENR;       //GPIO port mode register
	__vo uint32_t APB4ENR;       //GPIO port mode register
	uint32_t RESERVED8;
	__vo uint32_t AHB3LPENR;       //GPIO port mode register
	__vo uint32_t AHB1LPENR;       //GPIO port mode register
	__vo uint32_t AHB2LPENR;       //GPIO port mode register
	__vo uint32_t AHB4LPENR;       //GPIO port mode register
	__vo uint32_t APB3LPENR;       //GPIO port mode register
	__vo uint32_t APB1LLPENR;       //GPIO port mode register
	__vo uint32_t APB1HLPENR;       //GPIO port mode register
	__vo uint32_t APB2LPENR;       //GPIO port mode register
	__vo uint32_t APB4LPENR;       //GPIO port mode register
	uint32_t RESERVED9[4];
	__vo uint32_t C1_RSR;       //GPIO port mode register
	__vo uint32_t C1_AHB3ENR;       //GPIO port mode register
	__vo uint32_t C1_AHB1ENR;       //GPIO port mode register
	__vo uint32_t C1_AHB2ENR;       //GPIO port mode register
	__vo uint32_t C1_AHB4ENR;       //GPIO port mode register
	__vo uint32_t C1_APB3ENR;       //GPIO port mode register
	__vo uint32_t C1_APB1LENR;       //GPIO port mode register
	__vo uint32_t C1_APB1HENR;       //GPIO port mode register
	__vo uint32_t C1_APB2ENR;       //GPIO port mode register
	__vo uint32_t C1_APB4ENR;       //GPIO port mode register
	uint32_t RESERVED10;
	__vo uint32_t C1_AHB3LPENR;       //GPIO port mode register
	__vo uint32_t C1_AHB1LPENR;       //GPIO port mode register
	__vo uint32_t C1_AHB2LPENR;       //GPIO port mode register
	__vo uint32_t C1_AHB4LPENR;       //GPIO port mode register
	__vo uint32_t C1_APB3LPENR;       //GPIO port mode register
	__vo uint32_t C1_APB1LLPENR;       //GPIO port mode register
	__vo uint32_t C1_APB1HLPENR;       //GPIO port mode register
	__vo uint32_t C1_APB2LPENR;       //GPIO port mode register
	__vo uint32_t C1_APB4LPENR;       //GPIO port mode register
	uint32_t RESERVED11[4];
	__vo uint32_t C2_RSR;       //GPIO port mode register
	__vo uint32_t C2_AHB3ENR;       //GPIO port mode register
	__vo uint32_t C2_AHB1ENR;       //GPIO port mode register
	__vo uint32_t C2_AHB2ENR;       //GPIO port mode register
	__vo uint32_t C2_AHB4ENR;       //GPIO port mode register
	__vo uint32_t C2_APB3ENR;       //GPIO port mode register
	__vo uint32_t C2_APB1LENR;       //GPIO port mode register
	__vo uint32_t C2_APB1HENR;       //GPIO port mode register
	__vo uint32_t C2_APB2ENR;       //GPIO port mode register
	__vo uint32_t C2_APB4ENR;       //GPIO port mode register
	uint32_t RESERVED12;
	__vo uint32_t C2_AHB3LPENR;       //GPIO port mode register
	__vo uint32_t C2_AHB1LPENR;       //GPIO port mode register
	__vo uint32_t C2_AHB2LPENR;       //GPIO port mode register
	__vo uint32_t C2_AHB4LPENR;       //GPIO port mode register
	__vo uint32_t C2_APB3LPENR;       //GPIO port mode register
	__vo uint32_t C2_APB1LLPENR;       //GPIO port mode register
	__vo uint32_t C2_APB1HLPENR;       //GPIO port mode register
	__vo uint32_t C2_APB2LPENR;       //GPIO port mode register
	__vo uint32_t C2_APB4LPENR;       //GPIO port mode register
	uint32_t RESERVED13[8];
}RCC_RegDef_t;



/*
 * PERIPHERAL REGISTER DEFINITION STRUCTURE FOR EXTI
 */
typedef struct
{
   __vo uint32_t RTSR1;       //GPIO port mode register
   __vo uint32_t FTSR1;      //GPIO port output type register
   __vo uint32_t SWIER1;      //GPIO port output speed register
   __vo uint32_t D3PMR1;        //GPIO port pull-up/pull-down register
   __vo uint32_t D3PCR1L;          //GPIO port input data register
   __vo uint32_t D3PCR1H;          //GPIO port output data register
   uint32_t RESERVED0[2];
   __vo uint32_t RTSR2;          //GPIO port bit set/reset register
   __vo uint32_t FTSR2;          //GPIO port configuration lock register
   __vo uint32_t SWIER2;   //AFR[0]: GPIO alternate function low register;  AFR[1]: GPIO alternate function high register
   __vo uint32_t D3PMR2;
   __vo uint32_t D3PCR2L;
   __vo uint32_t D3PCR2H;
   uint32_t RESERVED1[2];
   __vo uint32_t RTSR3;
   __vo uint32_t FTSR3;
   __vo uint32_t SWIER3;
   __vo uint32_t D3PMR3;
   __vo uint32_t D3PCR3L;
   __vo uint32_t D3PCR3H;
   uint32_t RESERVED2[10];
   __vo uint32_t C1IMR1;
   __vo uint32_t C1EMR1;
   __vo uint32_t C1PR1;
   uint32_t RESERVED3;
   __vo uint32_t C1IMR2;
   __vo uint32_t C1EMR2;
   __vo uint32_t C1PR2;
   uint32_t RESERVED4;
   __vo uint32_t C1IMR3;
   __vo uint32_t C1EMR3;
   __vo uint32_t C1PR3;
   uint32_t RESERVED5[5];
   __vo uint32_t C2IMR1;
   __vo uint32_t C2EMR1;
   __vo uint32_t C2PR1;
   uint32_t RESERVED6;
   __vo uint32_t C2IMR2;
   __vo uint32_t C2EMR2;
   __vo uint32_t C2PR2;
   uint32_t RESERVED7;
   __vo uint32_t C2IMR3;
   __vo uint32_t C2EMR3;
   __vo uint32_t C2PR3;

 }EXTI_RegDef_t;

 /*
  * PERIPHERAL REGISTER DEFINITION STRUCTURE FOR SYSCFG
  */
 typedef struct
 {
	 uint32_t RESERVED0;
	__vo uint32_t PMCR;       //GPIO port mode register
    __vo uint32_t EXTICR[4];      //GPIO port output type register
    __vo uint32_t CFGR[2];          //GPIO port output data register
    __vo uint32_t CCSR;          //GPIO port bit set/reset register
    __vo uint32_t CCVR;          //GPIO port configuration lock register
    __vo uint32_t CCCR;        //AFR[0]: GPIO alternate function low register;  AFR[1]: GPIO alternate function high register
    __vo uint32_t PWRCR;
    uint32_t RESERVED1[52];
    __vo uint32_t SR0 ;
    uint32_t RESERVED2[8];
    __vo uint32_t PKGR;
    uint32_t RESERVED3[118];
    __vo uint32_t UR0;
    __vo uint32_t UR1;
    __vo uint32_t UR2;
    __vo uint32_t UR3;
    __vo uint32_t UR4;
    __vo uint32_t UR5;
    __vo uint32_t UR6;
    __vo uint32_t UR7;
    __vo uint32_t UR8;
    __vo uint32_t UR9;
    __vo uint32_t UR10;
    __vo uint32_t UR11;
    __vo uint32_t UR12;
    __vo uint32_t UR13;
    __vo uint32_t UR14;
    __vo uint32_t UR15;
    __vo uint32_t UR16;
    __vo uint32_t UR17;

 }SYSCFG_RegDef_t;



 /*
  * PERIPHERAL REGISTER DEFINITION STRUCTURE FOR SPI PERIPHERAL
  */
 typedef struct
 {
    __vo uint32_t CR1;       //GPIO port mode register
    __vo uint32_t CR2;      //GPIO port output type register
    __vo uint32_t CFG1;      //GPIO port output speed register
    __vo uint32_t CFG2;        //GPIO port pull-up/pull-down register
    __vo uint32_t IER;          //GPIO port input data register
    __vo uint32_t SR;          //GPIO port output data register
    __vo uint32_t IFCR;
    uint32_t RESERVED0;
    __vo uint32_t TXDR;
    uint32_t RESERVED1[3];
    __vo uint32_t RXDR ;
    uint32_t RESERVED2[3];
    __vo uint32_t CRCPOLY ;
    __vo uint32_t TXCRC;
    __vo uint32_t RXCRC;
    __vo uint32_t UDRDR;
    __vo uint32_t I2SCFGR;

 }SPI_RegDef_t;

#include "stm32h7xx_gpio_driver.h"
#include "stm32h7xx_spi_driver.h"

#endif /* INC_STM32H7XX_H_ */
