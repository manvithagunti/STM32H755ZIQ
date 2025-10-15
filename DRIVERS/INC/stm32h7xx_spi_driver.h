/*
 * stm32h7xx_spi_driver.h
 *
 *  Created on: Oct 9, 2025
 *      Author: manvitha.g
 */

#ifndef INC_STM32H7XX_SPI_DRIVER_H_
#define INC_STM32H7XX_SPI_DRIVER_H_

#include "stm32h7xx.h"

/*
 * configuration structure for SPIx peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;  //pin numbers can range from 0 to 15. so that's why uint8_t is used  @GPIO_PIN_NUMBERS
	uint8_t SPI_BusConfig; // possible values from  @GPIO_PIN_MODES
	uint8_t SPI_SclkSpeed;//possible values from  @GPIO_PIN_SPEED
	uint8_t SPI_DFF;//possible values from   @GPIO_PUPD
	uint8_t SPI_CPOL;//possible values from @GPIO_OP_TYPE
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPI_Config_t;

/*
 * Handle structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegDef_t     *pSPIx;   // this holds the base address of SPIx peripherals
	SPI_Config_t     SPIConfig;
}SPI_Handle_t;


/*
 * @SPI_DeviceMode
 */
#define     SPI_DEVICE_MODE_MASTER     1
#define     SPI_DEVICE_MODE_SLAVE      0

/*
 * @SPI_BusConfig
 */
#define     SPI_BUS_CONFIG_FD                      0
#define     SPI_BUS_CONFIG_SIMPLEX_TXONLY          1
#define     SPI_BUS_CONFIG_SIMPLEX_RXONLY          2
#define     SPI_BUS_CONFIG_HD                      3


/*
 * @SPI_SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2           0
#define SPI_SCLK_SPEED_DIV4           1
#define SPI_SCLK_SPEED_DIV8           2
#define SPI_SCLK_SPEED_DIV16          3
#define SPI_SCLK_SPEED_DIV32          4
#define SPI_SCLK_SPEED_DIV64          5
#define SPI_SCLK_SPEED_DIV128         6
#define SPI_SCLK_SPEED_DIV256         7

/*
 * @SPI_DFF
 */
#define SPI_DFF_4BITS                  3
#define SPI_DFF_5BITS                  4
#define SPI_DFF_6BITS                  5
#define SPI_DFF_7BITS                  6
#define SPI_DFF_8BITS                  7
#define SPI_DFF_9BITS                  8
#define SPI_DFF_10BITS                 9
#define SPI_DFF_11BITS                 10
#define SPI_DFF_12BITS                 11
#define SPI_DFF_13BITS                 12
#define SPI_DFF_14BITS                 13
#define SPI_DFF_15BITS                 14
#define SPI_DFF_16BITS                 15
#define SPI_DFF_17BITS                 16
#define SPI_DFF_18BITS                 17
#define SPI_DFF_19BITS                 18
#define SPI_DFF_20BITS                 19
#define SPI_DFF_21BITS                 20
#define SPI_DFF_22BITS                 21
#define SPI_DFF_23BITS                 22
#define SPI_DFF_24BITS                 23
#define SPI_DFF_25BITS                 24
#define SPI_DFF_26BITS                 25
#define SPI_DFF_27BITS                 26
#define SPI_DFF_28BITS                 27
#define SPI_DFF_29BITS                 28
#define SPI_DFF_30BITS                 29
#define SPI_DFF_31BITS                 30
#define SPI_DFF_32BITS                 31


/*
 * @CPOL
 */
#define SPI_CPOL_HIGH                  1
#define SPI_CPOL_LOW                   0

/*
 * @CPHA
 */
#define SPI_CPHA_HIGH                  1
#define SPI_CPHA_LOW                   0

/*
 * @SSM
 */
#define SPI_SSM_SW                     1
#define SPI_SSM_HW                     0

//SPI_CR1 reg bit fields
#define SPI_CR1_SPE      (1 << SPI_CR1_SPE_POS)   // SPI peripheral enable
#define SPI_CR1_CSTART   (1 << SPI_CR1_CSTART_POS)  // Communication start
#define SPI_CR1_CSUSP    (1 << SPI_CR1_CSUSP_POS)  // Communication suspend


/*
 * SPI related status flags definition
 */

#define SPI_SR_RXP_FLAG       (1 << SPI_SR_RXP_POS)   // Receive packet ready
#define SPI_SR_TXP_FLAG       (1 << SPI_SR_TXP_POS)   // Transmit packet ready
#define SPI_SR_DXP_FLAG       (1 << SPI_SR_DXP_POS)   // Duplex packet ready
#define SPI_SR_TXC_FLAG       (1 << SPI_SR_TXC_POS)   // Transmission complete
#define SPI_SR_EOT_FLAG       (1 << SPI_SR_EOT_POS)   // End of transfer
#define SPI_SR_TXTF_FLAG      (1 << SPI_SR_TXTF_POS)  // TXFIFO empty flag



/*
 * MACROS FOR SPI CONFIGURATIONS
 */


////BIT FIELD FOR SPI_CFG2 REG
#define SPI_CFG2_COMM_POS		               17
#define SPI_CFG2_CPHA_POS		               24
#define SPI_CFG2_CPOL_POS		               25
#define SPI_CFG2_SSM_POS                       26
#define SPI_CFG2_SSOE_POS                      29
#define SPI_CFG2_AFCNTR_POS                    31


//BIT FIELD FOR SPI_CFG1 REG
#define SPI_CFG1_DSIZE_POS                     0
#define SPI_CFG1_MBR_POS                       28




//BIT FIELD FOR SPI_CR1 REG
#define SPI_CR1_SPE_POS                        0
#define SPI_CR1_CSTART_POS                     9
#define SPI_CR1_CSUSP_POS                      10
#define SPI_CR1_SSI_POS                        12
#define SPI_CR1_IOLOCK_POS                     16



//BIT FIELDS FOR SPI_sr reg
#define SPI_SR_RXP_POS                         0
#define SPI_SR_TXP_POS                         1
#define SPI_SR_DXP_POS                         2
#define SPI_SR_EOT_POS                         3
#define SPI_SR_TXTF_POS                        4
#define SPI_SR_UDR_POS                         5
#define SPI_SR_OVR_POS                         6
#define SPI_SR_CRCE_POS                        7
#define SPI_SR_TIFRE_POS                       8
#define SPI_SR_MODF_POS                        9
#define SPI_SR_TSERF_POS                       10
#define SPI_SR_SUSP_POS                        11
#define SPI_SR_TXC_POS                         12








/*********************************************************************************************************
 *                          APIs supported by this driver
 *                For some information about APIs check function definitions
 *********************************************************************************************************/

/*
 *Peripheral clock setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/*
 *Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);


/*
 *Data send and receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer, uint32_t Len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pRxBuffer, uint32_t Len);

/*
 *IRQ Configurations and ISR Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pHandle);

/*
 * other peripheral control APIs
 */

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

#endif /* INC_STM32H7XX_SPI_DRIVER_H_ */
