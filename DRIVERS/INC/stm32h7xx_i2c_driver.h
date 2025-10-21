/*
 * stm32h7xx_i2c_driver.h
 *
 *  Created on: Oct 21, 2025
 *      Author: manvitha.g
 */

#ifndef INC_STM32H7XX_I2C_DRIVER_H_
#define INC_STM32H7XX_I2C_DRIVER_H_


#include "stm32h7xx.h"



/*
 * CONFIGURATION structure for I2Cx peripheral
 */
typedef struct
{
    uint32_t I2C_SCLSpeed;        // Desired SCL speed (100kHz, 400kHz, 1MHz)
    uint8_t  I2C_DeviceAddress;   // Own 7-bit device address
    uint8_t  I2C_AddressMode;     // 0 = 7-bit, 1 = 10-bit addressing
    uint8_t  I2C_AckControl;      // Enable/Disable ACK (for slave mode only)
    uint8_t  I2C_DualAddressMode; // ENABLE / DISABLE dual addressing
    uint8_t  I2C_SecondAddress;   // Secondary address (if dual mode)
    uint8_t  I2C_GeneralCall;     // ENABLE / DISABLE general call
    uint8_t  I2C_ClockStretching; // ENABLE / DISABLE clock stretching
    uint32_t I2C_Timing;          // Precomputed TIMINGR value for speed mode
    uint8_t  I2C_AnalogFilter;    // ENABLE / DISABLE analog filter
    uint8_t  I2C_DigitalFilter;   // 0–15: digital noise filter (number of clock cycles)
    uint8_t  I2C_AutoEnd;         // ENABLE / DISABLE automatic STOP after NBYTES
} I2C_Config_t;

/*
 * HANDLE structure for I2Cx peripheral
 */
typedef struct
{
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2C_Config;
	uint8_t     *pTxBuffer;    // Pointer to application TX buffer
	uint32_t    TxLen;
	uint8_t      TxRxState;
	uint8_t      DevAddr;

	void (*I2C_AppEventCallback)(struct I2C_Handle_t *pI2CHandle, uint8_t event);

}I2C_Handle_t;




#define I2C_ADDRMODE_7BIT           0
#define I2C_ADDRMODE_10BIT          1

#define I2C_ACK_ENABLE              1
#define I2C_ACK_DISABLE             0

#define I2C_DUALADDR_DISABLE        0
#define I2C_DUALADDR_ENABLE         1

#define I2C_GENCALL_DISABLE         0
#define I2C_GENCALL_ENABLE          1

#define I2C_STRETCH_ENABLE          1
#define I2C_STRETCH_DISABLE         0

#define I2C_ANALOGFILTER_ENABLE     1
#define I2C_ANALOGFILTER_DISABLE    0

#define I2C_AUTOEND_DISABLE         0
#define I2C_AUTOEND_ENABLE          1

// Digital filter depth 0–15
#define I2C_DIGITALFILTER_0   0
#define I2C_DIGITALFILTER_1   1
#define I2C_DIGITALFILTER_2   2
#define I2C_DIGITALFILTER_3   3
#define I2C_DIGITALFILTER_4   4
#define I2C_DIGITALFILTER_5   5
#define I2C_DIGITALFILTER_6   6
#define I2C_DIGITALFILTER_7   7
#define I2C_DIGITALFILTER_8   8
#define I2C_DIGITALFILTER_9   9
#define I2C_DIGITALFILTER_10   10
#define I2C_DIGITALFILTER_11  11
#define I2C_DIGITALFILTER_12  12
#define I2C_DIGITALFILTER_13  13
#define I2C_DIGITALFILTER_14  14
#define I2C_DIGITALFILTER_15  15

//MACROS FOR SCL_SPEEDS
#define I2C_SCL_SPEED_SM       100000    // Standard mode 100 kHz
#define I2C_SCL_SPEED_FM2K     200000    // Fast mode 200 kHz
#define I2C_SCL_SPEED_FM       400000    // Fast mode 400 kHz
#define I2C_SCL_SPEED_FMP      1000000   // Fast mode plus 1 MHz



//BIT FIELD FOR I2C_CR1 REG
#define I2C_CR1_PE_POS		               0
#define I2C_CR1_TXIE_POS		           1
#define I2C_CR1_RXIE_POS		           2
#define I2C_CR1_ADDRIE_POS		           3
#define I2C_CR1_NACKIE_POS		           4
#define I2C_CR1_STOPIE_POS		           5
#define I2C_CR1_TCIE_POS		           6
#define I2C_CR1_ERRIE_POS		           7
#define I2C_CR1_DNF_POS		               8
#define I2C_CR1_ANFOFF_POS		           12
#define I2C_CR1_TXDMAEN_POS		           14
#define I2C_CR1_RXDMAEN_POS		           15
#define I2C_CR1_SBC_POS		               16
#define I2C_CR1_NOSTRETCH_POS		       17
#define I2C_CR1_WUPEN_POS		           18
#define I2C_CR1_GCEN_POS		           19
#define I2C_CR1_SMBHEN_POS		           20
#define I2C_CR1_SMBDEN_POS		           21
#define I2C_CR1_ALERTEN_POS		           22
#define I2C_CR1_PECEN_POS		           23



#define I2C_CR2_START    (1 << 13)
#define I2C_CR2_STOP     (1 << 14)
#define I2C_CR2_RELOAD   (1 << 24)
#define I2C_CR2_AUTOEND  (1 << 25)
#define I2C_ISR_TXIS     (1 << 1)
#define I2C_ISR_STOPF    (1 << 5)
#define I2C_ICR_STOPCF   (1 << 5)

#define I2C_CR2_RD_WRN   (1 << 10)
#define I2C_ISR_RXNE     (1 << 2)


//MACROS FOR INTERRUPTS

#define I2C_READY              0
#define I2C_BUSY_IN_TX         1
#define I2C_BUSY_IN_RX         2

#define I2C_EV_TX_COMPLETE     0
#define I2C_EV_RX_COMPLETE     1
#define I2C_EV_STOP            2
#define I2C_EV_NACK            3

/*********************************************************************************************************
 *                          APIs supported by this driver
 *                For some information about APIs check function definitions
 *********************************************************************************************************/

/*
 *Peripheral clock setup
 */
void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi);

/*
 *Init and De-init
 */
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);



void I2C_MasterSendData(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t *pTxBuffer, uint32_t Len);
void I2C_MasterReceiveData(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t *pRxBuffer, uint32_t Len);



uint8_t I2C_MasterSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t *pTxBuffer, uint32_t Len);
uint8_t I2C_MasterReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t SlaveAddr, uint8_t *pRxBuffer, uint32_t Len);

/*
 *IRQ Configurations and ISR Handling
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnorDi);
void I2C_IRQPriorityConfig(uint8_t IRQNumber,uint32_t IRQPriority);


/*
 * other peripheral control APIs
 */

void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);//PE BIT IN CR1 REGISTER
uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx,uint32_t FlagName);


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv);
//this has to be implemented by the applocation. and since we dont know what the application might implement it as,
//we need to give it a weak implementation











#endif /* INC_STM32H7XX_I2C_DRIVER_H_ */
