/*******************************************************************************
* @copyright: Shenzhen Hangshun Chip Technology R&D Co., Ltd
* @filename:  hk32g003_dbgmcu.h
* @brief:     This file provides firmware functions to manage the following
*             functionalities of the Debug MCU (DBGMCU) peripheral:
*             + Device and Revision ID management
*             + Peripherals Configuration
* @author:    AE Team
*******************************************************************************/


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HK32G003_DBGMCU_H
#define __HK32G003_DBGMCU_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "hk32g003.h"
 
/** @addtogroup DBGMCU 
  * @{
  */ 
/* Exported types ------------------------------------------------------------*/ 
/* Exported constants --------------------------------------------------------*/


/** @defgroup DBGMCU_Exported_Constants DBGMCU_Exported_Constants
  * @{ 
  */

#define DBGMCU_STOP                  DBGMCU_CR_DBG_STOP
#define IS_DBGMCU_PERIPH(PERIPH) ((((PERIPH) & 0xFFFFFFFD) == 0x00) && ((PERIPH) != 0x00))

#define DBGMCU_TIM1_STOP             DBGMCU_APB_FZ_DBG_TIM1_STOP  
#define DBGMCU_TIM2_STOP             DBGMCU_APB_FZ_DBG_TIM2_STOP
#define DBGMCU_TIM6_STOP             DBGMCU_APB_FZ_DBG_TIM6_STOP 
#define DBGMCU_WWDG_STOP             DBGMCU_APB_FZ_DBG_WWDG_STOP
#define DBGMCU_IWDG_STOP             DBGMCU_APB_FZ_DBG_IWDG_STOP
#define DBGMCU_I2C1_SMBUS_TIMEOUT    DBGMCU_APB_FZ_DBG_I2C1_SMBUS_TIMEOUT
#define IS_DBGMCU_APB1PERIPH(PERIPH) ((((PERIPH) & 0xFFDFE7EC) == 0x00) && ((PERIPH) != 0x00))


/**
  * @}
  */ 
  
/**
  * @}
  */ 

 
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */ 

/* Device and Revision ID management functions ********************************/ 
uint16_t DBGMCU_GetREVID(void);
uint16_t DBGMCU_GetDEVID(void);
uint16_t DBGMCU_GetENGID(void);
uint16_t DBGMCU_GetDEVID2(void);
/* Peripherals Configuration functions ****************************************/ 
void DBGMCU_Config(uint32_t DBGMCU_Periph, FunctionalState NewState);
void DBGMCU_APB1PeriphConfig(uint32_t DBGMCU_Periph, FunctionalState NewState);

#ifdef __cplusplus
}
#endif

#endif /* __HK32G003_DBGMCU_H */
 
 
