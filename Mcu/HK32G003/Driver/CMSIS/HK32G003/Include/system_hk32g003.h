/*******************************************************************************
* @copyright: Shenzhen Hangshun Chip Technology R&D Co., Ltd
* @filename:  system_hk32g003.h
* @brief:     ADC initialization and configuration
* @author:    AE Team
*******************************************************************************/


#ifndef __SYSTEM_HK32G003_H
#define __SYSTEM_HK32G003_H

#ifdef __cplusplus
 extern "C" {
#endif 

 
#include "hk32g003.h"
/** hk32f0301mxxC_System_Exported_types  */
extern uint32_t SystemCoreClock;          /*!< System Clock Frequency (Core Clock) */
extern const uint16_t AHBPrescTable[16];   /*!< AHB prescalers table values */
extern const uint8_t APBPrescTable[8];    /*!< APB prescalers table values */


extern void SystemInit(void);
extern void SystemCoreClockUpdate(void);
#ifdef __cplusplus
}
#endif

#endif /*__SYSTEM_HK32G003_H */


