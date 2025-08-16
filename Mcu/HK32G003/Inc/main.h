/* USER CODE BEGIN Header */
/*
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Define to prevent recursive inclusion
 * -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes
 * ------------------------------------------------------------------*/
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "targets.h"
#include    "datatypes.h"
#include "hk32g003.h"
#include "system_hk32g003.h"
#include "button.h"
#include "peripherals.h"
#include "mcpwm_foc.h"
#include    "qmi8658b.h"
//#include "hk32g003_adc.h"
//#include "stm32g0xx_ll_bus.h"
//#include "stm32g0xx_ll_comp.h"
//#include "stm32g0xx_ll_cortex.h"
//#include "stm32g0xx_ll_dma.h"
//#include "stm32g0xx_ll_exti.h"
//#include "hk32g003_gpio.h"
//#include "hk32g003_flash.h"
//#include "stm32g0xx_ll_iwdg.h"
//#include "stm32g0xx_ll_pwr.h"
//#include "stm32g0xx_ll_rcc.h"
//#include "stm32g0xx_ll_system.h"
//#include "hk32g003_tim.h"
//#include "stm32g0xx_ll_usart.h"
//#include "stm32g0xx_ll_utils.h"

//#if defined(USE_FULL_ASSERT)
//#include "stm32_assert.h"
//#endif /* USE_FULL_ASSERT */

/* Private includes
 * ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types
 * ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants
 * --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro
 * ------------------------------------------------------------*/
/* USER CODE BEGIN EM */
/***
 * 
 * main variable 
 * 
*/
typedef struct 
{
    /* data */
    uint8_t IspowerON;  //开机标志
}mainState;

extern mainState mainState_t;


/* USER CODE END EM */

uint8_t GetONOFF(void); //获取开关机状态
void poweron(void);  //开机
void poweroff(void); //关机

/* Exported functions prototypes
 * ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines
 * -----------------------------------------------------------*/
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF
 * FILE****/
 