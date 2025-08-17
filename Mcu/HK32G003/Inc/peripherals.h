/*
 * peripherals.h
 *
 *  Created on: July. 31, 2025
 *      Author: MaxwellWang
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include "main.h"
//#include "parameters_conversion.h"

//init GPIO
void MX_GPIO_Init(void);
//init tim
void MX_TIM_Init(void);
void MX_ADC_Init(void);
void MX_Hall_init(void);
void MX_NVIC_Init(void);
void initCorePeripherals(void);
void Delay_ms(__IO uint32_t Delay);
void PWMC_OFFPWM(void);
void PWMC_ONPWM(void);


#endif
