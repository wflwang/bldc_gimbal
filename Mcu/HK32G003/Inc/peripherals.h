/*
 * peripherals.h
 *
 *  Created on: July. 31, 2025
 *      Author: MaxwellWang
 */

#ifndef PERIPHERALS_H_
#define PERIPHERALS_H_

#include    "mc_type.h"
//#include <stdint.h>
//#include "main.h"
//#include "parameters_conversion.h"
//缓冲区大小
#define RX_BUFFER_SIZE           64
typedef struct 
{
	uint8_t Data[RX_BUFFER_SIZE];
	uint8_t Index;
	uint8_t Len;
	uint8_t FinishedFlag;
}UartRxBufDef;

//init GPIO
void MX_GPIO_Init(void);
//init tim
void MX_TIM_Init(void);
void MX_ADC_Init(void);
void MX_Hall_init(void);
void MX_NVIC_init(void);
void initCorePeripherals(void);
void Delay_ms(__IO uint32_t Delay);
void PWMC_OFFPWM(void);
void PWMC_ONPWM(void);
//切换IO口 SWD到正常IO
void SWD_Pin_To_PB5_PD5_Configuration(void);
void MX_Uart_Init(void);
void UartSendDatas(uint8_t *p, uint8_t len);
void delay_us(uint16_t us_x10);


#endif
