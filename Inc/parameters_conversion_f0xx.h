/**
  ******************************************************************************
  * @file    parameters_conversion_f0xx.h 
  * @version V1.0.0
  * @date    2021-07-30
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PARAMETERS_CONVERSION_F0XX_H
#define __PARAMETERS_CONVERSION_F0XX_H


#include "drive_parameters.h"


/*********************** SENSORLESS REV-UP PARAMETERS *************************/
#define FIRST_SLESS_ALGO_PHASE (ENABLE_SL_ALGO_FROM_PHASE-1u)  


/*************************  IRQ Handler Mapping  *********************/														  
#define CURRENT_REGULATION_IRQHandler          ADC1_IRQHandler
//#define TIMx_UP_BRK_M1_IRQHandler              TIM1_IRQHandler


/**********  AUXILIARY TIMER (SINGLE SHUNT) *************/
#define R1_PWM_AUX_TIM                  TIM3

#define TRIG_CONV_LATENCY_NS	259ul /* Referred to Maximum value indicated 
                                    in the Datasheet Table 50 if ADC clock = HSI14 */ 				   
#define SAMPLING_TIME_CORRECTION_FACTOR_NS           500ul/ADC_CLK_MHz                 /* 0.5*1000/ADC_CLK_MHz */ 
#define SAMPLING_TIME_NS ((7 * 1000uL/ADC_CLK_MHz)+SAMPLING_TIME_CORRECTION_FACTOR_NS)

#define ADC_CONV_NB_CK 13u
#define ADC_CONV_TIME_NS    (uint16_t) (((ADC_CONV_NB_CK*1000ul)-500ul)/ADC_CLK_MHz)

#define TW_BEFORE (((uint16_t)(((2*SAMPLING_TIME_NS)+ TRIG_CONV_LATENCY_NS + ADC_CONV_TIME_NS)*ADV_TIM_CLK_MHz)/1000ul)+1u)

#define M1_VBUS_SW_FILTER_BW_FACTOR      10u

/* Sampling time allowed for F0xx are: 1, 7, 13, 28 ADC clock cycle */    
#define M1_VBUS_SAMPLING_TIME  ADC_SampleTime_28_5Cycles

/* Sampling time allowed for F0xx are:  1, 7, 13, 28 ADC clock cycle */    
#define M1_TEMP_SAMPLING_TIME  ADC_SampleTime_28_5Cycles

#endif /*__PARAMETERS_CONVERSION_F0XX_H*/


