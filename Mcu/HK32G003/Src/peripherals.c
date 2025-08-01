/*
 * peripherals.c
 *
 *  Created on: July. 28, 2025
 *      Author: MaxwellWang
 */

#include "peripherals.h"

#include "targets.h"
/**
 * system init
*/
void initCorePeripherals(void){
    MX_GPIO_Init();
}
/**
 * 初始化 跳转
*/
#if 0
void initAfterJump(void){
    SCB->VTOR = 0x08001000;
    __enable_irq();
}
#endif
/**
 * 时钟配置
*/
#if 0
void SystemClock_Config(void)
{
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_2);
    if (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_2) {
        //  Error_Handler();
    };

    /* HSI configuration and activation */
    LL_RCC_HSI_Enable();
    while (LL_RCC_HSI_IsReady() != 1) {
    };

    /* LSI configuration and activation */
    LL_RCC_LSI_Enable();
    while (LL_RCC_LSI_IsReady() != 1) {
    };

    /* Main PLL configuration and activation */
    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8,
        LL_RCC_PLLR_DIV_2);
    LL_RCC_PLL_Enable();
    LL_RCC_PLL_EnableDomain_SYS();
    while (LL_RCC_PLL_IsReady() != 1) {
    };

    /* Set AHB prescaler*/
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

    /* Sysclk activation on the main PLL */
    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);
    while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
    };

    /* Set APB1 prescaler*/
    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_Init1msTick(64000000);
    LL_SetSystemCoreClock(64000000);
    /* Update CMSIS variable (which can be updated also through
     * SystemCoreClockUpdate function) */
    LL_SetSystemCoreClock(64000000);
    LL_RCC_SetTIMClockSource(LL_RCC_TIM1_CLKSOURCE_PCLK1);
    LL_RCC_SetADCClockSource(LL_RCC_ADC_CLKSOURCE_SYSCLK);
}
#endif
/**
 * GPIO inital
 * IO口配置 ->power set..
*/
void MX_GPIO_Init(void){

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
void MX_TIM1_Init(void)
{
    /* USER CODE BEGIN TIM1_Init 0 */

    /* USER CODE END TIM1_Init 0 */

    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure = {0};
    TIM_OCInitTypeDef  TIM_OCInitStructure={0};
    GPIO_InitTypeDef GPIO_InitStructure={0};
    /* Peripheral clock enable */
    //LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM1);
	RCC_AHBPeriphClockCmd(PHASE_A_GPIO_CLK|PHASE_B_GPIO_CLK|PHASE_C_GPIO_CLK, ENABLE);	
    /* USER CODE BEGIN TIM1_Init 1 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;		
	GPIO_InitStructure.GPIO_Pin = PHASE_A_GPIO_PIN;
	GPIO_Init(PHASE_A_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = PHASE_B_GPIO_PIN;
	GPIO_Init(PHASE_B_GPIO_PORT, &GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Pin = PHASE_C_GPIO_PIN;
	GPIO_Init(PHASE_C_GPIO_PORT, &GPIO_InitStructure);

	GPIO_PinAFConfig(PHASE_A_GPIO_PORT, PHASE_A_GPIO_SOURCE, PHASE_A_GPIO_AF);
	GPIO_PinAFConfig(PHASE_B_GPIO_PORT, PHASE_B_GPIO_SOURCE, PHASE_B_GPIO_AF);
	GPIO_PinAFConfig(PHASE_C_GPIO_PORT, PHASE_C_GPIO_SOURCE, PHASE_C_GPIO_AF);
    /* USER CODE END TIM1_Init 1 */
    /* TIM1 clock enable */
    RCC_APBPeriph2ClockCmd(RCC_APBPeriph2_TIM1, ENABLE);
    TIM_DeInit(TIM1);
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.Prescaler = 0;
    TIM_TimeBaseStructure.CounterMode = TIM_CounterMode_CenterAligned1;
    TIM1_TimeBaseStructure.TIM_Period = ((PWM_PERIOD_CYCLES) / 2);
    //TIM_TimeBaseStructure.Autoreload = 3000;
    TIM_TimeBaseStructure.ClockDivision = TIM_CKD_DIV2;
    TIM1_TimeBaseStructure.TIM_RepetitionCounter = (REP_COUNTER);
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    LL_TIM_Init(TIM1, &TIM_InitStruct);
    LL_TIM_EnableARRPreload(TIM1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH1);
#ifdef USE_SWAPPED_OUPUT
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM2;
#else
    TIM_OC_InitStruct.OCMode = LL_TIM_OCMODE_PWM1;
#endif
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.CompareValue = 0;
    TIM_OC_InitStruct.OCPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCNPolarity = LL_TIM_OCPOLARITY_HIGH;
    TIM_OC_InitStruct.OCIdleState = LL_TIM_OCIDLESTATE_LOW;
    TIM_OC_InitStruct.OCNIdleState = LL_TIM_OCIDLESTATE_LOW;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH1, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH1);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH2);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH2, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH2);
    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH3);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH3, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH3);

    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH4);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH4, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH4);

    LL_TIM_OC_EnablePreload(TIM1, LL_TIM_CHANNEL_CH5);
    TIM_OC_InitStruct.OCState = LL_TIM_OCSTATE_DISABLE;
    TIM_OC_InitStruct.OCNState = LL_TIM_OCSTATE_DISABLE;
    LL_TIM_OC_Init(TIM1, LL_TIM_CHANNEL_CH5, &TIM_OC_InitStruct);
    LL_TIM_OC_DisableFast(TIM1, LL_TIM_CHANNEL_CH5);

    LL_TIM_SetTriggerOutput(TIM1, LL_TIM_TRGO_RESET);
    LL_TIM_SetTriggerOutput2(TIM1, LL_TIM_TRGO2_RESET);
    LL_TIM_DisableMasterSlaveMode(TIM1);
    TIM_BDTRInitStruct.OSSRState = LL_TIM_OSSR_DISABLE;
    TIM_BDTRInitStruct.OSSIState = LL_TIM_OSSI_DISABLE;
    TIM_BDTRInitStruct.LockLevel = LL_TIM_LOCKLEVEL_OFF;
    TIM_BDTRInitStruct.DeadTime = DEAD_TIME;
    TIM_BDTRInitStruct.BreakState = LL_TIM_BREAK_DISABLE;
    TIM_BDTRInitStruct.BreakPolarity = LL_TIM_BREAK_POLARITY_HIGH;
    TIM_BDTRInitStruct.BreakFilter = LL_TIM_BREAK_FILTER_FDIV1;
    TIM_BDTRInitStruct.BreakAFMode = LL_TIM_BREAK_AFMODE_INPUT;
    TIM_BDTRInitStruct.Break2State = LL_TIM_BREAK2_DISABLE;
    TIM_BDTRInitStruct.Break2Polarity = LL_TIM_BREAK2_POLARITY_HIGH;
    TIM_BDTRInitStruct.Break2Filter = LL_TIM_BREAK2_FILTER_FDIV1;
    TIM_BDTRInitStruct.Break2AFMode = LL_TIM_BREAK_AFMODE_INPUT;
    TIM_BDTRInitStruct.AutomaticOutput = LL_TIM_AUTOMATICOUTPUT_DISABLE;
    LL_TIM_BDTR_Init(TIM1, &TIM_BDTRInitStruct);
    /* USER CODE BEGIN TIM1_Init 2 */

    /* USER CODE END TIM1_Init 2 */
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
    LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
    /**TIM1 GPIO Configuration
    PA7   ------> TIM1_CH1N
    PB0   ------> TIM1_CH2N
    PB1   ------> TIM1_CH3N
    PA8   ------> TIM1_CH1
    PA9 [PA11]   ------> TIM1_CH2
    PA10 [PA12]   ------> TIM1_CH3
    */
#ifdef PWM_ENABLE_BRIDGE
#define PHASE_C_GPIO_LOW PHASE_C_GPIO_ENABLE
#define PHASE_B_GPIO_LOW PHASE_B_GPIO_ENABLE
#define PHASE_A_GPIO_LOW PHASE_A_GPIO_ENABLE
#define PHASE_C_GPIO_PORT_LOW PHASE_C_GPIO_PORT_ENABLE
#define PHASE_B_GPIO_PORT_LOW PHASE_B_GPIO_PORT_ENABLE
#define PHASE_A_GPIO_PORT_LOW PHASE_A_GPIO_PORT_ENABLE

#define PHASE_C_GPIO_HIGH PHASE_C_GPIO_PWM
#define PHASE_B_GPIO_HIGH PHASE_B_GPIO_PWM
#define PHASE_A_GPIO_HIGH PHASE_A_GPIO_PWM
#define PHASE_C_GPIO_PORT_HIGH PHASE_C_GPIO_PORT_PWM
#define PHASE_B_GPIO_PORT_HIGH PHASE_B_GPIO_PORT_PWM
#define PHASE_A_GPIO_PORT_HIGH PHASE_A_GPIO_PORT_PWM
#endif
#ifndef OPEN_DRAIN_PWM
#define PWM_OUTPUT_TYPE LL_GPIO_OUTPUT_PUSHPULL
#else
#define PWM_OUTPUT_TYPE LL_GPIO_OUTPUT_OPENDRAIN
#endif

    GPIO_InitStruct.Pin = PHASE_C_GPIO_LOW;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(PHASE_C_GPIO_PORT_LOW, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PHASE_B_GPIO_LOW;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(PHASE_B_GPIO_PORT_LOW, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PHASE_A_GPIO_LOW;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(PHASE_A_GPIO_PORT_LOW, &GPIO_InitStruct);

    // high side gate / PWM outputs
    GPIO_InitStruct.Pin = PHASE_C_GPIO_HIGH;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = PWM_OUTPUT_TYPE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(PHASE_C_GPIO_PORT_HIGH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PHASE_B_GPIO_HIGH;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = PWM_OUTPUT_TYPE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(PHASE_B_GPIO_PORT_HIGH, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = PHASE_A_GPIO_HIGH;
    GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
    GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.OutputType = PWM_OUTPUT_TYPE;
    GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
    GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
    LL_GPIO_Init(PHASE_A_GPIO_PORT_HIGH, &GPIO_InitStruct);
}