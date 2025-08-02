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
    TIM_TimeBaseStructure.TIM_Period = ((PWM_PERIOD_CYCLES) / 2);
    //TIM_TimeBaseStructure.Autoreload = 3000;
    TIM_TimeBaseStructure.ClockDivision = TIM_CKD_DIV2;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = (REP_COUNTER);
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);
	/* Channel 1, 2,3 in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_Pulse = 0; //dummy value
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState =TIM_OCNIdleState_Reset;  // LOW_SIDE_POLARITY;          

	TIM_OC1Init(TIM1, &TIM_OCInitStructure); 
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC4Ref);
	TIM_OCStructInit(&TIM_OCInitStructure);
	/* Channel 4 Configuration in OC */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable; 
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
	TIM_OCInitStructure.TIM_Pulse = (((PWM_PERIOD_CYCLES) / 2) - (HTMIN));

	TIM_OC4Init(TIM1, &TIM1_OCInitStructure);
    /* CC4 ENABLE */

    TIM_CCxCmd(TIM1,TIM_Channel_4,TIM_CCx_Enable);
	/* CC4_TO_ADC_SELConfig  */
    TIM_CC_TRIGADC(TIM1,TIM_Channel_4, CC_TRIGADC_OCREF);
    //TIM_BrakInputRemap(TIM1, TIM_Break_Remap_COMP1OUT);
}

/**
 * @brief ADC initial
 * @param   None
 * @retval  None
 * 采样电压 电流 两路霍尔的AD
*/
void MX_ADC_Init(void){
    GPIO_InitTypeDef GPIO_InitStruct;
    ADC_InitTypeDef ADC_InitStructure;
    Queue_InitType
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
}