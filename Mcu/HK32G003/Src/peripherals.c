/****
 * @file peripherals.c
 * @date Created on: July. 28, 2025
 * @author MaxwellWang
 */

#include "peripherals.h"

//#include "targets.h"
/**
 * system init
*/
void initCorePeripherals(void){
    //设置tick time
    SysTick_Config(SystemCoreClock /SYS_TICK_FREQUENCY);
    //EE_Read();
    MX_GPIO_Init();
    MX_ADC_Init();
    MX_TIM_Init();
    //qmi8658x_init(GYPO_SDA_GPIO_PORT,GYPO_SDA_GPIO_PIN,GYPO_SCL_GPIO_PORT,GYPO_SCL_GPIO_PIN);
    //SysTick_Config(SystemCoreClock /SYS_TICK_FREQUENCY);
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
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_AHBPeriphClockCmd(Button_PWR_GPIO_CLK|Button_LR_GPIO_CLK|Button_RR_GPIO_CLK|PowerEn_GPIO_CLK|LEDG_GPIO_CLK|LEDR_GPIO_CLK, ENABLE);
    /* Configure Button pin as input */
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = PowerEn_GPIO_PIN;
	GPIO_Init(PowerEn_GPIO_PORT, &GPIO_InitStructure);
    PowerEn_Write(1);
    GPIO_InitStructure.GPIO_Pin = LEDR_GPIO_PIN;
	GPIO_Init(LEDR_GPIO_PORT, &GPIO_InitStructure);
    LEDR_Write(0);
    GPIO_InitStructure.GPIO_Pin = LEDG_GPIO_PIN;
	GPIO_Init(LEDG_GPIO_PORT, &GPIO_InitStructure);
    LEDR_Write(0);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Pin = Button_PWR_GPIO_PIN;
    GPIO_Init(Button_PWR_GPIO_PORT, &GPIO_InitStructure); 
    GPIO_InitStructure.GPIO_Pin = Button_LR_GPIO_PIN;
    GPIO_Init(Button_LR_GPIO_PORT, &GPIO_InitStructure); 
    GPIO_InitStructure.GPIO_Pin = Button_RR_GPIO_PIN;
    GPIO_Init(Button_RR_GPIO_PORT, &GPIO_InitStructure); 
}

/**
 * @brief TIM Initialization Function
 * @param None
 * @retval None
 */
void MX_TIM_Init(void)
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
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseStructure.TIM_Period = ((PWM_PERIOD_CYCLES) / 2);
    //TIM_TimeBaseStructure.Autoreload = 3000;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
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

	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    /* CC4 ENABLE */

    TIM_CCxCmd(TIM1,ADC_TrigTIMCH,TIM_CCx_Enable);
	/* CC4_TO_ADC_SELConfig  */
    TIM_CC_TRIGADC(TIM1,ADC_TrigTIMCH, CC_TRIGADC_OCREF);
    //TIM_BrakInputRemap(TIM1, TIM_Break_Remap_COMP1OUT);
}
//关闭所有PWM
void PWMC_OFFPWM(void){
    PHASE_A_duty(0);
    PHASE_B_duty(0);
    PHASE_C_duty(0);
    //关闭PWM
    TIM1->BDTR &= ( uint32_t )~TIM_BDTR_MOE;
}
//开启PWM 处于刹车状态
void PWMC_ONPWM(void){
    //TIM_ClearFlag_UPDATE( TIM1 );
    //while ( TIM_IsActiveFlag_UPDATE( TIM1 ) == RESET )
    //{}
    /* Clear Update Flag */
    //TIM_ClearFlag_UPDATE( TIM1 );
    PHASE_A_duty(PWM_PERIOD_CYCLES/2);
    PHASE_B_duty(PWM_PERIOD_CYCLES/2);
    PHASE_C_duty(PWM_PERIOD_CYCLES/2);
    //while ( TIM_IsActiveFlag_UPDATE( TIM1 ) == RESET )
    //{}
    //开启PWM
    TIM1->BDTR |= TIM_OSSIState_Enable;  //不工作时候输出空闲电平
    TIM1->BDTR |= TIM_BDTR_MOE;
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
	RCC_AHBPeriphClockCmd(Voltage_ADC_CLK|Hall_x_GPIO_CLK|Hall_y_GPIO_CLK, ENABLE);	
      /* ADC Periph clock enable */
    RCC_APBPeriph2ClockCmd(RCC_APBPeriph2_ADC, ENABLE);
    /* USER CODE BEGIN TIM1_Init 1 */
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStruct.GPIO_Pin = Voltage_ADC_PIN;
	GPIO_Init(Voltage_ADC_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = Hall_x_GPIO_PIN;
	GPIO_Init(Hall_x_GPIO_PORT, &GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Pin = Hall_y_GPIO_PIN;
	GPIO_Init(Hall_y_GPIO_PORT, &GPIO_InitStruct);
    ADC_DeInit(ADC);
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExtTrig;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
    ADC_Init(ADC, &ADC_InitStructure);
    /* Convert the ADC1 Channel4 with 239.5 Cycles as sampling time */
    // (sampletime+12.5)*ADC CLK
    ADC_ChannelConfig(ADC, Voltage_ADC_CH, ADC_SampleTime_1_5Cycles);
    ADC_ChannelConfig(ADC, Hall_x_CH, ADC_SampleTime_1_5Cycles);
    ADC_ChannelConfig(ADC, Hall_y_CH, ADC_SampleTime_1_5Cycles);
    /* Enable the ADC peripheral */
    ADC_ClearITPendingBit(ADC, ADC_IT_EOC);
    ADC_Cmd(ADC, ENABLE);
}
/**
 * @brief Hall initial
 * @param   None
 * @retval  None
 * 初始化霍尔?
*/
void MX_Hall_init(void){

}
/**
 * 
 * delay function
*/
static __IO uint32_t msTick=0;
void Delay_ms(__IO uint32_t Delay)
{
  uint32_t tickstart = msTick;
  uint32_t wait = Delay;
  
  while((msTick - tickstart) < wait)
  {
  }
}
/**
  * @brief NVIC Configuration.
  * @retval None
  */
void MX_NVIC_Init(void)
{
    NVIC_InitTypeDef NVIC_InitStruct;
    /* TIM1_BRK_UP_TRG_COM_IRQn interrupt configuration */
    //NVIC_InitStruct.NVIC_IRQChannel=TIM1_IRQn;
    //NVIC_InitStruct.NVIC_IRQChannelPriority = 1;  //0;
    //NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;	
    //NVIC_Init(&NVIC_InitStruct);
    /* TIM2_IRQn interrupt configuration */
    //NVIC_InitStruct.NVIC_IRQChannel=TIM6_IRQn;
    //NVIC_InitStruct.NVIC_IRQChannelPriority = 2;
    //NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;	
    //NVIC_Init(&NVIC_InitStruct);	
    /* ADC interrupt configuration */
    NVIC_InitStruct.NVIC_IRQChannel=ADC1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
    NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;		
    NVIC_Init(&NVIC_InitStruct);

    /* UART1_IRQn interrupt configuration */
    //NVIC_InitStruct.NVIC_IRQChannel=UART1_IRQn;
    //NVIC_InitStruct.NVIC_IRQChannelPriority = 3;
    //NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;	
    //NVIC_Init(&NVIC_InitStruct);

    /* EXTI2_3_IRQn interrupt configuration */
    //NVIC_InitStruct.NVIC_IRQChannel=PIN_plus_EXTI_IRQn;  //EXTI4_15_IRQn;
    //NVIC_InitStruct.NVIC_IRQChannelPriority = 0;
    //NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
    //NVIC_Init(&NVIC_InitStruct);
	
}
