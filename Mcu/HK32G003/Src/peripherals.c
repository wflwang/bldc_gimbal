/****
 * @file peripherals.c
 * @date Created on: July. 28, 2025
 * @author MaxwellWang
 */

#include "peripherals.h"
#include "main.h"
#include "button.h"
#include    "mcpwm_foc.h"
//#include    "EEPROM.h"


static __IO uint32_t msTick=0;
UartRxBufDef Uart_t;
//extern FOC_Component FOC_Component_M1;
//#include "targets.h"
/**
 * system init
*/
void initCorePeripherals(void){
    //设置tick time
    SysTick_Config(SystemCoreClock /SYS_TICK_FREQUENCY);
    //EE_Read();
    MX_GPIO_Init();

    while(GetONOFF()==0){
        //LEDG_Xor();
        Delay_ms(1);
        fScanButton();   //扫描按键功能
    }
		//EE_WriteFOC(&FOC_Component_M1.lc);
		//EE_ReadFOC(&FOC_Component_M1.lc);
    MX_Uart_Init();
    MX_ADC_Init();
    MX_TIM_Init();
    MX_NVIC_init();
    //qmi8658x_init(GYPO_SDA_GPIO_PORT,GYPO_SDA_GPIO_PIN,GYPO_SCL_GPIO_PORT,GYPO_SCL_GPIO_PIN);
    //SysTick_Config(SystemCoreClock /SYS_TICK_FREQUENCY);
}
/**
  * @brief  This function handles SysTick exception,Run MotorControl Tasks.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
    //static uint8_t SystickDividerCounter = SYSTICK_DIVIDER;
    //if (SystickDividerCounter == SYSTICK_DIVIDER)
    //{
        msTick++;
    //    SystickDividerCounter = 0;
    //}
    ////if(Enable_start_button == 0)
    ////{
    ////    button_delay++;
    ////    if(button_delay>500)
    ////    Enable_start_button = 1;   
    ////}
    //SystickDividerCounter++;  
    //if(IsMCCompleted==1){
        MC_RunMotorControlTasks();
    //}
}
/**/
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
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
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
    //TIM_DeInit(TIM1);
    //TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Prescaler = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_CenterAligned1;
    TIM_TimeBaseStructure.TIM_Period = ((PWM_PERIOD_CYCLES) >> 1);
    //TIM_TimeBaseStructure.Autoreload = 3000;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV2;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = (REP_COUNTER);
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_OCStructInit(&TIM_OCInitStructure);
	/* Channel 1, 2,3 in PWM mode */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; 
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
	TIM_OCInitStructure.TIM_Pulse = 0; //dummy value
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; 
	TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_High;         
	TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Reset;
	TIM_OCInitStructure.TIM_OCNIdleState =TIM_OCNIdleState_Reset;  // LOW_SIDE_POLARITY;          

	TIM_OC1Init(TIM1, &TIM_OCInitStructure); 
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	//TIM_OC4Init(TIM1, &TIM_OCInitStructure);

	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);

	TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_OC4Ref);
	TIM_OCStructInit(&TIM_OCInitStructure);
	/* Channel 4 Configuration in OC */
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;  
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Disable;  //TIM_OutputState_Disable; 
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;                  
	TIM_OCInitStructure.TIM_Pulse = (((PWM_PERIOD_CYCLES) >> 1) - (HTMIN));

	TIM_OC4Init(TIM1, &TIM_OCInitStructure);
    /* CC4 ENABLE */

    TIM_CCxCmd(TIM1,ADC_TrigTIMCH,TIM_CCx_Enable);
	/* CC4_TO_ADC_SELConfig  */
    TIM_CC_TRIGADC(TIM1,ADC_TrigTIMCH, CC_TRIGADC_OCREF);
    //TIM_BrakInputRemap(TIM1, TIM_Break_Remap_COMP1OUT);
    /* TIM1 counter enable */
    TIM_Cmd(TIM1, ENABLE);
}
//关闭所有PWM
void PWMC_OFFPWM(void){
    PHASE_A_duty(0);
    PHASE_B_duty(0);
    PHASE_C_duty(0);
    //关闭PWM
    TIM1->BDTR &= ( uint32_t )~TIM_BDTR_MOE;
    //TIM_CtrlPWMOutputs(TIM1, DISABLE);
}
//开启PWM 处于刹车状态
void PWMC_ONPWM(void){
    //TIM_ClearFlag_UPDATE( TIM1 );
    //while ( TIM_IsActiveFlag_UPDATE( TIM1 ) == RESET )
    //{}
    /* Clear Update Flag */
    //TIM_ClearFlag_UPDATE( TIM1 );
    PHASE_A_duty(PWM_PERIOD_CYCLES>>2);
    PHASE_B_duty(PWM_PERIOD_CYCLES>>2);
    PHASE_C_duty(PWM_PERIOD_CYCLES>>2);
    ADC_PHASE_duty((PWM_PERIOD_CYCLES>>1)-5u);
    //while ( TIM_IsActiveFlag_UPDATE( TIM1 ) == RESET )
    //{}
    //开启PWM
    TIM1->BDTR |= TIM_OSSIState_Enable;  //不工作时候输出空闲电平
    TIM1->BDTR |= TIM_BDTR_MOE;
    //TIM_CtrlPWMOutputs(TIM1, ENABLE);
    //ADC_ExtTrigConfig(ADC, QUEUE_0, EXT_TIM1_CC4, EXT_TRIG_RISING);
    //ADC_ITConfig(ADC,ADC_IT_EOC,ENABLE);  
    //ADC_ClearITPendingBit(ADC, ADC_IT_EOC);
    ADC_ITConfig(ADC,ADC_IT_EOSEQ,ENABLE);  
    ADC_ClearITPendingBit(ADC, ADC_IT_EOSEQ);
    ADC_Cmd(ADC, ENABLE);
    ADC_StartOfConversion(ADC);
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
    //ADC_DeInit(ADC);
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExtTrig;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_ScanDirection = ADC_ScanDirection_Upward;
    ADC_Init(ADC, &ADC_InitStructure);
    /* Convert the ADC1 Channel4 with 239.5 Cycles as sampling time */
    // (sampletime+12.5)*ADC CLK
    //ADC_ChannelConfig(ADC, Voltage_ADC_CH, ADC_SampleTime_1_5Cycles);
    //ADC_ChannelConfig(ADC, Hall_x_CH, ADC_SampleTime_1_5Cycles);
    ADC_ChannelConfig(ADC, Hall_y_CH|Hall_x_CH|Voltage_ADC_CH, ADC_SampleTime_1_5Cycles);
    /* Enable the ADC peripheral */
    //ADC_ClearITPendingBit(ADC, ADC_IT_EOC);
    //ADC_Cmd(ADC, ENABLE);
}
///**
// * @brief Hall initial
// * @param   None
// * @retval  None
// * 初始化霍尔?
//*/
//void MX_Hall_init(void){
// 
//}
/**
 * @brief uart initial
 * @param   None
 * @retval  None
 * 
*/
void MX_Uart_Init(void){
    UART_InitTypeDef UART_InitStructure;
    /* Enable UART1 clock */
    RCC_AHBPeriphClockCmd(UartTX_CLK | UartRX_CLK, ENABLE);
    GPIO_PinAFConfig(UartTX_PORT, UartTX_SOURCE, UartTX_GPIO_AF);
    GPIO_PinAFConfig(UartRX_PORT, UartRX_SOURCE, UartRX_GPIO_AF);
    RCC_APBPeriph2ClockCmd(RCC_APBPeriph2_UART1, ENABLE);
    UART_InitStructure.UART_BaudRate = bps_rate;
    UART_InitStructure.UART_WordLength = UART_WordLength_8b;
    UART_InitStructure.UART_StopBits = UART_StopBits_1;
    UART_InitStructure.UART_Parity = UART_Parity_No;
    UART_InitStructure.UART_Mode = UART_Mode_Rx | UART_Mode_Tx;
    UART_Init(UART1, &UART_InitStructure);
    UART_ITConfig(UART1, UART_IT_RXNE, ENABLE);
    UART_ITConfig(UART1, UART_IT_IDLE, DISABLE);
    Uart_t.Index = 0;
    Uart_t.Len = 0;
    Uart_t.FinishedFlag = RESET;
    //Uart1Rx.Index = 0;
    //Uart1Rx.Len = 0;
    //Uart1Rx.FinishedFlag = RESET;
    /* Enable the UART1 */
    UART_Cmd(UART1, ENABLE);
}
/**
  * @brief  This function handles UART1 global interrupt request.
  * @param  None
  * @retval None
  */
void UART1_IRQHandler(void)
{
	if( UART_GetITStatus(UART1, UART_IT_RXNE) != RESET )
	{
		UART_ClearITPendingBit(UART1, UART_IT_RXNE);
        /* receive data */
        Uart_t.Index = (Uart_t.Index+1)%RX_BUFFER_SIZE;
        Uart_t.Data[Uart_t.Index] = UART_ReceiveData(UART1);
		
		if(Uart_t.Index == 1)
		{
			UART_ITConfig(UART1, UART_IT_IDLE, ENABLE);
		}
	}
	
	if(UART_GetITStatus(UART1, UART_IT_IDLE) != RESET)
	{
		UART_ClearITPendingBit(UART1, UART_IT_IDLE);
		UART_ITConfig(UART1, UART_IT_IDLE, DISABLE);
		Uart_t.Len = Uart_t.Index;
		Uart_t.Index = 0;
		Uart_t.FinishedFlag = SET;
	}
}
/**
  * @brief  UART1 Send some data.
  * @param  p - the start address of data to be send
  * @param  len - the len of data to be send
  * @retval None
  */
void UartSendDatas(uint8_t *p, uint8_t len)
{
  while( len -- )
  {
    UART_SendData(UART1, *p++);

    while( UART_GetFlagStatus(UART1, UART_FLAG_TC) == RESET )
    {
    }
  }
}
/***
 * @brief 获取串口调试信息 P/I/D
 * 
 * 
*/
void GetUartDebug(void){

}
/**
  * @brief  Configure SWDIO pin to GPIO function
  * @param  None
  * @retval None
  */
void SWD_Pin_To_PB5_PD5_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Enable IOMUX clock */
  RCC_APBPeriph1ClockCmd(RCC_APBPeriph1_IOMUX, ENABLE);

  /* Enable GPIOB adn GPIOD clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOD, ENABLE);

  /* Configure SWDIO to PD5 */
#if defined (HK32F0301MJ4M7C) /* for SOP8 - PIN8*/
  GPIO_IOMUX_ChangePin(IOMUX_PIN8, IOMUX_PD5_SEL_PD5);
#elif defined (HK32F0301MD4P7C) /*for TSSOP16 - PIN 15*/
  GPIO_IOMUX_ChangePin(IOMUX_PIN15, IOMUX_PD5_SEL_PD5);
#elif defined (HK32G003F4P7) /* for TSSOP20 - PIN 2 */
  GPIO_IOMUX_ChangePin(IOMUX_PIN2, IOMUX_PD5_SEL_PD5);
#elif defined (HK32F0301MF4N7C)  /* for QFN20 - PIN19*/
  GPIO_IOMUX_ChangePin(IOMUX_PIN19, IOMUX_PD5_SEL_PD5);
#endif

  /* Configure SWCLK to PB5 */
#if defined (HK32F0301MJ4M7C) /* for SOP8 - PIN5*/
  GPIO_IOMUX_ChangePin(IOMUX_PIN5, IOMUX_PB5_SEL_PB5);
#elif defined (HK32F0301MD4P7C) /*for TSSOP16 - PIN 9*/
  GPIO_IOMUX_ChangePin(IOMUX_PIN9, IOMUX_PB5_SEL_PB5);
#elif defined (HK32G003F4P7) /* for TSSOP20 - PIN 11 */
  GPIO_IOMUX_ChangePin(IOMUX_PIN11, IOMUX_PB5_SEL_PB5);
#elif defined (HK32F0301MF4N7C)  /* for QFN20 - PIN8*/
  GPIO_IOMUX_ChangePin(IOMUX_PIN8, IOMUX_PB5_SEL_PB5);
#endif

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOD, &GPIO_InitStructure);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}
/**
 * 
 * delay function
*/
void Delay_ms(__IO uint32_t Delay)
{
  uint32_t tickstart = msTick;
  uint32_t wait = Delay;
  
  while((msTick - tickstart) < wait)
  {
  }
}
/***
 * @brief 
 * @param us_x10 微秒*10 提高描述精度
*/
void delay_us(uint16_t us_x10){
   // 动态计算所需循环次数（主频单位MHz）
    uint32_t cycles = (uint32_t)us_x10 * (SystemCoreClock / 300000) ; // 每循环3周期
    __asm volatile (
        "MOV R0, %0 \n"      // 通过%0传入循环次数
        "1: SUBS R0, #1 \n"   // 1周期
        "BNE 1b \n"           // 2周期（跳转时）
        : : "r" (cycles)      // 输入操作数，将cycles值赋给R0
        : "r0"                // 声明R0被修改
    );
}
/**
  * @brief NVIC Configuration.
  * @retval None
  */
void MX_NVIC_init(void)
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
    /* UART1 IRQ Channel configuration */
    NVIC_InitStruct.NVIC_IRQChannel = UART1_IRQn;
    NVIC_InitStruct.NVIC_IRQChannelPriority = 0x01;
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
