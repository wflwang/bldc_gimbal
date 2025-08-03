#ifndef USB_MAKE
#define HK32G003
#endif

#ifdef HK32G003
#define FIRMWARE_NAME   "gimbal"
#define FILE_NAME   "bldc_gimbal_singal"
#define DEAD_TIME   10
//#define Current_ADC_PIN 
//#define Current_ADC_PORT    GPIOC
#define ResPu       10  //上拉电阻
#define ResPD       10  //下拉电阻
#define ADRef       330 //3.3v 采样基准3.3v  330/100
//外部触发选择CC4
#define ADC_ExtTrig ADC_ExternalTrigConv_T1_CC4
#define ADC_TrigTIMCH   TIM_Channel_4   //ADC 转换触发通道

#define Voltage_ADC_PIN     GPIO_Pin_6
#define Voltage_ADC_PORT    GPIOD
#define Voltage_ADC_CLK     RCC_AHBPeriph_GPIOD
#define Voltage_ADC_SOURCE  GPIO_PinSource6
#define Voltage_ADC_CH      ADC_Channel_1
#define GetVolAD()    ADC->DR1
//把AD值转换为 实际电压 *100  最后结果是电压*100倍值
#define GetVol_Value(x)  x*330*(ResPu+ResPD)/ResPD/4095
//#define Millivolt_PER_AMP   16

#define 

#define PHASE_A_GPIO_PIN    GPIO_Pin_6
#define PHASE_A_GPIO_PORT   GPIOC
#define PHASE_A_GPIO_CLK    RCC_AHBPeriph_GPIOC
#define PHASE_A_GPIO_SOURCE GPIO_PinSource6
#define PHASE_A_GPIO_AF     GPIO_AF_3

#define PHASE_B_GPIO_PIN    GPIO_Pin_7
#define PHASE_B_GPIO_PORT   GPIOC
#define PHASE_B_GPIO_CLK    RCC_AHBPeriph_GPIOC
#define PHASE_B_GPIO_SOURCE GPIO_PinSource7
#define PHASE_B_GPIO_AF     GPIO_AF_3

#define PHASE_C_GPIO_PIN    GPIO_Pin_3
#define PHASE_C_GPIO_PORT   GPIOD
#define PHASE_C_GPIO_CLK    RCC_AHBPeriph_GPIOD
#define PHASE_C_GPIO_SOURCE GPIO_PinSource3
#define PHASE_C_GPIO_AF     GPIO_AF_3

#define Hall_x_GPIO_PIN     GPIO_Pin_2
#define Hall_x_GPIO_PORT    GPIOD
#define Hall_x_GPIO_CLK     RCC_AHBPeriph_GPIOD
#define Hall_x_GPIO_SOURCE  GPIO_PinSource2
#define Hall_x_CH           ADC_Channel_4
#define GetHallxAD()   ADC->DR4

#define Hall_y_GPIO_PIN     GPIO_Pin_4
#define Hall_y_GPIO_PORT    GPIOC
#define Hall_y_GPIO_CLK     RCC_AHBPeriph_GPIOC
#define Hall_y_GPIO_SOURCE  GPIO_PinSource4
#define Hall_y_CH           ADC_Channel_2
#define GetHallyAD()  ADC->DR2

//按键定义
#define Button_PWR_GPIO_PIN     GPIO_Pin_5
#define Button_PWR_GPIO_PORT    GPIOC
#define Button_PWR_GPIO_CLK     RCC_AHBPeriph_GPIOC
#define Button_PWR_SOURCE       GPIO_PinSource5
//左旋
#define Button_LR_GPIO_PIN     GPIO_Pin_4
#define Button_LR_GPIO_PORT    GPIOD
#define Button_LR_GPIO_CLK     RCC_AHBPeriph_GPIOD
#define Button_LR_SOURCE       GPIO_PinSource4
//右旋
#define Button_RR_GPIO_PIN     GPIO_Pin_3
#define Button_RR_GPIO_PORT    GPIOA
#define Button_RR_GPIO_CLK     RCC_AHBPeriph_GPIOA
#define Button_RR_SOURCE       GPIO_PinSource3
#endif
//*******************************************
#ifndef FIRMWARE_NAME
#error  "Missing defines for target"

#endif // !1


#ifndef TARGET_VOLTAGE_DIVIDER
#define TARGET_VOLTAGE_DIVIDER  110
#endif