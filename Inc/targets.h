/*******
 * targets.h
 * 
 * Auth:    MaxwellWang
*/
#ifndef USB_MAKE
#define HK32G003
#endif

#ifdef HK32G003
#define FIRMWARE_NAME   "gimbal"
#define FILE_NAME   "bldc_gimbal_singal"

#define cLearnWaitTime  40  //学习等待时间 间隔这个时间后才开始记录位置 每次到一个极点等待速度陀螺仪时间 太快会导致读取数据不准
#define cLearnCount     16  //学习记录位置的次数
#define cLearnEAngOnce  0x800   //学习时候每次增加的角度 /65536(0x10000)


#define DEAD_TIME   10
#define HallXY_dir       4  //霍尔XY方向 000(x,y不变) 001(x,-y) 010(-x,y) 011(-x,-y) 100(y,x) 101(y,-x) 110(-y,x) 111(-y,-x)
#define AccXY_dir   2
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
#define GetVol_Value(x)  x*ADRef*(ResPu+ResPD)/ResPD/4095
//#define Millivolt_PER_AMP   16

//#define 

#define PHASE_A_GPIO_PIN    GPIO_Pin_6
#define PHASE_A_GPIO_PORT   GPIOC
#define PHASE_A_GPIO_CLK    RCC_AHBPeriph_GPIOC
#define PHASE_A_GPIO_SOURCE GPIO_PinSource6
#define PHASE_A_GPIO_AF     GPIO_AF_3
#define PHASE_A_duty(x)     TIM1->CCR1 = x

#define PHASE_B_GPIO_PIN    GPIO_Pin_7
#define PHASE_B_GPIO_PORT   GPIOC
#define PHASE_B_GPIO_CLK    RCC_AHBPeriph_GPIOC
#define PHASE_B_GPIO_SOURCE GPIO_PinSource7
#define PHASE_B_GPIO_AF     GPIO_AF_3
#define PHASE_B_duty(x)     TIM1->CCR2 = x

#define PHASE_C_GPIO_PIN    GPIO_Pin_3
#define PHASE_C_GPIO_PORT   GPIOD
#define PHASE_C_GPIO_CLK    RCC_AHBPeriph_GPIOD
#define PHASE_C_GPIO_SOURCE GPIO_PinSource3
#define PHASE_C_GPIO_AF     GPIO_AF_3
#define PHASE_C_duty(x)     TIM1->CCR3 = x

#define ADC_PHASE_duty(x)   TIM1->CCR4 = x

#define Hall_x_GPIO_PIN     GPIO_Pin_2
#define Hall_x_GPIO_PORT    GPIOD
#define Hall_x_GPIO_CLK     RCC_AHBPeriph_GPIOD
#define Hall_x_GPIO_SOURCE  GPIO_PinSource2
#define Hall_x_CH           ADC_Channel_4
#define GetHallxAD()        ADC->DR4

#define Hall_y_GPIO_PIN     GPIO_Pin_4
#define Hall_y_GPIO_PORT    GPIOC
#define Hall_y_GPIO_CLK     RCC_AHBPeriph_GPIOC
#define Hall_y_GPIO_SOURCE  GPIO_PinSource4
#define Hall_y_CH           ADC_Channel_2
#define GetHallyAD()        ADC->DR2

//陀螺仪IIC口  SDA
#define GYPO_SDA_GPIO_PIN     GPIO_Pin_2
#define GYPO_SDA_GPIO_PORT    GPIOA
#define GYPO_SDA_GPIO_CLK     RCC_AHBPeriph_GPIOA
#define GYPO_SDA_SOURCE       GPIO_PinSource2
#define GYPO_SDA_Set()      GYPO_SDA_GPIO_PORT->BSRR = GYPO_SDA_GPIO_PIN
#define GYPO_SDA_Reset()    GYPO_SDA_GPIO_PORT->BRR = GYPO_SDA_GPIO_PIN
#define GYPO_SDA_Write(x)   ((x)?GYPO_SDA_Set():GYPO_SDA_Reset())

//陀螺仪IIC口   SCL
#define GYPO_SCL_GPIO_PIN     GPIO_Pin_1
#define GYPO_SCL_GPIO_PORT    GPIOA
#define GYPO_SCL_GPIO_CLK     RCC_AHBPeriph_GPIOA
#define GYPO_SCL_SOURCE       GPIO_PinSource1
#define GYPO_SCL_Set()      GYPO_SCL_GPIO_PORT->BSRR = GYPO_SCL_GPIO_PIN
#define GYPO_SCL_Reset()    GYPO_SCL_GPIO_PORT->BRR = GYPO_SCL_GPIO_PIN
#define SetGYPO_SCL(x)      ((x)?GYPO_SCL_Set():GYPO_SCL_Reset())

//按键定义
//#define Button_PWR_GPIO_PIN     GPIO_Pin_5
//#define Button_PWR_GPIO_PORT    GPIOC
//#define Button_PWR_GPIO_CLK     RCC_AHBPeriph_GPIOC
//#define Button_PWR_SOURCE       GPIO_PinSource5
//#define GetButonPWR()   GPIO_ReadInputDataBit(Button_PWR_GPIO_PORT,Button_PWR_GPIO_PIN)   
////左旋
//#define Button_LR_GPIO_PIN     GPIO_Pin_4
//#define Button_LR_GPIO_PORT    GPIOD
//#define Button_LR_GPIO_CLK     RCC_AHBPeriph_GPIOD
//#define Button_LR_SOURCE       GPIO_PinSource4
//#define GetButonLR()   GPIO_ReadInputDataBit(Button_LR_GPIO_PORT,Button_LR_GPIO_PIN)   
////右旋
//#define Button_RR_GPIO_PIN     GPIO_Pin_3
//#define Button_RR_GPIO_PORT    GPIOA
//#define Button_RR_GPIO_CLK     RCC_AHBPeriph_GPIOA
//#define Button_RR_SOURCE       GPIO_PinSource3
//#define GetButonRR()   GPIO_ReadInputDataBit(Button_RR_GPIO_PORT,Button_RR_GPIO_PIN)   

//drver ETA86321 sleep set
#define DrvSleepEn_PIN      GPIO_Pin_5
#define DrvSleepEn_PORT     GPIOC
#define DrvSleepEn_CLK      RCC_AHBPeriph_GPIOC
#define DrvSleepEn_SOURCE   GPIO_PinSource5
#define DrvSleepEn_Set()          (DrvSleepEn_PORT->BSRR = DrvSleepEn_PIN)
#define DrvSleepEn_Reset()        (DrvSleepEn_PORT->BRR = DrvSleepEn_PIN)
#define DrvSleepEn_Write(x)       ((x)?DrvSleepEn_Set():DrvSleepEn_Reset())
#define DrvSleepEn()            DrvSleepEn_Reset()
#define DrvSleepDis()           DrvSleepEn_Set()
//PWMAEn
#define PWMAEn_PIN      GPIO_Pin_3
#define PWMAEn_PORT     GPIOC
#define PWMAEn_CLK      RCC_AHBPeriph_GPIOC
#define PWMAEn_SOURCE   GPIO_PinSource3
#define PWMAEn_Set()          (PWMAEn_PORT->BSRR = PWMAEn_PIN)
#define PWMAEn_Reset()        (PWMAEn_PORT->BRR = PWMAEn_PIN)
#define PWMAEn_Write(x)       ((x)?PWMAEn_Set():PWMAEn_Reset())
//PWMBEn
#define PWMBEn_PIN      GPIO_Pin_4
#define PWMBEn_PORT     GPIOB
#define PWMBEn_CLK      RCC_AHBPeriph_GPIOB
#define PWMBEn_SOURCE   GPIO_PinSource4
#define PWMBEn_Set()          (PWMBEn_PORT->BSRR = PWMBEn_PIN)
#define PWMBEn_Reset()        (PWMBEn_PORT->BRR = PWMBEn_PIN)
#define PWMBEn_Write(x)       ((x)?PWMBEn_Set():PWMBEn_Reset())
//PWMCEn
#define PWMCEn_PIN      GPIO_Pin_3
#define PWMCEn_PORT     GPIOA
#define PWMCEn_CLK      RCC_AHBPeriph_GPIOA
#define PWMCEn_SOURCE   GPIO_PinSource3
#define PWMCEn_Set()          (PWMCEn_PORT->BSRR = PWMCEn_PIN)
#define PWMCEn_Reset()        (PWMCEn_PORT->BRR = PWMCEn_PIN)
#define PWMCEn_Write(x)       ((x)?PWMCEn_Set():PWMCEn_Reset())

//灯和控制脚定义
//#define PowerEn_GPIO_PIN        GPIO_Pin_1
//#define PowerEn_GPIO_PORT       GPIOD
//#define PowerEn_GPIO_CLK        RCC_AHBPeriph_GPIOD
//#define PowerEn_GPIO_SOURCE     GPIO_PinSource1
//#define PowerEn_Set()       (PowerEn_GPIO_PORT->BSRR = PowerEn_GPIO_PIN)
//#define PowerEn_Reset()     (PowerEn_GPIO_PORT->BRR  = PowerEn_GPIO_PIN)
//#define PowerEn_Write(x)    ((x) ? PowerEn_Set() : PowerEn_Reset())
//lamp R
//#define LEDR_GPIO_PIN           GPIO_Pin_3       
//#define LEDR_GPIO_PORT          GPIOC
//#define LEDR_GPIO_CLK           RCC_AHBPeriph_GPIOC
//#define LEDR_GPIO_SOURCE        GPIO_PinSource3
//#define LEDR_Xor()          GPIO_Toggle(LEDR_GPIO_PORT,LEDR_GPIO_PIN)
//#define LEDR_Set()          (LEDR_GPIO_PORT->BSRR = LEDR_GPIO_PIN)
//#define LEDR_Reset()        (LEDR_GPIO_PORT->BRR = LEDR_GPIO_PIN)
//#define LEDR_Write(x)       ((x)?LEDR_Set():LEDR_Reset())
////lamp G
//#define LEDG_GPIO_PIN           GPIO_Pin_4       
//#define LEDG_GPIO_PORT          GPIOB
//#define LEDG_GPIO_CLK           RCC_AHBPeriph_GPIOB
//#define LEDG_GPIO_SOURCE        GPIO_PinSource4
//#define LEDG_Xor()          GPIO_Toggle(LEDG_GPIO_PORT,LEDG_GPIO_PIN)
//#define LEDG_Set()          (LEDG_GPIO_PORT->BSRR = LEDG_GPIO_PIN)
//#define LEDG_Reset()        (LEDG_GPIO_PORT->BRR = LEDG_GPIO_PIN)
//#define SetLEDG(x)          ((x)?LEDG_Set():LEDG_Reset())

#define UartTX_PIN      GPIO_Pin_4
#define UartTX_PORT     GPIOD
#define UartTX_CLK      RCC_AHBPeriph_GPIOD
#define UartTX_SOURCE   GPIO_PinSource4
#define UartTX_GPIO_AF     GPIO_AF_1

#define UartRX_PIN      GPIO_Pin_7
#define UartRX_PORT     GPIOD
#define UartRX_CLK      RCC_AHBPeriph_GPIOD
#define UartRX_SOURCE   GPIO_PinSource7
#define UartRX_GPIO_AF     GPIO_AF_1

#define bps_rate        115200

#endif
//*******************************************
#ifndef FIRMWARE_NAME
#error  "Missing defines for target"

#endif // !1


//#ifndef TARGET_VOLTAGE_DIVIDER
//#define TARGET_VOLTAGE_DIVIDER  110
//#endif
