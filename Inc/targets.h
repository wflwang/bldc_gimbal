#ifndef USB_MAKE
#define HK32G003
#endif

#ifdef HK32G003
#define FIRMWARE_NAME   "gimbal"
#define FILE_NAME   "bldc_gimbal_singal"
#define DEAD_TIME   10
#define Current_ADC_PIN 
#define Voltage_ADC_PIN
#define Millivolt_PER_AMP   16
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

#define Hall_x_GPIO_PIN
#define Hall_x_GPIO_PORT
#define Hall_x_GPIO_CLK

#define Hall_y_GPIO_PIN
#define Hall_y_GPIO_PORT
#define Hall_y_GPIO_CLK
#endif
//*******************************************
#ifndef FIRMWARE_NAME
#error  "Missing defines for target"

#endif // !1


#ifndef TARGET_VOLTAGE_DIVIDER
#define TARGET_VOLTAGE_DIVIDER  110
#endif