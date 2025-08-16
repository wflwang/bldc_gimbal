/**
  ******************************************************************************
  * @file    hk32g003_conf.h
  * @brief   configuration file.
  ******************************************************************************
  * @attention 
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HK32G003_CONF_H
#define __HK32G003_CONF_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

/*system clock source*/
#define SYSCLK_SRC_HSI   			0x4
#define SYSCLK_SRC_LSI      	0x5
#define SYSCLK_SCR_EXTCLK_IO  0x6

#define SYSCLK_SOURCE SYSCLK_SRC_HSI

// #define VECT_TAB_SRAM
#define VECT_TAB_OFFSET  0x0 /*!< Vector Table base offset field. This value must be a multiple of 0x200. */
 

/* ########################## EXTCLK/HSI Values adaptation ##################### */
 
  
#define EXTCLK_VALUE	((uint32_t)32000000) /*!< Value of the Internal oscillator in Hz*/

 
#define HSI_VALUE    ((uint32_t)48000000) /*!< Value of the Internal oscillator in Hz*/



/**
  * @brief In the following line adjust the Internal High Speed oscillator (HSI) Startup 
  *        Timeout value 
  */

 #define STARTUP_TIMEOUT   ((uint32_t)0xFFFF) /*!< Time out for start up */
 

/**
  * @brief Internal Low Speed oscillator (LSI) value.
  */

 #define LSI_VALUE  ((uint32_t)60000)    
  /*!< Value of the Internal Low Speed oscillator in Hz
    The real value may vary depending on the variations*/


/* Includes ------------------------------------------------------------------*/
/**
  * @brief Include module's header file 
  */

 #include "hk32g003_rcc.h"

 #include "hk32g003_crc.h"

 #include "hk32g003_exti.h"

 #include "hk32g003_flash.h"

 #include "hk32g003_gpio.h"

 #include "hk32g003_misc.h"

 #include "hk32g003_adc.h"

 #include "hk32g003_syscfg.h"

 #include "hk32g003_def.h"

 #include "hk32g003_i2c.h"

 #include "hk32g003_iwdg.h"

 #include "hk32g003_pwr.h"

 #include "hk32g003_spi.h"

 #include "hk32g003_tim.h"

 #include "hk32g003_uart.h"

 #include "hk32g003_wwdg.h"

 #include "hk32g003_awut.h"
  
 #include "hk32g003_dbgmcu.h"
  
/* Exported macro ------------------------------------------------------------*/
/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *         drivers code
  */
	
	
#define USE_FULL_ASSERT    

#ifdef  USE_FULL_ASSERT
/**
  * @brief  The assert_param macro is used for function's parameters check.
  * @param  expr: If expr is false, it calls assert_failed function
  *         which reports the name of the source file and the source
  *         line number of the call that failed. 
  *         If expr is true, it returns no value.
  * @retval None
  */
  #define assert_param(expr) ((expr) ? (void)0U : assert_failed((char *)__FILE__, __LINE__))
/* Exported functions ------------------------------------------------------- */
  void assert_failed(char* file, uint32_t line);
#else
  #define assert_param(expr) ((void)0U)
#endif /* USE_FULL_ASSERT */    
    
#ifdef __cplusplus
}
#endif

#endif /* __HK32G003_CONF_H */

/************************ (C) COPYRIGHT MKMcircoChuip *****END OF FILE****/
