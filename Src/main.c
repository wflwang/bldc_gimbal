/****
 * @file main.c
 *
 * @date Created on: July 28, 2025
 * @author: MaxwellWang
 */
#include    "main.h"
#include "button.h"
#include "peripherals.h"
#include "mcpwm_foc.h"
#include    "qmi8658b.h"
#include "LED.h"
//#include    "ADC.h" 
//#include    "IO.h"
//#include    "common.h"
//#include    "sensor.h"
static uint32_t timetemp=0;
mainState mainState_t={0};

int main(void){
    initCorePeripherals();
    //初始化IO
    //MX_GPIO_init();
    //初始化AD
    //MX_ADC_init();
    //初始化TIM
    //MX_TIM_init();
    //初始化gyro 60aixs
    //MX_GYRO_init();
    //初始化陀螺仪
    qmi8658x_init(GYPO_SDA_GPIO_PORT,GYPO_SDA_GPIO_PIN,GYPO_SCL_GPIO_PORT,GYPO_SCL_GPIO_PIN);
    #ifdef testQMI
    #ifdef cUartDebugEn
    sendstart();
    #endif
    while(1){
      Delay_ms(1);
      getOrientation_1ms(); //获取当前角度值
      LEDControl();   //LED控制
      #ifdef cUartDebugEn
      GetUartDebug(); //获取串口调试数据
      #endif
    }
    #endif
    //初始化hall 
    //MX_Hall_init();
    //初始化马达
    //MX_Motor_init();
    mcpwm_foc_init();
    //初始化串口 
    //MX_Uart_init();
    //初始化EEPROM
    //EEPROM_init();  
    Delay_ms(100);  //增加延迟方便SWD debug
    //初始化中断
    //MX_NVIC_init();
    while(1){
        //Scan_GYPO();    //扫描陀螺仪控制
        uint32_t time1ms = Get1msTick();
        if((time1ms-timetemp)>1){
          timetemp = time1ms;
          //LEDR_Set();
          //Delay_ms(1);  //增加延迟方便SWD debug
          getOrientation_1ms();
          fScanButton();   //扫描按键功能
          LEDControl();   //LED控制
          fScanVdd(); //扫描VDD电压
		      #ifdef cUartDebugEn
          GetUartDebug(); //获取串口调试数据
          #endif
          //LEDR_Xor();
        }
        if(GetGyroFin()==0){
          calibrationGyro();  //先校准陀螺仪中点值
        }
    }
}
/**
 * 
 * @brief Get on off state
*/
uint8_t GetONOFF(){
    return mainState_t.IspowerON;
}
void poweron(void){
    mainState_t.IspowerON = 1;
    PowerEn_Set();  //开机
}
void poweroff(void){
    mainState_t.IspowerON = 0;
    PowerEn_Reset();    //关机
}
#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(char* file , uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */	
       /* Infinite loop */
	
	while (1)
  {		
  }
}
#endif /* USE_FULL_ASSERT */
