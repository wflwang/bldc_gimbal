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
//#include    "ADC.h"
//#include    "IO.h"
//#include    "common.h"
//#include    "sensor.h"

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
    qmi8658x_init(GYPO_SDA_GPIO_PORT,GYPO_SDA_GPIO_PIN,GYPO_SCL_GPIO_PORT,GYPO_SCL_GPIO_PIN);
    //初始化hall 
    //MX_Hall_init();
    //初始化马达
    //MX_Motor_init();
    mcpwm_foc_init();
    //初始化串口
    //MX_Uart_init();
    //初始化EEPROM
    //EEPROM_init();  
    Delay_ms(200);  //增加延迟方便SWD debug
    //初始化中断
    MX_NVIC_init();
    while(1){
        //Scan_GYPO();    //扫描陀螺仪控制
        Delay_ms(1);  //增加延迟方便SWD debug
        fScanButton();   //扫描按键功能
        //LEDControl();   //LED控制
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
