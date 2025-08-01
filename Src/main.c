/*
 * main.c
 *
 *  Created on: July 28, 2025
 *      Author: MaxwellWang
 */
#include    "main.h"
#include    "ADC.h"
#include    "IO.h"
#include    "common.h"
#include    "sensor.h"


int main(void){

    //初始化IO
    MX_GPIO_init();
    //初始化AD
    MX_ADC_init();
    //初始化gyro 60aixs
    MX_GYRO_init();
    //初始化hall 
    MX_Hall_init();
    //初始化马达
    MX_Motor_init();
    //初始化串口
    MX_Uart_init();
    Delay_ms(200);  //增加延迟方便SWD debug
    //初始化中断
    MX_NVIC_init();
    while(1){
        
    }
}