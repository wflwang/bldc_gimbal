/******
 * @file led.c
 *  
 * @date Created on: Aug. 19, 2025
 * @author  MaxwellWang
 * 
 */


#include    "LED.h"
#include    "mc_config.h"

/**
 * @brief green led control
 * 
 * 
*/
void LEDControl(void){
    static uint16_t blinkcount=0;
    if(GetONOFF()){
        //开机时候灯亮
        if(GetLearnState()){
            LEDG_Set();
        }else{
            //闪烁提示
            blinkcount++;
            if(blinkcount&0x80){
                LEDG_Reset();
            }else{
                LEDG_Set();
            }
        }
    }else{
        //关机时候灯灭掉
        LEDG_Reset();
    }
}
