/******
 * @file led.c
 *  
 * @date Created on: Aug. 19, 2025
 * @author  MaxwellWang
 * 
 */


#include    "LED.h"
#include    "mc_config.h"
#include    "mcpwm_foc.h"

/**
 * @brief green led control
 * 
 * 
*/
void LEDControl(void){
    static uint16_t blinkcount=0;
    blinkcount++;
    if(GetONOFF()){
        //开机时候灯亮
        if(GetLearnState()){
            //正常模式
            if(fGetVddState()==lvdErr){
                LEDG_Reset();
                if(blinkcount&0x100)
                    LEDR_Reset();
                else
                    LEDR_Set();
            }else{
                LEDR_Reset();
                if(blinkcount&0x100)
                    LEDG_Reset();
                else
                    LEDG_Set();
            }
            //LEDG_Set();
        }else{
            //闪烁提示
            if(GetLearnAtt()){
                //爆闪
                if(blinkcount&0x20)
                    LEDG_Reset();
                else
                    LEDG_Set();
            }else if(blinkcount&0x40){
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
