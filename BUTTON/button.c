/******
 * @file button.c
 *  
 * @date Created on: Aug. 16, 2025
 * @author  MaxwellWang
 * 
 */

#include    "button.h"
#include    "mc_config.h"
#include    "mcpwm_foc.h"

button_t bt={0};
void fScanButton(void){
    ScanButton(&bt);
}


/***
 * @brief scan button
 * 
 * 
*/
void ScanButton(button_t *bt){
    //有按键按下 长按开关机 短按水平/垂直 双击旋转模式
    uint8_t tmp;
    tmp = (!GetButonPWR())|((!GetButonLR())<<1)|((!GetButonRR())<<2);
    if(tmp!=bt->nowbt){
        bt->deboune = 0;
        bt->nowbt = tmp;
    }else{
        //if(bt->deboune<60000)
        bt->deboune++;
    }
    if(bt->deboune>debouneTime){
        bt->deboune = 0;
        tmp = ((bt->nowbt!=0)<<1)|(bt->lastbt!=0);
        switch(tmp){
            case 0: //都没按下
                if(bt->BtTime<60000)
                bt->BtTime++;   //长按计时
                if((bt->BtTime==shortHTime)&&((bt->BtCount&0xf0))){
                    //执行单机动作
                    switch (bt->BtCount&0xf0)
                    {
                        case 0x10:
                            //水平和垂直切换
                            Hor_Turn_Ver();
                        break;
                        case 0x20:
                            //左转90度
                            SetTurnLeft();
                        break;
                        case 0x30:
                            //右转90度
                            SetTurnRight();
                        break;
                        default:
                        break;
                    }     
                    bt->BtCount = 0;
                }
            break;
            case 1: //01 -> 上次有按键本次释放了
                if((bt->BtTime>shortLTime)&&(bt->BtTime<shortHTime))
                bt->BtCount++;  //记按键次数
                if((bt->BtCount&0x0f)==2){
                    //双击
                    switch (bt->lastbt)
                    {
                        case bt_pwrEn:
                            //特定旋转动作
                            HorOrVerRoll();
                        break;
                        case bt_LREn:
                            //左转360度
                            SetTurnLeftCycle();
                        break;
                        case bt_RREn:
                            //右转360度
                            SetTurnRightCycle();
                        break;
                        default:
                        break;
                    }         
                    bt->BtTime = 0; //清除计时
                    bt->BtCount = 0;
                }else if((bt->BtCount&0x0f)==1){
                    switch (bt->lastbt)
                    {
                        case bt_pwrEn:
                            bt->BtCount |= 0x10;
                        break;
                        case bt_LREn:
                            bt->BtCount |= 0x20;
                        break;
                        case bt_RREn:
                            bt->BtCount |= 0x30;
                        break;
                        default:
                        break;
                    }     
                }
                bt->BtTime = 0; //清除计时
            break;
            case 2:
                bt->BtTime = 0; //清除计时
            break;
            case 3:
            default:
                if(bt->BtTime<60000)
                bt->BtTime++;   //长按计时
                switch (bt->lastbt)
                {
                    case bt_pwrEn:
                    //长安开关 开关机
                    if(bt->BtTime==longONOFF){
                        if(GetONOFF()==0)
                        poweron(); //开机
                        else
                        poweroff(); //关机
                    }
                    break;
                    case (bt_LREn|bt_RREn):
                        if(bt->BtTime==longReLearn){
                            MC_initLearn();
                        }
                    break;
                    default:
                    break;
                }
            break;
        }
        bt->lastbt = bt->nowbt;
    }
}
