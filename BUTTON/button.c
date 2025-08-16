/******
 * @file button.c
 *  
 * @date Created on: Aug. 16, 2025
 * @author  MaxwellWang
 * 
 */

#include    "button.h"




/***
 * @brief scan button
 * 
 * 
*/
void ScanButton(button_t *bt){
    //有按键按下 长按开关机 短按水平/垂直 双击旋转模式
    uint8_t tmp;
    tmp = (!GetButonPWR())|((!GetButonLR())<<1)|((!GetButonLR())<<2);
    if(tmp!=bt->nowbt){
        bt->deboune = 0;
        bt->nowbt = tmp;
    }else{
        //if(bt->deboune<60000)
        bt->deboune++;
    }
    if(bt->deboune>debouneTime){
        bt->deboune = 0;
        bt->BtTime++;  //10ms
        if((bt->nowbt==0)&&(bt->lastbt!=0)){
            //本次没按键按下,上次不等于0
            bt->BtCount++;
            if(bt->BtCount==2){
                //双击
                switch (bt->lastbt)
                {
                    case bt_pwrEn:
                        Hor_Turn_Ver();
                    break;
                    case bt_LREn:
                    break;
                    case bt_RREn:
                    break;
                    default:
                    break;
                }
            }
        }else if((bt->nowbt==0)&&(bt->lastbt==0)){
            if(bt->BtTime==30){ //打到时间赋值一次
                switch (bt->lastbt)
                {
                    case bt_pwrEn:
                        Hor_Turn_Ver();
                    break;
                    case bt_LREn:
                        SetTurnLeft();
                    break;
                    case bt_RREn:
                        SetTurnRight();
                    break;
                    default:
                    break;
                }
            }
        }else{
            bt->BtTime = 0;
        }
        if(GetONOFF()==0){
            //关机 只判断有没开机
            if((tmp==bt_pwrEn)&&(bt->lastbt==0)&&(bt->deboune>longONOFF)){
                //开机计时
                poweron(); //开机
                bt->lastbt = tmp;   //更新按键值
            }
        }else{
            //已经开机了
            switch(tmp){
                case 0:
                    switch(bt->lastbt){
                        case 0:
                        break;
                        case bt_pwrEn:
                        break;
                        case bt_LREn:
                        break;
                        case bt_RREn:
                        break;
                    }
                    bt->lastbt = tmp;   //更新按键值
                break;
                case bt_pwrEn:
                    if((bt->lastbt==0)&&(bt->deboune>longONOFF)){
                        poweroff(); //关机
                        bt->lastbt = tmp;   //更新按键值
                    }
                break;
                case bt_LREn:
                    //if((bt->lastbt==0)){
                    //    poweroff(); //关机
                    bt->lastbt = tmp;   //更新按键值
                    //}
                break;
                case bt_RREn:
                    bt->lastbt = tmp;   //更新按键值
                break;
                case (bt_LREn|bt_RREn):     //两个按键同时长安关机
                    if((bt->lastbt==0)&&(bt->deboune>longReLearn)){
                        MC_initLearn();
                        bt->lastbt = tmp;   //更新按键值
                    }
                break;
                default:
                    bt->lastbt = tmp;   //更新按键值
                break;
            }
        }
    }
}