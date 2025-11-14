/******
 * @file button.h
 *  
 * @date Created on: Aug. 16, 2025
 * @author  MaxwellWang
 */
#ifndef __BUTTON_H
#define __BUTTON_H

#include    "main.h"
//#include    "mc_type.h"

#define bt_pwrEn    0x01
#define bt_LREn     0x02
#define bt_RREn     0x04    

//消抖时间
#define debouneTime     5
#define shortLTime      1  //(1+1)*10 = 20ms
#define shortHTime      18  //40  //41*10 = 410ms
#define shortHTimeS     40  //40  //41*10 = 410ms
//长安开关机时间
#define longONOFF       150    
#define longReLearn     300


typedef struct
{
    //按键buff
    uint8_t lastbt; //上次按键值
    uint8_t nowbt; //当前按键值
    uint8_t BtCount;   //按键计数
    uint16_t deboune;   //消抖
    uint16_t BtTime;   //稳定按键计时
}button_t;



//扫描按键
void ScanButton(button_t *bt);
void fScanButton(void);

#endif
