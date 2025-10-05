/******
 * @file mcpwm_foc.h
 *  
 * @date Created on: Aug. 12, 2025
 * @author  MaxwellWang
 */
#ifndef __MCPWM_FOC_H
#define __MCPWM_FOC_H
#include    "main.h"
#include    "filter.h"

#if 0
/***
 * 
 * 马达控制命令集合
*/
typedef enum{
    lock = 0,   //锁住目标角度
    leftCycle,  //左旋转360
    leftCycle1,  //左旋转360
    rightCycle, //右旋转360
    rightCycle1, //右旋转360
    Hor_LR_roll,   //0->右 225度 左 135度 -> 右0
    Hor_LR_roll1,
    Hor_LR_roll2,
    Ver_LR_roll,    //垂直滚动90-> 左 135 右255 ->左135 -> 右90
    Ver_LR_roll1,
    Ver_LR_roll2,
    Ver_LR_roll3,
}mc_cmd;
#endif


#define lvdErr 2

typedef struct 
{
    //FOC_Component *fc;
    int16_t step;    //变化的步骤
    int16_t OverStep;
}UpRunMode;

//typedef struct 
//{
//    int16_t Add;    //增量
//    int16_t EndAngle;  //本次结束角度
//}RunModeParam;



void mcpwm_foc_init(void);
//霍尔比例增益调整
//void GetHallXYScale(Learn_Componets *lc,int16_t *x,int16_t *y);
int16_t PosPISControl(FOC_Component *fc);
int16_t Get_HallAngle(FOC_Component *fc);
Err_FOC MotorRunControl(FOC_Component *fc);
void MC_learnHall(FOC_Component *fc);
uint32_t GetRealElAngle(FOC_Component *fc);
void MC_RunMotorControlTasks(void);
void CURRENT_REGULATION_IRQHandler(void);
void SysTick_Handler(void);
uint8_t UpNextRunModeAngle(UpRunMode *rm);
void SetTurnLeftCycle(void);
void SetTurnRightCycle(void);
void SetTurnHorRoll(void);
void SetTurnVerRoll(void);
void HorOrVerRoll(void);
void SetSPIDInterval(int16_t in);
int16_t GetSpeedRun(void);
void SetDeadErr(int16_t in);
int16_t Speed_Sample(filter_t *ft,int16_t raw);
int16_t GetTorque(void);
int16_t MecA_Sample(filter_t *ft,int16_t raw);  //物理角度采样滤波
int16_t CalMecAngle(FOC_Component *fc); //获取当前物理角度
void SetPosLoopInv(int16_t in);
int16_t fGetMHdir(void);
bool GetLearnAtt(void);
void SetLearnAttStart(void);
int16_t GetGyroZero(void);
void ClearRunMode(void);    //清除runmode标志
void fScanVdd(void);
uint8_t fGetVddState(void);
void fSetVddState(uint8_t err);
int16_t CalXYAngle(FOC_Component *fc,HallXYs *xynow);
void LearnPolePairAngle(FOC_Component *fc,HallXYs xynow);
int16_t GetAccXoffset(void);
int16_t GetAccYoffset(void);
int16_t GetAccZoffset(void);
void fSetGyroInitMid(int16_t in);

#endif
