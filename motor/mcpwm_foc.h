/******
 * @file mcpwm_foc.h
 *  
 * @date Created on: Aug. 12, 2025
 * @author  MaxwellWang
 */
#ifndef __MCPWM_FOC_H
#define __MCPWM_FOC_H
#include    "main.h"


void mcpwm_foc_init(void);
//霍尔比例增益调整
void GetHallXYScale(Learn_Componets *lc,int16_t *x,int16_t *y);
int16_t PosPISControl(FOC_Component *fc);
int16_t Get_HallAngle(FOC_Component *fc);
Err_FOC MotorRunControl(FOC_Component *fc);
void MC_learnHall(FOC_Component *fc);
uint32_t GetRealElAngle(FOC_Component *fc);
void MC_RunMotorControlTasks(void);
void CURRENT_REGULATION_IRQHandler(void);
void SysTick_Handler(void);


#endif
