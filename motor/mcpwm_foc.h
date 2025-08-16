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
Err_FOC MotorRunControl(FOC_Component *fc);
void MC_learnHall(FOC_Component *fc);


#endif
