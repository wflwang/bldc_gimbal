/**
  ******************************************************************************
  * @file    mc_config.h 
  * @version V1.0.0
  * @date    2021-07-30
  ******************************************************************************
  */  
#ifndef __MC_CONFIG_H
#define __MC_CONFIG_H

#include "pid_regulator.h"
//#include "revup_ctrl.h"
//#include "speed_torq_ctrl.h"
//#include "virtual_speed_sensor.h"
//#include "ntc_temperature_sensor.h"
//#include "pwm_curr_fdbk.h"
//#include "r_divider_bus_voltage_sensor.h"
//#include "virtual_bus_voltage_sensor.h"
//#include "pqd_motor_power_measurement.h"
//#include "user_interface.h"
//#include "motor_control_protocol.h"
//#include "r3_f0xx_pwm_curr_fdbk.h"
//#include "hall_speed_pos_fdbk.h"
//#include "encoder_speed_pos_fdbk.h"
//#include "enc_align_ctrl.h"
//#include "enc_KTH7823.h"
//#include "ramp_ext_mngr.h"
//#include "circle_limitation.h"
//#include "ui_task.h"

//extern RevUpCtrl_Handle_t RevUpControlM1;
//#include "ramp_ext_mngr.h"
//#include "circle_limitation.h"
//#include "sto_speed_pos_fdbk.h"
//#include "sto_pll_speed_pos_fdbk.h"
//#include "usart_frame_communication_protocol.h"

extern PID_Handle_t PIDPosHandle_M1;
extern PID_Handle_t PIDSpeedHandle_M1;
extern PWMC_Handle PWMC_Handle_M1;
extern FOC_Component FOC_Component_M1;
Volt_Components GetVqd(void);
void SetTorque(int16_t hTorque);
void SetFlux(int16_t hFlux);
void Hor_Turn_Ver(void);  //水平垂直切换
void MC_initLearn(void);
void SetTurnLeft(void);
void SetTurnRight(void);
void SetHorizontal(void);
void SetVertical(void);
uint8_t GetLearnState(void);
int16_t GetLearnGyroZBais(void);
void SetLearnGyroZBais(int16_t vZ);
int16_t GetMecA(void);  //获取物理角度
int16_t GetElA(void); //获取电角度
void SetPosPIDKp(int16_t p);  //设置位置环比例
void SetPosPIDKi(int16_t i);  //设置位置环比例
void SetPosPIDKd(int16_t d);  //设置位置环比例
void SetSpeedPIDKp(int16_t p);  //速度环参数
void SetSpeedPIDKi(int16_t i);
void SetSpeedPIDKd(int16_t d);
//extern PID_Handle_t PIDSpeedHandle_M1;
//extern PID_Handle_t PIDIqHandle_M1;
//extern PID_Handle_t PIDIdHandle_M1;
//extern NTC_Handle_t TempSensorParamsM1;
//extern ENCODER_Handle_t ENCODER_M1;
//extern EncAlign_Handle_t EncAlignCtrlM1;

//extern PWMC_R3_F0_Handle_t PWM_Handle_M1;

//extern SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1;
//extern PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1;
//extern PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1;  
//extern VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1;
//extern STO_Handle_t STO_M1;
//extern STO_PLL_Handle_t STO_PLL_M1;
//extern HALL_Handle_t HALL_M1; 
//extern ENC_KTH7823_Handle_t ENC_KTH7823_M1;
//extern RDivider_Handle_t RealBusVoltageSensorParamsM1;
//extern CircleLimitation_Handle_t CircleLimitationM1;

//extern UI_Handle_t UI_Params;

//extern RampExtMngr_Handle_t RampExtMngrHFParamsM1;
//extern UFCP_Handle_t pUSART;

#define NBR_OF_MOTORS 1
#endif /* __MC_CONFIG_H */

