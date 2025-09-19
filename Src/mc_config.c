/**
  ******************************************************************************
  * @file    mc_config.c
  * @version V1.0.0
  * @date    2021-07-30
  ******************************************************************************
  */

#include "main.h"
//#include "mc_type.h"
#include "parameters_conversion.h"
//#include "mc_parameters.h"
#include "mc_config.h"
#include "mcpwm_foc.h"
//#include "enc_KTH7823.h"

//#ifndef HALL_En
//#define MAX_TWAIT 0                 /* Dummy value for single drive */
//#define FREQ_RATIO 1                /* Dummy value for single drive */
//#define FREQ_RELATION HIGHEST_FREQ  /* Dummy value for single drive */
//#endif

#define OFFCALIBRWAIT_MS     0
#define OFFCALIBRWAIT_MS2    0     
//#include "pqd_motor_power_measurement.h"

#if 0
PQD_MotorPowMeas_Handle_t PQD_MotorPowMeasM1 =
{
  .wConvFact = PQD_CONVERSION_FACTOR
};
PQD_MotorPowMeas_Handle_t *pPQD_MotorPowMeasM1 = &PQD_MotorPowMeasM1; 
#endif
//运行时候也不停锁定位置环? 间隔多长时间改变位置? 慢慢移动?
PID_Handle_t PIDPosHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_Pos_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_Pos_KI_DEFAULT, 
  .wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)Pos_KIDIV,
  .wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)Pos_KIDIV,
  .hUpperOutputLimit       = (int16_t)IQMAX, 
  .hLowerOutputLimit       = -(int16_t)IQMAX,
  .hKpDivisor          = (uint16_t)Pos_KPDIV,
  .hKiDivisor          = (uint16_t)Pos_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)Pos_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)Pos_KIDIV_LOG,
  .hDefKdGain           = (int16_t)PID_Pos_KD_DEFAULT,
  .hKdDivisor           = (uint16_t)Pos_KDDIV,
  .hKdDivisorPOW2       = (uint16_t)Pos_KDDIV_LOG,
};
//速度环PID参数
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT, 
  .wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)SP_KIDIV,
  .wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)SP_KIDIV,
  .hUpperOutputLimit       = (int16_t)IQMAX, 
  .hLowerOutputLimit       = -(int16_t)IQMAX,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain           = (int16_t)PID_SPEED_KD_DEFAULT,
  .hKdDivisor           = (uint16_t)SP_KDDIV,
  .hKdDivisorPOW2       = (uint16_t)SP_KDDIV_LOG,
};

PWMC_Handle PWMC_Handle_M1 =
{
  .hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)>>14,
  .hPWMperiod = PWM_PERIOD_CYCLES, 
  .TIMx = TIM1,
};

FOC_Component FOC_Component_M1 ={
  .Vqd.qV_Component1 = 0,
  .Vqd.qV_Component2 = 0,
  .lc.M_dir = 0,
  .lc.x_offset = 0,
  .lc.y_offset = 0,
  .lc.xy_scale = 1,
  .PolePariNum = POLE_PAIR_NUM,
  .lc.learnXYFin = 0,
  .hElAngle = 0,
  .hStepTime = 0,  //增加时间
  .hFinalTorque    =	FINAL_I_ALIGNMENT,            
  .hDurationms     =	ALIGNMENT_DURATION,   
  .hAddActTargetAngle = 0,
  .hAddTargetAngle = 0,
};

Volt_Components GetVqd(void){
  return FOC_Component_M1.Vqd;
}
//获取学习状态
uint8_t GetLearnState(void){
  return FOC_Component_M1.lc.LearnFinish;
}
//设置扭矩
void SetTorque(int16_t hTorque){
  FOC_Component_M1.Vqd.qV_Component1 = hTorque;
}
//设置磁链
void SetFlux(int16_t hFlux){
  FOC_Component_M1.Vqd.qV_Component1 = hFlux;
}
//PID 设置
void SetPosPIDKp(int16_t p){
  PIDPosHandle_M1.hKpGain = p;
}
void SetPosPIDKi(int16_t i){
  PIDPosHandle_M1.hKiGain = i;
}
void SetPosPIDKd(int16_t d){
  PIDPosHandle_M1.hKdGain = d;
}
//PID 设置
void SetSpeedPIDKp(int16_t p){
  PIDSpeedHandle_M1.hKpGain = p;
}
void SetSpeedPIDKi(int16_t i){
  PIDSpeedHandle_M1.hKiGain = i;
}
void SetSpeedPIDKd(int16_t d){
  PIDSpeedHandle_M1.hKdGain = d;
}
/**
 * 
 * 水平垂直转换
*/
void Hor_Turn_Ver(void){
  if(FOC_Component_M1.hAddTargetAngle!=0)
    SetHorizontal();
  else
    SetVertical();
}
/**
 * 
 * 设置到水平
*/
void SetHorizontal(void){
    FOC_Component_M1.hAddTargetAngle = 0;
    //FOC_Component_M1.endAngle = 0;
}
/**
 * 
 * 设置到垂直
*/
void SetVertical(void){
    FOC_Component_M1.hAddTargetAngle = 0x4000;
    //FOC_Component_M1.endAngle = 0x4000;
}
/***
 * 
 * 左转
*/
void SetTurnLeft(void){
    FOC_Component_M1.hAddTargetAngle -= 0x4000;
    //FOC_Component_M1.endAngle = FOC_Component_M1.hTargetAngle;
}
/***
 * 
 * R转
*/
void SetTurnRight(void){
    FOC_Component_M1.hAddTargetAngle += 0x4000;
    //FOC_Component_M1.endAngle = FOC_Component_M1.hTargetAngle;
}
//获取学习的Z轴中点
int16_t GetLearnGyroZBais(void){
  return FOC_Component_M1.lc.gyroVz_Bais;
}
//设置校准的Z轴值
void SetLearnGyroZBais(int16_t vZ){
  FOC_Component_M1.lc.gyroVz_Bais = vZ;
}
/***
 * 
 * 正转360度, 每次比当前增加0x1000度 达到再增加 每=
 * 
*/
//void SetTurnLeftCycle(void){
//}
//重新开始学习参数
void MC_initLearn(void){
    MC_learnHall(&FOC_Component_M1);
}
//获取电角度
int16_t GetMecA(void){
    return FOC_Component_M1.hMecAngle;
}
int16_t GetElA(void){
    return FOC_Component_M1.hElAngle;
}

//uint16_t GetHallX_offset(void){
//  return FOC_Component_M1.lc.x_offset;
//}
//uint16_t GetHallY_offset(void){
//  return FOC_Component_M1.lc.y_offset;
//}
//
//int16_t GetHallXYScale(void){
//  return FOC_Component_M1.lc.xy_scale;
//}
//获取磁对数
//uint8_t GetPolePair(void){
//}
//获取真实的电角度
//int16_t GetRealElAngle(int16_t hMecAngle){
//    int16_t hElAngle;
//    hElAngle = hMecAngle*GetPolePair();    //当前物理角度算出的电角度
//}

#if 0
/**
  * @brief  PI / PID Speed loop parameters Motor 1
  * 云台也不用速度环,只用位置环?
  * 做的好 也许需要速度环 限制匀速动作?
  * 运行时候是速度环 达到位置后改位置环 位置环一定要够力
  */
PID_Handle_t PIDSpeedHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_SPEED_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_SPEED_KI_DEFAULT, 
  .wUpperIntegralLimit = (int32_t)IQMAX * (int32_t)SP_KIDIV,
  .wLowerIntegralLimit = -(int32_t)IQMAX * (int32_t)SP_KIDIV,
  .hUpperOutputLimit       = (int16_t)IQMAX, 
  .hLowerOutputLimit       = -(int16_t)IQMAX,
  .hKpDivisor          = (uint16_t)SP_KPDIV,
  .hKiDivisor          = (uint16_t)SP_KIDIV,
  .hKpDivisorPOW2      = (uint16_t)SP_KPDIV_LOG,
  .hKiDivisorPOW2      = (uint16_t)SP_KIDIV_LOG,
  .hDefKdGain           = (int16_t)PID_SPEED_KD_DEFAULT,
  .hKdDivisor           = (uint16_t)SP_KDDIV,
  .hKdDivisorPOW2       = (uint16_t)SP_KDDIV_LOG,
};
#endif
#if 0
/**
  * @brief  PI / PID Iq loop parameters Motor 1
  * 电流环PID参数
  */
PID_Handle_t PIDIqHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_TORQUE_KP_DEFAULT,
  .hDefKiGain          = (int16_t)PID_TORQUE_KI_DEFAULT,
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV,
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,   
  .hUpperOutputLimit       = INT16_MAX,     
  .hLowerOutputLimit       = -INT16_MAX,           
  .hKpDivisor          = (uint16_t)TF_KPDIV,       
  .hKiDivisor          = (uint16_t)TF_KIDIV,       
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,       
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,        
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};
/**
  * @brief  PI / PID Id loop parameters Motor 1
  */
PID_Handle_t PIDIdHandle_M1 =
{
  .hDefKpGain          = (int16_t)PID_FLUX_KP_DEFAULT, 
  .hDefKiGain          = (int16_t)PID_FLUX_KI_DEFAULT, 
  .wUpperIntegralLimit = (int32_t)INT16_MAX * TF_KIDIV, 
  .wLowerIntegralLimit = (int32_t)-INT16_MAX * TF_KIDIV,
  .hUpperOutputLimit       = INT16_MAX,                 
  .hLowerOutputLimit       = -INT16_MAX,                
  .hKpDivisor          = (uint16_t)TF_KPDIV,          
  .hKiDivisor          = (uint16_t)TF_KIDIV,          
  .hKpDivisorPOW2      = (uint16_t)TF_KPDIV_LOG,       
  .hKiDivisorPOW2      = (uint16_t)TF_KIDIV_LOG,       
  .hDefKdGain           = 0x0000U,
  .hKdDivisor           = 0x0000U,
  .hKdDivisorPOW2       = 0x0000U,
};

#endif

#if 0
/**
  * @brief  SpeednTorque Controller parameters Motor 1
  * 速度环和启动也都不要
  */
SpeednTorqCtrl_Handle_t SpeednTorqCtrlM1 =
{
  .STCFrequencyHz =           		MEDIUM_FREQUENCY_TASK_RATE, 	 
  .MaxAppPositiveMecSpeed01Hz =	(uint16_t)(MAX_APPLICATION_SPEED), 
  .MinAppPositiveMecSpeed01Hz =	(uint16_t)(MIN_APPLICATION_SPEED), 
  .MaxAppNegativeMecSpeed01Hz =	(int16_t)(-MIN_APPLICATION_SPEED), 
  .MinAppNegativeMecSpeed01Hz =	(int16_t)(-MAX_APPLICATION_SPEED),
  .MaxPositiveTorque =				(int16_t)NOMINAL_CURRENT,		 
  .MinNegativeTorque =				-(int16_t)NOMINAL_CURRENT,       
  .ModeDefault =					DEFAULT_CONTROL_MODE,            
  .MecSpeedRef01HzDefault =		(int16_t)(DEFAULT_TARGET_SPEED_RPM),
  .TorqueRefDefault =				(int16_t)DEFAULT_TORQUE_COMPONENT,
  .IdrefDefault =					(int16_t)DEFAULT_FLUX_COMPONENT,                                                                     
};

RevUpCtrl_Handle_t RevUpControlM1 =
{
  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE,   
  .hStartingMecAngle       = (int16_t)((int32_t)(STARTING_ANGLE_DEG)* 65536/360),
  .bFirstAccelerationStage = FIRST_SLESS_ALGO_PHASE,   
  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED, 
  .hMinStartUpFlySpeed     = (int16_t)(OBS_MINIMUM_SPEED/2),  
  .OTFStartupEnabled       = false,  
  .OTFPhaseParams         = {(uint16_t)500,                 
                                         0,                 
                             (int16_t)PHASE5_FINAL_CURRENT,
                             (void*)MC_NULL},
  .ParamsData             = {{(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_RPM/6),(int16_t)PHASE1_FINAL_CURRENT,&RevUpControlM1.ParamsData[1]},
                             {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_RPM/6),(int16_t)PHASE2_FINAL_CURRENT,&RevUpControlM1.ParamsData[2]},
                             {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_RPM/6),(int16_t)PHASE3_FINAL_CURRENT,&RevUpControlM1.ParamsData[3]},
                             {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_RPM/6),(int16_t)PHASE4_FINAL_CURRENT,&RevUpControlM1.ParamsData[4]},
                             {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_RPM/6),(int16_t)PHASE5_FINAL_CURRENT,(void*)MC_NULL},
                            },
};
#endif

//RevUpCtrl_Handle_t RevUpControlM1 =
//{
//  .hRUCFrequencyHz         = MEDIUM_FREQUENCY_TASK_RATE,   
//  .hStartingMecAngle       = (int16_t)((int32_t)(STARTING_ANGLE_DEG)* 65536/360),
//  .bFirstAccelerationStage = FIRST_SLESS_ALGO_PHASE,   
//  .hMinStartUpValidSpeed   = OBS_MINIMUM_SPEED, 
//  .hMinStartUpFlySpeed     = (int16_t)(OBS_MINIMUM_SPEED/2),  
//  .OTFStartupEnabled       = false,  
//  .OTFPhaseParams         = {(uint16_t)500,                 
//                                         0,                 
//                             (int16_t)PHASE5_FINAL_CURRENT,
//                             (void*)MC_NULL},
//  .ParamsData             = {{(uint16_t)PHASE1_DURATION,(int16_t)(PHASE1_FINAL_SPEED_RPM/6),(int16_t)PHASE1_FINAL_CURRENT,&RevUpControlM1.ParamsData[1]},
//                             {(uint16_t)PHASE2_DURATION,(int16_t)(PHASE2_FINAL_SPEED_RPM/6),(int16_t)PHASE2_FINAL_CURRENT,&RevUpControlM1.ParamsData[2]},
//                             {(uint16_t)PHASE3_DURATION,(int16_t)(PHASE3_FINAL_SPEED_RPM/6),(int16_t)PHASE3_FINAL_CURRENT,&RevUpControlM1.ParamsData[3]},
//                             {(uint16_t)PHASE4_DURATION,(int16_t)(PHASE4_FINAL_SPEED_RPM/6),(int16_t)PHASE4_FINAL_CURRENT,&RevUpControlM1.ParamsData[4]},
//                             {(uint16_t)PHASE5_DURATION,(int16_t)(PHASE5_FINAL_SPEED_RPM/6),(int16_t)PHASE5_FINAL_CURRENT,(void*)MC_NULL},
//                            },
//};
#if 0
PWMC_R3_F0_Handle_t PWM_Handle_M1 =
{
  {
    .pFctGetPhaseCurrents              = &R3F0XX_GetPhaseCurrents,    
    .pFctSwitchOffPwm                  = &R3F0XX_SwitchOffPWM,             
    .pFctSwitchOnPwm                   = &R3F0XX_SwitchOnPWM,              
    .pFctCurrReadingCalib              = &R3F0XX_CurrentReadingCalibration,
    .pFctTurnOnLowSides                = &R3F0XX_TurnOnLowSides,             
    .pFctSetADCSampPointSect1          = &R3F0XX_SetADCSampPointSect1,     
    .pFctSetADCSampPointSect2          = &R3F0XX_SetADCSampPointSect2,     
    .pFctSetADCSampPointSect3          = &R3F0XX_SetADCSampPointSect3,     
    .pFctSetADCSampPointSect4          = &R3F0XX_SetADCSampPointSect4,     
    .pFctSetADCSampPointSect5          = &R3F0XX_SetADCSampPointSect5,     
    .pFctSetADCSampPointSect6          = &R3F0XX_SetADCSampPointSect6,     
    .pFctIsOverCurrentOccurred         = &R3F0XX_IsOverCurrentOccurred,    
    .pFctOCPSetReferenceVoltage        = MC_NULL,
    .pFctRLDetectionModeEnable         = MC_NULL,    
    .pFctRLDetectionModeDisable        = MC_NULL,   
    .pFctRLDetectionModeSetDuty        = MC_NULL,   
    .hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u,     
    .hSector = 0,    
    .hCntPhA = 0,
    .hCntPhB = 0,
    .hCntPhC = 0,
    .SWerror = 0,
    .bTurnOnLowSidesAction = false, 
    .hOffCalibrWaitTimeCounter = 0, 
    .bMotor = 0,     
    .RLDetectionMode = false,
    .hIa = 0, 
    .hIb = 0, 
    .hIc = 0, 
    .DTTest = 0,    
    .DTCompCnt = 0,
    .hPWMperiod = PWM_PERIOD_CYCLES,
    .hOffCalibrWaitTicks = (uint16_t)((SYS_TICK_FREQUENCY * OFFCALIBRWAIT_MS)/ 1000),
    .hDTCompCnt          = DTCOMPCNT,
    .Ton                 = TON,
    .Toff                = TOFF
  },
  .wPhaseAOffset = 0,  
  .wPhaseBOffset = 0,  
  .wPhaseCOffset = 0,  
  .Half_PWMPeriod = PWM_PERIOD_CYCLES/2u, 
  .hRegConv = 0,
  .OverCurrentFlag = false,  
  .OverVoltageFlag = false,  
  .BrakeActionLock = false,  
  .bIndex = 0,
  .hFlags = 0,
  .ADC1_DMA_converted[0] = 0, 
  .ADC1_DMA_converted[1] = 0,
  .ADC1_DMA_converted[2] = 0,
  .bCalib_A_index = 0,
  .bCalib_B_index = 0,
  .bCalib_C_index = 0,
  .pParams_str = &R3_F0XX_Params
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Base Class
  */
VirtualSpeedSensor_Handle_t VirtualSpeedSensorM1 =
{
  
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM, 
    .hMaxReliableMecSpeed01Hz          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED/6),
    .hMinReliableMecSpeed01Hz          =	(uint16_t)(MIN_APPLICATION_SPEED/6),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,      
    .hMaxReliableMecAccel01HzP         =	65535,                             
    .hMeasurementFrequency             =	TF_REGULATION_RATE,                 
    },
  .hSpeedSamplingFreqHz =	MEDIUM_FREQUENCY_TASK_RATE, 
  .hTransitionSteps     =	(int16_t)(TF_REGULATION_RATE * TRANSITION_DURATION/ 1000.0),
                           
};

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - State Observer + PLL
  */
STO_PLL_Handle_t STO_PLL_M1 =
{
  ._Super = {
	.bElToMecRatio                     =	POLE_PAIR_NUM,              
    .hMaxReliableMecSpeed01Hz          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED/6),
    .hMinReliableMecSpeed01Hz          =	(uint16_t)(MIN_APPLICATION_SPEED/6),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,           
    .hMaxReliableMecAccel01HzP         =	65535,                               
    .hMeasurementFrequency             =	TF_REGULATION_RATE,                  
  },
 .hC1                         =	C1,                             
 .hC2                         =	C2,                             
 .hC3                         =	C3,                             
 .hC4                         =	C4,                             
 .hC5                         =	C5,                             
 .hF1                         =	F1,                             
 .hF2                         =	F2,                             
 .PIRegulator = {
     .hDefKpGain = PLL_KP_GAIN, 
     .hDefKiGain = PLL_KI_GAIN, 
	 .hDefKdGain = 0x0000U,     
     .hKpDivisor = PLL_KPDIV,   
     .hKiDivisor = PLL_KIDIV,   
	 .hKdDivisor = 0x0000U,			 
     .wUpperIntegralLimit = INT32_MAX, 
     .wLowerIntegralLimit = -INT32_MAX,
     .hUpperOutputLimit = INT16_MAX, 
     .hLowerOutputLimit = -INT16_MAX, 
     .hKpDivisorPOW2 = PLL_KPDIV_LOG,  
     .hKiDivisorPOW2 = PLL_KIDIV_LOG, 
     .hKdDivisorPOW2       = 0x0000U, 
   },      			
 .SpeedBufferSize01Hz                =	STO_FIFO_DEPTH_01HZ,           
 .SpeedBufferSizedpp                 =	STO_FIFO_DEPTH_DPP,            
 .VariancePercentage                 =	PERCENTAGE_FACTOR,             
 .SpeedValidationBand_H              =	SPEED_BAND_UPPER_LIMIT,        
 .SpeedValidationBand_L              =	SPEED_BAND_LOWER_LIMIT,        
 .MinStartUpValidSpeed               =	OBS_MINIMUM_SPEED,             
 .StartUpConsistThreshold            =	NB_CONSECUTIVE_TESTS,  	       
 .Reliability_hysteresys             =	OBS_MEAS_ERRORS_BEFORE_FAULTS, 
 .BemfConsistencyCheck               =	BEMF_CONSISTENCY_TOL,          
 .BemfConsistencyGain                =	BEMF_CONSISTENCY_GAIN,         
 .MaxAppPositiveMecSpeed01Hz         =	(uint16_t)(MAX_APPLICATION_SPEED*1.15/6.0), 
 .F1LOG                              =	F1_LOG,                            
 .F2LOG                              =	F2_LOG,                            
 .SpeedBufferSizedppLOG              =	STO_FIFO_DEPTH_DPP_LOG             
};
STO_PLL_Handle_t *pSTO_PLL_M1 = &STO_PLL_M1; 

STO_Handle_t STO_M1 = 
{
  ._Super                        = (SpeednPosFdbk_Handle_t*)&STO_PLL_M1,
  .pFctForceConvergency1         = &STO_PLL_ForceConvergency1,
  .pFctForceConvergency2         = &STO_PLL_ForceConvergency2,
  .pFctStoOtfResetPLL            = &STO_OTF_ResetPLL,
  .pFctSTO_SpeedReliabilityCheck = &STO_PLL_IsVarianceTight                              
};

#ifdef encoder_En
ENC_KTH7823_Handle_t ENC_KTH7823_M1 = 
{
  ._Super = {
    .bElToMecRatio  =   POLE_PAIR_NUM,
    .hMaxReliableMecSpeed01Hz   = 32767,  //(uint16_t)(1.15*MAX_APPLICATION_SPEED),
    .hMinReliableMecSpeed01Hz          =	(uint16_t)(MIN_APPLICATION_SPEED),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,            
    .hMaxReliableMecAccel01HzP         =	65535,                             
    .hMeasurementFrequency             =	TF_REGULATION_RATE,                 
  },
  //.SensorPlacement  =   HALL_SENSORS_PLACEMENT,
  .SpeedSamplingFreq01Hz = 30*MEDIUM_FREQUENCY_TASK_RATE,
  .bElDiffMec = true,
  .I_feed = false,
  .hEACFrequencyHz =	MEDIUM_FREQUENCY_TASK_RATE,
  .hElAngle        =	ALIGNMENT_ANGLE_S16,      
  .hFinalTorque    =	FINAL_I_ALIGNMENT,            
  .hDurationms     =	ALIGNMENT_DURATION,              
  .offsetElAngle  = 0,
  .Maxspeed = MOTOR_MAX_SPEED_RPM,
  .bBatNum = 2,  //默认两节电池
};
/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Encoder
  */
//ENCODER_Handle_t ENCODER_M1 =
//{
//  ._Super = {
//    .bElToMecRatio                     =	POLE_PAIR_NUM,              
//    .hMaxReliableMecSpeed01Hz          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED/6),
//    .hMinReliableMecSpeed01Hz          =	(uint16_t)(MIN_APPLICATION_SPEED/6),
//    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,         
//    .hMaxReliableMecAccel01HzP         =	65535,                               
//    .hMeasurementFrequency             =	TF_REGULATION_RATE,                
//  },  
//  .PulseNumber           =	M1_ENCODER_PPR*4,		 
//  .RevertSignal           =	(FunctionalState)ENC_INVERT_SPEED,
//  .SpeedSamplingFreq01Hz =	10*MEDIUM_FREQUENCY_TASK_RATE, 
//  .SpeedBufferSize       =	ENC_AVERAGING_FIFO_DEPTH,  
//  .TIMx                =	ENC_TIM2,	    
//
//};
/**
  * @brief  Encoder Alignment Controller parameters Motor 1
  */
//EncAlign_Handle_t EncAlignCtrlM1 =
//{
//  .hEACFrequencyHz =	MEDIUM_FREQUENCY_TASK_RATE,
//  .hFinalTorque    =	FINAL_I_ALIGNMENT,            
//  .hElAngle        =	ALIGNMENT_ANGLE_S16,      
//  .hDurationms     =	ALIGNMENT_DURATION,              
//  .bElToMecRatio   =	POLE_PAIR_NUM,            
//};

#endif

/**
  * @brief  SpeedNPosition sensor parameters Motor 1 - Base Class
  */
#ifdef HallEn
HALL_Handle_t HALL_M1 =
{
  ._Super = {
    .bElToMecRatio                     =	POLE_PAIR_NUM,              
    .hMaxReliableMecSpeed01Hz          =	(uint16_t)(1.15*MAX_APPLICATION_SPEED/6),
    .hMinReliableMecSpeed01Hz          =	(uint16_t)(MIN_APPLICATION_SPEED/6),
    .bMaximumSpeedErrorsNumber         =	MEAS_ERRORS_BEFORE_FAULTS,            
    .hMaxReliableMecAccel01HzP         =	65535,                             
    .hMeasurementFrequency             =	TF_REGULATION_RATE,                 
  }, 
  .SensorPlacement     = HALL_SENSORS_PLACEMENT,
  .PhaseShift          = (int16_t)(HALL_PHASE_SHIFT * 65536/360),
  .SpeedSamplingFreqHz = MEDIUM_FREQUENCY_TASK_RATE,
  .SpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH, 
 .TIMClockFreq       = HALL_TIM_CLK,         
 .TIMx                = HALL_TIM2,        

 .H1Port             =  M1_HALL_H1_GPIO_Port, 
 .H1Pin              =  M1_HALL_H1_Pin,       
 .H2Port             =  M1_HALL_H2_GPIO_Port, 
 .H2Pin              =  M1_HALL_H2_Pin,       
 .H3Port             =  M1_HALL_H3_GPIO_Port, 
 .H3Pin              =  M1_HALL_H3_Pin,       									 
};
#endif

/**
  * temperature sensor parameters Motor 1
  */
NTC_Handle_t TempSensorParamsM1 =
{
  .bSensorType = REAL_SENSOR,
  .hLowPassFilterBW        = M1_TEMP_SW_FILTER_BW_FACTOR,
  .hOverTempThreshold      = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d),
  .hOverTempDeactThreshold = (uint16_t)(OV_TEMPERATURE_THRESHOLD_d - OV_TEMPERATURE_HYSTERESIS_d),
  .hSensitivity            = (uint16_t)(ADC_REFERENCE_VOLTAGE/dV_dT),
  .wV0                     = (uint16_t)(V0_V *65536/ ADC_REFERENCE_VOLTAGE),
  .hT0                     = T0_C,											 
};

/* Bus voltage sensor value filter buffer */
uint16_t RealBusVoltageSensorFilterBufferM1[M1_VBUS_SW_FILTER_BW_FACTOR];
uint16_t RealIBusFilterBufferM1[M1_VBUS_SW_FILTER_BW_FACTOR];

/**
  * Bus voltage sensor parameters Motor 1
  */
RDivider_Handle_t RealBusVoltageSensorParamsM1 =
{
  ._Super                =
  {
    .SensorType          = REAL_SENSOR,                 
    .ConversionFactor    = (uint16_t)(ADC_REFERENCE_VOLTAGE / BUS_ADC_CONV_RATIO),                                                   
  },
  
  .LowPassFilterBW       =  M1_VBUS_SW_FILTER_BW_FACTOR,  
  .OverVoltageThreshold  = OVERVOLTAGE_THRESHOLD_d,   
  .UnderVoltageThreshold =  UNDERVOLTAGE_THRESHOLD_d,  
  .aBuffer = RealBusVoltageSensorFilterBufferM1,
};

UI_Handle_t UI_Params =
{
	      .bDriveNum = 0,
};

/** RAMP for Motor1.
  *
  */
RampExtMngr_Handle_t RampExtMngrHFParamsM1 =
{
  .FrequencyHz = TF_REGULATION_RATE 
};

#endif

#if 0
/**
  * @brief  CircleLimitation Component parameters Motor 1 - Base Component
  * 没有电流环 直接电压控制也不用环形限制 因为电压不可能超
  */
CircleLimitation_Handle_t CircleLimitationM1 =
{
  .MaxModule          = MAX_MODULE,        	
  .Circle_limit_table = MMITABLE,        	
  .Start_index        = START_INDEX, 		
};
#endif
#if 0
UFCP_Handle_t pUSART =
{
    ._Super.RxTimeout = 0, 

    .UARTx              = UART1,                
       
};
#endif


