/**
  ******************************************************************************
  * @file    drive_parameters.h 
  * @version V1.0.0
  * @date    2021-07-28
  * @brief   This file contains the parameters for MOTOR controler.
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __DRIVE_PARAMETERS_H
#define __DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/************************* CPU & ADC PERIPHERAL CLOCK CONFIG ******************/
#define SYSCLK_FREQ      48000000uL
#define TIM_CLOCK_DIVIDER  1 
#define ADV_TIM_CLK_MHz    48
#define ADC_CLK_MHz    24uL /* Maximum ADC Clock Frequency expressed in MHz */
#define HALL_TIM_CLK       48000000uL
#define ADC_REFERENCE_VOLTAGE  3.3 //5


/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
#define POLE_PAIR_NUM          7 //4 //7 //4 /* Number of motor pole pairs */
#define RS                     0.561   //0.465   //3.806 //0.031   //3.08 /* Stator resistance , ohm*/
#define LS                     0.0003  //0.0002304  //0.0023 //@0.000022   //0.00061 
/* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

#define NOMINAL_CURRENT         32767  //30000  //32768  //6000   //8000   //12797  //32768  //12797
//60s 的转速 1min转速 *1
#define MOTOR_MAX2_SPEED_RPM     14000
#define MOTOR_MAX3_SPEED_RPM     19000
#define MOTOR_MAX_SPEED_RPM     27000   //15000  //6000   //3200   //4000 /*!< Maximum rated speed  */
#define MOTOR_Mid_SPEED_RPM      30 //50
#define MOTOR_Min_SPEED_RPM      5  //5  //1
#define MOTOR_VOLTAGE_CONSTANT  4 /*!< Volts RMS ph-ph /kRPM */
#define ID_DEMAG                -13000 //-8000   //-8000  //-12797 //-32768 //-12797 /*!< Demagnetization current */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  DEGREES_120 
/*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT        120 
/*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
/*** Quadrature encoder ***/
#define M1_ENCODER_PPR             1000  
/*!< Number of pulses per
                                            revolution */

														 
/*********** Bus voltage sensing section ****************/
#define VBUS_PARTITIONING_FACTOR             0.1666   //0.091   /*!< It expresses how much the Vbus is attenuated before being converted into digital value */
#define NOMINAL_BUS_VOLTAGE_V                14 //24
/******** Current reading parameters section ******/
/*** Topology ***/
#define THREE_SHUNT

#define RSHUNT                        0.01   //0.05   //0.05

/*  ICSs gains in case of isolated current sensors,
        amplification gain for shunts based sensing */
#define AMPLIFICATION_GAIN            10  //5   //5.18 
#define AMPLIFICATION_GAIN_IBUS       10  //5

/*** Noise parameters ***/
#define TNOISE_NS                     1000
#define TRISE_NS                      1000 
   
/************ Temperature sensing section ***************/
/* V[V]=V0+dV/dT[V/Celsius]*(T-T0)[Celsius]*/
#define V0_V                          1.65f /*!< in Volts */
#define T0_C                          25 /*!< in Celsius degrees */
#define dV_dT                         0.023f /*!< V/Celsius degrees */
#define T_MAX                         110 
/*!< Sensor measured 
                                                       temperature at maximum 
                                                       power stage working 
                                                       temperature, Celsius degrees */

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/*** Speed measurement settings ***/
#define MAX_APPLICATION_SPEED           MOTOR_MAX_SPEED_RPM //3000 //3200 //4000 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED           0 
/*!< rpm, mechanical,  
                                                           absolute value */
#define MEAS_ERRORS_BEFORE_FAULTS       3 
/*!< Number of speed  
                                                             measurement errors before 
                                                             main sensor goes in fault */

/*** Encoder **********************/                                                                                                           
#define ENC_MEAS_ERRORS_BEFORE_FAULTS   3 
/*!< Number of failed   
                                                        derived class specific speed 
                                                        measurements before main sensor  
                                                        goes in fault */

#define ENC_INVERT_SPEED                DISABLE  
/*!< To be enabled for  
                                                            encoder (main or aux) if  
                                                            measured speed is opposite 
                                                            to real one */        
#define ENC_AVERAGING_FIFO_DEPTH        16 
/*!< depth of the FIFO used to 
                                                              average mechanical speed in 
                                                              0.1Hz resolution */
/****** Hall sensors ************/ 
#define HALL_MEAS_ERRORS_BEFORE_FAULTS    7  //5  //3 
/*!< Number of failed   
                                                           derived class specific speed 
                                                           measurements before main sensor  
                                                           goes in fault */

#define HALL_AVERAGING_FIFO_DEPTH        16 
/*!< depth of the FIFO used to 
                                                           average mechanical speed in 
                                                           0.1Hz resolution */                                                                                                           
/****** State Observer + PLL ****/
#define VARIANCE_THRESHOLD               0.1 
/*!<Maximum accepted 
                                                            variance on speed 
                                                            estimates (percentage) */
/* State observer scaling factors F1 */                    
#define F1                                   4096
#define F2                                   16384

/* State observer constants */
#define GAIN1                                -8200 //-6243
#define GAIN2                                17950
/*Only in case PLL is used, PLL gains */
#define PLL_KP_GAIN                          425
#define PLL_KI_GAIN                          30

#define OBS_MEAS_ERRORS_BEFORE_FAULTS        3   /*!< Number of consecutive errors on variance test before a speed feedback error is reported */
#define STO_FIFO_DEPTH_DPP                   64   /*!< Depth of the FIFO used to average mechanical speed in dpp format */
#define STO_FIFO_DEPTH_01HZ                  64   /*!< Depth of the FIFO used to average mechanical speed in dpp format */
#define BEMF_CONSISTENCY_TOL                 63.998   /*!< Parameter for B-emf amplitude-speed consistency */
#define BEMF_CONSISTENCY_GAIN                63   /*!< Parameter for B-emf amplitude-speed consistency */

/****** State Observer + CORDIC ***/
#define CORD_VARIANCE_THRESHOLD              4.0   /*!< Maxiumum accepted variance on speed estimates(percentage) */
#define CORD_F1                              4096
#define CORD_F2                              16384

/* State observer constants */
#define CORD_GAIN1                           -8200 //-6243
#define CORD_GAIN2                           17950

#define CORD_MEAS_ERRORS_BEFORE_FAULTS   3  
/*!< Number of consecutive errors   
                                                           on variance test before a speed 
                                                           feedback error is reported */
#define CORD_FIFO_DEPTH_DPP              64  
/*!< Depth of the FIFO used  
                                                            to average mechanical speed  
                                                            in dpp format */
#define CORD_FIFO_DEPTH_01HZ             64  
/*!< Depth of the FIFO used  
                                                           to average mechanical speed  
                                                           in dpp format */        
#define CORD_MAX_ACCEL_DPPP              164  
/*!< Maximum instantaneous 
                                                              electrical acceleration (dpp 
                                                              per control period) */
#define CORD_BEMF_CONSISTENCY_TOL        64  
/* Parameter for B-emf 
                                                           amplitude-speed consistency */
#define CORD_BEMF_CONSISTENCY_GAIN       64  
/* Parameter for B-emf 
                                                          amplitude-speed consistency */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */
#define PWM_FREQUENCY                        11000 //25000 //10000 //10000 //8000  //10000
#define LOW_SIDE_SIGNALS_ENABLING            LS_PWM_TIMER
#define SW_DEADTIME_NS                       200  //800   //1400   /*!< Dead-time to be inserted by FW, only if low side signals are enabled */

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE     1    
/*!< FOC execution rate in 
                                                           number of PWM cycles */     
/* Gains values for torque and flux control loops */
//#define PID_TORQUE_KP_DEFAULT                1000  //5800  //9800 //5800  //4200  //3100   //1300  //1200   //800   //1600	//3040
//#define PID_TORQUE_KI_DEFAULT                1000   //1300  //2200  //2200  //1684  //1300   //1000  //600   //450   //500  //2584

//#define PID_FLUX_KP_DEFAULT                  1000  //5800  //9800  //5900  //4200  //3100    //1300  //1200   //800   //1600	//3040
//#define PID_FLUX_KI_DEFAULT                  1000   //1300   //2200  //2200  //1684  //1300   //1000   //600   //450   //500  //2584


/* Torque/Flux control loop gains dividers*/
//#define TF_KPDIV                             8192
//#define TF_KIDIV                             8192

//#define testQMI //仅仅测试陀螺仪
#define GyroEn  //陀螺仪功能使能
#define cUartDebugEn    //开启串口调试功能
//#define debug_int
/* Speed control loop */ 
#define SPEED_LOOP_FREQUENCY_HZ       1000   //300 
/*!<Execution rate of speed regulation loop (Hz) */
//#define posLoop      //位置环 /  位置环+速度环
#define deadErr   100 //329   //死区范围
//速度环
#define PID_SPEED_KP_DEFAULT                 80 //70   //1520  //570 //175   //200 //#1600   //1100   //1500  //3700  //4800  //2700  //3300  //5200  //1200   //1400  //3400  //1800  //2600	//1459
#define PID_SPEED_KI_DEFAULT                 90 //80   //400  //1800   //#18000   //600   //1200   //1200  //2300  //600   //716
#define PID_SPEED_KD_DEFAULT                 380   //256  //#1500

/* Speed PID parameter dividers */
#define SP_KPDIV                       256   //64
#define SP_KIDIV                       8192  //16384
#define SP_KDDIV                       512   //512  //8192
//位置环
#define PID_Pos_KP_DEFAULT                 4560 //350  //55   //200 //#1600   //1100   //1500  //3700  //4800  //2700  //3300  //5200  //1200   //1400  //3400  //1800  //2600	//1459
#define PID_Pos_KI_DEFAULT                 0 //2400   //9620  //1800   //#18000   //600   //1200   //1200  //2300  //600   //716
#define PID_Pos_KD_DEFAULT                 1000 //31200  //#1500
/* Speed PID parameter dividers */
#define Pos_KPDIV                       256   //64
#define Pos_KIDIV                       8192  //16384
#define Pos_KDDIV                       512   //512  //8192

#define MaxPosSpeed     15000
//PID 间隔时间
#define cPIDDiff        2  //2ms once 500Hz
//#define filterFirstOrder      //一阶滤波
//#define filterAV 
//#define filterAVDeep    32

//输入数据去高去低滑动平均滤波 合成的角度做一阶滤波 让角度输出很稳定
//滑动平均滤波的滤波深度 2的倍数  4/8/16/32/64/128 
#define avFilterDeep    8
//#define HallfilterFirstEn     //使能霍尔一阶滤波

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV 1 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

/* Default settings */
#define DEFAULT_CONTROL_MODE           STC_SPEED_MODE   //STC_SPEED_MODE //STC_TORQUE_MODE //STC_SPEED_MODE	//STC_SPEED_MODE 
/*!< STC_TORQUE_MODE or 
                                                        STC_SPEED_MODE */  
#define DEFAULT_TARGET_SPEED_RPM             60 //420   //300   //150   //1000
#define DEFAULT_TORQUE_COMPONENT             0  //10000  //200  //2000  //6700  //4700  //4000  //2100  //4266
#define DEFAULT_FLUX_COMPONENT               0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
#define OV_VOLTAGE_PROT_ENABLING        DISABLE	//ENABLE
#define UV_VOLTAGE_PROT_ENABLING        DISABLE	//ENABLE
#define OV_VOLTAGE_THRESHOLD_V          18   //48   //30 
/*!< Over-voltage 
                                                         threshold */
#define UD_VOLTAGE_THRESHOLD_V          6 //8 
/*!< Under-voltage 
                                                          threshold */
#define R_BRAKE_SWITCH_OFF_THRES_V      32

#define OV_TEMPERATURE_THRESHOLD_C      110 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C     10 /*!< Celsius degrees */

#define HW_OV_CURRENT_PROT_BYPASS       DISABLE 
/*!< In case ON_OVER_VOLTAGE  
                                                          is set to TURN_ON_LOW_SIDES
                                                          this feature may be used to
                                                          bypass HW over-current
                                                          protection (if supported by 
                                                          power stage) */
/******************************   START-UP PARAMETERS   **********************/
/* Encoder alignment */
//-> 这里电流的配置全部直接转换位电压输出控制  大约70%的力 动到0度位置
#define ALIGN_TimeOnce        10    //每次对齐变化时间
#define ALIGNMENT_DURATION              800  //500 //700 /*!< milliseconds */
#define ALIGNMENT_ANGLE_DEG             270  //90 /*!< degrees [0...359] */
#define FINAL_I_ALIGNMENT               15000 //5000 //23000  //500 //7000 //3000 //9000   //20795 /*!< s16A */
//#define FINAL_I3_ALIGNMENT               5600 //5000 //23000  //500 //7000 //3000 //9000   //20795 /*!< s16A */
//#define FINAL_I4_ALIGNMENT               4700 //5000 //23000  //500 //7000 //3000 //9000   //20795 /*!< s16A */
// With ALIGNMENT_ANGLE_DEG equal to 90 degrees final alignment 
// phase current = (FINAL_I_ALIGNMENT * 1.65/ Av)/(32767 * Rshunt)  
// being Av the voltage gain between Rshunt and A/D input

/* Sensor-less rev-up sequence */
#define STARTING_ANGLE_DEG                   90   /*!< degrees [0...359] */

/* Phase 1 */
#define PHASE1_DURATION                      300   //700   //1000  //180   /*!< milliseconds */
#define PHASE1_FINAL_SPEED_RPM               0   /*!< rpm */
#define PHASE1_FINAL_CURRENT                 4266  //5800  //4266
/* Phase 2 */
#define PHASE2_DURATION                      2400  //600   //2400   /*!< milliseconds */
#define PHASE2_FINAL_SPEED_RPM               500   //1200   /*!< rpm */
#define PHASE2_FINAL_CURRENT                 6800  //4266
/* Phase 3 */
#define PHASE3_DURATION                      1000   /*!< milliseconds */
#define PHASE3_FINAL_SPEED_RPM               1000   /*!< rpm */
#define PHASE3_FINAL_CURRENT                 4664
/* Phase 4 */
#define PHASE4_DURATION                      1000   /*!< milliseconds */
#define PHASE4_FINAL_SPEED_RPM               1000   /*!< rpm */
#define PHASE4_FINAL_CURRENT                 4664
/* Phase 5 */
#define PHASE5_DURATION                      1000   /*!< milliseconds */
#define PHASE5_FINAL_SPEED_RPM               1000   /*!< rpm */
#define PHASE5_FINAL_CURRENT                 4664

#define ENABLE_SL_ALGO_FROM_PHASE      2

/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM          200   //300   //1000
#define NB_CONSECUTIVE_TESTS           2  //2 
/* corresponding to 
                                                         former NB_CONSECUTIVE_TESTS/
                                                         (TF_REGULATION_RATE/
                                                         MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT         17 
/*!< It expresses how much 
                                                            estimated speed can exceed 
                                                            forced stator electrical 
                                                            without being considered wrong. 
                                                            In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT         15  
/*!< It expresses how much 
                                                             estimated speed can be below 
                                                             forced stator electrical 
                                                             without being considered wrong. 
                                                             In 1/16 of forced speed */                        
#define TRANSITION_DURATION            25  /* Switch over duration, ms */                                                                          
/******************************   ADDITIONAL FEATURES   **********************/

#define FW_VOLTAGE_REF                985 /*!<Vs reference, tenth                                                         of a percent */
#define FW_KP_GAIN                    8000   //3000 /*!< Default Kp gain */
#define FW_KI_GAIN                    5000 /*!< Default Ki gain */
#define FW_KPDIV                      32768      
                                                /*!< Kp gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through       
                                                algebrical right shifts to speed up PIs execution. 
                                                Only in this case this parameter specifies the 
                                                number of right shifts to be executed */
#define FW_KIDIV                      32768
                                                /*!< Ki gain divisor.If FULL_MISRA_C_COMPLIANCY
                                                is not defined the divisor is implemented through       
                                                algebrical right shifts to speed up PIs execution. 
                                                Only in this case this parameter specifies the 
                                                number of right shifts to be executed */
/*  Feed-forward parameters */
#define FEED_FORWARD_CURRENT_REG_ENABLING DISABLE
#define CONSTANT1_Q                    891
#define CONSTANT1_D                    891
#define CONSTANT2_QD                   1196

#define IQMAX                          32767 //12797

#endif /*__DRIVE_PARAMETERS_H*/

