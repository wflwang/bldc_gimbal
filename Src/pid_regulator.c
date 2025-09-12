/**
 ******************************************************************************
 * @file    pid_regulator.c
 * @author  MaxwellWang
 * @date  2025-8-13
 * @brief   This file provides firmware functions that implement the following features
 *          of the PID regulator component of the Motor Control SDK:
 *
 *           * proportional, integral and derivative computation funcions
 *           * read and write gain functions
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "pid_regulator.h"

//#include "mc_type.h"

/** @addtogroup MCSDK
 * @{
 */

/**
 * @defgroup PIDRegulator PID Regulator
 * @brief PID regulator component of the Motor Control SDK
 *
 * The PID regulator component implements the following control function:
 *
 * @f[
 * u(t) = K_{p} e(t) + K_{i} \int_0^t e(\tau) \,d\tau + K_{d} \frac{de(t)}{dt}
 * @f]
 *
 * with the proportional, integral and derivative gains expressed as rational numbers, with a gain and a divisor parameter :
 *
 * @f[
 * K_{p} = \frac{K_{pg}}{K_{pd}}
 * @f]
 * @f[
 * K_{i} = \frac{K_{ig}}{K_{id}}
 * @f]
 * @f[
 * K_{d} = \frac{K_{dg}}{K_{dd}}
 * @f]
 *
 *  Each of the gain and divisor parameters, @f$K_{{p}g}@f$, @f$K_{{i}g}@f$, @f$K_{{d}g}@f$, @f$K_{{p}d}@f$,
 * @f$K_{id}@f$, @f$K_{dd}@f$, can be set independently. via the PID_SetKP(), PID_SetKPDivisorPOW2(), PID_SetKI(),
 * PID_SetKIDivisorPOW2(), PID_SetKD()
 *
 * @{
 */

/**
 * @brief  It initializes the handle
 * @param  pHandle: handler of the current instance of the PID component
 * @retval None
 */
void PID_HandleInit( PID_Handle_t * pHandle )
{
  pHandle->hKpGain =  pHandle->hDefKpGain;
  pHandle->hKiGain =  pHandle->hDefKiGain;
  pHandle->hKdGain =  pHandle->hDefKdGain;
  pHandle->wIntegralTerm = 0x00000000UL;
  pHandle->wPrevProcessVarError = 0x00000000UL;
}

/**
 * @brief  It updates the Kp gain
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKpGain: new Kp gain
 * @retval None
 */
void PID_SetKP( PID_Handle_t * pHandle, int16_t hKpGain )
{
  pHandle->hKpGain = hKpGain;
}

/**
 * @brief  It updates the Ki gain
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKiGain: new Ki gain
 * @retval None
 */
void PID_SetKI( PID_Handle_t * pHandle, int16_t hKiGain )
{
  pHandle->hKiGain = hKiGain;
}

/**
 * @brief  It returns the Kp gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kp gain
 */
int16_t PID_GetKP( PID_Handle_t * pHandle )
{
  return ( pHandle->hKpGain );
}

/**
 * @brief  It returns the Ki gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Ki gain
 */
int16_t PID_GetKI( PID_Handle_t * pHandle )
{
  return ( pHandle->hKiGain );
}

/**
 * @brief  It returns the Default Kp gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval default Kp gain
 */
int16_t PID_GetDefaultKP( PID_Handle_t * pHandle )
{
  return ( pHandle->hDefKpGain );
}

/**
 * @brief  It returns the Default Ki gain of the passed PI object
 * @param  pHandle: handler of the current instance of the PID component
 * @retval default Ki gain
 */
int16_t PID_GetDefaultKI( PID_Handle_t * pHandle )
{
  return ( pHandle->hDefKiGain );
}

/**
 * @brief  It set a new value into the PI integral term
 * pHandle: handler of the current instance of the PID component
 * @param  wIntegralTermValue: new integral term value
 * @retval None
 */
void PID_SetIntegralTerm( PID_Handle_t * pHandle, int32_t wIntegralTermValue )
{
  pHandle->wIntegralTerm = wIntegralTermValue;

  return;
}

/**
 * @brief  It returns the Kp gain divisor
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kp gain divisor
 */
uint16_t PID_GetKPDivisor( PID_Handle_t * pHandle )
{
  return ( pHandle->hKpDivisor );
}

/**
 * @brief  It updates the Kp divisor
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKpDivisorPOW2: new Kp divisor expressed as power of 2
 * @retval None
 */
void PID_SetKPDivisorPOW2( PID_Handle_t * pHandle, uint16_t hKpDivisorPOW2 )
{
  pHandle->hKpDivisorPOW2 = hKpDivisorPOW2;
  pHandle->hKpDivisor = ( ( uint16_t )( 1u ) << hKpDivisorPOW2 );
}

/**
 * @brief  It returns the Ki gain divisor of the passed PI object
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Ki gain divisor
 */
uint16_t PID_GetKIDivisor( PID_Handle_t * pHandle )
{
  return ( pHandle->hKiDivisor );
}

/**
 * @brief  It updates the Ki divisor
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKiDivisorPOW2: new Ki divisor expressed as power of 2
 * @retval None
 */
void PID_SetKIDivisorPOW2( PID_Handle_t * pHandle, uint16_t hKiDivisorPOW2 )
{
  int32_t wKiDiv = ( ( int32_t )( 1u ) << hKiDivisorPOW2 );
  pHandle->hKiDivisorPOW2 = hKiDivisorPOW2;
  pHandle->hKiDivisor = ( uint16_t )( wKiDiv );
  PID_SetUpperIntegralTermLimit( pHandle, ( int32_t )INT16_MAX * wKiDiv );
  PID_SetLowerIntegralTermLimit( pHandle, ( int32_t ) - INT16_MAX * wKiDiv );
}

/**
 * @brief  It set a new value for lower integral term limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wLowerLimit: new lower integral term limit value
 * @retval None
 */
void PID_SetLowerIntegralTermLimit( PID_Handle_t * pHandle, int32_t wLowerLimit )
{
  pHandle->wLowerIntegralLimit = wLowerLimit;
}

/**
 * @brief  It set a new value for upper integral term limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wUpperLimit: new upper integral term limit value
 * @retval None
 */
void PID_SetUpperIntegralTermLimit( PID_Handle_t * pHandle, int32_t wUpperLimit )
{
  pHandle->wUpperIntegralLimit = wUpperLimit;
}

/**
 * @brief  It set a new value for lower output limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hLowerLimit: new lower output limit value
 * @retval None
 */
void PID_SetLowerOutputLimit( PID_Handle_t * pHandle, int16_t hLowerLimit )
{
  pHandle->hLowerOutputLimit = hLowerLimit;
}

/**
 * @brief  It set a new value for upper output limit
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hUpperLimit: new upper output limit value
 * @retval None
 */
void PID_SetUpperOutputLimit( PID_Handle_t * pHandle, int16_t hUpperLimit )
{
  pHandle->hUpperOutputLimit = hUpperLimit;
}

/**
 * @brief  It set a new value into the PID Previous error variable required to
 *         compute derivative term
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wPrevProcessVarError: New previous error variable
 * @retval None
 */
void PID_SetPrevError( PID_Handle_t * pHandle, int32_t wPrevProcessVarError )
{
  pHandle->wPrevProcessVarError = wPrevProcessVarError;
  return;
}

/**
 * @brief  It updates the Kd gain
 * @param  pHandle: handler of the current instance of the PID component
 * @param  hKdGain: new Kd gain
 * @retval None
 */
void PID_SetKD( PID_Handle_t * pHandle, int16_t hKdGain )
{
  pHandle->hKdGain = hKdGain;
}

/**
 * @brief  It returns the Kd gain
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kd gain
 */
int16_t PID_GetKD( PID_Handle_t * pHandle )
{
  return pHandle->hKdGain;
}

/**
 * @brief  It returns the Kd gain divisor of the PID object passed
 * @param  pHandle: handler of the current instance of the PID component
 * @retval Kd gain divisor
 */
uint16_t PID_GetKDDivisor( PID_Handle_t * pHandle )
{
  return ( pHandle->hKdDivisor );
}

/**
 * @brief Sets @f$K_{dd}@f$, the derivative divisor parameter of the PID component
 *
 * @param pHandle handle on the instance of the PID component to update
 * @param hKdDivisorPOW2
 */
void PID_SetKDDivisorPOW2( PID_Handle_t * pHandle, uint16_t hKdDivisorPOW2 );


#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
 * @brief  This function compute the output of a PI regulator sum of its
 *         proportional and integral terms
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wProcessVarError: current process variable error, intended as the reference
 *         value minus the present process variable value
 * @retval computed PI output
 */
int16_t PI_Controller( PID_Handle_t * pHandle, int32_t wProcessVarError )
{
  int32_t wProportional_Term, wIntegral_Term, wOutput_32, wIntegral_sum_temp;
  int32_t wDischarge = 0;
  int16_t hUpperOutputLimit = pHandle->hUpperOutputLimit;
  int16_t hLowerOutputLimit = pHandle->hLowerOutputLimit;

  /* Proportional term computation*/
  wProportional_Term = pHandle->hKpGain * wProcessVarError;

  /* Integral term computation */
  if ( pHandle->hKiGain == 0 )
  {
    pHandle->wIntegralTerm = 0;
  }
  else
  {
    //比例和积分方向不一致时候 积分清0
    //wIntegral_Term = pHandle->hKiGain * wProcessVarError;
    //积分方向一定和比例方向一致 不一致积分清0
    if(((pHandle->wIntegralTerm^wProcessVarError)&0x80000000)){
      //正负方向不一致 积分迅速减小
      pHandle->wIntegralTerm = 0; //积分清0
      //pHandle->wIntegralTerm = pHandle->wIntegralTerm>>1; // /4倍
    }
    //误差够小时候才积分
    //if(((wProcessVarError<920)&&(wProcessVarError>-920))){
    //  wIntegral_Term = pHandle->hKiGain * wProcessVarError;
      //wIntegral_Term  = 0;
    //}else{
    //  wIntegral_Term  = 0;
      //pHandle->wIntegralTerm = 0;
    //}
    wIntegral_sum_temp = pHandle->wIntegralTerm + wIntegral_Term;

    if ( wIntegral_sum_temp < 0 )
    {
      if ( pHandle->wIntegralTerm > 0 )
      {
        if ( wIntegral_Term > 0 )
        {
          wIntegral_sum_temp = INT32_MAX;
        }
      }
    }
    else
    {
      if ( pHandle->wIntegralTerm < 0 )
      {
        if ( wIntegral_Term < 0 )
        {
          wIntegral_sum_temp = -INT32_MAX;
        }
      }
    }

    if ( wIntegral_sum_temp > pHandle->wUpperIntegralLimit )
    {
      pHandle->wIntegralTerm = pHandle->wUpperIntegralLimit;
    }
    else if ( wIntegral_sum_temp < pHandle->wLowerIntegralLimit )
    {
      pHandle->wIntegralTerm = pHandle->wLowerIntegralLimit;
    }
    else
    {
      pHandle->wIntegralTerm = wIntegral_sum_temp;
    }
  }

#ifdef FULL_MISRA_C_COMPLIANCY
  wOutput_32 = ( wProportional_Term / ( int32_t )pHandle->hKpDivisor ) + ( pHandle->wIntegralTerm /
               ( int32_t )pHandle->hKiDivisor );
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
             that Cortex-M3 assembly instruction ASR (arithmetic shift right)
             is used by the compiler to perform the shifts (instead of LSR
             logical shift right)*/
  wOutput_32 = ( wProportional_Term >> pHandle->hKpDivisorPOW2 ) + ( pHandle->wIntegralTerm >> pHandle->hKiDivisorPOW2 );
#endif

  if ( wOutput_32 > hUpperOutputLimit )
  {

    wDischarge = hUpperOutputLimit - wOutput_32;
    wOutput_32 = hUpperOutputLimit;
  }
  else if ( wOutput_32 < hLowerOutputLimit )
  {

    wDischarge = hLowerOutputLimit - wOutput_32;
    wOutput_32 = hLowerOutputLimit;
  }
  else { /* Nothing to do here */ }

  pHandle->wIntegralTerm += wDischarge;

  return ( ( int16_t )( wOutput_32 ) );
}
#if 0
/**
 * @brief  This function compute the output of a PI regulator sum of its
 *         proportional and integral terms
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wProcessVarError: current process variable error, intended as the reference
 *         value minus the present process variable value
 * @retval computed PI output
 * 速度环 低速PI控制
 */
int32_t LowSpeedPI_Controller( PID_Handle_t * pHandle, int32_t wProcessVarError )
{
  int64_t wProportional_Term, wIntegral_Term, wOutput_32, wIntegral_sum_temp;
  //int32_t wProportional_Term, wIntegral_Term, wOutput_32, wIntegral_sum_temp;
  //int32_t wDischarge = 0;
  int16_t hUpperOutputLimit = pHandle->hUpperOutputLimit;
  int16_t hLowerOutputLimit = pHandle->hLowerOutputLimit;

  /* Proportional term computation*/
  wProportional_Term = pHandle->hKpGain * wProcessVarError;

  /* Integral term computation */
  if ( pHandle->hKiGain == 0 )
  {
    pHandle->wIntegralTerm = 0;
  }
  else
  {
    wIntegral_Term = (int64_t)pHandle->hKiGain * wProcessVarError;
    //if(((pHandle->wIntegralTerm^wProcessVarError)&0x80000000)&&((pHandle->wIntegralTerm>30)||(pHandle->wIntegralTerm<-30))){
      //正负方向不一致 积分迅速减小
      //pHandle->wIntegralTerm = pHandle->wIntegralTerm>>2; // /4倍
    //}
    wIntegral_sum_temp = (int64_t)pHandle->wIntegralTerm + wIntegral_Term; //本次积分和
    if(wIntegral_sum_temp>INT32_MAX){
      wIntegral_sum_temp = INT64_MAX;
    }else if(wIntegral_sum_temp<-INT32_MAX){
      wIntegral_sum_temp = -INT64_MAX;
    }

    //if ( wIntegral_sum_temp > pHandle->wUpperIntegralLimit )
    //{
    //  pHandle->wIntegralTerm = pHandle->wUpperIntegralLimit;
    //}
    //else if ( wIntegral_sum_temp < pHandle->wLowerIntegralLimit )
    //{
    //  pHandle->wIntegralTerm = pHandle->wLowerIntegralLimit;
    //}
    //else
    //{
    //  pHandle->wIntegralTerm = wIntegral_sum_temp;
    //}
  }

#ifdef FULL_MISRA_C_COMPLIANCY
  wOutput_32 = ( wProportional_Term / ( int32_t )pHandle->hKpDivisor ) + ( pHandle->wIntegralTerm /
               ( int32_t )pHandle->hKiDivisor );
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
             that Cortex-M3 assembly instruction ASR (arithmetic shift right)
             is used by the compiler to perform the shifts (instead of LSR
             logical shift right)*/
  wOutput_32 = ( wProportional_Term >> pHandle->hKpDivisorPOW2 ) + ( pHandle->wIntegralTerm >> pHandle->hKiDivisorPOW2 );
#endif

  if ( wOutput_32 > INT32_MAX )
  {

    wDischarge = INT32_MAX - wOutput_32;
    wOutput_32 = INT32_MAX;
  }
  else if ( wOutput_32 < -INT32_MAX )
  {

    wDischarge = -INT32_MAX - wOutput_32;
    wOutput_32 = -INT32_MAX;
  }
  else { /* Nothing to do here */ }

  //pHandle->wIntegralTerm += wDischarge;

  return ((uint32_t)( wOutput_32 ) );
}
#endif
#if 1
#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
 * @brief  This function compute the output of a PID regulator sum of its
 *         proportional, integral and derivative terms
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wProcessVarError: current process variable error, intended as the
 *         reference value minus the present process variable value
 * @retval PID computed output
 * 角度误差PID控制 D进行一阶滤波,抑制噪声 有D时候刚开始小 慢慢逼近真实误差
 * 小于积分阈值才开始积分 大误差不对积分进行积分
 */
//int16_t PosPID_Controller( PID_Handle_t * pHandle, int32_t wProcessVarError ){
//}


/**
 * @brief  This function compute the output of a PID regulator sum of its
 *         proportional, integral and derivative terms
 * @param  pHandle: handler of the current instance of the PID component
 * @param  wProcessVarError: current process variable error, intended as the
 *         reference value minus the present process variable value
 * @retval PID computed output
 */

int16_t PID_Controller( PID_Handle_t * pHandle, int32_t wProcessVarError )
{
  int32_t wDifferential_Term;
  int32_t wDeltaError;
  int32_t wTemp_output;

  if ( pHandle->hKdGain != 0 ) /* derivative terms not used */
  {
    //本次误差
    wDeltaError = wProcessVarError - pHandle->wPrevProcessVarError;
    pHandle->wPreDeltaErr += wDeltaError;
    pHandle->wPreDeltaErr >>= 1;
    wDifferential_Term = pHandle->hKdGain * pHandle->wPreDeltaErr;  //wDeltaError;  //pHandle->wPreDeltaErr;  //

#ifdef FULL_MISRA_C_COMPLIANCY
    wDifferential_Term /= ( int32_t )pHandle->hKdDivisor;
#else
    /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right)
    is used by the compiler to perform the shifts (instead of LSR
    logical shift right)*/
    wDifferential_Term >>= pHandle->hKdDivisorPOW2;
#endif
    //if((wProcessVarError^wDeltaError)&0x80000000){
    //  pHandle->wIntegralTerm = 0;
    //}

    pHandle->wPrevProcessVarError = wProcessVarError;
    //if(((wProcessVarError^wDifferential_Term)&0x80000000)&&((wDifferential_Term>0x60)||(wDifferential_Term<-0x60))){
    //比例和微分方向不一致
    if(((wProcessVarError^wDifferential_Term)&0x80000000)){
      //微分和比例方向不一致时候积分减半
    //  pHandle->wIntegralTerm = (pHandle->wIntegralTerm *180)>>8;
      wTemp_output = (int32_t)PI_Controller( pHandle, wProcessVarError ) + wDifferential_Term;
    }else{
      //方向一致
      wTemp_output = (int32_t)PI_Controller( pHandle, wProcessVarError );
    }
    //wTemp_output = PI_Controller( pHandle, wProcessVarError ) + wDifferential_Term;
    //比例误差和输出扭力控制方向不一致 说明超调了
    //if((wProcessVarError^wTemp_output)&0x80000000){
      //int32_t tmp = wTemp_output;
      //if(tmp<0)
      //  tmp = - tmp;
      //if(tmp>0x4800){
      //  pHandle->wIntegralTerm = 0;
      //}else if(tmp>0x3050){
      //  pHandle->wIntegralTerm = (pHandle->wIntegralTerm)>>3;
      //}else if(tmp>0x2000){
      //  pHandle->wIntegralTerm = (pHandle->wIntegralTerm)>>2;
      //}else if(tmp>0x1250){
      //  pHandle->wIntegralTerm = (pHandle->wIntegralTerm)>>1;
      //}else if(tmp>0x650){
      //  pHandle->wIntegralTerm = (pHandle->wIntegralTerm*3)>>2;
      ////}else{
      ////  wTemp_output = 0;
      //}
      //pHandle->wIntegralTerm =0;
      //wTemp_output = 0;
    //}

    if ( (int16_t)wTemp_output > pHandle->hUpperOutputLimit )
    {
      wTemp_output = pHandle->hUpperOutputLimit;
    }
    else if ( (int16_t)wTemp_output < pHandle->hLowerOutputLimit )
    {
      wTemp_output = pHandle->hLowerOutputLimit;
    }
    else
    {}
  }
  else
  {
    wTemp_output = PI_Controller( pHandle, wProcessVarError );
  }
  return ( ( int16_t ) wTemp_output );
}
#endif
/**
 * @}
 */

/**
 * @}
 */

