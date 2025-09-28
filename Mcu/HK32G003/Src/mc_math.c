/**
  ******************************************************************************
  * @file    mc_math.c
  * @author  Motor Control SDK Team, HK AE Teams
  * @brief   This file provides mathematics functions useful for and specific to
  *          Motor Control.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mc_math.h"
//#include "mc_type.h"

/** @addtogroup MCSDK
  * @{
  */

/** @defgroup MC_Math Motor Control Math functions
  * @brief Motor Control Mathematic functions of the Motor Control SDK
  *
  * @todo Document the Motor Control Math "module".
  *
  * @{
  */

/* Private macro -------------------------------------------------------------*/
//0-90
#define SIN_COS_TABLE {\
    0x0000,0x00C9,0x0192,0x025B,0x0324,0x03ED,0x04B6,0x057F,\
    0x0648,0x0711,0x07D9,0x08A2,0x096A,0x0A33,0x0AFB,0x0BC4,\
    0x0C8C,0x0D54,0x0E1C,0x0EE3,0x0FAB,0x1072,0x113A,0x1201,\
    0x12C8,0x138F,0x1455,0x151C,0x15E2,0x16A8,0x176E,0x1833,\
    0x18F9,0x19BE,0x1A82,0x1B47,0x1C0B,0x1CCF,0x1D93,0x1E57,\
    0x1F1A,0x1FDD,0x209F,0x2161,0x2223,0x22E5,0x23A6,0x2467,\
    0x2528,0x25E8,0x26A8,0x2767,0x2826,0x28E5,0x29A3,0x2A61,\
    0x2B1F,0x2BDC,0x2C99,0x2D55,0x2E11,0x2ECC,0x2F87,0x3041,\
    0x30FB,0x31B5,0x326E,0x3326,0x33DF,0x3496,0x354D,0x3604,\
    0x36BA,0x376F,0x3824,0x38D9,0x398C,0x3A40,0x3AF2,0x3BA5,\
    0x3C56,0x3D07,0x3DB8,0x3E68,0x3F17,0x3FC5,0x4073,0x4121,\
    0x41CE,0x427A,0x4325,0x43D0,0x447A,0x4524,0x45CD,0x4675,\
    0x471C,0x47C3,0x4869,0x490F,0x49B4,0x4A58,0x4AFB,0x4B9D,\
    0x4C3F,0x4CE0,0x4D81,0x4E20,0x4EBF,0x4F5D,0x4FFB,0x5097,\
    0x5133,0x51CE,0x5268,0x5302,0x539B,0x5432,0x54C9,0x5560,\
    0x55F5,0x568A,0x571D,0x57B0,0x5842,0x58D3,0x5964,0x59F3,\
    0x5A82,0x5B0F,0x5B9C,0x5C28,0x5CB3,0x5D3E,0x5DC7,0x5E4F,\
    0x5ED7,0x5F5D,0x5FE3,0x6068,0x60EB,0x616E,0x61F0,0x6271,\
    0x62F1,0x6370,0x63EE,0x646C,0x64E8,0x6563,0x65DD,0x6656,\
    0x66CF,0x6746,0x67BC,0x6832,0x68A6,0x6919,0x698B,0x69FD,\
    0x6A6D,0x6ADC,0x6B4A,0x6BB7,0x6C23,0x6C8E,0x6CF8,0x6D61,\
    0x6DC9,0x6E30,0x6E96,0x6EFB,0x6F5E,0x6FC1,0x7022,0x7083,\
    0x70E2,0x7140,0x719D,0x71F9,0x7254,0x72AE,0x7307,0x735E,\
    0x73B5,0x740A,0x745F,0x74B2,0x7504,0x7555,0x75A5,0x75F3,\
    0x7641,0x768D,0x76D8,0x7722,0x776B,0x77B3,0x77FA,0x783F,\
    0x7884,0x78C7,0x7909,0x794A,0x7989,0x79C8,0x7A05,0x7A41,\
    0x7A7C,0x7AB6,0x7AEE,0x7B26,0x7B5C,0x7B91,0x7BC5,0x7BF8,\
    0x7C29,0x7C59,0x7C88,0x7CB6,0x7CE3,0x7D0E,0x7D39,0x7D62,\
    0x7D89,0x7DB0,0x7DD5,0x7DFA,0x7E1D,0x7E3E,0x7E5F,0x7E7E,\
    0x7E9C,0x7EB9,0x7ED5,0x7EEF,0x7F09,0x7F21,0x7F37,0x7F4D,\
    0x7F61,0x7F74,0x7F86,0x7F97,0x7FA6,0x7FB4,0x7FC1,0x7FCD,\
    0x7FD8,0x7FE1,0x7FE9,0x7FF0,0x7FF5,0x7FF9,0x7FFD,0x7FFE}
//tan 0 - 45  第一象限0-45  倒数 45-90
//第二象限 倒数 90-135 正数: 135-180
//第3象限 倒数 225-270 正数: 180-225
//第4象限 倒数 270-315 正数: 315-360
#define tan_Table { \
0x0064, 0x012d, 0x01f6, 0x02bf, 0x0388, 0x0451, 0x051b, 0x05e4, \
0x06ad, 0x0776, 0x083f, 0x0909, 0x09d2, 0x0a9b, 0x0b65, 0x0c2e, \
0x0cf8, 0x0dc1, 0x0e8b, 0x0f55, 0x101f, 0x10e9, 0x11b3, 0x127d, \
0x1347, 0x1411, 0x14db, 0x15a6, 0x1670, 0x173b, 0x1806, 0x18d1, \
0x199c, 0x1a67, 0x1b32, 0x1bfe, 0x1cc9, 0x1d95, 0x1e61, 0x1f2d, \
0x1ff9, 0x20c5, 0x2191, 0x225e, 0x232b, 0x23f8, 0x24c5, 0x2592, \
0x2660, 0x272d, 0x27fb, 0x28c9, 0x2997, 0x2a66, 0x2b35, 0x2c04, \
0x2cd3, 0x2da2, 0x2e72, 0x2f41, 0x3011, 0x30e2, 0x31b2, 0x3283, \
0x3354, 0x3425, 0x34f7, 0x35c9, 0x369b, 0x376d, 0x3840, 0x3913, \
0x39e6, 0x3ab9, 0x3b8d, 0x3c61, 0x3d36, 0x3e0a, 0x3edf, 0x3fb5, \
0x408a, 0x4160, 0x4237, 0x430d, 0x43e4, 0x44bc, 0x4593, 0x466c, \
0x4744, 0x481d, 0x48f6, 0x49d0, 0x4aaa, 0x4b84, 0x4c5f, 0x4d3a, \
0x4e15, 0x4ef1, 0x4fce, 0x50ab, 0x5188, 0x5266, 0x5344, 0x5422, \
0x5501, 0x55e1, 0x56c1, 0x57a1, 0x5882, 0x5963, 0x5a45, 0x5b27, \
0x5c0a, 0x5ced, 0x5dd1, 0x5eb6, 0x5f9a, 0x6080, 0x6166, 0x624c, \
0x6333, 0x641b, 0x6503, 0x65eb, 0x66d5, 0x67be, 0x68a9, 0x6994, \
0x6a7f, 0x6b6b, 0x6c58, 0x6d46, 0x6e34, 0x6f22, 0x7011, 0x7101, \
0x71f2, 0x72e3, 0x73d5, 0x74c8, 0x75bb, 0x76af, 0x77a3, 0x7899, \
0x798f, 0x7a86, 0x7b7d, 0x7c75, 0x7d6e, 0x7e68, 0x7f62, 0x805e, \
0x815a, 0x8256, 0x8354, 0x8452, 0x8552, 0x8652, 0x8752, 0x8854, \
0x8957, 0x8a5a, 0x8b5e, 0x8c63, 0x8d69, 0x8e70, 0x8f78, 0x9081, \
0x918a, 0x9295, 0x93a0, 0x94ac, 0x95ba, 0x96c8, 0x97d7, 0x98e8, \
0x99f9, 0x9b0b, 0x9c1f, 0x9d33, 0x9e48, 0x9f5f, 0xa076, 0xa18f, \
0xa2a9, 0xa3c3, 0xa4df, 0xa5fc, 0xa71b, 0xa83a, 0xa95a, 0xaa7c, \
0xab9f, 0xacc3, 0xade8, 0xaf0f, 0xb036, 0xb15f, 0xb289, 0xb3b5, \
0xb4e2, 0xb610, 0xb73f, 0xb870, 0xb9a2, 0xbad6, 0xbc0a, 0xbd41, \
0xbe78, 0xbfb1, 0xc0ec, 0xc228, 0xc365, 0xc4a4, 0xc5e5, 0xc727, \
0xc86a, 0xc9af, 0xcaf6, 0xcc3e, 0xcd88, 0xced4, 0xd021, 0xd16f, \
0xd2c0, 0xd412, 0xd566, 0xd6bc, 0xd813, 0xd96c, 0xdac7, 0xdc24, \
0xdd83, 0xdee3, 0xe046, 0xe1aa, 0xe310, 0xe479, 0xe5e3, 0xe74f, \
0xe8bd, 0xea2e, 0xeba0, 0xed14, 0xee8b, 0xf004, 0xf17e, 0xf2fc, \
0xf47b, 0xf5fc, 0xf780, 0xf906, 0xfa8f, 0xfc1a, 0xfda7, 0xff37, \
}

//#define tan_Table { \
0x0000, 0x00c9, 0x0192, 0x025b, 0x0324, 0x03ed, 0x04b6, 0x057f, \
0x0648, 0x0712, 0x07db, 0x08a4, 0x096d, 0x0a37, 0x0b00, 0x0bca, \
0x0c93, 0x0d5d, 0x0e26, 0x0ef0, 0x0fba, 0x1084, 0x114e, 0x1218, \
0x12e2, 0x13ac, 0x1476, 0x1541, 0x160b, 0x16d6, 0x17a0, 0x186b, \
0x1936, 0x1a01, 0x1acd, 0x1b98, 0x1c63, 0x1d2f, 0x1dfb, 0x1ec7, \
0x1f93, 0x205f, 0x212b, 0x21f8, 0x22c4, 0x2391, 0x245e, 0x252b, \
0x25f9, 0x26c6, 0x2794, 0x2862, 0x2930, 0x29ff, 0x2acd, 0x2b9c, \
0x2c6b, 0x2d3a, 0x2e0a, 0x2ed9, 0x2fa9, 0x3079, 0x314a, 0x321b, \
0x32eb, 0x33bd, 0x348e, 0x3560, 0x3632, 0x3704, 0x37d6, 0x38a9, \
0x397c, 0x3a4f, 0x3b23, 0x3bf7, 0x3ccb, 0x3da0, 0x3e75, 0x3f4a, \
0x401f, 0x40f5, 0x41cb, 0x42a2, 0x4379, 0x4450, 0x4528, 0x45ff, \
0x46d8, 0x47b0, 0x4889, 0x4963, 0x4a3d, 0x4b17, 0x4bf1, 0x4ccc, \
0x4da8, 0x4e83, 0x4f60, 0x503c, 0x5119, 0x51f7, 0x52d5, 0x53b3, \
0x5492, 0x5571, 0x5651, 0x5731, 0x5811, 0x58f2, 0x59d4, 0x5ab6, \
0x5b99, 0x5c7c, 0x5d5f, 0x5e43, 0x5f28, 0x600d, 0x60f3, 0x61d9, \
0x62c0, 0x63a7, 0x648f, 0x6577, 0x6660, 0x6749, 0x6833, 0x691e, \
0x6a09, 0x6af5, 0x6be2, 0x6ccf, 0x6dbc, 0x6eab, 0x6f9a, 0x7089, \
0x717a, 0x726b, 0x735c, 0x744e, 0x7541, 0x7635, 0x7729, 0x781e, \
0x7914, 0x7a0a, 0x7b01, 0x7bf9, 0x7cf2, 0x7deb, 0x7ee5, 0x7fe0, \
0x80dc, 0x81d8, 0x82d5, 0x83d3, 0x84d2, 0x85d1, 0x86d2, 0x87d3, \
0x88d5, 0x89d8, 0x8adc, 0x8be1, 0x8ce6, 0x8ded, 0x8ef4, 0x8ffc, \
0x9105, 0x920f, 0x931a, 0x9426, 0x9533, 0x9641, 0x9750, 0x985f, \
0x9970, 0x9a82, 0x9b95, 0x9ca9, 0x9dbe, 0x9ed4, 0x9feb, 0xa103, \
0xa21c, 0xa336, 0xa451, 0xa56e, 0xa68b, 0xa7aa, 0xa8ca, 0xa9eb, \
0xab0d, 0xac31, 0xad55, 0xae7b, 0xafa2, 0xb0cb, 0xb1f4, 0xb31f, \
0xb44b, 0xb579, 0xb6a7, 0xb7d7, 0xb909, 0xba3c, 0xbb70, 0xbca5, \
0xbddc, 0xbf15, 0xc04e, 0xc18a, 0xc2c6, 0xc405, 0xc544, 0xc686, \
0xc7c8, 0xc90d, 0xca52, 0xcb9a, 0xcce3, 0xce2e, 0xcf7a, 0xd0c8, \
0xd218, 0xd369, 0xd4bc, 0xd611, 0xd767, 0xd8bf, 0xda1a, 0xdb75, \
0xdcd3, 0xde33, 0xdf94, 0xe0f8, 0xe25d, 0xe3c4, 0xe52d, 0xe699, \
0xe806, 0xe975, 0xeae6, 0xec5a, 0xedcf, 0xef47, 0xf0c1, 0xf23d, \
0xf3bb, 0xf53b, 0xf6be, 0xf843, 0xf9ca, 0xfb54, 0xfce0, 0xfe6f, \
}
//0 - 45 /256 half  <now is real value max 127/128 -> 1
// tan(PI/4*?/128) => 0-1 => y/x * 65536
#define tan_table_half256 { \
0x00c9, 0x025b, 0x03ed, 0x057f, 0x0712, 0x08a4, 0x0a37, 0x0bca, \
0x0d5d, 0x0ef0, 0x1084, 0x1218, 0x13ac, 0x1541, 0x16d6, 0x186b, \
0x1a01, 0x1b98, 0x1d2f, 0x1ec7, 0x205f, 0x21f8, 0x2391, 0x252b, \
0x26c6, 0x2862, 0x29ff, 0x2b9c, 0x2d3a, 0x2ed9, 0x3079, 0x321b, \
0x33bd, 0x3560, 0x3704, 0x38a9, 0x3a4f, 0x3bf7, 0x3da0, 0x3f4a, \
0x40f5, 0x42a2, 0x4450, 0x45ff, 0x47b0, 0x4963, 0x4b17, 0x4ccc, \
0x4e83, 0x503c, 0x51f7, 0x53b3, 0x5571, 0x5731, 0x58f2, 0x5ab6, \
0x5c7c, 0x5e43, 0x600d, 0x61d9, 0x63a7, 0x6577, 0x6749, 0x691e, \
0x6af5, 0x6ccf, 0x6eab, 0x7089, 0x726b, 0x744e, 0x7635, 0x781e, \
0x7a0a, 0x7bf9, 0x7deb, 0x7fe0, 0x81d8, 0x83d3, 0x85d1, 0x87d3, \
0x89d8, 0x8be1, 0x8ded, 0x8ffc, 0x920f, 0x9426, 0x9641, 0x985f, \
0x9a82, 0x9ca9, 0x9ed4, 0xa103, 0xa336, 0xa56e, 0xa7aa, 0xa9eb, \
0xac31, 0xae7b, 0xb0cb, 0xb31f, 0xb579, 0xb7d7, 0xba3c, 0xbca5, \
0xbf15, 0xc18a, 0xc405, 0xc686, 0xc90d, 0xcb9a, 0xce2e, 0xd0c8, \
0xd369, 0xd611, 0xd8bf, 0xdb75, 0xde33, 0xe0f8, 0xe3c4, 0xe699, \
0xe975, 0xec5a, 0xef47, 0xf23d, 0xf53b, 0xf843, 0xfb54, 0xfe6f, \
};

#define SIN_MASK        0x0300u
#define U0_90           0x0200u
#define U90_180         0x0300u
#define U180_270        0x0000u
#define U270_360        0x0100u
#define divSQRT_3 (int32_t)0x49E6    /* 1/sqrt(3) in q1.15 format=0.5773315*/

/* Private variables ---------------------------------------------------------*/
const int16_t hSin_Cos_Table[256] = SIN_COS_TABLE;
//const uint16_t hTan_Table[128] = tan_table_half256;  //0-45
const uint16_t hTan_Table[256] = tan_Table;  //0-45

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
#if 0
/**
  * @brief  This function transforms stator currents Ia and qIb (which are
  *         directed along axes each displaced by 120 degrees) into currents
  *         Ialpha and Ibeta in a stationary qd reference frame.
  *                               Ialpha = Ia
  *                       Ibeta = -(2*Ib+Ia)/sqrt(3)
  * @param  Curr_Input: stator current Ia and Ib in Curr_Components format
  * @retval Stator current Ialpha and Ibeta in Curr_Components format
  */
Curr_Components MCM_Clarke( Curr_Components Curr_Input )
{
  Curr_Components Curr_Output;

  int32_t qIa_divSQRT3_tmp, qIb_divSQRT3_tmp ;
  int32_t wIbeta_tmp;
  int16_t hIbeta_tmp;

  /* qIalpha = qIas*/
  Curr_Output.qI_Component1 = Curr_Input.qI_Component1;

  qIa_divSQRT3_tmp = divSQRT_3 * ( int32_t )Curr_Input.qI_Component1;

  qIb_divSQRT3_tmp = divSQRT_3 * ( int32_t )Curr_Input.qI_Component2;

  /*qIbeta = -(2*qIbs+qIas)/sqrt(3)*/
#ifdef FULL_MISRA_C_COMPLIANCY
  wIbeta_tmp = ( -( qIa_divSQRT3_tmp ) - ( qIb_divSQRT3_tmp ) -
                 ( qIb_divSQRT3_tmp ) ) / 32768;
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */

  wIbeta_tmp = ( -( qIa_divSQRT3_tmp ) - ( qIb_divSQRT3_tmp ) -
                 ( qIb_divSQRT3_tmp ) ) >> 15;
#endif

  /* Check saturation of Ibeta */
  if ( wIbeta_tmp > INT16_MAX )
  {
    hIbeta_tmp = INT16_MAX;
  }
  else if ( wIbeta_tmp < ( -32768 ) )
  {
    hIbeta_tmp = ( -32768 );
  }
  else
  {
    hIbeta_tmp = ( int16_t )( wIbeta_tmp );
  }

  Curr_Output.qI_Component2 = hIbeta_tmp;

  if ( Curr_Output.qI_Component2 == ( int16_t )( -32768 ) )
  {
    Curr_Output.qI_Component2 = -32767;
  }

  return ( Curr_Output );
}
#endif

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif

#if 0
/**
  * @brief  This function transforms stator currents Ialpha and Ibeta, which
  *         belong to a stationary qd reference frame, to a rotor flux
  *         synchronous reference frame (properly oriented), so as Iq and Id.
  *                   Id= qIalpha *sin(theta)+qIbeta *cos(Theta)
  *                   Iq=qIalpha *cos(Theta)-qIbeta *sin(Theta)
  * @param  Curr_Input: stator current Ialpha and Ibeta in Curr_Components format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator current Iq and Id in Curr_Components format
  */
Curr_Components MCM_Park( Curr_Components Curr_Input, int16_t Theta )
{
  Curr_Components Curr_Output;
  int32_t qId_tmp_1, qId_tmp_2, qIq_tmp_1, qIq_tmp_2;
  Trig_Components Local_Vector_Components;
  int32_t wIqd_tmp;
  int16_t hIqd_tmp;

  Local_Vector_Components = MCM_Trig_Functions( Theta );

  /*No overflow guaranteed*/
  qIq_tmp_1 = Curr_Input.qI_Component1 * ( int32_t )Local_Vector_Components.hCos;

  /*No overflow guaranteed*/
  qIq_tmp_2 = Curr_Input.qI_Component2 * ( int32_t )Local_Vector_Components.hSin;

  /*Iq component in Q1.15 Format */
#ifdef FULL_MISRA_C_COMPLIANCY
  wIqd_tmp = ( qIq_tmp_1 - qIq_tmp_2 ) / 32768;
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  wIqd_tmp = ( qIq_tmp_1 - qIq_tmp_2 ) >> 15;
#endif

  /* Check saturation of Iq */
  if ( wIqd_tmp > INT16_MAX )
  {
    hIqd_tmp = INT16_MAX;
  }
  else if ( wIqd_tmp < ( -32768 ) )
  {
    hIqd_tmp = ( -32768 );
  }
  else
  {
    hIqd_tmp = ( int16_t )( wIqd_tmp );
  }

  Curr_Output.qI_Component1 = hIqd_tmp;

  if ( Curr_Output.qI_Component1 == ( int16_t )( -32768 ) )
  {
    Curr_Output.qI_Component1 = -32767;
  }

  /*No overflow guaranteed*/
  qId_tmp_1 = Curr_Input.qI_Component1 * ( int32_t )Local_Vector_Components.hSin;

  /*No overflow guaranteed*/
  qId_tmp_2 = Curr_Input.qI_Component2 * ( int32_t )Local_Vector_Components.hCos;

  /*Id component in Q1.15 Format */
#ifdef FULL_MISRA_C_COMPLIANCY
  wIqd_tmp = ( qId_tmp_1 + qId_tmp_2 ) / 32768;
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  wIqd_tmp = ( qId_tmp_1 + qId_tmp_2 ) >> 15;
#endif

  /* Check saturation of Id */
  if ( wIqd_tmp > INT16_MAX )
  {
    hIqd_tmp = INT16_MAX;
  }
  else if ( wIqd_tmp < ( -32768 ) )
  {
    hIqd_tmp = ( -32768 );
  }
  else
  {
    hIqd_tmp = ( int16_t )( wIqd_tmp );
  }

  Curr_Output.qI_Component2 = hIqd_tmp;

  if ( Curr_Output.qI_Component2 == ( int16_t )( -32768 ) )
  {
    Curr_Output.qI_Component2 = -32767;
  }

  return ( Curr_Output );
}
#endif

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function transforms stator voltage qVq and qVd, that belong to a rotor flux synchronous rotating frame, to a stationary reference frame, so as to obtain qValpha and qVbeta:
  *                  Valfa= Vq*Cos(theta)+ Vd*Sin(theta)
  *                  Vbeta=-Vq*Sin(theta)+ Vd*Cos(theta)
  * @param  Curr_Input: stator voltage Vq and Vd in Volt_Components format
  * @param  Theta: rotating frame angular position in q1.15 format
  * @retval Stator voltage Valpha and Vbeta in Volt_Components format
  */
Volt_Components MCM_Rev_Park( Volt_Components Volt_Input, int16_t Theta )
{
  int32_t qValpha_tmp1, qValpha_tmp2, qVbeta_tmp1, qVbeta_tmp2;
  Trig_Components Local_Vector_Components;
  Volt_Components Volt_Output;

  Local_Vector_Components = MCM_Trig_Functions( Theta );

  /*No overflow guaranteed*/
  qValpha_tmp1 = Volt_Input.qV_Component1 * ( int32_t )Local_Vector_Components.hCos;
  qValpha_tmp2 = Volt_Input.qV_Component2 * ( int32_t )Local_Vector_Components.hSin;

#ifdef FULL_MISRA_C_COMPLIANCY
  Volt_Output.qV_Component1 = ( int16_t )( ( ( qValpha_tmp1 ) + ( qValpha_tmp2 ) ) / 32768 );
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  Volt_Output.qV_Component1 = ( int16_t )( ( ( qValpha_tmp1 ) + ( qValpha_tmp2 ) ) >> 15 );
#endif

  qVbeta_tmp1 = Volt_Input.qV_Component1 * ( int32_t )Local_Vector_Components.hSin;
  qVbeta_tmp2 = Volt_Input.qV_Component2 * ( int32_t )Local_Vector_Components.hCos;

#ifdef FULL_MISRA_C_COMPLIANCY
  Volt_Output.qV_Component2 = ( int16_t )( ( qVbeta_tmp2 - qVbeta_tmp1 ) / 32768 );
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
  that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
  the compiler to perform the shift (instead of LSR logical shift right) */
  Volt_Output.qV_Component2 = ( int16_t )( ( qVbeta_tmp2 - qVbeta_tmp1 ) >> 15 );
#endif

  return ( Volt_Output );
}

/**
  * @brief  Converts input voltages @f$ V_{\alpha} @f$ and @f$ V_{\beta} @f$ into PWM duty cycles
  *         and feed them to the inverter.
  * @param  pHandle handler on the target PWMC component.
  * @param  Valfa_beta Voltage Components expressed in the @f$(\alpha, \beta)@f$ reference frame
  *
  * This function computes the the time during which the transistors of each phase are to be switched on in
  * a PWM cycle in order to achieve the reference phase voltage set by @p Valfa_beta. Then, the function
  * programs the resulting duty cycles in the related timer channels. It also sets the phase current
  * sampling point for the next PWM cycle accordingly.
  *
  * This function is used in the FOC frequency loop and needs to complete before the next PWM cycle starts
  * so that the duty cycles it computes can be taken into account. Failing to do so (for instance because
  * the PWM Frequency is too high) results in the functions returning #MC_FOC_DURATION which entails a
  * Motor Control Fault that stops the motor.
  *
  * @retval Returns #MC_NO_ERROR if no error occurred or #MC_FOC_DURATION if the duty cycles were
  *         set too late for being taken into account in the next PWM cycle.
  */
void PWMC_SetPhaseVoltage( PWMC_Handle *pHandle, Volt_Components Valfa_beta )
{
  int32_t wX, wY, wZ, wUAlpha, wUBeta, wTimePhA, wTimePhB, wTimePhC;
  //PWMC_SetSampPointSectX_Cb_t pSetADCSamplingPoint;

  wUAlpha = Valfa_beta.qV_Component1 * ( int32_t )(pHandle->hT_Sqrt3);
  //wUBeta = -( Valfa_beta.qV_Component2 * ( int32_t )( pHandle->hPWMperiod ) ) * 2;
  wUBeta = -( Valfa_beta.qV_Component2 * ( int32_t )( pHandle->hPWMperiod ) ) << 1;

  wX = wUBeta;
  wY = ( wUBeta + wUAlpha ) / 2;
  wZ = ( wUBeta - wUAlpha ) / 2;

  /* Sector calculation from wX, wY, wZ */
  if ( wY < 0 )
  {
    if ( wZ < 0 )
    {
      //pHandle->hSector = SECTOR_5;
      //wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhA = (( int32_t )( pHandle->hPWMperiod ) >>2) + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhB = wTimePhA + wZ / 131072;
      wTimePhC = wTimePhA - wY / 131072;
      //pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect5;
    }
    else /* wZ >= 0 */
      if ( wX <= 0 )
      {
        //pHandle->hSector = SECTOR_4;
        //wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhA = (( int32_t )( pHandle->hPWMperiod ) >>2) + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhB = wTimePhA + wZ / 131072;
        wTimePhC = wTimePhB - wX / 131072;
        //pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect4;
      }
      else /* wX > 0 */
      {
        //pHandle->hSector = SECTOR_3;
        //wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhA = (( int32_t )( pHandle->hPWMperiod ) >>2) + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;
        //pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect3;
      }
  }
  else /* wY > 0 */
  {
    if ( wZ >= 0 )
    {
      //pHandle->hSector = SECTOR_2;
      //wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhA = (( int32_t )( pHandle->hPWMperiod ) >>2) + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhB = wTimePhA + wZ / 131072;
      wTimePhC = wTimePhA - wY / 131072;
      //pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect2;
    }
    else /* wZ < 0 */
      if ( wX <= 0 )
      {
        //pHandle->hSector = SECTOR_6;
        //wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhA = (( int32_t )( pHandle->hPWMperiod ) >>2) + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;
        //pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect6;
      }
      else /* wX > 0 */
      {
        //pHandle->hSector = SECTOR_1;
        //wTimePhA = ( int32_t )( pHandle->hPWMperiod ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhA = (( int32_t )( pHandle->hPWMperiod ) >>2) + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhB = wTimePhA + wZ / 131072;
        wTimePhC = wTimePhB - wX / 131072;
        //pSetADCSamplingPoint = pHandle->pFctSetADCSampPointSect1;
      }
  }

  pHandle->TIMx->CCR1 = ( uint16_t )wTimePhA;
  pHandle->TIMx->CCR2 = ( uint16_t )wTimePhB;
  pHandle->TIMx->CCR3 = ( uint16_t )wTimePhC;
  //__TIM_OC_DisablePreload( pHandle->TIMx, TIM_CHANNEL_CH4 );
  //TIM_OC_SetCompareCH4 ( pHandle->TIMx, 0xFFFFu );
  //__TIM_OC_EnablePreload( pHandle->TIMx, TIM_CHANNEL_CH4 );
  //TIM_OC_SetCompareCH4 ( pHandle->TIMx, (pHandle->hPWMperiod/2-1) );

  //if ( pHandle->DTTest == 1u )
  //{
  //  /* Dead time compensation */
  //  if ( pHandle->hIa > 0 )
  //  {
  //    pHandle->hCntPhA += pHandle->DTCompCnt;
  //  }
  //  else
  //  {
  //    pHandle->hCntPhA -= pHandle->DTCompCnt;
  //  }
//
  //  if ( pHandle->hIb > 0 )
  //  {
  //    pHandle->hCntPhB += pHandle->DTCompCnt;
  //  }
  //  else
  //  {
  //    pHandle->hCntPhB -= pHandle->DTCompCnt;
  //  }
//
  //  if ( pHandle->hIc > 0 )
  //  {
  //    pHandle->hCntPhC += pHandle->DTCompCnt;
  //  }
  //  else
  //  {
  //    pHandle->hCntPhC -= pHandle->DTCompCnt;
  //  }
  //}

  //return ( pSetADCSamplingPoint( pHandle ) );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  This function returns cosine and sine functions of the angle fed in
  *         input
  * @param  hAngle: angle in q1.15 format
  * @retval Sin(angle) and Cos(angle) in Trig_Components format
  */

Trig_Components MCM_Trig_Functions( int16_t hAngle )
{
  int32_t shindex;
  uint16_t uhindex;

  Trig_Components Local_Components;

  /* 10 bit index computation  */
  shindex = ( ( int32_t )32768 + ( int32_t )hAngle );
  uhindex = ( uint16_t )shindex;
  //uhindex /= ( uint16_t )64;
  uhindex >>= 6;

 
  switch ( ( uint16_t )( uhindex ) & SIN_MASK )
  {
    case U0_90:
      Local_Components.hSin = hSin_Cos_Table[( uint8_t )( uhindex )];
      Local_Components.hCos = hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
      break;

    case U90_180:
      Local_Components.hSin = hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
      Local_Components.hCos = -hSin_Cos_Table[( uint8_t )( uhindex )];
      break;

    case U180_270:
      Local_Components.hSin = -hSin_Cos_Table[( uint8_t )( uhindex )];
      Local_Components.hCos = -hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
      break;

    case U270_360:
      Local_Components.hSin =  -hSin_Cos_Table[( uint8_t )( 0xFFu - ( uint8_t )( uhindex ) )];
      Local_Components.hCos =  hSin_Cos_Table[( uint8_t )( uhindex )];
      break;
    default:
      break;
  }
  return ( Local_Components );
}
/****
 * @brief 从0-45 度表中 搜索arctan 值
 * @param pr = y/x*65536
 * @return 0-256
*/
uint16_t arctanSearch(uint16_t pr){
  uint16_t max = 256;  //实在表格中不存在
  uint16_t min = 0;
  uint16_t index = 0x80;
  while(max>min){
    index = (max+min)>>1;
    if(pr<hTan_Table[index]){
      max = index;
    }else{
      min = index;
      if(min==(max-1))
        return max;
    }
  }
  return index;
}
/****
 * @brief arctan 查表0-45 /128
 * 
 * 
*/
int16_t arctan(int16_t x, int16_t y){
// 处理零值
#if 0
    if (x == 0 && y == 0) return 0; // 未定义角度，返回0
    if (x == 0) return (y > 0) ? 0x4000 : 0xC000; // 90°或270°
    if (y == 0) return (x > 0) ? 0x0000 : 0x8000; // 0°或180°
    int32_t abs_x = abs((int32_t)x);
    int32_t abs_y = abs((int32_t)y);
    // 计算斜率（Q16格式）
    uint32_t ratio;
    int16_t base_angle;

    // 特殊角度判断（45°倍数）
    if (abs_x == abs_y) {
        if (x > 0) return (y > 0) ? 0x2000 : 0xE000; // 45°/315°
        else return (y > 0) ? 0x6000 : 0xA000;         // 135°/225°
    }

    if (abs_x > abs_y) { //1<45 2>135 3<225 4<315
        // 第一象限 <45° 的情况：直接查表
        ratio = ((uint32_t)abs_y << 16) / (uint32_t)abs_x; // Q16: y/x
        base_angle = (int16_t)(arctanSearch((uint16_t)ratio) << 5);    // 0-45° → 0x0000-0x2000
        if(x>0){
          if(y>0){
            return base_angle;    //1
          }else{
            return -base_angle;   //4
          }
        }else{
            if(y>0){
            return 0x8000-base_angle; //2
          }else{
            return 0x8000+base_angle; //3
          }
        }
    } else { 
        // 第一象限 >45° 的情况：用 90° - arctan(|x|/|y|)
        ratio = ((uint32_t)abs_x << 16) / (uint32_t)abs_y; // Q16: x/y
        base_angle = (int16_t)(arctanSearch((uint16_t)ratio) << 5); // 90° - 锐角
        if(x>0){
          if(y>0){
            return 0x4000-base_angle; //1
          }else{
            return 0xc000+base_angle; //4
          }
        }else{
            if(y>0){
            return 0x4000+base_angle; //2
          }else{
            return 0xc000-base_angle; //3
          }
        }
    }
#endif
#if 1
    if (x == 0 && y == 0) return 0;
    //if (x == 0) return (y > 0) ? 0x4000 : 0xC000; // 90°或270°
    //if (y == 0) return (x > 0) ? 0x0000 : 0x8000; // 0°或180°
    
    // 计算绝对值比
    int32_t abs_x = abs((int32_t)x);  //(0-32768)
    int32_t abs_y = abs((int32_t)y);  //(0-32768)
    uint32_t ratio;
    
    if (abs_x > abs_y) {
        ratio = ((uint32_t)abs_y << 16) / (uint32_t)abs_x;
    } else {
        if(abs_x == abs_y)
        ratio = 0xffff;
        else
        ratio = ((uint32_t)abs_x << 16) / (uint32_t)abs_y;
    }
    
    // 查表得到锐角 (0-45°)
    uint16_t acute_angle = arctanSearch((uint16_t)ratio)<<5;  //0-256 <<5 =>0-0x2000
    
    // 计算基础角度
    int32_t angle;
    if (abs_x > abs_y) {
        angle = acute_angle; // 0-45° 0-0x2000
    } else {
        angle = 0x4000 - acute_angle; // 45-90° 0x4000-0x2000
    }
    
    // 根据象限调整
    if (x >= 0) {
        angle = (y >= 0) ? angle : -angle; // 第一或第四象限  (0-0x2000),(0-0xe000),(0x4000-0x2000),(0xc000,0xe000)
    } else {
      //(0x8000,0x6000),(0xc000,0xa000)
        angle = (y >= 0) ? 0x8000 - angle : 0x8000 + angle; // 第二或第三象限
    }
    
    // 规范化到 [0, 0xFFFF]
    if (angle < 0) angle += 0x10000;
    return (int16_t)angle;
#endif
}
/***
 * @brief 范围平移 输入一个范围之间的数比例对应到另一个输入范围之间的数
 * @param in_min
 * @param in_max
 * @param in_raw
 * @param out_min
 * @param out_max
 * @return out_value
 * 
*/
int dataRangeMov(int in_raw,int in_min,int in_max,int out_min,int out_max){
  if(in_raw<=in_min)
    return out_min;
  if(in_raw>=in_max)
    return out_max;
  if(in_min>in_max)
    return dataRangeMov(in_raw,in_max,in_min,out_min,out_max);
  if(in_min==in_max)
    return out_min;
  const int in_mid = (in_max+in_min)>>1;
  const int out_mid = (out_max+out_min)>>1;
  if(in_min==in_mid)
    return out_mid;
  if(in_raw<=in_mid)
    return dataRangeMov(in_raw,in_min,in_mid,out_min,out_mid);
  else
    return dataRangeMov(in_raw,in_mid+1,in_max,out_mid,out_max);
}
/***
 * @brief 更新最大最小值
 * 
 * 
*/
void MaxMinUpDate(uint16_t *now,uint16_t *max,uint16_t *min){
  if(*now>*max){
    *max = *now;
  }else if(*now<*min){
    *min = *now;
  }
}
/***
 * @brief 带进位的加法 限制幅度  防止溢出的加法
 * 
*/
int16_t CalculateAdd16(int16_t A,int16_t B){
  int16_t c = A + B;
  if(c<0){
    if((A>0)&&(B>0)){
      c = INT16_MAX;
    }
  }else{
    if((A<0)&&(B<0)){
      c = INT16_MIN;
    }
  }
  return c;
}
/***
 * @brief int16 环形加减
 * @param sum a+b / a - b
 * @param b 
 * a + b / a-b
 * 
*/
int32_t CalculateLoopAddSub(int32_t sum){
  if(sum>INT16_MAX){
    sum = sum - 65536;
  }else if(sum<INT16_MIN){
    sum = 65536 - sum;
  }
  return sum;
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
  * @brief  It calculates the square root of a non-negative int32_t. It returns 0
  *         for negative int32_t.
  * @param  Input int32_t number
  * @retval int32_t Square root of Input (0 if Input<0)
  */
int32_t MCM_Sqrt( int32_t wInput )
{
  uint8_t biter = 0u;
  int32_t wtemproot;
  int32_t wtemprootnew;

  if ( wInput > 0 )
  {

    if ( wInput <= ( int32_t )2097152 )
    {
      wtemproot = ( int32_t )128;
    }
    else
    {
      wtemproot = ( int32_t )8192;
    }

    do
    {
      //wtemprootnew = ( wtemproot + wInput / wtemproot ) / ( int32_t )2;
      wtemprootnew = ( wtemproot + wInput / wtemproot )>>1;
      if ( wtemprootnew == wtemproot )
      {
        biter = 6u;
      }
      else
      {
        biter ++;
        wtemproot = wtemprootnew;
      }
    }
    while ( biter < 6u );
  }
  else
  {
    wtemprootnew = ( int32_t )0;
  }

  return ( wtemprootnew );
}

/**
  * @brief  This function codify a floting point number into the relative
  *         32bit integer.
  * @param  float Floting point number to be coded.
  * @retval uint32_t Coded 32bit integer.
  */
uint32_t MCM_floatToIntBit( float x )
{
  uint32_t * pInt;
  pInt = ( uint32_t * )( &x );
  return *pInt;
}

/**
  * @}
  */

/**
  * @}
  */
