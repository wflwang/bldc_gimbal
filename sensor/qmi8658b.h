/*
 * peripherals.h
 *
 *  Created on: Aug. 11, 2025
 *      Author: MaxwellWang
 */
#ifndef __QMI8658B_H
#define __QMI8658B_H
#include    "main.h"
#include    "i2c.h"
//#define SDA_PullDown
#ifdef SDA_PullDown
#define QMI8658B_ADDRESS            0x6b
#else
#define QMI8658B_ADDRESS            0x6a
#endif

#define accX_alp_raw    6000    //当前滤波系数
#define accX_alp_min    6000    //最小滤波系数
#define accX_alp_max    65535    //最大滤波系数
#define accY_alp_raw    6000    //当前滤波系数
#define accY_alp_min    6000    //最小滤波系数
#define accY_alp_max    65535    //最大滤波系数
#define gyroZ_alp_raw    1000    //当前滤波系数
#define gyroZ_alp_min    1000    //最小滤波系数
#define gyroZ_alp_max    65535    //最大滤波系数

#define gyroCaliErr     0x100   //陀螺仪最大误差范围

//#define MPU6050_CONFIG              0x1A
//#define BITS_DLPF_CFG_256HZ         0x00
//#define BITS_DLPF_CFG_188HZ         0x01
//#define BITS_DLPF_CFG_98HZ          0x02
//#define BITS_DLPF_CFG_42HZ          0x03

#define ACCEL_SCALE_FACTOR 0.00119708f  // (1/8192) * 9.8065  (8192 LSB = 1 G)
#define GYRO_SCALE_FACTOR  0.00026646f  // (1/65.5) * pi/180   (65.5 LSB = 1 DPS)


uint8_t qmi8658x_init(GPIO_TypeDef *sda_gpio,uint32_t sda_pin,GPIO_TypeDef *scl_gpio,uint32_t scl_pin);
//读出传感器数据
void readQmi8658b(void);
int16_t GetACC_X(void);
int16_t GetACC_Y(void);
int16_t GetACC_Z(void);
int16_t GetGYRO_Z(void);
void calibrationGyro(void); //校准陀螺仪
int16_t getOrientation_1ms(void);
void writeQMIregInit(i2c_t *it);
void writeQMIreg(i2c_t *it,uint8_t adr,uint8_t dat);

#endif
