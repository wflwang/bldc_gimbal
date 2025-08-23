/*
 * peripherals.h
 *
 *  Created on: Aug. 11, 2025
 *      Author: MaxwellWang
 */
#ifndef __QMI8658B_H
#define __QMI8658B_H
#include    "main.h"a
#include    "i2c.h"
//#define SDA_PullDown
#ifdef SDA_PullDown
#define QMI8658B_ADDRESS            0x6b
#else
#define QMI8658B_ADDRESS            0x6a
#endif

#define MPU6050_CONFIG              0x1A

#define BITS_DLPF_CFG_256HZ         0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03

#define ACCEL_SCALE_FACTOR 0.00119708f  // (1/8192) * 9.8065  (8192 LSB = 1 G)
#define GYRO_SCALE_FACTOR  0.00026646f  // (1/65.5) * pi/180   (65.5 LSB = 1 DPS)


void qmi8658x_init(GPIO_TypeDef *sda_gpio,uint32_t sda_pin,GPIO_TypeDef *scl_gpio,uint32_t scl_pin);


#endif
