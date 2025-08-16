/*
 * peripherals.h
 *
 *  Created on: Aug. 11, 2025
 *      Author: MaxwellWang
 */
#ifndef __QMI8658B_H
#define __QMI8658B_H
#include    "main.h"


void qmi8658x_init(GPIO_InitTypeDef *sda_gpio,int sda_pin,GPIO_InitTypeDef *scl_gpio,int scl_pin);


#endif
