/*
 * peripherals.h
 *
 *  Created on: Aug. 11, 2025
 *      Author: MaxwellWang
 */
#ifndef __QMI8658B_H
#define __QMI8658B_H
#include    "main.h"


void qmi8658x_init(GPIO_TypeDef *sda_gpio,uint32_t sda_pin,GPIO_TypeDef *scl_gpio,uint32_t scl_pin);


#endif
