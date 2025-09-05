/*
 * i2c.h
 *
 *  Created on: Aug. 22, 2025
 *      Author: MaxwellWang
    */

//#include <stdint.h>
#ifndef __I2C_H
#define __I2C_H
#include "main.h"
#include "peripherals.h"

typedef struct 
{
    uint8_t iic_adr;    //IIC Write 地址
    uint8_t data_adr;   //数据地址
    uint8_t len;        //数据长度
    uint8_t *data;      //数据指针
    uint16_t delay;     //us*10
    GPIO_TypeDef *sda_gpio;
    uint32_t sda_pin;
    GPIO_TypeDef *scl_gpio;
    uint32_t scl_pin;
}i2c_t;

void i2cStart(i2c_t *it);
void i2cStop(i2c_t *it);
void i2cWrite(i2c_t* it);
void i2cRead(i2c_t* it);
void i2cAck(i2c_t *it);
void i2cNoAck(i2c_t *it);
void i2cOncClock(i2c_t *it);
void i2cBitWrite(i2c_t *it,uint8_t data);
uint8_t i2cBitRead(i2c_t *it);
#define IIC_delay(x)    delay_us(x);
void i2cSDA_IN(i2c_t *it);
void i2cSDA_OUT(i2c_t *it);

#endif
