/**
 *  @file i2c.c
 *  @brief
 *  @date Created on: Aug. 22, 2025
 *  @author: MaxwellWang
*/
#include "i2c.h"

/**
 * @brief i2c read
 * @param
*/
void i2cRead(i2c_t* it){
    i2cStart(it);
    i2cBitWrite(it,(it->iic_adr<<1));
    i2cBitWrite(it,it->data_adr);
    i2cStart(it);
    i2cBitWrite(it,(it->iic_adr<<1)|0x01);
    for(uint8_t i=0;i<it->len;i++){
        i2cSDA_IN(it);
        it->data[i] = i2cBitRead(it);
        i2cSDA_OUT(it);
        if(i==(it->len-1))
        i2cNoAck(it);
        else
        i2cAck(it);
    }
    i2cStop(it);
}
/**
 * @brief i2c write
 * @param
*/
void i2cWrite(i2c_t* it){
    i2cStart(it);
    i2cBitWrite(it,(it->iic_adr<<1));
    i2cBitWrite(it,it->data_adr);
    for(int i=0;i<it->len;i++){
        i2cBitWrite(it,it->data[i]);
    }
    i2cStop(it);
}
/***
 * @brief i2c start
 * 
*/
void i2cStart(i2c_t *it){
    GPIO_SetBits(it->sda_gpio,it->sda_pin);
    IIC_delay(it->delay);
    GPIO_SetBits(it->scl_gpio,it->scl_pin);
    IIC_delay(it->delay);
    GPIO_ResetBits(it->sda_gpio,it->sda_pin);
    IIC_delay(it->delay);
    GPIO_ResetBits(it->scl_gpio,it->scl_pin);
    IIC_delay(it->delay);
}
/***
 * @brief i2c start
 * 
*/
void i2cStop(i2c_t *it){
    GPIO_ResetBits(it->sda_gpio,it->sda_pin);
    IIC_delay(it->delay);
    GPIO_SetBits(it->scl_gpio,it->scl_pin);
    IIC_delay(it->delay);
    GPIO_SetBits(it->sda_gpio,it->sda_pin);
    IIC_delay(it->delay);
}
/***
 * @brief i2c start
 * 
*/
void i2cAck(i2c_t *it){
    GPIO_ResetBits(it->sda_gpio,it->sda_pin);
    IIC_delay(it->delay);
    i2cOncClock(it);
}
/***
 * @brief i2c start
 * 
*/
void i2cNoAck(i2c_t *it){
    GPIO_SetBits(it->sda_gpio,it->sda_pin);
    IIC_delay(it->delay);
    i2cOncClock(it);
}
/**
 * @brief one clock
 * 
 * 
*/
void i2cOncClock(i2c_t *it){
    GPIO_SetBits(it->scl_gpio,it->scl_pin);
    IIC_delay(it->delay);
    GPIO_ResetBits(it->scl_gpio,it->scl_pin);
    IIC_delay(it->delay);
}
/**
 * @brief write one byte data
 * 
*/
void i2cBitWrite(i2c_t *it,uint8_t data){
    for(uint8_t i=0;i<9;i++){
        if(data&0x80){
            GPIO_SetBits(it->sda_gpio,it->sda_pin);
        }else{
            GPIO_ResetBits(it->sda_gpio,it->sda_pin);
        }
        data <<= 1;
        i2cOncClock(it);
    }
}
/**
 * @brief read one byte data
 * 
*/
uint8_t i2cBitRead(i2c_t *it){
    uint8_t data=0;
    for(uint8_t i=0;i<8;i++){
        data <<= 1;
        GPIO_SetBits(it->scl_gpio,it->scl_pin);
        IIC_delay(it->delay);
        data |= GPIO_ReadInputDataBit(it->sda_gpio,it->sda_pin);
        GPIO_ResetBits(it->scl_gpio,it->scl_pin);
        IIC_delay(it->delay);
    }
		return data;
}
/***
 * @brief sda-<input
 * 
*/
void i2cSDA_IN(i2c_t *it){
    it->sda_gpio->MODER &= ~(GPIO_MODER_MODER0 << (GYPO_SDA_SOURCE * 2));
}

/***
 * @brief sda-< output
 * 
*/
void i2cSDA_OUT(i2c_t *it){
    it->sda_gpio->MODER &= ~(GPIO_MODER_MODER0 << (GYPO_SDA_SOURCE * 2));
    it->sda_gpio->MODER |= (GPIO_Mode_OUT << (GYPO_SDA_SOURCE * 2));
}




