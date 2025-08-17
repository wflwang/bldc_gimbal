/*
 * eeprom.h
 *
 *  Created on: Aug. 12, 2025
 *      Author: MaxwellWang
    */
#ifndef __EEPROM_H
#define __EEPROM_H
   #include "main.h"
   #include "flash_helper.h"
   #include "mc_type.h"

//hall state & angle(256) tanɵ = hally/hallx actan hally/hallx = @(-1 - 1)
#define ADDR_HALL_Ready     ADDR_FLASH_EEPROM_PAGE
#define ADDR_GYPO_Ready     ADDR_FLASH_EEPROM_PAGE+4
#define GYPO_Wz_offset      0
#define Acc_x_offset        0
#define Acc_y_offset        0

//EEPROM read data
int EE_ReadFOC(Learn_Componets *lc);
void EE_WriteFOC(Learn_Componets *lc);

#endif
