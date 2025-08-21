/*
 * eeprom.c
 *
 *  Created on: Aug. 12, 2025
 *      Author: MaxwellWang
    */
   #include     "EEPROM.h"
   //#include   "mc_type.h"
   #include   <stdio.h>
/**
 * 
 * 初始化EEPROM 
 * 读出EEPROM中的数据 对应的是256*2 => 256 个角度的x y霍尔值
 * 
*/
void EE_init(void){
  /* Check if the programmed data is OK
      MemoryProgramStatus = 0: data programmed correctly
      MemoryProgramStatus != 0: number of words not programmed correctly ******/
  //Address = FLASH_USER_START_ADDR;
  //MemoryProgramStatus = PASSED;
  // 
  //while (Address < FLASH_USER_END_ADDR)
  //{
  //  Data = *(__IO uint32_t *)Address;
  // 
  //  if (Data != DATA_32)
  //  {
  //    MemoryProgramStatus = FAILED;
  //  }
  // 
  //  Address = Address + 4;
  //}
}
/**
 * 
 * 读出EEPROM 存储的数据 hall角度 & 陀螺仪校准值
 * 
*/
void EE_ReadState(void){
    //Data = *(__IO uint32_t *)Address;
}
/**
 * @brief read eeprom data
 * 
*/
int EE_ReadFOC(Learn_Componets *lc){
	// Read flash
  //memcpy(lc, (uint8_t *)ADDR_FLASH_EEPROM_PAGE, sizeof(Learn_Componets));
  uint32_t data = *(__IO uint32_t *)ADDR_FLASH_EEPROM_PAGE;
  lc->LearnFinish =  (uint8_t)((data>>24)&0xff);
  lc->M_dir =  (uint8_t)((data>>16)&0xff);
  lc->xyScaleDir =  (uint8_t)((data>>8)&0xff);
  data = *(__IO uint32_t *)(ADDR_FLASH_EEPROM_PAGE+4);
  lc->ElAngele_offset = (int16_t)(data>>16);
  lc->x_offset = (int16_t)(data&0xffff);
  data = *(__IO uint32_t *)(ADDR_FLASH_EEPROM_PAGE+8);
  lc->y_offset = (int16_t)(data>>16);
  lc->xy_scale = (int16_t)(data&0xffff);
  if(lc->LearnFinish!=1){
    lc->LearnFinish = 0;
    return -1;
  }
  return 0;

	//memcpy(&eepromConfig, (char *)EEPROM_BASE, sizeof(eepromConfig_t));
//
	//accConfidenceDecay = 1.0f / sqrt(eepromConfig.accelCutoff);
//
	//mechanical2electricalDegrees[ROLL ] = eepromConfig.rollMotorPoles  / 2.0f;
	//mechanical2electricalDegrees[PITCH] = eepromConfig.pitchMotorPoles / 2.0f;
	//mechanical2electricalDegrees[YAW  ] = eepromConfig.yawMotorPoles   / 2.0f;
//
	//electrical2mechanicalDegrees[ROLL ] = 1.0f / mechanical2electricalDegrees[ROLL ];
	//electrical2mechanicalDegrees[PITCH] = 1.0f / mechanical2electricalDegrees[PITCH];
	//electrical2mechanicalDegrees[YAW  ] = 1.0f / mechanical2electricalDegrees[YAW  ];
}

void EE_WriteFOC(Learn_Componets *lc){  
  uint32_t data[3];
  data[0] = ((lc->LearnFinish<<24)|(lc->M_dir<<16)|(lc->xyScaleDir<<8));
  data[1] = ((lc->ElAngele_offset<<16)|(lc->x_offset));
  data[2] = ((lc->y_offset<<16)|(lc->xy_scale));
  //memcpy(data, (uint8_t *)lc, sizeof(Learn_Componets));
  uint32_t primask = __get_PRIMASK();  // 读取PRIMASK寄存器[1](@ref)
  __disable_irq();                     // 强制关闭中断
  erase_page(EEPROM_BASE,1);
  write_data(ADDR_FLASH_EEPROM_PAGE, data, 3);
  // 智能恢复中断状态
  if ((primask & 0x1) == 0) {          // 判断原状态是否为开启
    __enable_irq();                   // 原开启则恢复开启[4](@ref)
  }
}
