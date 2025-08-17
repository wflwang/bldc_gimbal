/*
 * flash_helper.h
 *
 *  Created on: Aug. 12, 2025
 *      Author: MaxwellWang
    */

#ifndef FLASH_HELPER_H_
#define FLASH_HELPER_H_

//#include "conf_general.h"
#include "main.h"

//#define CODE_IND_QML	0
//#define CODE_IND_LISP	1
/*
 * Defines
 */
#define FLASH_PAGE							64	//64*256 = 16,384byte
#define FLASH_PAGE_SIZE						256	//256 byte
#define BOOTLOADER_BASE							0
#define BOOTLOADER_SIZE							1
#define APP_BASE								2
#define APP_BASE_SIZE							60		//APP 使用的page大小
#define EEPROM_BASE								62
#define EEPROM_BASE_SIZE						2		//EEPROM 使用大小

// Base address of the Flash sectors
#define ADDR_FLASH_START_PAGE					((uint32_t)0x08000000) // Base @ of Sector 0, 16 Kbytes
#define ADDR_FLASH_BOOTLOADER_PAGE				(ADDR_FLASH_START_PAGE+(BOOTLOADER_BASE*FLASH_PAGE_SIZE))
#define ADDR_FLASH_APP_PAGE						(ADDR_FLASH_START_PAGE+(APP_BASE*FLASH_PAGE_SIZE))
#define ADDR_FLASH_EEPROM_PAGE					(ADDR_FLASH_START_PAGE+(EEPROM_BASE*FLASH_PAGE_SIZE)) 
// Functions
FLASH_Status write_data(uint32_t base, uint8_t *data, uint32_t len);
FLASH_Status erase_page(uint32_t start_addr_page,uint32_t new_app_size);
FLASH_Status flash_helper_erase_new_app(uint32_t new_app_size);
FLASH_Status flash_helper_erase_bootloader(void);
FLASH_Status flash_helper_write_new_app_data(uint32_t offset, uint8_t *data, uint32_t len);
void flash_helper_jump_to_bootloader(void);

#endif /* FLASH_HELPER_H_ */
