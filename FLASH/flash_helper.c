/*
 * flash helper.c
 *
 *  Created on: Aug. 12, 2025
 *      Author: MaxwellWang
    */

#include "flash_helper.h"
//#include "ch.h"
//#include "hal.h"
//#include "stm32f4xx_conf.h"
//#include "utils_sys.h"
//#include "mc_interface.h"
//#include "timeout.h"
//#include "hw.h"
//#include "crc.h"
//#include "buffer.h"
//#include <string.h>
//#include	"main.h"

#ifdef USE_LISPBM
#include "lispif.h"
#endif

//#define VECTOR_TABLE_ADDRESS					((uint32_t*)ADDR_FLASH_SECTOR_0)
//#define VECTOR_TABLE_SIZE						((uint32_t)(ADDR_FLASH_SECTOR_1 - ADDR_FLASH_SECTOR_0))
//#define EEPROM_EMULATION_SIZE					((uint32_t)(ADDR_FLASH_SECTOR_4 - ADDR_FLASH_SECTOR_2))
//#define	APP_CRC_WAS_CALCULATED_FLAG				((uint32_t)0x00000000)
//#define	APP_CRC_WAS_CALCULATED_FLAG_ADDRESS		((uint32_t*)(ADDR_FLASH_SECTOR_0 + APP_MAX_SIZE - 8))
//#define APP_CRC_ADDRESS							((uint32_t*)(ADDR_FLASH_SECTOR_0 + APP_MAX_SIZE - 4))
//#define ERASE_VOLTAGE_RANGE						(uint8_t)((PWR->CSR & PWR_CSR_PVDO) ? VoltageRange_2 : VoltageRange_3)

// Private functions

/**
 * @param new_app_size  max is 64 page
 * 擦除目标APP FLASH 数据
*/
FLASH_Status erase_page(uint32_t start_addr_page,uint32_t new_app_size) {
	if(new_app_size>64){
		return FLASH_ERASE_PAGE_OVER;
	}
	FLASH_Unlock();
	/* Clear pending flags (if any) */
  	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR);
	//timeout_configure_IWDT_slowest();

	for (uint32_t i = 0;i < new_app_size ;i++) {
    	if(FLASH_ErasePage(ADDR_FLASH_START_PAGE+((start_addr_page+i)*FLASH_PAGE_SIZE)) != FLASH_COMPLETE)
    	{
    	  /* Error occurred while sector erase.
    	      User can add here some code to deal with this error */
			  //没有擦除成功
			  break;
    	}
	}

	FLASH_Lock();
	//timeout_configure_IWDT();
	//mc_interface_ignore_input_both(100);
	//utils_sys_unlock_cnt();

	return FLASH_COMPLETE;
}
/**
 * @param new_app_size  max is 64 page
 * 写 目标 APP FLASH 数据
*/
FLASH_Status write_data(uint32_t base, uint8_t *data, uint32_t len) {
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_WRPERR);
	/* 
	mc_interface_ignore_input_both(5000);
	mc_interface_release_motor_override_both();
	if (!mc_interface_wait_for_motor_release_both(3.0)) {
		return 100;
	}
	*/
	//utils_sys_lock_cnt();
	//timeout_configure_IWDT_slowest();

	for (uint32_t i = 0;i < len;i=i+4) {
		if(FLASH_ProgramWord(base + i, data[i])!=FLASH_COMPLETE){
			break;
		}
	}

	FLASH_Lock();
	//timeout_configure_IWDT();
	//mc_interface_ignore_input_both(100);
	//utils_sys_unlock_cnt();

	return FLASH_COMPLETE;
}

/**
 * @param new_app_size  max is 64 page
 * 擦除目标APP FLASH 数据
*/
FLASH_Status flash_helper_erase_new_app(uint32_t new_app_size) {
	return erase_page(APP_BASE,APP_BASE_SIZE);
}

FLASH_Status flash_helper_erase_bootloader(void) {
	return erase_page(ADDR_FLASH_BOOTLOADER_PAGE,BOOTLOADER_SIZE);
}

FLASH_Status flash_helper_write_new_app_data(uint32_t offset, uint8_t *data, uint32_t len) {
	return write_data(ADDR_FLASH_APP_PAGE + offset, data, len);
}

/**
 * Stop the system and jump to the bootloader.
 */
void flash_helper_jump_to_bootloader(void) {
	typedef void (*pFunction)(void);

	//mc_interface_release_motor_override();
	//usbDisconnectBus(&USBD1);
	//usbStop(&USBD1);

	//sdStop(&HW_UART_DEV);
	//palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_INPUT);
	//palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_INPUT);

	// Disable watchdog
	//timeout_configure_IWDT_slowest();

	//chSysDisable();

	pFunction jump_to_bootloader;

	// Variable that will be loaded with the start address of the application
	volatile uint32_t* jump_address;
	const volatile uint32_t* bootloader_address = (volatile uint32_t*)ADDR_FLASH_BOOTLOADER_PAGE;

	// Get jump address from application vector table
	jump_address = (volatile uint32_t*) bootloader_address[1];

	// Load this address into function pointer
	jump_to_bootloader = (pFunction) jump_address;

	__disable_irq();

	// 8.2 清除所有挂起的中断 (M0没有ICSR，使用NVIC->ICPR)
    #ifdef NVIC_ICPR0
        // 如果定义了多个ICPR寄存器
        for (int i = 0; i < sizeof(NVIC->ICPR) / sizeof(NVIC->ICPR[0]); i++) {
            NVIC->ICPR[i] = 0xFFFFFFFF; // 清除所有挂起中断
        }
    #else
        // 单个ICPR寄存器
        NVIC->ICPR[0] = 0xFFFFFFFF; // 清除所有挂起中断
    #endif
    
    // 8.3 禁用所有中断
    #ifdef NVIC_ICER0
        // 如果定义了多个ICER寄存器
        for (int i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++) {
            NVIC->ICER[i] = 0xFFFFFFFF; // 禁用所有中断
        }
    #else
        // 单个ICER寄存器
        NVIC->ICER[0] = 0xFFFFFFFF; // 禁用所有中断
    #endif

	// Set stack pointer
	__set_MSP((uint32_t) (bootloader_address[0]));

	// Jump to the bootloader
	jump_to_bootloader();
}
