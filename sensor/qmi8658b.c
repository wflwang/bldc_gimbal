/******
 * @file peripherals.c
 *  
 * @date Created on: Aug. 11, 2025
 * @author  MaxwellWang
 */

#include "qmi8658b.h"

///////////////////////////////////////////////////////////////////////////////
// MPU6050 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

// Registers

#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_FIFO_EN             0x23
#define MPU6050_INT_PIN_CFG         0x37
#define MPU6050_INT_ENABLE          0x38
#define MPU6050_INT_STATUS          0x3A
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_TEMP_OUT_L          0x42
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48
#define MPU6050_USER_CTRL           0x6A
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_PWR_MGMT_2          0x6C
#define MPU6050_FIFO_COUNTH         0x72
#define MPU6050_FIFO_COUNTL         0x73
#define MPU6050_FIFO_R_W            0x74
#define MPU6050_WHOAMI              0x75

// Bits

#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01


//*****************************************
float accelOneG = 9.8065;           // 
int16_t accelData500Hz[3];

float accelTCBias[3] = { 0.0f, 0.0f, 0.0f };

//int16andUint8_t rawAccel[3];
//********************************************

void qmi8658x_init(GPIO_TypeDef *sda_gpio,uint32_t sda_pin,GPIO_TypeDef *scl_gpio,uint32_t scl_pin){
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = sda_pin;
	GPIO_Init(sda_gpio, &GPIO_InitStructure);
	GYPO_SDA_Set();
    GPIO_InitStructure.GPIO_Pin = scl_pin;
	GPIO_Init(scl_gpio, &GPIO_InitStructure);
	GYPO_SCL_Set();
}

#if 0
static int reset_init_mpu(void) {
	i2c_bb_restore_bus(&i2cs);

	// Set clock source to gyro x
	tx_buf[0] = MPU9150_PWR_MGMT_1;
	tx_buf[1] = 0x01;
	bool res = i2c_bb_tx_rx(&i2cs, mpu_addr, tx_buf, 2, rx_buf, 0);

	// Try the other address
	if (!res) {
		if (mpu_addr == MPU_ADDR1) {
			mpu_addr = MPU_ADDR2;
		} else {
			mpu_addr = MPU_ADDR1;
		}

		// Set clock source to gyro x
		tx_buf[0] = MPU9150_PWR_MGMT_1;
		tx_buf[1] = 0x01;
		res = i2c_bb_tx_rx(&i2cs, mpu_addr, tx_buf, 2, rx_buf, 0);

		if (!res) {
			return 0;
		}
	}

	// Set accelerometer full-scale range to +/- 16g
	tx_buf[0] = MPU9150_ACCEL_CONFIG;
	tx_buf[1] = MPU9150_ACCEL_FS_16 << MPU9150_ACONFIG_AFS_SEL_BIT;
	res = i2c_bb_tx_rx(&i2cs, mpu_addr, tx_buf, 2, rx_buf, 0);

	if (!res) {
		return 0;
	}

	// Set gyroscope full-scale range to +/- 2000 deg/s
	tx_buf[0] = MPU9150_GYRO_CONFIG;
	tx_buf[1] = MPU9150_GYRO_FS_2000 << MPU9150_GCONFIG_FS_SEL_BIT;
	res = i2c_bb_tx_rx(&i2cs, mpu_addr, tx_buf, 2, rx_buf, 0);

	if (!res) {
		return 0;
	}

	// Set low pass filter to 256Hz (1ms delay)
	tx_buf[0] = MPU9150_CONFIG;
	tx_buf[1] = MPU9150_DLPF_BW_256;
	res = i2c_bb_tx_rx(&i2cs, mpu_addr, tx_buf, 2, rx_buf, 0);

	if (!res) {
		return 0;
	}

	if(use_magnetometer){
		// Set the i2c bypass enable pin to true to access the magnetometer
		tx_buf[0] = MPU9150_INT_PIN_CFG;
		tx_buf[1] = 0x02;
		res = i2c_bb_tx_rx(&i2cs, mpu_addr, tx_buf, 2, rx_buf, 0);

		if (!res) {
			return 0;
		}
	}

	is_mpu9250 = read_single_reg(MPU9150_WHO_AM_I) == 0x71;

	return 1;
}
#endif
