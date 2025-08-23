/******
 * @file peripherals.c
 *  
 * @date Created on: Aug. 11, 2025
 * @author  MaxwellWang
 */

#include "qmi8658b.h"
#include "peripherals.h"
//#include	"i2c.h"

///////////////////////////////////////////////////////////////////////////////
// MPU6050 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

// Registers

#define QMI8658B_ACCEL_XOUT_H        0x36
#define QMI8658B_ACCEL_XOUT_L        0x35
#define QMI8658B_ACCEL_YOUT_H        0x38
#define QMI8658B_ACCEL_YOUT_L        0x37
#define QMI8658B_ACCEL_ZOUT_H        0x3a
#define QMI8658B_ACCEL_ZOUT_L        0x39
#define QMI8658B_TEMP_OUT_H          0x34
#define QMI8658B_TEMP_OUT_L          0x33
#define QMI8658B_GYRO_XOUT_H         0x3c
#define QMI8658B_GYRO_XOUT_L         0x3b
#define QMI8658B_GYRO_YOUT_H         0x3e
#define QMI8658B_GYRO_YOUT_L         0x3d
#define QMI8658B_GYRO_ZOUT_H         0x40
#define QMI8658B_GYRO_ZOUT_L         0x3f
#define QMI8658B_FIFO_REG          	0x13
#define QMI8658B_FIFO_SMPL_CNT      0x75
#define QMI8658B_FIFO_STATUS        0x16
#define QMI8658B_FIFO_R_W           0x17

#define QMI8658B_WHOAMI             0x00
#define QMI8658B_WHOAMI_V           0x05
#define QMI8658B_RESET				0x60
#define QMI8658B_RESET_V			0x4b	//reset

#define QMI8658B_ACC_Set			0x03
#define QMI8658B_ACC_FS_selftest	0x80
#define QMI8658B_ACC_FS_2G			0x00
#define QMI8658B_ACC_FS_4G			0x10
#define QMI8658B_ACC_FS_8G			0x20
#define QMI8658B_ACC_FS_16G			0x30
#define QMI8658B_ACC_ODR_7174HZ		0x00
#define QMI8658B_ACC_ODR_3587HZ		0x01
#define QMI8658B_ACC_ODR_1793HZ		0x02
#define QMI8658B_ACC_ODR_896HZ		0x03
#define QMI8658B_ACC_ODR_448HZ		0x04
#define QMI8658B_ACC_ODR_224HZ		0x05
#define QMI8658B_ACC_ODR_112HZ		0x06
#define QMI8658B_ACC_ODR_56HZ		0x07
#define QMI8658B_ACC_ODR_28HZ		0x08
#define QMI8658B_ACC_LP_ODR_128HZ		0x0c
#define QMI8658B_ACC_LP_ODR_21HZ		0x0d
#define QMI8658B_ACC_LP_ODR_11HZ		0x0e
#define QMI8658B_ACC_LP_ODR_3HZ			0x0f

#define QMI8658B_GYRO_Set			0x03
#define QMI8658B_GYRO_FS_selftest	0x80
#define QMI8658B_GYRO_FS_16DPS		0x00
#define QMI8658B_GYRO_FS_32DPS		0x10
#define QMI8658B_GYRO_FS_64DPS		0x20
#define QMI8658B_GYRO_FS_128DPS		0x30
#define QMI8658B_GYRO_FS_256DPS		0x40
#define QMI8658B_GYRO_FS_512DPS		0x50
#define QMI8658B_GYRO_FS_1024DPS	0x60
#define QMI8658B_GYRO_FS_2048DPS	0x70
#define QMI8658B_GYRO_ODR_7174HZ	0x00
#define QMI8658B_GYRO_ODR_3587HZ	0x01
#define QMI8658B_GYRO_ODR_1793HZ	0x02
#define QMI8658B_GYRO_ODR_896HZ		0x03
#define QMI8658B_GYRO_ODR_448HZ		0x04
#define QMI8658B_GYRO_ODR_224HZ		0x05
#define QMI8658B_GYRO_ODR_112HZ		0x06
#define QMI8658B_GYRO_ODR_56HZ		0x07
#define QMI8658B_GYRO_ODR_28HZ		0x08

#define QMI8658B_EN_Sensors			0x08
#define QMI8658B_GYRO_EN			0x02
#define QMI8658B_ACC_EN				0x01



//*****************************************
float accelOneG = 9.8065;           // 
int16_t accelData500Hz[3];

float accelTCBias[3] = { 0.0f, 0.0f, 0.0f };

//int16andUint8_t rawAccel[3];
//********************************************

uint8_t qmi8658x_init(GPIO_TypeDef *sda_gpio,uint32_t sda_pin,GPIO_TypeDef *scl_gpio,uint32_t scl_pin){
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Pin = sda_pin;
	GPIO_Init(sda_gpio, &GPIO_InitStructure);
	GPIO_SetBits(sda_gpio,sda_pin);
    GPIO_InitStructure.GPIO_Pin = scl_pin;
	GPIO_Init(scl_gpio, &GPIO_InitStructure);
	GPIO_SetBits(scl_gpio,scl_pin);
	i2c_t it={
		.scl_gpio = scl_gpio,
		.scl_pin = scl_pin,
		.sda_gpio = sda_gpio,
		.sda_pin = sda_pin,
		.iic_adr = QMI8658B_ADDRESS,
	};
	it.len =1;
	it.data_adr = QMI8658B_RESET;
	it.data = QMI8658B_RESET_V;
	i2cWrite(&it);		//reset
	delay_ms(150);
	it.data_adr = QMI8658B_ACC_Set;
	it.data = QMI8658B_ACC_FS_4G|QMI8658B_ACC_ODR_896HZ;
	i2cWrite(&it);		//reset
	it.data_adr = QMI8658B_GYRO_Set;
	it.data = QMI8658B_GYRO_FS_512DPS|QMI8658B_GYRO_ODR_7174HZ;
	i2cWrite(&it);		//reset
	it.data_adr = QMI8658B_EN_Sensors;
	it.data = QMI8658B_GYRO_EN|QMI8658B_ACC_EN;
	i2cWrite(&it);		//reset
	it.data_adr = QMI8658B_WHOAMI;
	uint8_t ret = 0;
	it.data = &ret;
	i2cRead(&it);
	if(it.data != QMI8658B_WHOAMI_V){
		return -1;	//fail
	}
	//it.data_adr = QMI8658B_LP_config;	//配置低通滤波
	//it.data = QMI8658B_GYRO_LP_EN|QMI8658B_ACC_LP_EN;
	//i2cWrite(&it);		//reset

	//i2cWrite(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1,   MPU_CLK_SEL_PLLGYROZ);      // Clock Source
	//i2cWrite(MPU6050_ADDRESS, MPU6050_PWR_MGMT_2,   0x00);                      // turn off all standby
	//i2cWrite(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV,   0x00);                      // Accel Sample Rate 1000 Hz, Gyro Sample Rate 8000 Hz
	//i2cWrite(MPU6050_ADDRESS, MPU6050_CONFIG,       eepromConfig.dlpfSetting);  // Accel and Gyro DLPF Setting
	//i2cWrite(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, BITS_FS_4G);                // Accel +/- 4 G Full Scale
	//i2cWrite(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG,  BITS_FS_500DPS);            // Gyro +/- 500 DPS Full Scale
	//    i2cWrite(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG,  0x00);                      // int pin config
	//    i2cWrite(MPU6050_ADDRESS, MPU6050_INT_ENABLE,   0x00);                      // int pin disable

	///////////////////////////////////

	delay_ms(100);

	computeMPU6050RTData();
	return 0;
}
/***
 * @brief 陀螺仪初始化 ；连续采样陀螺仪和加速度 算出平均值 当检测到误差过大时候fail 用以前的校准数据
 * 没有存储校准过灯一直闪烁等待陀螺仪和角速度校准完成
 * 
 * 
*/
void initOrientation()
{
	int initLoops = 150;
	//float accAngle[NUMAXIS] = { 0.0f, 0.0f, 0.0f };
	int i;

	for (i = 0; i < initLoops; i++)
	{
		readQmi8658b();//从MPU6050得到加速度和陀螺仪数据，并进行 与初始化方位估计矩阵（根据IMU单元的方位确定的矩阵A ） 相乘后的数据

		computeMPU6050TCBias();//计算温度补偿偏差值

		//（矩阵相乘后的加速度数据-温度补偿偏差）* （(1/8192) * 9.8065）
		//(1/8192) * 9.8065  (8192 LSB = 1 G)
		//1G量程的8192个数字量分之1，对应重力加速度9.8065m/1G的8192分之1
		sensors.accel500Hz[XAXIS] = ((float)rawAccel[XAXIS].value - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
		sensors.accel500Hz[YAXIS] = ((float)rawAccel[YAXIS].value - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
		sensors.accel500Hz[ZAXIS] = -((float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

		//进行欧拉角积分运算
		accAngle[ROLL]  += atan2f(-sensors.accel500Hz[YAXIS], -sensors.accel500Hz[ZAXIS]);
		accAngle[PITCH] += atan2f(sensors.accel500Hz[XAXIS], -sensors.accel500Hz[ZAXIS]);

		//求取欧拉角算数平均值
		accAngleSmooth[ROLL ] = accAngle[ROLL ] / (float)initLoops;
		accAngleSmooth[PITCH] = accAngle[PITCH] / (float)initLoops;

		delay(2);
	}

	//得到当前方位 ,初始化一次，不要振动云台，因为这里只用了加速度数据计算欧拉角（加速度数据是长期可信的），但是加速度计对
	//振动很敏感，所以为了减小误差，初始化方位的时候不要振动云台。
	sensors.evvgcCFAttitude500Hz[PITCH] = accAngleSmooth[PITCH];
	sensors.evvgcCFAttitude500Hz[ROLL ] = accAngleSmooth[ROLL ];
	sensors.evvgcCFAttitude500Hz[YAW  ] = 0.0f;
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
