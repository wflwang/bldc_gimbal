/******
 * @file peripherals.c
 *  
 * @date Created on: Aug. 11, 2025
 * @author  MaxwellWang
 */

#include "qmi8658b.h"
#include "peripherals.h"
#include "filter.h"
#include "mc_math.h"
#include "mc_config.h"
#include	"i2c.h"
//陀螺仪参数滤波主要滤掉低频 低误差时候滤波系数小 高误差时候滤波系数大
//陀螺仪滤波系数表格 只需要过滤掉 变化大的  ?/65536(滤波系数)=>每次增加的滤波系数
const int16_t gyroFilter[] = {
    80,220,820,1500,5000,10000
};
//对应不同误差的滤波系数增量
const int16_t gyroFilterV[]={
    312,622,900,1500,3000,6000,9000
};
//加速度滤波 滤波系数要大,不能增长过快
//加速率滤波表格
const int16_t accFilter[] = {
    80,160,820,2640,8500
};
const int16_t accFilterV[] = {
    360,500,700,1500,3500,8000
};
//互补滤波 误差越大滤波系数快速增大
//互补滤波表格 误差越大 陀螺仪占比越多 误差越小 加速度占比越多
//const int16_t complementFilter[]={
//    80,360,820,2640,6500
//};
//const int16_t complementFilterV[]={
//    300,600,1200,3200,7000,12500
//};
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
static i2c_t it;
static int16_t GYRO_vZ = 0;
static int16_t ACC_vX = 0;
static int16_t ACC_vY = 0;
static int16_t ACC_vZ = 0;
//static int16_t TEMP = 0;
static int16_t lastOriA=0;	//上次计算出的角度
static uint8_t gyroInitFin=0;	//陀螺仪初始化完成标志
filter_t accXft;
filter_t accYft;
filter_t gyroZft;
//filter_t copft;	//互补滤波参数

//int16andUint8_t rawAccel[3];
//********************************************

/***
 * @brief 陀螺仪初始化 ；连续采样陀螺仪和加速度 算出平均值 当检测到误差过大时候fail 用以前的校准数据
 * 没有存储校准过灯一直闪烁等待陀螺仪和角速度校准完成
 * 
 * 
*/
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
	it.scl_gpio = scl_gpio;
	it.scl_pin = scl_pin;
	it.sda_gpio = sda_gpio;
	it.sda_pin = sda_pin;
	it.iic_adr = QMI8658B_ADDRESS;
	it.len =1;
	it.data_adr = QMI8658B_RESET;
	it.data = &QMI8658B_RESET_V;
	i2cWrite(&it);		//reset
	Delay_ms(150);
	it.data_adr = QMI8658B_ACC_Set;
	it.data = &(QMI8658B_ACC_FS_4G|QMI8658B_ACC_ODR_896HZ);
	i2cWrite(&it);		//reset
	it.data_adr = QMI8658B_GYRO_Set;
	it.data = &(QMI8658B_GYRO_FS_512DPS|QMI8658B_GYRO_ODR_7174HZ);
	i2cWrite(&it);		//reset
	it.data_adr = QMI8658B_EN_Sensors;
	it.data = &(QMI8658B_GYRO_EN|QMI8658B_ACC_EN);
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
	readQmi8658b();	//读出参数
	//初始化所有滤波参数
	accXft.alpha_diff = accFilter;
	accXft.alpha_diff_addV = accFilterV;
	accXft.alpha_diff_len = sizeof(accFilter)/sizeof(int16_t);
    accXft.alpha_raw = accX_alp_raw;
    accXft.alpha_min= accX_alp_min;
    accXft.alpha_max= accX_alp_max;
	accXft.filter = ACC_vX;

	accYft.alpha_diff = accFilter;
	accYft.alpha_diff_addV = accFilterV;
	accYft.alpha_diff_len = sizeof(accFilter)/sizeof(int16_t);
    accYft.alpha_raw = accY_alp_raw;
    accYft.alpha_min= accY_alp_min;
    accYft.alpha_max= accY_alp_max;
	accYft.filter = ACC_vY;

	gyroZft.alpha_diff = gyroFilter;
	gyroZft.alpha_diff_addV = gyroFilterV;
	gyroZft.alpha_diff_len = sizeof(gyroFilter)/sizeof(int16_t);
	gyroZft.alpha_raw = gyroZ_alp_raw;
	gyroZft.alpha_min = gyroZ_alp_min;	//3%
	gyroZft.alpha_max = gyroZ_alp_max;
	gyroZft.filter = GYRO_vZ;
	//互补滤波参数
	//raw -> 陀螺仪参数(误差越大越灵敏占比多) filter-> 加速度(误差越小占比越大)
    //filterInit = 1;
	//开机校准陀螺仪和加速计
	//computeMPU6050RTData();
	//校准陀螺仪和加速度 静置桌面不动 陀螺仪积分5000次校准得出陀螺仪的偏移点
	//没校准过就会自动校准
	calibrationGyro();

	return 0;
}
/***
 * @brief 校准陀螺仪
 * 
 * 
*/
void calibrationGyro(void){
	//校准512次 取平均值 如果最大最小值波动小于阈值 校准成功 否则重新校准直到校准成功
	while(gyroInitFin==0){
		//学习时候 G灯闪烁
		//没有学习成功会一直学习
		int gyroSum = 0;
		int gyroMin =INT32_MAX;
		int gyroMax =INT32_MIN;
		for(int i=0;i<258;i++){
			Delay_ms(1);		//1ms读一次陀螺仪 最快0.5s校准
			readQmi8658b();	
			ACC_vX = firstOrderFilter(&accXft,ACC_vX);
			ACC_vY = firstOrderFilter(&accYft,ACC_vY);
			GYRO_vZ = firstOrderFilter(&gyroZft,GYRO_vZ);
			if(GYRO_vZ>gyroMax)
				gyroMax = (int)GYRO_vZ;
			else if(GYRO_vZ<gyroMin)
				gyroMin = (int)GYRO_vZ;
			gyroSum += (int)GYRO_vZ;
		}
		LEDG_Xor();		//学习完一次绿灯切换一次
		gyroSum -=gyroMin; 
		gyroSum -=gyroMax;
		gyroSum >>=8; 
		if((gyroMax-gyroMin)<gyroCaliErr){
			//陀螺仪误差在一个很小范围内
			SetLearnGyroZBais((int16_t)gyroSum);
			gyroInitFin = 1;
		}else{
			//如果不在误差范围内
			if(GetLearnState()==1){
				//如果学习完成可以直接退出 否则继续校准陀螺仪
				gyroInitFin = 1;
			}
		}
		LEDG_Set();	//结束时候绿灯亮
		//校准完了算出当前初始角度 直接角速度角度为初始角度
		//重新算出初始角度
		lastOriA = (int)arctan(ACC_vX,ACC_vY);	//算出加速度角度
	}
}
/**
 * @brief 从qmi8658b 读出陀螺仪 和 加速度数据 和后面函数配合使用读出对应数据
 * 
*/
void readQmi8658b(void){
	uint8_t data[14];
	it.len =14;
	it.data_adr = QMI8658B_TEMP_OUT_L;
	it.data = data;
	i2cRead(&it);
	TEMP	= (int16_t)(it.data[0]|((uint16_t)it.data[1]<<8));
	ACC_vX	= (int16_t)(it.data[2]|((uint16_t)it.data[3]<<8));
	ACC_vY	= (int16_t)(it.data[4]|((uint16_t)it.data[5]<<8));
	ACC_vZ	= (int16_t)(it.data[6]|((uint16_t)it.data[7]<<8));
	GYRO_vZ	= (int16_t)(it.data[12]|((uint16_t)it.data[13]<<8));
	//中值滤波加一阶滤波 滤波 ACC&GYRO
	//accXft.alpha_diff = accFilter;
	//accYft.alpha_diff = accFilter;
	//gyroZft.alpha_diff = gyroFilter;
	//firstOrderFilter(&accXft);
	//firstOrderFilter(&accYft);
	//firstOrderFilter(&gyroZft);
}
int16_t GetACC_X(void){
	return ACC_vX;
}
int16_t GetACC_Y(void){
	return ACC_vY;
}
int16_t GetACC_Z(void){
	return ACC_vZ;
}
int16_t GetGYRO_Z(void){
	return GYRO_vZ;
}
/***
 * @brief 获取陀螺仪算出的角度
 * 固定采样时间位1000us 固定参数方便整型运算 DPS 512 固定参数的值
 * DPS/32768 * dt * 65536/360 * vz = 积分的角度值
 * 
 * 
 * 
*/
int16_t getOrientation_1ms(void){
	if(gyroInitFin==1){
		readQmi8658b();	//读出参数	
		//Acc X&Y 算出加速度轴的角度
		int accA = (int)arctan(ACC_vX,ACC_vY)*5625;	//算出加速度角
		//DPS = 512  DPS/32768 * vZ * time(1000us)*351.5625*16*65536/360 = 增加的角度*6525
 		//gyro = lastgyro*5625 + addgyro*16
		int gyroA = (int)(GYRO_vZ-GetLearnGyroZBais())<<4 + (int)lastOriA*5625;		//本次Z轴加速度
		gyroA = complementFilter(gyroA,accA);
		lastOriA = (int16_t)(gyroA/5625);	//算出当前实际角度
		return lastOriA;
	}
	return 0;
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
