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
#include 	"button.h"

#if AccXY_dir == 0
#define ACCXY	ACC_vX,ACC_vY
#elif AccXY_dir == 1
#define ACCXY	ACC_vX,-ACC_vY
#elif AccXY_dir == 2
#define ACCXY	-ACC_vX,ACC_vY
#elif AccXY_dir == 3
#define ACCXY	-ACC_vX,-ACC_vY
#elif AccXY_dir == 4
#define ACCXY	ACC_vY,ACC_vX
#elif AccXY_dir == 5
#define ACCXY	ACC_vY,-ACC_vX
#elif AccXY_dir == 6
#define ACCXY	-ACC_vY,ACC_vX
#else 
#define ACCXY	-ACC_vY,-ACC_vX
#endif

#define accX_alp_raw    1000    //6000    //当前滤波系数
#define accX_alp_min    70    //6000    //最小滤波系数
#define accX_alp_max    65535    //最大滤波系数
#define accY_alp_raw    1000    //6000    //当前滤波系数
#define accY_alp_min    70    //6000    //最小滤波系数
#define accY_alp_max    65535    //最大滤波系数
#define gyroZ_alp_raw    1000    //当前滤波系数
#define gyroZ_alp_min    500    //最小滤波系数
#define gyroZ_alp_max    65535    //最大滤波系数

#define accvq2_Min  0x3f00*0x3f00
#define accvq2_Max  0x4100*0x4100
#define accvq2_Mid  0x4000*0x4000

#define MinGyroRun	5	//最小陀螺仪动作幅度

#define accMaxPro   16  //32/256 最大可信度
//陀螺仪参数滤波主要滤掉低频 低误差时候滤波系数小 高误差时候滤波系数大
//陀螺仪滤波系数表格 只需要过滤掉 变化大的  ?/65536(滤波系数)=>每次增加的滤波系数
const int16_t gyroFilter[] = {
    60,180,670,1800,6000,10000
};
//对应不同误差的滤波系数增量
const int16_t gyroFilterV[]={
    42,102,250,1000,3700,6000,9000
};
//加速度滤波 滤波系数要大,不能增长过快
//加速率滤波表格
const int16_t accFilter[] = {
    80,560,1820,4640,8500,12000
};
const int16_t accFilterV[] = {
    60,120,200,350,600,800,1000
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
#define QMI8658B_REVID             	0x01
#define QMI8658B_REVID_V           	0x7c
#define QMI8658B_RESET				0x60
#define QMI8658B_RESET_V			0xb0	//0x4b reset

#define QMI8658B_ConfReg			0x02
#define QMI8658B_ConfReg_ADR_AI		0x40	//1 addr auto add
#define QMI8658B_ConfReg_ADR_BE		0x20	//read Big-endian
#define QMI8658B_Dis_HiOSC			0x01

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

#define QMI8658B_GYRO_Set			0x04
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

#define QMI8658B_LPF				0x06
#define QMI8658B_LPF_AccEn			0x01
#define QMI8658B_LPF_Acc_ODR_2d66	0x00
#define QMI8658B_LPF_Acc_ODR_3d63	0x02
#define QMI8658B_LPF_Acc_ODR_5d39	0x04
#define QMI8658B_LPF_Acc_ODR_13d37	0x06
#define QMI8658B_LPF_GYROEn			0x10
#define QMI8658B_LPF_GYRO_ODR_2d66	0x00
#define QMI8658B_LPF_GYRO_ODR_3d63	0x20
#define QMI8658B_LPF_GYRO_ODR_5d39	0x40
#define QMI8658B_LPF_GYRO_ODR_13d37	0x60

#define QMI8658B_EN_Sensors			0x08
#define QMI8658B_GYRO_EN			0x02
#define QMI8658B_ACC_EN				0x01



//*****************************************
//float accelOneG = 9.8065;           // 
//int16_t accelData500Hz[3];

//float accelTCBias[3] = { 0.0f, 0.0f, 0.0f };
static i2c_t it;
static int16_t GYRO_vZ = 0;
static int16_t ACC_vX = 0;
static int16_t ACC_vY = 0;
static int16_t ACC_vZ = 0;
//static int16_t TEMP = 0;
static int16_t lastOriA=0;	//上次计算出的角度
static uint8_t gyroInitFin=0;	//陀螺仪初始化完成标志
static int accA = 0;	//本次加速度算出的脚位
static int gyroA = 0;	//本次陀螺仪算出的脚位
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
		uint8_t ret = 0;
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
	it.delay = 0;
	//it.iic_adr = QMI8658B_ADDRESS;
	//it.len =1;
	//it.data_adr = QMI8658B_RESET;
	//it.data = &dat;
	//i2cWrite(&it);		//reset
	writeQMIregInit(&it);
	//it.data_adr = QMI8658B_WHOAMI;
	//it.data = &ret;
	//i2cRead(&it);	
	writeQMIreg(&it,QMI8658B_RESET,QMI8658B_RESET_V);
	Delay_ms(250);
	//it.data_adr = QMI8658B_ACC_Set;
	//dat = (QMI8658B_ACC_FS_4G|QMI8658B_ACC_ODR_896HZ);
	//it.data = &dat;
	//i2cWrite(&it);		//reset
	writeQMIreg(&it,QMI8658B_ConfReg,QMI8658B_ConfReg_ADR_AI);
	writeQMIreg(&it,QMI8658B_ACC_Set,(QMI8658B_ACC_FS_2G|QMI8658B_ACC_ODR_896HZ));
	//it.data_adr = QMI8658B_GYRO_Set;
	//it.data = (QMI8658B_GYRO_FS_512DPS|QMI8658B_GYRO_ODR_7174HZ);
	//i2cWrite(&it);		//reset
	writeQMIreg(&it,QMI8658B_GYRO_Set,(QMI8658B_GYRO_FS_512DPS|QMI8658B_GYRO_ODR_7174HZ));
	//it.data_adr = QMI8658B_EN_Sensors;
	//it.data = (QMI8658B_GYRO_EN|QMI8658B_ACC_EN);
	//i2cWrite(&it);		//reset
	//writeQMIreg(&it,QMI8658B_LPF,(QMI8658B_LPF_AccEn|QMI8658B_LPF_Acc_ODR_2d66|QMI8658B_LPF_GYROEn|QMI8658B_LPF_GYRO_ODR_13d37));
	writeQMIreg(&it,QMI8658B_LPF,(QMI8658B_LPF_AccEn|QMI8658B_LPF_Acc_ODR_2d66));
	writeQMIreg(&it,QMI8658B_EN_Sensors,(QMI8658B_GYRO_EN|QMI8658B_ACC_EN));
	
	//it.data_adr = QMI8658B_ACC_Set;
	//it.data = &ret;
	//i2cRead(&it);
	//it.data_adr = QMI8658B_GYRO_Set;
	//it.data = &ret;
	//i2cRead(&it);
	//it.data_adr = QMI8658B_REVID;
	//it.data = &ret;
	//i2cRead(&it);
	it.data_adr = QMI8658B_WHOAMI;
	it.data = &ret;
	i2cRead(&it);
	if(it.data[0] != QMI8658B_WHOAMI_V){
		return 0xff;	//fail
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

	Delay_ms(200);
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
 * @brief 写qmi 的寄存器参数
 * 
 * 
*/
void writeQMIregInit(i2c_t *it){
	it->iic_adr = QMI8658B_ADDRESS;
	it->len =1;
}
void writeQMIreg(i2c_t *it,uint8_t adr,uint8_t dat){
	it->data_adr = adr;
	it->data = &dat;
	i2cWrite(it);		//reset
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
		for(int i=0;i<514;i++){
			Delay_ms(1);		//1ms读一次陀螺仪 最快0.5s校准
			fScanButton();
			readQmi8658b();	
			//ACC_vX = firstOrderFilter(&accXft,ACC_vX);
			//ACC_vY = firstOrderFilter(&accYft,ACC_vY);
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
		gyroSum >>=9; 
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
	}
	LEDG_Set();	//结束时候绿灯亮
		//校准完了算出当前初始角度 直接角速度角度为初始角度
		//重新算出初始角度
	lastOriA = (int)arctan(ACCXY);	//算出加速度角度
}
/**
 * @brief 从qmi8658b 读出陀螺仪 和 加速度数据 和后面函数配合使用读出对应数据
 * 
*/
void readQmi8658b(void){
	uint8_t data[12];
	it.len =12;
	it.data_adr = QMI8658B_ACCEL_XOUT_L;	//QMI8658B_TEMP_OUT_L;
	it.data = data;
	i2cRead(&it);
	//TEMP	= (int16_t)(it.data[0]|((uint16_t)it.data[1]<<8));
	ACC_vX	= (int16_t)((uint16_t)it.data[0]|((uint16_t)it.data[1]<<8));
	ACC_vY	= (int16_t)((uint16_t)it.data[2]|((uint16_t)it.data[3]<<8));
	ACC_vZ	= (int16_t)((uint16_t)it.data[4]|((uint16_t)it.data[5]<<8));
	GYRO_vZ	= (int16_t)((uint16_t)it.data[10]|((uint16_t)it.data[11]<<8));
	//中值滤波加一阶滤波 滤波 ACC&GYRO
	//accXft.alpha_diff = accFilter;
	//accYft.alpha_diff = accFilter;
	//gyroZft.alpha_diff = gyroFilter;
	//ACC_vX = firstOrderFilter(&accXft,ACC_vX);
	//ACC_vY = firstOrderFilter(&accYft,ACC_vY);
	//GYRO_vZ = firstOrderFilter(&gyroZft,GYRO_vZ);
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
 * @brief 解码加速度是否处于匀速/稳定范围 稳定时候占5%比例 不稳定不能用
 * 稳定时候还可以慢慢校准陀螺仪中点
 * 
 * 
*/
int8_t CheckCorrect(void){
	int vx = (int16_t)ACC_vX;
	int vy = (int16_t)ACC_vY;
	int vz = (int16_t)ACC_vZ;
	static uint8_t count=0;
	int vq2 = vx*vx + vy*vy + vz*vz;
	//模值在有效范围内可信 不在范围内不可信, 只要有一次不可信就不可信
	//持续可信 就真的可信
	if((vq2>accvq2_Min)&&(vq2<accvq2_Max)){
		if(count<8){
			count++;
			return 0;	//不可信
		}
		return 1;	//可信
	}else{
		count = 0;
		return 0;	//不可信
	}
	//if((vq2>accvq2_Min)&&(vq2<accvq2_Max)&&(abs(GYRO_vZ-GetLearnGyroZBais())<MinGyroRun)){
	//	//在可信范围内 越接近1G 越可信 可信度越高 加速度占比越大
	//	if(count<10){
	//		count++;
	//		return 0;	//不可信
	//	}
	//	if(vq2<accvq2_Mid){
	//		return (vq2-accvq2_Min)*accMaxPro/(accvq2_Mid-accvq2_Min);
	//	}else{
	//		return (accvq2_Max-vq2)*accMaxPro/(accvq2_Max-accvq2_Mid);
	//	}
	//}else{
	//	count = 0;
	//	return 0;	//不可信
	//}
}
/***
 * @brief 获取陀螺仪算出的角度
 * 固定采样时间位1000us 固定参数方便整型运算 DPS 512 固定参数的值
 * DPS/32768 * dt * 65536/360 * vz = 积分的角度值
 * 
*/
int16_t getOrientation_1ms(void){
	//uint8_t data[2];
	if(gyroInitFin==1){
		//LEDR_Set();
		readQmi8658b();	//读出参数	
		ACC_vX = firstOrderFilter(&accXft,ACC_vX);
		ACC_vY = firstOrderFilter(&accYft,ACC_vY);
		//LEDR_Reset();
		//Acc X&Y 算出加速度轴的角度
		accA = ((int)arctan(ACCXY))*5625;	//算出加速度角
		//DPS = 512  DPS/32768 * vZ * time(1000us)*351.5625*16*65536/360 = 增加的角度*6525
 		//gyro = lastgyro*5625 + addgyro*16
		gyroA = (((int)(GYRO_vZ-GetLearnGyroZBais()))<<5) + ((int)lastOriA)*5625;		//本次Z轴加速度
		//int gyroA1 = complementFilter(gyroA,accA);
		if(gyroA<-32768*5625){
			gyroA = 65536*5625 + gyroA;	//环形2进制 越界环形回来
		}else if(gyroA>32767*5625){
			gyroA = gyroA-65536*5625;
		}
		//int gyroA1 = complementFilter(gyroA,accA);

    	//int diff = accA-gyroA;    //本次角度误差 不同误差对应不同滤波系数
		//if(diff<-32768*5625){
		//	diff = 65536*5625 + diff;
		//}else if(diff>32767*5625){
		//	diff = diff-65536*5625;
		//}
		//前提是加速度的模值在有效范围内 超出范围信任陀螺仪
		//误差越大 越信任陀螺仪 误差越小 越信任加速度(消除陀螺仪累计误差)
		//gyrp *(1-a) + acc *a = gyrp + (acc-gyrp)a
		int gyroA1;
		if(CheckCorrect()){
			//启动融合算法 误差越大 越信任陀螺仪 误差越小越信任加速度
			gyroA1 = complementFilter(gyroA,accA);
		}else{
			//不可信
			int diff = accA - gyroA;    //本次角度误差 不同误差对应不同滤波系数
   			if(diff<-32768*5625){   //误差 超出最大负数 认为是正向误差
    			diff = 65536*5625 + diff;
    		}else if(diff>32767*5625){
    			diff = diff-65536*5625;
    		}
			gyroA1 = (int)((diff)>>7)+gyroA;
    		//int result = ((diff>>8) * 255)+accA;
    		//保证结果一定是在一个正确的范围内 环形加法 不溢出
    		if(gyroA1<-32768*5625){   //误差 超出最大负数 认为是正向误差
    			gyroA1 = 65536*5625 + gyroA1;
    		}else if(gyroA1>32767*5625){
    			gyroA1 = gyroA1-65536*5625; 
    		}
			//gyroA1 = gyroA;
		}
    	//int gyroA1 = ((diff * (int)(256-CheckCorrect()))>>8)+gyroA;	//accA;
		//if(gyroA1<-32768*5625){
		//	gyroA1 = 65536*5625 + gyroA1;
		//}else if(gyroA1>32767*5625){
		//	gyroA1 = gyroA1-65536*5625;
		//}
		lastOriA = (int16_t)(gyroA1/5625);	//算出当前实际角度
		//data[0] = 0xaa;
		//data[1] = (uint8_t)(lastOriA>>8);
		//data[2] = (uint8_t)(lastOriA&0xff);
      	//UartSendDatas(data,3);
		return lastOriA;
	}
	return 0;
}
/***
 * @brief 对加速度角度进行一阶滤波
 * 
 * 
*/
//int16_t accA_Sample(filter_t *ft,int16_t raw){
//    static int8_t accAInit=0; //物理角度滤波初始化
//    if(accAInit==0){
//        ft->filter = raw;  //初始化上次速度
//        ft->alpha_diff = accFilter;
//        ft->alpha_diff_addV = accFilterV;
//        ft->alpha_diff_len = sizeof(accFilter)/sizeof(int16_t);
//        ft->alpha_raw = MecA_alp_raw;
//        ft->alpha_min = MecA_alp_min;
//        ft->alpha_max = MecA_alp_max
//        accAInit = 1;
//    }else{
//        //求出一阶滤波后速度
//        firstOrderFilter(ft,raw);
//    }
//    return ft->filter;
//}
/***
 * @brief 获取陀螺仪算出的角度
 * 
 * 
*/
int16_t GetOriGyroA(void){
	return lastOriA;
}
//加速度算出的角度
int16_t GetGyroA(void){
	return (int16_t)(gyroA/5625);
}
//陀螺仪积分算出的角度
int16_t GetAccA(void){
	return (int16_t)(accA/5625);
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
