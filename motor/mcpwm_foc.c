/******
 * @file mcpwm_foc.c
 *  
 * @date Created on: Aug. 12, 2025
 * @author  MaxwellWang
 * @brief foc control motor
 */

#include    "mcpwm_foc.h"
#include    "mc_config.h"
#include    "pid_regulator.h"
#include    "mc_math.h"
#include    "EEPROM.h"
#include "peripherals.h"
#include    "parameters_conversion.h"
#include "qmi8658b.h"

#define cTurnLCycle  0
#define cTurnLCycleLen  2
#define cTurnRCycle  2
#define cTurnRCycleLen  2
#define cTurnHorRoll  4
#define cTurnHorRollLen 3
#define cTurnVerRoll  7
#define cTurnVerRollLen  4
//static uint8_t Aligned_hall = 0;
static uint8_t IsMCCompleted = 0;
static uint8_t RunModeEn = 0;

static uint8_t vPIDInt = cPIDDiff;  //PID间隔时间 
static int16_t vDeadErr = deadErr; //死区时间
filter_t speedft;   //速度滤波 结构体
filter_t mecAft;   //物理角度滤波 结构体
//static mc_cmd mc_cmd_t = lock;
const RunModeParam RunModeParam_t[]={ \
   // {.Add=8,.EndAngle = 0x4000},{.Add=-8,.EndAngle=-0x4000} 
{.Add=8,.EndAngle=0x8000},{.Add=8,.EndAngle=0x0},\
{.Add=-8,.EndAngle=0x8000},{.Add=-8,.EndAngle=0x0},\
{.Add=-8,.EndAngle=0xa000},{.Add=8,.EndAngle=0x6000},{.Add=-8,.EndAngle=0x0},\
{.Add=8,.EndAngle=0x6000},{.Add=-8,.EndAngle=0xb000},{.Add=8,.EndAngle=0x6000},{.Add=-8,.EndAngle=0x4000}};
UpRunMode urm={0};
//当前速度
//speed 滤波表格
const int16_t speedFilter[] = {
    20,45,85,190,520,840
};
//speed 速率滤波表格
const int16_t speedFilterV[] = {
    75,130,260,600,2200,3000,7000
};
#define speed_alp_raw   1000    //当前滤波系数
#define speed_alp_min   75    //当前滤波系数
#define speed_alp_max   65535    //当前滤波系数
//物理角度滤波
const int16_t MecAFilter[] = {
    128,550,912,1524,2248,4096
};
//speed 速率滤波表格
const int16_t MecAFilterV[] = {
    55,85,150,450,900,2000,4500
};
#define MecA_alp_raw   1000    //当前滤波系数
#define MecA_alp_min   75    //当前滤波系数
#define MecA_alp_max   65535    //当前滤波系数
//左360度动作
//const uint16_t leftCycle[10]={};
void CURRENT_REGULATION_IRQHandler(void){
    /* Clear EOC flag */
    int16_t hElAngle = 0;
	//ADC_ClearITPendingBit(ADC,ADC_IT_EOC);
	ADC_ClearITPendingBit(ADC,ADC_IT_EOSEQ);
    //LEDR_Xor();
    //ADC->SQUE0_CFG1 &= (~(uint32_t)ADC_SQUE0_CFG1_QUE_DONE);
    //AD 采样出 HallX&Y  VDD
    //根据Hall 值算出当前角度
    //GetVolAD();
    #ifdef debug_int
    GPIO_Toggle(LEDR_GPIO_PORT,LEDR_GPIO_PIN);
    #endif
    if(IsMCCompleted){
        hElAngle = Get_HallAngle(&FOC_Component_M1); //从霍尔获取电角度
        //反park变换算出 a b 轴的力
	    //Valphabeta = MCM_Rev_Park(Vqd, hElAngle);
        //svpwm 换算出三相PWM
        PWMC_SetPhaseVoltage(&PWMC_Handle_M1, MCM_Rev_Park(GetVqd(), hElAngle));
    }
    #ifdef debug_int
    GPIO_Toggle(LEDR_GPIO_PORT,LEDR_GPIO_PIN);
    #endif
}

/****
 * @brief 获取实时霍尔角度 校准时候返回固定角度 正常动作时候返回当前hall采样的角度
 * @return deta  真实电角度
 * 采样时候不停的采样平均角度值
*/
int16_t Get_HallAngle(FOC_Component *fc){
    uint32_t hElAngle;
    fc->xy_now = MX_Hall_Sample((GetHallxAD()&0xFFF)<<4,(GetHallyAD()&0xFFF)<<4);
    //fc->x_now = hxy.Hallx;
    //fc->y_now = hxy.Hally;
    if(fc->lc.LearnFinish==0){
        uint16_t maxtemp = fc->x_Max;
        uint16_t mintemp = fc->x_Min;    
        if(fc->lc.learnXYFin==0){
            //fc->x_now = ((GetHallxAD()&0xFFF)<<4);  //获取X轴的最大值,最小值
            MaxMinUpDate(&fc->xy_now.Hallx,&maxtemp,&mintemp);
            fc->x_Max = maxtemp;
            fc->x_Min = mintemp;
            maxtemp = fc->y_Max;
            mintemp = fc->y_Min;
            //fc->y_now = ((GetHallyAD()&0xFFF)<<4);  //获取Y轴的最大值,最小值
            MaxMinUpDate(&fc->xy_now.Hally,&maxtemp,&mintemp);
            fc->y_Max = maxtemp;
            fc->y_Min = mintemp;
        }
        //else{  //学习误差 和方向
        //    hElAngle = GetRealElAngle(fc); //当前物理角度算出的电角度
        //}
    }else{
        //获取当前X Y 霍尔的值 换算出角度
        hElAngle = GetRealElAngle(fc); //当前物理角度算出的电角度
        //hElAngle = (uint16_t)fc->hMecAngle*fc->PolePariNum; 
        if(fc->lc.M_dir)   //电角度增加 物理角度是否方向一致 也是增加
        //offset = ela - mecEla
        fc->hElAngle = (uint32_t)((uint16_t)((int16_t)(hElAngle) + fc->lc.ElAngele_offset));
        else        //方向不一致
        //offset = mec + ela
        fc->hElAngle = (uint32_t)((uint16_t)(fc->lc.ElAngele_offset - (int16_t)(hElAngle)));
        //fc->hElAngle = CalculateLoopAddSub();
        //换算出实际电角度
    }
    return (int16_t)(fc->hElAngle);
}
/***
 * @brief 计算当前物理角度
 * 
 * @param hxy hall xy 的角度
*/
int16_t CalMecAngle(FOC_Component *fc){
    int16_t hX,hY,angle;
    hX = (int16_t)(fc->xy_now.Hallx-fc->lc.x_offset);
	hY = (int16_t)(fc->xy_now.Hally-fc->lc.y_offset);
    GetHallXYScale(&fc->lc,&hX,&hY);
    #if HallXY_dir==0
    angle = arctan(hX,hY);
    #elif HallXY_dir==1    
	angle = arctan(hX,-hY);
    #elif HallXY_dir==2
    angle = arctan(-hX,hY);
    #elif HallXY_dir==3
    angle = arctan(-hX,-hY);
    #elif HallXY_dir==4
	//LEDR_Set();
    angle = arctan(hY,hX);
	//LEDR_Reset();
    #elif HallXY_dir==5
    angle = arctan(hY,-hX);
    #elif HallXY_dir==6
    angle = arctan(-hY,hX);
    #else
    angle = arctan(-hY,-hX);
    #endif
    fc->hMecAngle = MecA_Sample(&mecAft,angle); //对计算的物理角度一阶滤波
    return fc->hMecAngle;
}
/***
 * @brief 物理角度滤波采样
 * 
 * 
*/
int16_t MecA_Sample(filter_t *ft,int16_t raw){
    static int16_t MecAInit=0; //物理角度滤波初始化
    if(MecAInit==0){
        ft->filter = raw;  //初始化上次速度
        ft->alpha_diff = MecAFilter;
        ft->alpha_diff_addV = MecAFilterV;
        ft->alpha_diff_len = sizeof(MecAFilter)/sizeof(int16_t);
        ft->alpha_raw = MecA_alp_raw;
        ft->alpha_min = MecA_alp_min;
        ft->alpha_max = MecA_alp_max;
        MecAInit = 1;
    }else{
        //求出一阶滤波后速度
        firstOrderFilter(ft,raw);
    }
    return ft->filter;
}
/****
 * 
 * 获取真实电角度,和物理角度
 * 在中点会抖动的原因就是 没有对角度做滤波的原因 现在改为 中断中只采样数据做滑动平均滤波
 * 在主程序中 再求角度 减小中断时间的占用 主程序中对角度再做一阶滤波
 * 
*/
uint32_t GetRealElAngle(FOC_Component *fc){
    //static int32_t hX=0,hY=0,hcount=0;
    #if 0
    int16_t hx1,hy1;
    #ifdef filterAV
	static int32_t hX=0,hY=0,hcount=0;
    //MX_Hall_init(int16_t xRaw,int16_t yRaw);
    hX += (int32_t)(fc->xy_now.Hallx-fc->lc.x_offset);
	hY += (int32_t)(fc->xy_now.Hally-fc->lc.y_offset);
    hcount++;
    if(hcount>=filterAVDeep){
        hcount = 0;
        hx1 = (int16_t)(hX >>LOG2(filterAVDeep)); //LOG2(filterAVDeep)
        hy1 = (int16_t)(hY >>LOG2(filterAVDeep));   //LOG2(filterAVDeep)
        hX = 0;
        hY = 0;
        GetMecAngle(fc);
    }
    #else
    GetMecAngle(fc);
    #endif
    #endif
    return (uint32_t)((uint16_t)fc->hMecAngle*fc->PolePariNum);    //当前物理角度算出的电角度
}
/***
 * 
 * 获取hall xy 的比例增益
 * 
*/
void GetHallXYScale(Learn_Componets *lc,int16_t *x,int16_t *y){
    if(lc->xyScaleDir==0){  //X>Y
        *x  =   (int16_t)(((int32_t)lc->xy_scale * (*x)) >>16);
    }else{  // X<Y
        *y  =   (int16_t)(((int32_t)lc->xy_scale * (*y)) >>16);
    }
}
//**************************************************************
//PID 控制
int16_t PosPISControl(FOC_Component *fc){
    //static int hErrCount=0;
    //static uint8_t errTime=0;
    int16_t hTorqueReference;   //生成的扭力
    int16_t hError; //位置误差
    static int16_t hSpeed; //误差对应的速度
    static int16_t posCount=0;  //位置环计次 3次位置环调整一次 速度换每次都调整
    if(fc->hAddTargetAngle!=fc->hAddActTargetAngle){
        hError = fc->hAddTargetAngle-fc->hAddActTargetAngle;
        if(hError>0){
            fc->hAddActTargetAngle += 16;
            hError = fc->hAddTargetAngle-fc->hAddActTargetAngle;
            if(hError<0)
                fc->hAddActTargetAngle = fc->hAddTargetAngle;
        }else{
            fc->hAddActTargetAngle -= 16;
            hError = fc->hAddTargetAngle-fc->hAddActTargetAngle;
            if(hError>0)
                fc->hAddActTargetAngle = fc->hAddTargetAngle;
        }
        //if(FOC_Component_M1.hAddTargetAngle>FOC_Component_M1.hAddActTargetAngle){
        //    FOC_Component_M1.hAddActTargetAngle += 12;
        //    if(FOC_Component_M1.hAddActTargetAngle>FOC_Component_M1.hAddTargetAngle)
        //        FOC_Component_M1.hAddActTargetAngle = FOC_Component_M1.hAddTargetAngle;
        //}else{
        //    FOC_Component_M1.hAddActTargetAngle -= 12;
        //    if(FOC_Component_M1.hAddActTargetAngle<FOC_Component_M1.hAddTargetAngle)
        //        FOC_Component_M1.hAddActTargetAngle = FOC_Component_M1.hAddTargetAngle;
        //}
    }
    int16_t realyAngle = fc->hAddActTargetAngle;
    if(fc->hAddActTargetAngle>0){
        realyAngle = (realyAngle * 64200) >>16;
    }else{
        realyAngle = (realyAngle * 63536) >>16;
    }
    if(fc->lc.M_dir){
        //电角度和物理角度同相变化
        #ifdef GyroEn
        hError = -GetOriGyroA() + realyAngle;
        #else
        hError = -fc->hMecAngle + realyAngle;
        #endif
    }
    else{
        #ifdef GyroEn
        hError = GetOriGyroA() - realyAngle;
        #else
        hError = fc->hMecAngle - realyAngle;
        #endif
    }

    //PID调试 直接误差PWM 输出改为PWM的增量 以上次为基准 增/减

    #ifdef posLoop  //位置环
    hTorqueReference = PID_Controller(&PIDPosHandle_M1, ( int32_t )hError);
    #else
    //不同的角度误差 => 对应不同速度 位置环 
    //速度环 转速熊超过1min 1转 1s->1/6转 err 对应角度的反数 角度期望总是要无限接近0
    if(++posCount>fc->posCount){
        posCount = 0;
        hSpeed = PID_Controller(&PIDPosHandle_M1, ( int32_t )hError);
    }
    
    //限制一下最大速度 
    //if(hError>vDeadErr)
    //hSpeed = 2000;    //匀速 不管在哪里都是匀速处理
    //else if(hError<-vDeadErr)
    //hSpeed = -2000;
    //else{   //速度逐渐变到目标速度
    ////    //if(hSpeed>0){
    ////    //    hSpeed -= 5;
    ////    //    if(hSpeed<0)
    ////    //        hSpeed = 0;
    ////    //}
    //    hSpeed = 0;     //只设置三种速度
    //}
    //hSpeed = -hSpeed;
    if(hSpeed>MaxPosSpeed)
        hSpeed = MaxPosSpeed;
    if(hSpeed<-MaxPosSpeed)
        hSpeed = -MaxPosSpeed;
    if((hSpeed<vDeadErr)&&(hSpeed>-vDeadErr)){
        hSpeed = 0;
    }
    //不同速度对应不同扭矩 速度环 速度恒定
    //目标速度 - 实际速度  = 当前误差速度
    //本次 
    //fc->hSpeed = fc->hMecAngle - fc->hLastMecAngle;
    //当前速度算法改为1阶滤波
    //fc->hSpeed += fc->hMecAngle - fc->hLastMecAngle;
    //速度一阶滤波
    //fc->hSpeed >>= 1;
    int16_t tempsp;
    if(fc->lc.M_dir){
        tempsp = fc->hMecAngle - fc->hLastMecAngle;
    }
    else{
        tempsp = fc->hLastMecAngle - fc->hMecAngle;
    }
    fc->hLastMecAngle = fc->hMecAngle;
    //if(tempsp>INT16_MAX){
    //    tempsp = tempsp - 65536; 
    //}else if(tempsp<INT16_MIN){
    //    tempsp = 65536 + tempsp; 
    //}
    //if((tempsp>INT16_MAX)||(tempsp<INT16_MIN)){
    //    tempsp = -tempsp;
    //}
    fc->hSpeed = Speed_Sample(&speedft,tempsp);
    fc->hSpeed = tempsp;
    //fc->hLastMecAngle = fc->hMecAngle;
    //获取目标速度和本次速度的误差
    int16_t errspeed = (hSpeed-fc->hSpeed);
    //if(errspeed>INT16_MAX){
    //    errspeed = errspeed - 65536; 
    //}else if(errspeed<INT16_MIN){
    //    errspeed = 65536 + errspeed; 
    //}
    //if((errspeed>INT16_MAX)||(errspeed<INT16_MIN))
    //    errspeed = -errspeed;
    //检测速度 如果速度也接近0 就认为已经到一个平衡点了 用上次的扭力就可以了
    //为Error增加一个死区
    //if(((hError>-vDeadErr)&&(hError<vDeadErr))&&((fc->hSpeed>-64)&&(fc->hSpeed<64))){
    //    if(errTime<100)
    //    errTime++;
    //    else{
    //        //认为打到目标 不再积分(PWM 不用再变化?)
    //        //hError = 0; //扭矩不变化 用上次的扭矩
    //        return GetTorque();    //获取上次的扭矩
    //    }
    //}else{
    //    //超出误差
    //    errTime = 0;    //超出了误差
    //}
    //速度环算出当前扭矩的增量
    hTorqueReference = PID_Controller(&PIDSpeedHandle_M1, ( int32_t )errspeed);
    #endif
    //if((hError<0x200)&&(hError>-0x200)){
    //    if(hErrCount<200)
    //    hErrCount++;
    //    else{
    //        PIDPosHandle_M1.wIntegralTerm = 0;
    //        hError = 0;
    //    }
    //}else{
    //    hErrCount=0;
    //}
    //hTorqueReference = PID_Controller(&PIDPosHandle_M1, ( int32_t )hError);
    return hTorqueReference;
}
//设置PID调节间隔
void SetSPIDInterval(int16_t in){
  vPIDInt = in;
}
//获取运行速度
int16_t GetSpeedRun(void){
  return FOC_Component_M1.hSpeed;
}
void SetDeadErr(int16_t in){
    vDeadErr = in;
}
void SetPosLoopInv(int16_t in){
    FOC_Component_M1.posCount = in;
}
//获取扭矩
int16_t GetTorque(void){
    return FOC_Component_M1.Vqd.qV_Component1;
}
/***
 * @brief 采样当前速度滤波
 * 
 * 
*/
int16_t Speed_Sample(filter_t *ft,int16_t raw){
    static int16_t speedInit=0; //速度滤波初始化
    if(speedInit==0){
        ft->filter = raw;  //初始化上次速度
        ft->alpha_diff = speedFilter;
        ft->alpha_diff_addV = speedFilterV;
        ft->alpha_diff_len = sizeof(speedFilter)/sizeof(int16_t);
        ft->alpha_raw = speed_alp_raw;
        ft->alpha_min = speed_alp_min;
        ft->alpha_max = speed_alp_max;
        speedInit = 1;
    }else{
        //求出一阶滤波后速度
        firstOrderFilter(ft,raw);
    }
    return ft->filter;
}
/***
 * 
 * motor run timing
 * 
*/
Err_FOC MotorRunControl(FOC_Component *fc){
    static uint8_t CountTime=0;
    static int16_t  offsetErr1;
    static int16_t  offsetErr2;
    static int32_t  offsetErrTmp1=0; 
    static int32_t  offsetErrTmp2=0; 
    uint32_t hElAngle=0;

    if(fc->lc.learnXYFin){
        //学习了XY值后不停计算物理角度 1ms一次并滤波处理
        CalMecAngle(&FOC_Component_M1); //计算出当前的物理角度
    }
    
    if(fc->lc.LearnFinish==0){
        //是否没有学习,没有学习要学习角度 给一个固定力旋转
        if(fc->hStepTime<fc->hDurationms){
            fc->hStepTime++;
            fc->hElAngle = 0;  //初始定位到电角度0度
            //FOC_Component_M1.hLastElAngle = 0;  //初始定位到电角度0度
            fc->Vqd.qV_Component2 = fc->hStepTime*fc->hFinalTorque/fc->hDurationms;
            fc->Vqd.qV_Component1 = 0;
            //fc->x_step = 0;
            //fc->y_step = 0;
            CountTime = 0;
            if(fc->hStepTime<50){
                fc->x_Max = fc->xy_now.Hallx;
                fc->x_Min = fc->xy_now.Hallx;
                //fc->x_Start = fc->x_now;
                fc->y_Max = fc->xy_now.Hally;
                fc->y_Min = fc->xy_now.Hally;
                //fc->y_Start = fc->y_now;
            //}else{
            //    fc->x_Start = (fc->x_now+fc->x_Start)>>1;
            //    fc->y_Start = (fc->y_now+fc->y_Start)>>1;
            }
        }else{
            //达到上升时间缓慢增加角度,扭力不变 Hallxy 不停采样最大最小值 10ms 1/1024度 10s一圈
            if(CountTime<ALIGN_TimeOnce){
                CountTime++;
            }else{
                CountTime = 0;  //达到计时时候计算出当前位置(x,y)
                if(fc->lc.learnXYFin==0){
                    fc->hElAngle += 0x80;   //0x40; //每次0.35度 开环转起来
                    if(((fc->hElAngle>>16)&0xffff)==(fc->PolePariNum+1)){
                        //极对数一对上 得出hall xy 的中点
                        fc->lc.x_offset = (((uint32_t)fc->x_Max+fc->x_Min)>>1);
                        fc->lc.y_offset = (((uint32_t)fc->y_Max+fc->y_Min)>>1);
                        uint16_t tempx = (fc->x_Max-fc->x_Min); //x max range
                        uint16_t tempy = (fc->y_Max-fc->y_Min); //y max range
                        if((tempx<0x4000)||(tempy<0x4000)){
                            //范围过小 学习不正常 重新学习
                            fc->hStepTime = 0;
                            return err_learn;
                        }
                        if(tempx>tempy){
                            fc->lc.xyScaleDir = 0;
                            fc->lc.xy_scale = (uint16_t)(((uint32_t)tempy <<16) / (uint32_t)tempx);
                        }else{
                            fc->lc.xyScaleDir = 1;
                            fc->lc.xy_scale = (uint16_t)(((uint32_t)tempx <<16) / (uint32_t)tempy);
                        }
                        //X Y 角度对上后 再旋转 比较物理角度和实际电角度偏差 算出马达方向和 偏差值
                        fc->lc.learnXYFin = 1; //XY 校准完 校准角度偏差和方向
                        //获取换算出的电角度
                        fc->hElAngle = 0x00;
                        //fc->lc.ElAngele_offset = (int16_t)fc->hElAngle + (int16_t)hElAngle;
                    }
                }else{
										//fc->hElAngle += 0x100; //每次1.4度 开环转起来
                    if(fc->hElAngle==0){
                        //获取当前物理角度算出的电角度
                        hElAngle = GetRealElAngle(fc);
                        offsetErr1 = fc->hMecAngle;     //获取本次物理角度
                        offsetErr2 = 0;
                        //当前物理角度换算出的电角度和实际电角度偏差
                        //当前电角度 - 计算出的物理角度
                        //offsetErr1 = (int32_t)((fc->hElAngle&0xffff) - (hElAngle&0xffff)); //不知道是正还是反
                        //offsetErr2 = (int32_t)((fc->hElAngle&0xffff) + (hElAngle&0xffff));
                        offsetErrTmp1 = ((int32_t)(fc->hElAngle&0xffff) - (int32_t)(hElAngle&0xffff));
                        offsetErrTmp1 = CalculateLoopAddSub(offsetErrTmp1); //算出差值
                        offsetErrTmp2 = ((int32_t)(fc->hElAngle&0xffff) + (int32_t)(hElAngle&0xffff));
                        offsetErrTmp2 = CalculateLoopAddSub(offsetErrTmp2); //算出差值
					    fc->hElAngle += 0x80; //每次1.4度 开环转起来
                    }else{
                        fc->hElAngle += 0x80; //每次1.4度 开环转起来
                        hElAngle = GetRealElAngle(fc);
                        //本次物理角度和上次物理角度差值
                        offsetErr2 += (fc->hMecAngle - offsetErr1);     //获取本次物理角度
                        offsetErr1 = fc->hMecAngle;     //获取本次物理角度
                        //算出电角度的偏差
                        offsetErrTmp1 += CalculateLoopAddSub(((int32_t)(fc->hElAngle&0xffff) - (int32_t)(hElAngle&0xffff))); //算出差值
                        offsetErrTmp2 += CalculateLoopAddSub(((int32_t)(fc->hElAngle&0xffff) + (int32_t)(hElAngle&0xffff))); //算出差值
                        //offsetErrTmp1 += offsetErr2;    //差值的和
                        //offsetErr2 += offsetErrTmp2;
                        if(fc->hElAngle==0x1000){
                            //前面加了32次
                            offsetErrTmp1 = offsetErrTmp1>>5; 
                            offsetErrTmp2 = offsetErrTmp2>>5; 
                            //offsetErr1 = offsetErr1>>5;
                            //offsetErr2 = offsetErr2>>5;
                            //谁的平均误差小 说明
                            if(offsetErr2>0){
                                //电角度增大 物理角度也是增加
                                //正向误差小
                                fc->lc.M_dir = 1;   //马达方向正    实际电角度-换算电角度
                                fc->lc.ElAngele_offset = (int16_t)offsetErrTmp1;
                            }else{
                                //电角度增加 物理角度减小
                                fc->lc.M_dir = 0;
                                fc->lc.ElAngele_offset = (int16_t)offsetErrTmp2;
                            }
                            fc->lc.LearnFinish  =   1;  //学习完成
                            EE_WriteFOC(&fc->lc); //把学习的参数写入EEPROM
                        }
                    }
                }
            }
        }
    }else{
		fc->hStepTime++;
        if(fc->hStepTime>(uint16_t)vPIDInt){
            fc->hStepTime = 0;
            //位置PID 当前角度和 目标角度之间的误差
            //增量PID 算出的是当前扭矩的增量
            //int16_t tpVq = PosPISControl(fc);   //当前扭力的增量
            //每次扭力不能超过限制值?
            //增量算出新扭力?
            //fc->Vqd.qV_Component1 = CalculateAdd16(fc->Vqd.qV_Component1,tpVq);
            //每次更新扭力
            fc->Vqd.qV_Component1 = PosPISControl(fc);   //当前扭力的增量
            fc->Vqd.qV_Component2 = 0;
        }
    }
    return no_err;
}
/***
 * 
 * 开始学习角度等数据
*/
void MC_learnHall(FOC_Component *fc){
    fc->lc.learnXYFin  = 0;
    fc->hStepTime = 0;
    fc->lc.LearnFinish = 0;
}


/***
 * @brief 马达运行任务
 * 
*/
void MC_RunMotorControlTasks(void){
    //马达一直动作? 
    //static int16_t endAngle;
    if((IsMCCompleted==1)&&(GetONOFF())){
        //初始化完成
        if(RunModeEn){
            //学习时候自动增加角度功能
            RunModeEn = UpNextRunModeAngle(&urm);
        }
        #ifndef testQMI
        //int16_t oriA = getOrientation_1ms();
        //当前陀螺仪多少度 目标值是当前角度-陀螺仪的角度 = 目标角度 误差就是陀螺仪角度的反向
        //FOC_Component_M1.hTargetAngle = -GetOriGyroA();   //getOrientation_1ms();    //当前陀螺仪的角度
        //FOC_Component_M1.hTargetAngle += FOC_Component_M1.hAddTargetAngle;
        else{
            #ifdef GyroEn
            FOC_Component_M1.hTargetAngle = FOC_Component_M1.hMecAngle - GetOriGyroA();
            FOC_Component_M1.hTargetAngle += FOC_Component_M1.hAddTargetAngle;
            #else
            FOC_Component_M1.hTargetAngle = FOC_Component_M1.hAddTargetAngle;
            #endif
        }
        #endif
    #if 0
        switch(mc_cmd_t){
            case lock:
            break;
            case leftCycle:
                FOC_Component_M1.hTargetAngle = FOC_Component_M1.hMecAngle+0x8; //每次增加1/8192度 一圈时间8s?
                if(FOC_Component_M1.hTargetAngle == endAngle){
                    mc_cmd_t = leftCycle1;
                    endAngle = 0;
                }
            break;
            case leftCycle1:
                FOC_Component_M1.hTargetAngle = FOC_Component_M1.hMecAngle+0x8; //每次增加1/8192度 一圈时间8s?
                if(FOC_Component_M1.hTargetAngle == endAngle){
                        mc_cmd_t = lock;
                }
            break;
            case rightCycle:
                FOC_Component_M1.hTargetAngle = FOC_Component_M1.hMecAngle-0x8; //每次增加1/8192度 一圈时间8s?
                if(FOC_Component_M1.hTargetAngle == endAngle){
                    mc_cmd_t = rightCycle1;
                    endAngle = 0;
                }
            break;
            case rightCycle1:
                FOC_Component_M1.hTargetAngle = FOC_Component_M1.hMecAngle-0x8; //每次增加1/8192度 一圈时间8s?
                if(FOC_Component_M1.hTargetAngle == endAngle){
                    mc_cmd_t = lock;
                }
            break;
            case Hor_LR_roll:
                FOC_Component_M1.hTargetAngle = FOC_Component_M1.hMecAngle - 0x8; //每次增加1/8192度 一圈时间8s?
                if(FOC_Component_M1.hTargetAngle == endAngle){
                    mc_cmd_t = Hor_LR_roll1;
                    endAngle = 0x6000;
                }
            break;
            case Hor_LR_roll1:
                FOC_Component_M1.hTargetAngle = FOC_Component_M1.hMecAngle + 0x8; //每次增加1/8192度 一圈时间8s?
                if(FOC_Component_M1.hTargetAngle == endAngle){
                    mc_cmd_t = Hor_LR_roll2;
                    endAngle = 0;
                }
            break;
            case Hor_LR_roll2:
                FOC_Component_M1.hTargetAngle = FOC_Component_M1.hMecAngle - 0x8; //每次增加1/8192度 一圈时间8s?
                if(FOC_Component_M1.hTargetAngle == endAngle){
                    mc_cmd_t = lock;
                }
            break;
            case Ver_LR_roll:
                FOC_Component_M1.hTargetAngle = FOC_Component_M1.hMecAngle + 0x8; //每次增加1/8192度 一圈时间8s?
                if(FOC_Component_M1.hTargetAngle == endAngle){
                    mc_cmd_t = Ver_LR_roll1;
                    endAngle = 0xb000;
                }
            break;
            case Ver_LR_roll1:
                FOC_Component_M1.hTargetAngle = FOC_Component_M1.hMecAngle - 0x8; //每次增加1/8192度 一圈时间8s?
                if(FOC_Component_M1.hTargetAngle == endAngle){
                    mc_cmd_t = Ver_LR_roll2;
                    endAngle = 0x6000;
                }
            break;
            case Ver_LR_roll2:
                FOC_Component_M1.hTargetAngle = FOC_Component_M1.hMecAngle + 0x8; //每次增加1/8192度 一圈时间8s?
                if(FOC_Component_M1.hTargetAngle == endAngle){
                    mc_cmd_t = Ver_LR_roll3;
                    endAngle = 0x4000;
                }
            break;
            case Ver_LR_roll3:
                FOC_Component_M1.hTargetAngle = FOC_Component_M1.hMecAngle - 0x8; //每次增加1/8192度 一圈时间8s?
                if(FOC_Component_M1.hTargetAngle == endAngle){
                    mc_cmd_t = lock;
                }
            break;
        }
    #endif
        MotorRunControl(&FOC_Component_M1);
    }
}
/**
 * @brief 更新下一个角度和动作位置
 * @param add 增量
 * @param enAngle 结束角度
 * @param nextAngle 下一次角度
 * @param runmode 下一次模式
 * @return 0: fail 1: over
*/
uint8_t UpNextRunModeAngle(UpRunMode *rm){
    rm->fc->hTargetAngle = rm->fc->hMecAngle + RunModeParam_t[rm->step].Add; //每次增加1/8192度 一圈时间8s?
    if(rm->fc->hTargetAngle == RunModeParam_t[rm->step].EndAngle){
        rm->step++;
        if(rm->step>rm->OverStep){
            return 0;   //over
        }
    }
    return 1;
}

void mcpwm_foc_init(void) {
/******************************************************/
/*   PID component initialization: speed regulation   */
/******************************************************/
    //位置PID
    PID_HandleInit(&PIDPosHandle_M1);
    PID_HandleInit(&PIDSpeedHandle_M1);
    /* disable IT and flags  */
    //ADC_ClearITPendingBit(ADC, ADC_IT_EOC);
    /* Enables the ADC peripheral */
    //ADC_Cmd(ADC, ENABLE);
    //while ( (ADC->ISR & ADC_IT_ADRDY) == RESET )
    //{
        /* wait */
    //}
    //PWM 输出刹车
    PWMC_ONPWM();   //开启PWM ADC 采样
    //EE_WriteFOC(&FOC_Component_M1.lc);
    //EE_ReadFOC(&FOC_Component_M1.lc);
    IsMCCompleted = 1;
    //if(Aligned_hall==0){
        //霍尔 对应角度位置没有校准 要进行一次校准
        //给固定一个角度力慢慢旋转1周 不断更新hall X Y 轴数据
    //}
}

/***
 * 
 * 正转360度, 每次比当前增加0x1000度 达到再增加 每=
 * 
*/
void SetTurnLeftCycle(void){
    RunModeEn = 1;
    urm.step= cTurnLCycle;    //开始位置
    urm.OverStep= cTurnLCycle+cTurnRCycleLen;    
}
/***
 * 
 * 正转360度, 每次比当前增加0x1000度 达到再增加 每=
 * 
*/
void SetTurnRightCycle(void){
    RunModeEn = 1;
    urm.step= cTurnRCycle;    //开始位置
    urm.OverStep= cTurnRCycle+cTurnLCycleLen;    
}
/***
 * @brief 水平还是垂直 摇摆动作
 * 
*/
void HorOrVerRoll(void){
    if(FOC_Component_M1.hTargetAngle==0){
        SetTurnHorRoll();
    }else{
        SetTurnVerRoll();
    }
}
/***
 * 
 * 正转360度, 每次比当前增加0x1000度 达到再增加 每=
 * 
*/
void SetTurnHorRoll(void){
    RunModeEn = 1;
    urm.step= cTurnHorRoll;    //开始位置
    urm.OverStep= cTurnHorRoll+cTurnHorRollLen;    
}
/***
 * 
 * 反转360度, 每次比当前增加0x1000度 达到再增加 每=
 * 
*/
void SetTurnVerRoll(void){
    RunModeEn = 1;
    urm.step= cTurnVerRoll;    //开始位置
    urm.OverStep= cTurnVerRoll+cTurnVerRollLen;    
}
