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
#define cTurnLCycleLen  4
#define cTurnRCycle  4
#define cTurnRCycleLen  4
#define cTurnHorRoll  8
#define cTurnHorRollLen 6
#define cTurnVerRoll  14
#define cTurnVerRollLen  7
//static uint8_t Aligned_hall = 0;
static uint8_t IsMCCompleted = 0;
static uint8_t RunModeEn = 0;

static uint8_t vPIDInt = cPIDDiff;  //PID间隔时间 
static int16_t vDeadErr = deadErr; //死区时间
//static uint16_t vddAD;
filter_t speedft;   //速度滤波 结构体
filter_t mecAft;   //物理角度滤波 结构体
filter_t vddft;   //物理角度滤波 结构体
//static mc_cmd mc_cmd_t = lock;
const int16_t RunModeParam_t[]={ \
   // {.Add=8,.EndAngle = 0x4000},{.Add=-8,.EndAngle=-0x4000} 
    0x4000,0x4000,0x4000,0x4000,\
    -0x4000,-0x4000,-0x4000,-0x4000,\
    -0x4000,-0x2000,0x4000,0x4000,0x4000,-0x6000,\
    0x2000,-0x4000,-0x4000,-0x2500,0x4500,0x6000,-0x2000
};
UpRunMode urm={0};
//xy_Componets *xyc_t;
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
    //128,550,912,1524,2248,4096
    256,550,1024,2048,4096,8192
};
//speed 速率滤波表格
const int16_t MecAFilterV[] = {
    //55,85,150,450,900,2000,4500
    55,85,130,200,300,550,4500
};
#define MecA_alp_raw   1000    //当前滤波系数
#define MecA_alp_min   60   //75    //当前滤波系数
#define MecA_alp_max   65535    //当前滤波系数
//电压滤波
const int16_t vddFilter[] = {
    228,550,1312,2524,4248,7096
};
//电压滤波参数
const int16_t vddFilterV[] = {
    55,85,150,300,500,900,1500
};
#define vdd_alp_raw   1000    //当前滤波系数
#define vdd_alp_min   60   //75    //当前滤波系
#define vdd_alp_max   65535   //75    //当前滤波系
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
    FOC_Component_M1.vddAD = GetVolAD()&0xfff;
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
    //int32_t hElAngle;
    fc->xy_now = MX_Hall_Sample((GetHallxAD()&0xFFF)<<4,(GetHallyAD()&0xFFF)<<4);
    //fc->x_now = hxy.Hallx;
    //fc->y_now = hxy.Hally;
    if(fc->lc->LearnFinish==0){
        if(fc->lc->learnXYFin==0){
            //更新最大最小值
			uint16_t maxtemp = fc->lc->Max.Hallx;
			uint16_t mintemp = fc->lc->Min.Hallx;
            MaxMinUpDate(&fc->xy_now.Hallx,&maxtemp,&mintemp);
            fc->lc->Max.Hallx = maxtemp;
            fc->lc->Min.Hallx = mintemp;
            maxtemp = fc->lc->Max.Hally;
            mintemp = fc->lc->Min.Hally;
            MaxMinUpDate(&fc->xy_now.Hally,&maxtemp,&mintemp);
            fc->lc->Max.Hally = maxtemp;
            fc->lc->Min.Hally = mintemp;
        }
    }
    //正常直接得到电角度
    //else{
    //    //获取当前X Y 霍尔的值 换算出角度
    //    hElAngle = GetRealElAngle(fc); 
    //    //hElAngle = fc->hMecAngle*fc->PolePariNum;
    //    int16_t ElA = (int16_t)hElAngle;    //当前物理角度算出的电角度
    //    //hElAngle = GetRealElAngle(fc); //当前物理角度算出的电角度
    //    //hElAngle = (uint16_t)fc->hMecAngle*fc->PolePariNum; 
    //    int16_t tmp;
    //    if(fc->lc.M_dir){   //电角度增加 物理角度是否方向一致 也是增加
    //    //offset = ela - mecEla
    //        tmp = ElA + fc->lc.ElAngele_offset;
    //        fc->hElAngle = (uint32_t)(tmp);
    //    }
    //    else{        //方向不一致
    //    //offset = mec + ela
    //        tmp = fc->lc.ElAngele_offset - ElA;
    //        fc->hElAngle = (uint32_t)(tmp);
    //    }
    //    //fc->hElAngle = CalculateLoopAddSub();
    //    //换算出实际电角度
    //}
    return (int16_t)(fc->hElAngle);
}
/**
 * @brief 根据当前XY算出角度 当前物理角度
 */
int16_t CalXYAngle(FOC_Component *fc,HallXYs *xynow){
    int16_t hX,hY;
    uint16_t x_offset,y_offset;
    uint16_t x_Amp,y_Amp;
    x_offset = (uint16_t)(((uint32_t)fc->lc->Min.Hallx + (uint32_t)fc->lc->Max.Hallx)>>1);
    y_offset = (uint16_t)(((uint32_t)fc->lc->Min.Hally + (uint32_t)fc->lc->Max.Hally)>>1);
    x_Amp = (fc->lc->Max.Hallx - fc->lc->Min.Hallx)>>1;  //x amp
    y_Amp = (fc->lc->Max.Hally - fc->lc->Min.Hally)>>1;  //y amp
    hX = (int16_t)(xynow->Hallx-x_offset);
    hY = (int16_t)(xynow->Hally-y_offset);
    if(x_Amp>y_Amp){
        hX = hX * y_Amp / x_Amp;
    }else{
        hY = hY * x_Amp / y_Amp;
    }
    return arctan(hY,hX);  //求出当前XY 对应相应极对的电角度   
}
/***
 * @brief 计算当前 xy 值的电角度
 * 高16bit is polepair 在第几极对
 * 新程序中不需要准确知道物理角度 只需要知道xy对应的电角度
 * 
 * 
*/
int16_t CalElAngle(FOC_Component *fc){
    HallXYs xynow = fc->xy_now; //采样到的XY值
    int16_t atmp = CalXYAngle(fc,&xynow);   //当换算出的角度
    int16_t elA1,elA2,tmpA,tmpB;
    uint8_t i=0;  //两个角度值
    int32_t reA;
    fc->hMecAngle = MecA_Sample(&mecAft,atmp); //对计算的物理角度一阶滤波
    while(1){
        //不同极对有不同的XY->实际角度
        elA1 = fc->lc->ZeroAngle[i];   //换算出当前极对初始角度
        tmpA = atmp - elA1; //与本次的差值
        if(fc->lc->M_dir){
            //同向
            if(tmpA>=0){
                if(i==(POLE_PAIR_NUM-1))
                    i = 0;
                else
                    i++;
                elA2 = fc->lc->ZeroAngle[i];   //换算出当前极对初始角度
                tmpB = elA2 - elA1;
                if(tmpB>tmpA){ //A B 都是正
                    //范围内
                    reA =  ((int32_t)(tmpA)<<16) / (int32_t)tmpB;  //算出当前电角度
                    fc->hElAngle = (int16_t)reA;
                    return fc->hElAngle;
                }
            }else{
                //往回退
                if(i==0)
                    i = POLE_PAIR_NUM-1;
                else
                    i--;
                elA2 = fc->lc->ZeroAngle[i];   //换算出当前极对初始角度
                tmpB = elA2 - elA1;
                if(tmpB<tmpA){
                    //在范围内
                    reA =  ((int32_t)(tmpA)<<16) / (int32_t)tmpB;  //算出当前电角度
                    fc->hElAngle = 0xffff - (int16_t)reA;
                    return fc->hElAngle;
                }
            }
        }else{
            //反向
            //tmpA = atmp - elA1; //与本次的差值
            if(tmpA>=0){
                //往回退
                if(i==0)
                    i = POLE_PAIR_NUM-1;
                else
                    i--;
                elA2 = fc->lc->ZeroAngle[i];   //换算出当前极对初始角度
                tmpB = elA2 - elA1;
                if(tmpB>tmpA){
                    //在范围内
                    reA =  ((int32_t)(tmpA)<<16) / (int32_t)tmpB;  //算出当前电角度
                    int16_t tp = 0xffff - (int16_t)reA;
                    fc->hElAngle = tp;
                    return fc->hElAngle;
                }
                //不在范围内
            }else{
                //往前进
                if(i==(POLE_PAIR_NUM-1))
                    i = 0;
                else
                    i++;
                elA2 = fc->lc->ZeroAngle[i];   //换算出当前极对初始角度
                tmpB = elA2 - elA1;
                if(tmpB<tmpA){ //A B 都是负
                    //范围内
                    reA =  ((int32_t)(tmpA)<<16) / (int32_t)tmpB;  //算出当前电角度
                    fc->hElAngle = (int16_t)reA;
                    return fc->hElAngle;
                }
            }
        }
    }
    //return 0; //没有检测到相应的角度
}
/**
 * @brief 极对中角度信息学习
 * 
*/
void LearnPolePairAngle(FOC_Component *fc,HallXYs xynow){
    static int16_t El_90d;
    static int16_t count=0;
    //int16_t Err1,Err2;
    uint8_t num = (fc->hElAngle>>16);
    if(num>=fc->PolePairNum){
        num -= fc->PolePairNum;
        int16_t tmp = El_90d - fc->lc->ZeroAngle[0];
        if(tmp>0){
            //方向一致
            //Err1 = 0-El_0d;
            //Err2 = 0x4000-El_90d;
            //Err1 = (int16_t)(((int32_t)Err1+(int32_t)Err2)>>1);
            fc->lc->M_dir = 1; //same
            //xyc->ElAngleOffset[num] = Err1;
        }else{
            //方向不一致
            //Err1 = El_0d;
            //Err2 = 0x4000+El_90d;
            //Err1 = (int16_t)(((int32_t)Err1+(int32_t)Err2)>>1);
            fc->lc->M_dir = 0;   //diff
            //xyc->ElAngleOffset[num] = Err1;
        }
        //这里已经完整的转了一圈了 找出最大最小值算出角度
        fc->lc->ZeroAngle[0] = CalXYAngle(fc,&fc->xyZero[0]);
        fc->lc->ZeroAngle[1] = CalXYAngle(fc,&fc->xyZero[1]);
        fc->lc->ZeroAngle[2] = CalXYAngle(fc,&fc->xyZero[2]);
        fc->lc->ZeroAngle[3] = CalXYAngle(fc,&fc->xyZero[3]);
        fc->lc->ZeroAngle[4] = CalXYAngle(fc,&fc->xyZero[4]);
        fc->lc->ZeroAngle[5] = CalXYAngle(fc,&fc->xyZero[5]);
        fc->lc->ZeroAngle[6] = CalXYAngle(fc,&fc->xyZero[6]);
        fc->lc->learnXYFin = 1;
        fc->hElAngle = 0;
        fc->lc->accVx_offset = fc->accVxSum/(POLE_PAIR_NUM*10);
        fc->lc->accVy_offset = fc->accVySum/(POLE_PAIR_NUM*10);
        fc->lc->accVz_offset = fc->accVzSum/(POLE_PAIR_NUM*10);
        return;
    }
    //每次0度时记录下当前的XY
    if((fc->hElAngle&0xffff)==0){
        //每次电角度0度时候先延迟1s 校准加速度 再存入XY值
        //->只能记住XY 全部结束后才可以记忆角度
        //fc->lc->ZeroAngle[num] = CalXYAngle(fc,&xynow);      //算出0的角度
        if(fc->hElAngle==0){
            //开始学习赋值最大最小值 更新第一个极对的极值
            fc->lc->Max = xynow;
            fc->lc->Min = xynow;    //第一次直接赋值最大最小值
            fc->accVxSum = 0;
            fc->accVySum = 0;
            fc->accVzSum = 0;
        }
        count++;
		readQmi8658b();	//读出参数	
        if(count>10){
            //11-21
            fc->accVxSum += GetACC_X();
            fc->accVySum += GetACC_Y();
            fc->accVzSum += GetACC_Z();
        }
        if(count>20){   //进来一次1ms*200 = 200ms 矫正加速度值
            fc->xyZero[num] = xynow;
            fc->hElAngle += 0x80;
        }
    }else if(fc->hElAngle==0x4000){
        El_90d = CalXYAngle(fc,&xynow);      //算出0的角度
        fc->hElAngle += 0x80;
        count = 0;
    }else{
        fc->hElAngle += 0x80;
        count = 0;
    }
}
/***
 * @brief 计算当前物理角度
 * 
 * @param hxy hall xy 的角度
*/
int16_t CalMecAngle(FOC_Component *fc){
    //int16_t hX,hY,angle;
    //GetHallXYScale(&fc->lc,&fc->xy_now.Hallx,&fc->xy_now.Hally);
    HallXYs xynow = fc->xy_now; //采样到的XY值
    int16_t angle = CalXYAngle(fc,&xynow);   //当换算出的物理角度
    //hX = (int16_t)(fc->xy_now.Hallx-fc->lc.x_offset);
	//hY = (int16_t)(fc->xy_now.Hally-fc->lc.y_offset);
    //GetHallXYScale(&fc->lc,&hX,&hY);
    //hX = (int16_t)hX1;
    //hY = (int16_t)hY1;
    //#if HallXY_dir==0
    //angle = arctan(hX,hY);
    //#elif HallXY_dir==1    
	//angle = arctan(hX,-hY);
    //#elif HallXY_dir==2
    //angle = arctan(-hX,hY);
    //#elif HallXY_dir==3
    //angle = arctan(-hX,-hY);
    //#elif HallXY_dir==4
	////LEDR_Set();
    //angle = arctan(hY,hX);
	////LEDR_Reset();
    //#elif HallXY_dir==5
    //angle = arctan(hY,-hX);
    //#elif HallXY_dir==6
    //angle = arctan(-hY,hX);
    //#else
    //angle = arctan(-hY,-hX);
    //#endif
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
    return (uint32_t)((uint16_t)fc->hMecAngle*fc->PolePairNum);    //当前物理角度算出的电角度
}
/***
 * 
 * 获取hall xy 的比例增益
 * 
*/
//void GetHallXYScale(Learn_Componets *lc,int16_t *x,int16_t *y){
//    if(lc->xyScaleDir==0){  //X>Y
//        *x  =   (int16_t)((lc->xy_scale * (*x)) >>16);
//    }else{  // X<Y
//        *y  =   (int16_t)((lc->xy_scale * (*y)) >>16);
//    }
//}
/**
 * @brief 获取VDD值
 * 
*/
int16_t fGetVDDAD(){
    return FOC_Component_M1.vddAD;
}
/***
 * @brief 获取电压状态
 * 
*/
uint8_t fGetVddState(void){
    return FOC_Component_M1.vddErr;
}
void fSetVddState(uint8_t err){
    FOC_Component_M1.vddErr = err;
}
/**
 * @brief 扫描VDD
 * 
*/
void fScanVdd(void){
    static int16_t vddInit=0;
    static int16_t lvdCount=0;
    if(vddInit==0){
	    vddft.alpha_diff = vddFilter;
	    vddft.alpha_diff_addV = vddFilterV;
	    vddft.alpha_diff_len = sizeof(vddFilter)/sizeof(int16_t);
        vddft.alpha_raw = vdd_alp_raw;
        vddft.alpha_min= vdd_alp_min;
        vddft.alpha_max= vdd_alp_max;
	    vddft.filter = fGetVDDAD();
        vddInit = 1;
    }else{
        firstOrderFilter(&vddft,fGetVDDAD());
        int32_t vdd = (int32_t)GetVol_Value(vddft.filter);
        int32_t vddtemp;
        if(fGetVddState()==lvdErr)
            vddtemp = 301;
        else
            vddtemp = 331;
        if(vdd<vddtemp){
            //电池持续低于3.3v进入低压
            if(lvdCount>2000){
                if(fGetVddState()==lvdErr){
                    //直接关机
                    poweroff();
                }else{
                    fSetVddState(lvdErr);   //低压
                    lvdCount = 0; //
                }
            }else{
                lvdCount++;
            }
        }else{
            lvdCount = 0; //
        }
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
    }else{
        if(RunModeEn){
            //学习时候自动增加角度功能
            RunModeEn = UpNextRunModeAngle(&urm);
        }
    }
    int16_t realyAngle = fc->hAddActTargetAngle;
    //realyAngle += 32768;
    //if(fc->hAddActTargetAngle>0){
    //    realyAngle = (realyAngle * 64200) >>16;
    //}else{
    //    realyAngle = (realyAngle * 63536) >>16;
    //}
    if(fc->lc->M_dir){
        //电角度和物理角度同相变化
        #ifdef GyroEn
        hError = realyAngle-GetOriGyroA();
        #else
        hError = realyAngle-fc->hMecAngle;
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
    //if((hError>-vDeadErr)&&(hError<vDeadErr)){
    //    hError = 0;
    //}
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
        PIDPosHandle_M1.wIntegralTerm = 0;
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
    if(fc->lc->M_dir){
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
    //fc->hSpeed = tempsp;
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
void fSetGyroInitMid(int16_t in){
    FOC_Component_M1.lc->GyroInitAngle = in;
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
//获取马达和霍尔方向关系
int16_t fGetMHdir(void){
    return FOC_Component_M1.lc->M_dir;
}
/**
 * @brief Get the Acc Xoffset object
 * 
 */
int16_t GetAccXoffset(void){
    if(FOC_Component_M1.lc->learnXYFin==1)
    return FOC_Component_M1.lc->accVx_offset;
    else
    return 0;
}
int16_t GetAccYoffset(void){
    if(FOC_Component_M1.lc->learnXYFin==1)
    return FOC_Component_M1.lc->accVy_offset;
    else
    return 0;
}
int16_t GetAccZoffset(void){
    if(FOC_Component_M1.lc->learnXYFin==1)
    return FOC_Component_M1.lc->accVz_offset;
    else
    return 0x4000;
}
//获取陀螺仪0度位置
int16_t GetGyroZero(void){
    return FOC_Component_M1.lc->GyroInitAngle;
}
//获取是否处于等待姿态检测状态
bool GetLearnAtt(void){
    if((FOC_Component_M1.lc->LearnFinish==0)&&(FOC_Component_M1.LearnAttitude==1))
        return true;
    else 
        return false;
}
//进入姿态检测状态
void SetLearnAttStart(void){
    FOC_Component_M1.LearnAttitude = 2;
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
    static uint16_t CountTime=0;
    //static int16_t  offsetErr1;
    //static int16_t  offsetErr2;
    //static int32_t  offsetErrTmp1=0; 
    //static int32_t  offsetErrTmp2=0; 
    //static HallXYs Elstart; //一般存储0度
    //static HallXYs ElAtemp; //一般存储90度
    //uint32_t hElAngle=0;
    HallXYs xynow = fc->xy_now; //采样到的XY值

    if(fc->lc->learnXYFin){
        //学习了XY值后不停计算物理角度 1ms一次并滤波处理
        //CalMecAngle(&FOC_Component_M1); //计算出当前的物理角度
        CalElAngle(&FOC_Component_M1);  //计算出电角度顺便计算出物理角度
    }
    
    if(fc->lc->LearnFinish==0){
        //是否没有学习,没有学习要学习角度 给一个固定力旋转
        if(fc->hStepTime<fc->hDurationms){
            fc->hStepTime++;
            fc->hElAngle = 0;  //初始定位到电角度0度
            //FOC_Component_M1.hLastElAngle = 0;  //初始定位到电角度0度
            fc->Vqd.qV_Component2 = fc->hStepTime*fc->hFinalTorque/fc->hDurationms;
            fc->Vqd.qV_Component1 = 0;
            CountTime = 0;
            //if(fc->hStepTime<50){
            //    fc->Max = xynow;
            //    fc->Min = xynow;
            //}
        }else{
            //判断是否可以进入姿态校准
            if(fc->LearnAttitude==2){   //
                if(CountTime<(ALIGN_qmiTime/2)){
                    CountTime++;
                    return no_err;
                }
                //学习姿态时候读取陀螺仪并校准
                ResetGyroInit();
                //calibrationGyro();  //先校准陀螺仪中点值
                //静止时候读取当前真实的角度位置
                CountTime = 0; 
                fc->LearnAttitude=3;   //进入下一阶段读取陀螺仪稳定值
                return no_err;
            }
            if(fc->LearnAttitude==3){
                if(GetGyroFin()==1){
                    if(CountTime<ALIGN_qmiTime){
                        CountTime++;
                        return no_err;
                    }else{
                        //获取陀螺仪的静止角度
                        fc->lc->GyroInitAngle = GetOriGyroA();
                        fc->lc->LearnFinish  =   1;  //学习完成
                        fc->LearnAttitude = 0;
                        EE_WriteFOC(fc->lc); //把学习的参数写入EEPROM
                        return no_err;
                    }
                }else{
                    CountTime = 0; 
                    return no_err;
                }
            }
            //达到上升时间缓慢增加角度,扭力不变 Hallxy 不停采样最大最小值 10ms 1/1024度 10s一圈
            if(CountTime<ALIGN_TimeOnce){
                CountTime++;
            }else{
                CountTime = 0;  //达到计时时候计算出当前位置(x,y)
                if(fc->lc->learnXYFin==0){
                    LearnPolePairAngle(fc,xynow);
                }else{
                    fc->LearnAttitude = 1;      //开始学习姿态 马达停止输出
                    fc->Vqd.qV_Component2 = 0;
                    fc->Vqd.qV_Component1 = 0;
                    //EE_WriteFOC(&fc->lc); //把学习的参数写入EEPROM
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
    fc->lc->learnXYFin  = 0;
    fc->hStepTime = 0;
    fc->lc->LearnFinish = 0;
    fc->LearnAttitude = 0;  //不能学习姿态
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
        //if(RunModeEn){
            //学习时候自动增加角度功能
        //    RunModeEn = UpNextRunModeAngle(&urm);
        //}
        #ifndef testQMI
        //int16_t oriA = getOrientation_1ms();
        //当前陀螺仪多少度 目标值是当前角度-陀螺仪的角度 = 目标角度 误差就是陀螺仪角度的反向
        //FOC_Component_M1.hTargetAngle = -GetOriGyroA();   //getOrientation_1ms();    //当前陀螺仪的角度
        //FOC_Component_M1.hTargetAngle += FOC_Component_M1.hAddTargetAngle;
        //else{
        //    #ifdef GyroEn
        //    FOC_Component_M1.hTargetAngle = FOC_Component_M1.hMecAngle - GetOriGyroA();
        //    FOC_Component_M1.hTargetAngle += FOC_Component_M1.hAddTargetAngle;
        //    #else
        //    FOC_Component_M1.hTargetAngle = FOC_Component_M1.hAddTargetAngle;
        //    #endif
        //}
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
    if(rm->step>=rm->OverStep){
        return 0;   //over
    }
    FOC_Component_M1.hAddTargetAngle += RunModeParam_t[rm->step++]; //每次增加1/8192度 一圈时间8s?
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
    SetHorizontal();
    //EE_WriteFOC(&FOC_Component_M1.lc);
    //EE_ReadFOC(&FOC_Component_M1.lc);
    IsMCCompleted = 1;
    //if(Aligned_hall==0){
        //霍尔 对应角度位置没有校准 要进行一次校准
        //给固定一个角度力慢慢旋转1周 不断更新hall X Y 轴数据
    //}
}

void ClearRunMode(void){
    RunModeEn = 0;
}
/***
 * 
 * 正转360度, 每次比当前增加0x1000度 达到再增加 每=
 * 
*/
void SetTurnLeftCycle(void){
    if(RunModeEn==0){
        RunModeEn = 1;
        urm.step= cTurnLCycle;    //开始位置
        urm.OverStep= cTurnLCycle+cTurnRCycleLen;    
    }
}
/***
 * 
 * 正转360度, 每次比当前增加0x1000度 达到再增加 每=
 * 
*/
void SetTurnRightCycle(void){
    if(RunModeEn==0){
        RunModeEn = 1;
        urm.step= cTurnRCycle;    //开始位置
        urm.OverStep= cTurnRCycle+cTurnLCycleLen;
    }    
}
/***
 * @brief 特定旋转动作
 * 
*/
void HorOrVerRoll(void){
    if(FOC_Component_M1.hAddTargetAngle==FOC_Component_M1.lc->GyroInitAngle){
        SetTurnHorRoll();
    }else{
        SetTurnVerRoll();
    }
}
/***
 * @brief 水平时候自定义动作
 * 正转360度, 每次比当前增加0x1000度 达到再增加 每=
 * 
*/
void SetTurnHorRoll(void){
    if(RunModeEn==0){
        RunModeEn = 1;
        urm.step= cTurnHorRoll;    //开始位置
        urm.OverStep= cTurnHorRoll+cTurnHorRollLen;    
    }
}
/***
 * @brief 垂直时候自定义动作
 * 反转360度, 每次比当前增加0x1000度 达到再增加 每=
 * 
*/
void SetTurnVerRoll(void){
    if(RunModeEn==0){
        RunModeEn = 1;
        urm.step= cTurnVerRoll;    //开始位置
        urm.OverStep= cTurnVerRoll+cTurnVerRollLen;    
    }
}
