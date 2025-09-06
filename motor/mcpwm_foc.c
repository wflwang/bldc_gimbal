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
static uint16_t vDeadErr = deadErr; //死区时间
//static mc_cmd mc_cmd_t = lock;
const RunModeParam RunModeParam_t[]={ \
   // {.Add=8,.EndAngle = 0x4000},{.Add=-8,.EndAngle=-0x4000} 
{.Add=8,.EndAngle=0x8000},{.Add=8,.EndAngle=0x0},\
{.Add=-8,.EndAngle=0x8000},{.Add=-8,.EndAngle=0x0},\
{.Add=-8,.EndAngle=0xa000},{.Add=8,.EndAngle=0x6000},{.Add=-8,.EndAngle=0x0},\
{.Add=8,.EndAngle=0x6000},{.Add=-8,.EndAngle=0xb000},{.Add=8,.EndAngle=0x6000},{.Add=-8,.EndAngle=0x4000}};
UpRunMode urm={0};
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
}

/****
 * @brief 获取实时霍尔角度 校准时候返回固定角度 正常动作时候返回当前hall采样的角度
 * @return deta
 * 采样时候不停的采样平均角度值
*/
int16_t Get_HallAngle(FOC_Component *fc){
    uint32_t hElAngle;
    uint16_t maxtemp = fc->x_Max;
    uint16_t mintemp = fc->x_Min;    
    HallXYs hxy = MX_Hall_Sample((GetHallxAD()&0xFFF)<<4,(GetHallyAD()&0xFFF)<<4);
    fc->x_now = hxy.Hallx;
    fc->y_now = hxy.Hally;
    if(fc->lc.LearnFinish==0){
        if(fc->learnXYFin==0){
            //fc->x_now = ((GetHallxAD()&0xFFF)<<4);  //获取X轴的最大值,最小值
            MaxMinUpDate(&fc->x_now,&maxtemp,&mintemp);
            fc->x_Max = maxtemp;
            fc->x_Min = mintemp;
            maxtemp = fc->y_Max;
            mintemp = fc->y_Min;
            //fc->y_now = ((GetHallyAD()&0xFFF)<<4);  //获取Y轴的最大值,最小值
            MaxMinUpDate(&fc->y_now,&maxtemp,&mintemp);
            fc->y_Max = maxtemp;
            fc->y_Min = mintemp;
        }else{  //学习误差 和方向
            hElAngle = GetRealElAngle(fc); //当前物理角度算出的电角度
        }
    }else{
        //获取当前X Y 霍尔的值 换算出角度
        hElAngle = GetRealElAngle(fc); //当前物理角度算出的电角度

        if(fc->lc.M_dir)   //电角度增加 物理角度是否方向一致 也是增加
        fc->hElAngle = (int16_t)hElAngle + fc->lc.ElAngele_offset;
        else
        fc->hElAngle = fc->lc.ElAngele_offset - (int16_t)hElAngle;
        //换算出实际电角度
    }
    return (int16_t)(fc->hElAngle&0xffff);
}
/****
 * 
 * 获取真实电角度,和物理角度
 * 
*/
uint32_t GetRealElAngle(FOC_Component *fc){
    //static int32_t hX=0,hY=0,hcount=0;
    int16_t hx1,hy1;
    uint32_t hElAngle;

    #ifdef filterAV
	static int32_t hX=0,hY=0,hcount=0;
    //MX_Hall_init(int16_t xRaw,int16_t yRaw);
    hX += (int32_t)(fc->x_now-fc->lc.x_offset);
	hY += (int32_t)(fc->y_now-fc->lc.y_offset);
    hcount++;
    if(hcount>=filterAVDeep){
        hcount = 0;
        hx1 = (int16_t)(hX >>LOG2(filterAVDeep)); //LOG2(filterAVDeep)
        hy1 = (int16_t)(hY >>LOG2(filterAVDeep));   //LOG2(filterAVDeep)
        hX = 0;
        hY = 0;
        GetHallXYScale(&fc->lc,&hx1,&hy1);
        #if HallXY_dir==0
        fc->hMecAngle = arctan(hx1,hy1);
        #elif HallXY_dir==1    
	    fc->hMecAngle = arctan(hx1,-hy1);
        #elif HallXY_dir==2
        fc->hMecAngle = arctan(-hx1,hy1);
        #elif HallXY_dir==3
        fc->hMecAngle = arctan(-hx1,-hy1);
        #elif HallXY_dir==4
	    //LEDR_Set();
        fc->hMecAngle = arctan(hy1,hx1);
	    //LEDR_Reset();
        #elif HallXY_dir==5
        fc->hMecAngle = arctan(hy1,-hx1);
        #elif HallXY_dir==6
        fc->hMecAngle = arctan(-hy1,hx1);
        #else
        fc->hMecAngle = arctan(-hy1,-hx1);
        #endif
    }
    #else
    hx1 = (int16_t)(fc->x_now-fc->lc.x_offset);
	hy1 = (int16_t)(fc->y_now-fc->lc.y_offset);
    GetHallXYScale(&fc->lc,&hx1,&hy1);
    #if HallXY_dir==0
    fc->hMecAngle = arctan(hx1,hy1);
    #elif HallXY_dir==1    
	fc->hMecAngle = arctan(hx1,-hy1);
    #elif HallXY_dir==2
    fc->hMecAngle = arctan(-hx1,hy1);
    #elif HallXY_dir==3
    fc->hMecAngle = arctan(-hx1,-hy1);
    #elif HallXY_dir==4
	//LEDR_Set();
    fc->hMecAngle = arctan(hy1,hx1);
	//LEDR_Reset();
    #elif HallXY_dir==5
    fc->hMecAngle = arctan(hy1,-hx1);
    #elif HallXY_dir==6
    fc->hMecAngle = arctan(-hy1,hx1);
    #else
    fc->hMecAngle = arctan(-hy1,-hx1);
    #endif
    #endif
    hElAngle = (uint32_t)fc->hMecAngle*(uint32_t)fc->PolePariNum;    //当前物理角度算出的电角度
    return hElAngle;
}
/***
 * 
 * 获取hall xy 的比例增益
 * 
*/
void GetHallXYScale(Learn_Componets *lc,int16_t *x,int16_t *y){
    if(lc->xyScaleDir==0){  //X>Y
        *x  =   ((lc->xy_scale * (*x)) >>16);
    }else{  // X<Y
        *y  =   ((lc->xy_scale * (*y)) >>16);
    }
}
//**************************************************************
//PID 控制
int16_t PosPISControl(FOC_Component *fc){
    static int hErrCount=0;
    static uint8_t errTime=0;
    int16_t hTorqueReference;   //生成的扭力
    int16_t hError; //位置误差
    int16_t hSpeed; //误差对应的速度
    if(fc->lc.M_dir)
    //电角度和物理角度同相变化
    hError = fc->hTargetAngle - fc->hMecAngle;
    else
    hError = fc->hMecAngle - fc->hTargetAngle;

    #ifdef posLoop  //位置环
    hTorqueReference = PID_Controller(&PIDPosHandle_M1, ( int32_t )hError);
    #else
    //不同的角度误差 => 对应不同速度 位置环 
    //速度环 转速熊超过1min 1转 1s->1/6转 err 对应角度的反数 角度期望总是要无限接近0
    //hSpeed = PID_Controller(&PIDPosHandle_M1, ( int32_t )hError);
    //限制一下最大速度 
    if(hError>vDeadErr)
    hSpeed = 64;    //匀速 不管在哪里都是匀速处理
    else if(hError<vDeadErr)
    hSpeed = -64;
    else{   //速度逐渐变到目标速度
        if(hSpeed>0){
            hSpeed -= 5;
            if(hSpeed<0)
                hSpeed = 0;
        }
        hSpeed = 0;     //只设置三种速度
    }
    //if(hSpeed>250)
    //    hSpeed = 250;
    //if(hSpeed<-250)
    //    hSpeed = -250;
    //不同速度对应不同扭矩 速度环 速度恒定
    //目标速度 - 实际速度  = 当前误差速度
    //本次 
    //fc->hSpeed = fc->hMecAngle - fc->hLastMecAngle;
    fc->hSpeed += fc->hMecAngle - fc->hLastMecAngle;
    fc->hSpeed >>= 1;
    fc->hLastMecAngle = fc->hMecAngle;
    int16_t errspeed = (hSpeed-fc->hSpeed);
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
void SetDeadErr(uint16_t in){
  vDeadErr = in;
}
//获取扭矩
int16_t GetTorque(void){
    return FOC_Component_M1.Vqd.qV_Component1;
}
/***
 * 
 * motor run timing
 * 
*/
Err_FOC MotorRunControl(FOC_Component *fc){
    static uint8_t CountTime=0;
    static uint32_t  offsetErr1;
    static uint32_t  offsetErr2;
    uint32_t  offsetErrTmp1=0; 
    uint32_t  offsetErrTmp2=0; 
    uint32_t hElAngle=0;
    
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
                fc->x_Max = fc->x_now;
                fc->x_Min = fc->x_now;
                //fc->x_Start = fc->x_now;
                fc->y_Max = fc->y_now;
                fc->y_Min = fc->y_now;
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
                if(fc->learnXYFin==0){
                    fc->hElAngle += 0x80;   //0x40; //每次0.35度 开环转起来
                    if(((fc->hElAngle>>16)&0xffff)==(fc->PolePariNum+1)){
                        //极对数一对上 得出hall xy 的中点
                        fc->lc.x_offset = (fc->x_Max+fc->x_Min)>>1;
                        fc->lc.y_offset = (fc->y_Max+fc->y_Min)>>1;
                        uint16_t tempx = (fc->x_Max-fc->x_Min); //x max range
                        uint16_t tempy = (fc->y_Max-fc->y_Min); //y max range
                        if((tempx<0x4bff)||(tempy<0x4bff)){
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
                        fc->learnXYFin = 1; //XY 校准完 校准角度偏差和方向
                        //获取换算出的电角度
                        fc->hElAngle = 0x00;
                        //fc->lc.ElAngele_offset = (int16_t)fc->hElAngle + (int16_t)hElAngle;
                    }
                }else{
										//fc->hElAngle += 0x100; //每次1.4度 开环转起来
                    if(fc->hElAngle==0){
                        hElAngle = (uint32_t)fc->hMecAngle*(uint32_t)fc->PolePariNum;
                        offsetErr1 = (fc->hElAngle&0xffff) - (hElAngle&0xffff); //不知道是正还是反
                        offsetErr2 = (fc->hElAngle&0xffff) + (hElAngle&0xffff);
												fc->hElAngle += 0x100; //每次1.4度 开环转起来
                    }else{
                        fc->hElAngle += 0x100; //每次1.4度 开环转起来
                        hElAngle = (uint32_t)fc->hMecAngle*(uint32_t)fc->PolePariNum;
                        offsetErrTmp1 = (fc->hElAngle&0xffff) - (hElAngle&0xffff);
                        offsetErrTmp2 = (fc->hElAngle&0xffff) + (hElAngle&0xffff);
                        offsetErr1 += offsetErrTmp1;
                        offsetErr2 += offsetErrTmp2;
                        if(fc->hElAngle==0x1000){
                            //前面加了16次
                            offsetErr1 = offsetErr1>>4;
                            offsetErr2 = offsetErr2>>4;
                            if((offsetErr1-offsetErrTmp1)<(offsetErr2-offsetErrTmp2)){
                                //正向误差小
                                fc->lc.M_dir = 1;   //马达方向正    实际电角度-换算电角度
                                fc->lc.ElAngele_offset = (int16_t)offsetErr1;
                            }else{
                                fc->lc.M_dir = 0;
                                fc->lc.ElAngele_offset = (int16_t)offsetErr2;
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
            fc->Vqd.qV_Component1 = PosPISControl(fc);
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
    fc->learnXYFin  = 0;
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
