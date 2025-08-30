/***
 * @file filter.c
 * 
 * 
 * 
*/
    #include "filter.h"
    #include <math.h>
/***
 * @brief 一阶滤波 滤波系数可调
 * @brief filter = filter*(1-a) + a*raw =>a*(raw-filter) + filter
 * @param diff 误差值
 * @param raw
 * @param alpha >> /16
 * @return filter 返回滤波后的值
*/
int16_t firstOrderFilter(filter_t *ft,int16_t raw){
    //如果本次误差方向和上次方向不一致 滤波系数降到最小
    int diff = (int)raw - (int)ft->filter;
    if(((diff>0)&&(ft->lastDiff<0))||((diff<0)&&(ft->lastDiff>0))){
        //误差方向不一致 滤波系数降到最小
        ft->alpha_raw = ft->alpha_min;
        ft->lastDiff = diff;    //只需要误差的方向不关心误差值
    }else{
        //不同误差对应到不同的滤波系数增量
        uint8_t index=0;
        for(;index<ft->alpha_diff_len;index++){
            if((int)ft->alpha_diff[index]<abs(diff)){
                break;
            }
        }
        ft->alpha_raw += (int)ft->alpha_diff_addV[index];
        if(ft->alpha_raw>ft->alpha_max)
            ft->alpha_raw = ft->alpha_max;
    }
    ft->filter = (int16_t)((ft->alpha_raw*diff)>>16)+ft->filter;
    return ft->filter;
}
/***
 * @brief 一阶互补滤波  陀螺仪和加速度误差决定 滤波系数大小
 * 误差越大 陀螺仪占比越高 误差越小 加速度占比越大
 * @param gyroA 陀螺仪本次角加速度*单位时间+上次角度 = 本次角度
 * @param accA 加速度直接算出的角度
 * gyro*a + (1-a)accA => (gyro-accA)*a + accA
 * a = (gyro-accA)*k <min - max> PI控制 (diff)*p+f(diff)*I
 * P = ?/65536  -> <0.8 - 0.98> -> <52428, 64225> <0,11797>
 * 1024us 采样一次 *360倍 => 直接算出本次增加的角度*360倍的值
 * gyro = lastgyro*5625 + addgyro*16
 * accA = accA*5625
 * diff = gyro - accA 
 * @return 
*/
int complementFilter(int gyroA,int accA){
    int diff = gyroA - accA;    //本次角度误差 不同误差对应不同滤波系数
    //误差对应范围内的位置 对应到滤波系数对应到的位置
    //误差越大 alpha 值越大 越信任陀螺仪 反之信任加速度
    int in_min = complementFLP_minDiff;  //最小误差范围
    int in_max = complementFLP_maxDiff; //最大误差范围
    int out_min = complementFLP_minAlpha;   //最小滤波系数
    int out_max = complementFLP_maxAlpha;   //最大滤波系数
    int pro = dataRangeMov(abs(diff),in_min,in_max,out_min,out_max);
    int result = (((long long)diff * pro)>>8)+accA;
    return result;
}
