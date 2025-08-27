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
 * 
 * 
*/