/***
 * @file filter.h
 * 
 * 
 * 
*/
#ifndef __FILTER_H
#define __FILTER_H
    #include    "main.h"

#define complementFLP_minDiff   16*5625        //互补滤波最小的误差 误差是*5625(>>4 = 351.5625)倍后的误差
#define complementFLP_maxDiff   300*5625        //互补滤波最大的误差
#define complementFLP_minAlpha   208        //互补滤波最小的滤波系数 208/256
#define complementFLP_maxAlpha   252        //互补滤波最大滤波系数  252/256

/**
 * 滤波参数结构体
*/
typedef struct
{
    uint8_t alpha_diff_len; //滤波误差表格长度
    int16_t filter; //滤波后的值
    //int16_t raw;    //当前值
    const int16_t *alpha_diff;  //不同误差对应级别表格
    const int16_t *alpha_diff_addV;   //不同误差对应级别滤波系数增量表格
    int lastDiff;    //上次的滤波误差
    int alpha_raw;  //当前滤波系数
    int alpha_min;  //最小滤波系数
    int alpha_max;  //最大滤波系数
}filter_t;

//一阶滤波
int16_t firstOrderFilter(filter_t *ft,int16_t raw);
//一阶互补滤波
int complementFilter(int gyroA,int accA);



#endif

