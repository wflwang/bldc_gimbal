/***
 * @file filter.h
 * 
 * 
 * 
*/
#ifndef __FILTER_H
#define __FILTER_H
    #include    "main.h"
/**
 * 滤波参数结构体
*/
typedef struct
{
    uint8_t alpha_diff_len; //滤波误差表格长度
    int16_t filter; //滤波后的值
    //int16_t raw;    //当前值
    int16_t *alpha_diff;  //不同误差对应级别表格
    int16_t *alpha_diff_addV;   //不同误差对应级别滤波系数增量表格
    int lastDiff;    //上次的滤波误差
    int alpha_raw;  //当前滤波系数
    int alpha_min;  //最小滤波系数
    int alpha_max;  //最大滤波系数
}filter_t;

//一阶滤波
int16_t firstOrderFilter(filter_t *ft,int16_t raw);



#endif

