/***
 * @file filter.h
 * 
 * 
 * 
*/
#ifndef __FILTER_H
#define __FILTER_H
    #include    "main.h"

#define complementFLP_minDiff   5*5625        //互补滤波最小的误差 误差是*5625(>>4 = 351.5625)倍后的误差
#define complementFLP_maxDiff   100*5625        //互补滤波最大的误差
#define complementFLP_minAlpha   5  //25     //228        //互补滤波最小的滤波系数 208/256
#define complementFLP_maxAlpha   1    //255    //252        //互补滤波最大滤波系数  252/256

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
/**
 * 平均滤波参数结构体
*/
typedef struct
{
    uint8_t index; //滤波误差表格长度
    uint8_t avFilterInitFin;    //初始化完成标志
    #ifdef AvFilterSubMaxMin
    uint16_t buff[avFilterDeep+2]; //滤波后的值
    #else
    uint16_t buff[avFilterDeep]; //滤波后的值
    #endif
    uint32_t sum;        //累加和
}Avfilter_t;

//一阶滤波
int16_t firstOrderFilter(filter_t *ft,int16_t raw);
//一阶互补滤波
int complementFilter(int gyroA,int accA);
//平均滤波
uint16_t AvFilter(Avfilter_t *aft,uint16_t raw);



#endif

