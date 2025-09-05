/**
  ******************************************************************************
  * @file    debugTab.h 
  * @version V1.0.0
  * @date    2025-09-1
  * @brief   debug inst tab.	调试模式下的指令表
  * @author MaxwellWang
  ******************************************************************************
  */

//1byte head
#define UartHead        0xaa    //串口开始头    第一个byte
//2byte 数据长度
//3byte 指令数 多少组指令帧
//4byte 输出次数 0/0xff(0 一直输出)(0xff 不输出)
//5byte 输出间隔 *10ms 
//指令  - 6yte  ......
#define GetMecAngle     0x01    //获取物理角度
#define GetElAngle      0x02    //获取电角度
#define GetGyroAngle    0x03    //获取陀螺仪角度
#define GetMCElSpeed    0x04    //获取电机 电转速
#define GetMCMecSpeed   0x05    //获取电机 物理转速
#define GetMecAngleAcc  0x13    //加速度角度
#define GetMecAngleGyro 0x23    //陀螺仪角度
#define GetMotorSpeed   0x33    //获取运行的速度

#define SetMCElSpeed    0x84    //设置电机电转速
#define SetSpeedPID_P   0x89    //速度环PID P
#define SetSpeedPID_I   0x8a    //速度环PID I 
#define SetSpeedPID_D   0x8b    //速度环PID D
#define SetPosPID_P     0x8c    //设置位置环PID P
#define SetPosPID_I     0x8d    //设置位置环PID P
#define SetPosPID_D     0x8e    //设置位置环PID P
#define SetPIDInt       0x8f    //PID调整间隔
#define SetDead_Err     0x90    //设置死区角度
//end 0x55

