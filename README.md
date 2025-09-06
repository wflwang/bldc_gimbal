# 单轴云台

## 使用说明
MDK 工程文件可以编译

drive_paramters.h  马达参数配置
deadErr: 设置死区大小
PID 可以调节速度环和位置环 和PID调整间隔
targets.h  IO 和一些功能配置
可以选择 GyroEn 开启陀螺仪 否则直接固定位置 方便调PID分离陀螺仪干扰影响

串口115200 可以调参

MCU/HK32G003/Inc/peripherals.h 可以看可以调哪些参数




### 关于

flash 32个word 256byte 一个 page 共 64 个page 第1&2个page 作为bootloader  最后2个page作为EEPROM(存储磁编每一个角度值(256byte*2) 和上次陀螺仪的校准值)
中间 60个page 作为flash 程序空间


## 历史

## 版本

## 邮箱
* [email](https://B5106D@Outlook.com)

## 仓库
* [Gitee](https://gitee.com/szdctek/bldc_gimbal.git)
* [Github](https://github.com/wflwang/***)