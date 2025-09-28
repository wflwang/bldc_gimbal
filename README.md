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
### 自动学习
在新的结构或第一次下载程序时候 需要自动学习,没有学习过的云台没有功能
学习错误的云台功能错乱
##### 学习步骤
1. 开机前云台需要水平放在水平的桌面上,开口朝上
2. 长按 L M R 三个按键3s开机,识别到云台水平朝上放置时候会开机并进入自动学习模式
3. 绿灯快速闪烁svpwm慢慢逐个输出不同角度 一直到经历8个极对,实际7极对(主要学习霍尔最大最小值)
4. 通过旋转一圈求出最大最小值换算出霍尔的中点值,继续旋转
绿灯慢闪 表示进入下一步矫正
5. 把云台垂直放置在桌面上,按键朝上,短按按键绿灯快闪进入第二步学习
6. 绿灯快速闪烁检测陀螺仪是否处于静止,静止时候矫正陀螺仪和加速度
##### 修改学习步骤
1. 7极对电机需要学习7组数据 xy max min 需要对应不同极对里
记录每个极对x范围和y范围 后面查询的时候直接定位到根据x查询到哪个极对范围 再用y最后筛选
换算出不同的 x_offset y_offset x_Amp y_Amp(求出xy 增益的比例,不同)



### 关于

flash 32个word 256byte 一个 page 共 64 个page 第1&2个page 作为bootloader  最后2个page作为EEPROM(存储磁编每一个角度值(256byte*2) 和上次陀螺仪的校准值)
中间 60个page 作为flash 程序空间


## 历史

## 版本

## 邮箱
* [email](https://B5106D@Outlook.com)

## 仓库
* [Gitee](https://gitee.com/szdctek/bldc_gimbal.git)
* [Github](https://github.com/wflwang/bldc_gimbal.git)