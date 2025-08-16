# 单轴云台


### 关于

flash 32个word 256byte 一个 page 共 64 个page 第1&2个page 作为bootloader  最后2个page作为EEPROM(存储磁编每一个角度值(256byte*2) 和上次陀螺仪的校准值)
中间 60个page 作为flash 程序空间