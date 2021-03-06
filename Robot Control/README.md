# ROBOT CONTROL  
 控制对象：小车底盘  
 控制处理器：STM32F4  
 姿态传感器：板载MPU6050  

# 技术方案    
1、姿态解算：解算MPU6050，获取小车姿态信息；  
2、卡尔曼滤波：数据噪声过大，且由于抖动数据不够平稳，不利于小车的闭环控制，通过卡尔曼滤波获取更加平滑的姿态角度；  
3、电机PID控制：距离环+电流环、角度环+电流环；  
4、串口通信：STM32与Raspberry Pi之间的数据通信  

# 文件说明
<wheel_control.c>文件中包含距离PID计算和角度PID计算，最后的输出适当放大匹配电机的电流值，输出至小车电机，控制小车运动；  
<Wheel_out.c>文件中包含电机速度环控制算法，还有CAN通信发送协议；  
<Serial_port_communication>文件中包含STM32与Raspberry Pi之间通信协议部分，包括了发送协议，接收协议，STM32暂时不需要向树莓派发送数据，因此发送函数未调用；  
<MPU6050>文件中包含陀螺仪零点校准，读取陀螺仪数据及用卡尔曼滤波对数据进行处理。    
<KalmanFilter.c>文件中包含卡尔曼滤波函数，使用时只需要根据文件说明定义2个参数，并给定Data入口，即可获得较好的滤波效果。  
# 说明
由于前期对Git使用不是很了解，文件放在项目Gravity-N1的其他仓库，详见：https://github.com/Gravity-N1/Communication-between-miniPC-with-STM32  
