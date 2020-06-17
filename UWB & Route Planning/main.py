import threading
import numpy as np
import route_planning
import UWB_serial




if __name__ == '__main__':
    REAL_TIME = 0
    WRITE_MES = 1
    READ_MES = 2
    q = 0.001  # 卡尔曼滤波的系统噪声协方差
    max_loop = 600  # WRITE_MES模式下的循环次数
    port = '/dev/ttyACM0'  # 串口端口
    plot = False  # 绘制XYZ曲线
    # anchor_pos = np.mat([[0, 0, 2000], [1000, 0, 2000], [1000, 1000, 2000], [0, 1000, 3800]])


    #与小车的通讯
    sent_port = '/dev/ttyUSB0' # 车的串口端口
#    sent_port = 'COM1' #电脑测试使用
    start = 600 #人和车的距离
    test = 300 #judge whether move to target point

    # 基站的坐标
    anchor_pos =  np.mat([[0, 3110, 3110, 0],
                                [0, 0, 6920, 6920],
                                [0, 0, 0, 0]])
    mode = REAL_TIME  # UWB 工作模式

    global loc0_record, loc1_record  # 车子（loc0_record）与人（loc0_record）的坐标，最后的100次记录

    t1 = threading.Thread(target=UWB_serial.runUWB, args=(q, port, mode, max_loop, plot))  # UWB线程
    t2 = threading.Thread(target=route_planning.route_planning, args=(sent_port,start,test))  # 路径规划线程
    t1.start()
    t2.start()

    # runUWB(q, port, mode, max_loop, plot)
