import threading

import serial as pyserial
import time
import numpy as np
from numpy.linalg import *
import math
from pykalman import KalmanFilter
import matplotlib.pyplot as plt


class UWBMessage:

    def __init__(self, str='', t=0):
        self.R = [-1, -1, -1, -1]
        if str == '':
            self.isempty = True
            self.t = t
            self.MID = 'NA'
            self.MASK = 0
            self.R0 = -1
            self.R1 = -1
            self.R2 = -1
            self.R3 = -1
            self.RSEQ = -1
            self.tagID = -1
        else:
            self.isempty = False
            d = str.split(' ')
            self.t = round(t)
            self.MID = d[0]
            self.MASK = int(d[1], 16)
            for i in range(4):
                self.R[i] = int(d[i + 2], 16)
            self.R0 = int(d[2], 16)
            self.R1 = int(d[3], 16)
            self.R2 = int(d[4], 16)
            self.R3 = int(d[5], 16)
            self.RSEQ = int(d[7], 16)
            self.tagID = int(d[9][1], 16)

    def from_file(self, str):
        elem = str.split(',')
        self.t = int(elem[0])
        self.MASK = int(elem[1])
        self.R0 = int(elem[2])
        self.R1 = int(elem[3])
        self.R2 = int(elem[4])
        self.R3 = int(elem[5])
        self.RSEQ = int(elem[6])
        for i in range(4):
            self.R[i] = int(elem[i + 2])
        self.isempty = False


class RecordBuffer:
    def __init__(self, size):
        self.record = []
        self.t = []
        self.size = size

    def append(self, t, obj):
        self.record.append(obj)
        self.t.append(t)
        if len(self.record) > self.size:
            del self.record[0]
            del self.t[0]

    def get_record(self):
        return self.record

    def __len__(self):
        return len(self.record)


def UWB_Initial(port):
    serial = pyserial.Serial(port, 115200, timeout=0.15)  # /dev/ttyUSB0
    pyserial.Serial()
    if serial.isOpen():
        print("open success")
    else:
        print("open failed")
    return serial


def recv(ser, t0):
    mes0 = UWBMessage()
    mes1 = UWBMessage()
    lin = 2
    while lin > 0:
        data = ser.readline().decode("gbk")
        t = (time.perf_counter() - t0) * 1000

        if data[0:2] == 'mc':
            mes0 = UWBMessage(data, t)
            lin = lin - 1
        elif data[0:2] == 'st':
            mes1 = UWBMessage(data, t)
            lin = lin - 1
        elif data == '':
            break
        else:
            print(data)
    return mes0, mes1


def find_anchor_pos(anchor_record):
    a = b = c = 0
    for mes in anchor_record.record:
        c += mes.R1
        b += mes.R2
        a += mes.R3
    a = a / len(anchor_record)
    b = b / len(anchor_record)
    c = c / len(anchor_record)
    x = (b * b + c * c - a * a) / (2 * c)
    y = math.sqrt(b * b - x * x)
    A = np.mat([[0, 0, 0], [c, 0, 0], [x, y, 0], [0, y, 0]])
    return A


def triangulation(Anc, A_n, R):
    if len(R.shape) > 1:
        R = R[:, 0]
    k = [0] * A_n
    h = np.zeros((A_n, 1))
    Ga = np.zeros((A_n, 4))
    for i in range(A_n):
        k[i] = Anc[0, i] ** 2 + Anc[1, i] ** 2 + Anc[2, i] ** 2

    for i in range(A_n):
        h[i] = R[i] ** 2 - k[i]

    for i in range(A_n):
        Ga[i, 0] = -2 * Anc[0, i]
        Ga[i, 1] = -2 * Anc[1, i]
        Ga[i, 2] = -2 * Anc[2, i]
        Ga[i, 3] = 1
    # Q为TDOA系统的协方差矩阵
    Q = np.eye(A_n)
    B = np.diag(R)
    psi = B.dot(Q).dot(B)
    # za, 距离较远时
    za1 = pinv(Ga.T.dot(pinv(psi)).dot(Ga)).dot(Ga.T).dot(pinv(psi)).dot(h)

    # 第二次WLS
    h2 = np.mat([[za1[0, 0] ** 2],
                 [za1[1, 0] ** 2],
                 [za1[2, 0] ** 2],
                 [za1[3, 0]]])

    Ga2 = np.mat([[1, 0, 0],
                  [0, 1, 0],
                  [0, 0, 1],
                  [1, 1, 1]])
    B2 = np.diag([za1[0, 0], za1[1, 0], za1[2, 0], 0.5])
    za2 = pinv(Ga2.T.dot(pinv(B2)).dot(Ga.T).dot(pinv(Q)).dot(Ga).dot(pinv(B2)).dot(Ga2)).dot(
        Ga2.T.dot(pinv(B2)).dot(Ga.T).dot(pinv(Q)).dot(Ga).dot(pinv(B2))).dot(h2)
    zp = np.zeros((3, 1))
    zp[0, 0] = math.sqrt(abs(za2[0, 0]))
    zp[1, 0] = math.sqrt(abs(za2[1, 0]))
    zp[2, 0] = math.sqrt(abs(za2[2, 0]))
    return np.asarray(zp)[:, 0]


'''
def localization(anchor_pos, tag_mes):
    if tag_mes.isempty:
        loc = np.ma.masked_all(3)
    else:
        ranges = np.mat([tag_mes.R0, tag_mes.R1, tag_mes.R2, tag_mes.R3])
        loc = triangulation(anchor_pos, ranges)
    return loc


def transition_function(state):
    next_state = np.zeros((1, 6))
    next_state[0] = state[0] + 0.1 * state[3]
    next_state[1] = state[1] + 0.1 * state[4]
    next_state[2] = state[2] + 0.1 * state[5]
    next_state[3] = state[3]
    next_state[4] = state[4]
    next_state[5] = state[5]
    return next_state


def observation_function(state):
    observe = np.zeros(4, 1)
    for i in range(4):
        observe[i] = norm(state[0:3] - anchor_pos[i].transpose())
    return observe
'''


def measure(mes):
    r = np.ma.masked_all((4, 1))
    if not mes.isempty:
        for i in range(4):
            if mes.MASK & (1 << i):
                r[i] = mes.R[i]
            else:
                r[i] = np.ma.masked
    return r


def monitor(buff):
    plt.ion()  # 开启一个画图的窗口
    plt.clf()  # 清除之前画的图
    t = buff.t
    xyz = buff.record
    plt.plot(np.asarray(t) / 1000, np.asarray(xyz) / 1000)  # 画出当前 ax 列表和 ay 列表中的值的图形
    plt.pause(0.001)
    plt.ioff()


def kalman_initial(q):
    kf = KalmanFilter(transition_matrices=np.mat([[1, 0.1], [0, 1]]), observation_matrices=np.mat([1, 0]))
    '''
    kf.transition_matrices = np.mat([[1, 0, 0, 1, 0, 0],
                                     [0, 1, 0, 0, 1, 0],
                                     [0, 0, 1, 0, 0, 1],
                                     [0, 0, 0, 1, 0, 0],
                                     [0, 0, 0, 0, 1, 0],
                                     [0, 0, 0, 0, 0, 1]])
    kf.observation_matrices = np.mat([[1, 0, 0, 0, 0, 0],
                                      [0, 1, 0, 0, 0, 0],
                                      [0, 0, 1, 0, 0, 0]])

    kf.observation_covariance = np.eye(3)
    # kf0.initial_state_covariance = np.eye(6)
    kf.transition_covariance = np.eye(6)
    kf.n_dim_obs = 3
    kf.n_dim_state = 6
    '''
    # 定义初始状态协方差矩阵
    kf.initial_state_covariance = np.mat([[1, 0], [0, 1]])
    # 定义状态转移矩阵，因为每秒钟采一次样，所以delta_t = 1
    kf.transition_matrices = np.mat([[1, 1], [0, 1]])
    # 定义状态转移协方差矩阵，这里我们把协方差设置的很小，因为觉得状态转移矩阵准确度高
    kf.transition_covariance = np.mat([[q, 0], [0, q]])
    # 定义观测矩阵
    kf.observation_matrices = np.mat([1, 0])
    # 定义观测噪声协方差
    kf.observation_covariance = np.mat([758])
    return kf


def runUWB(kal_q, port, mode, max_loop, plot):
    t0 = time.perf_counter()
    rec_size = 100
    tag0_record = RecordBuffer(rec_size)
    tag1_record = RecordBuffer(rec_size)
    global loc0_record, loc1_record
    loc0_record = RecordBuffer(rec_size)
    loc1_record = RecordBuffer(rec_size)
    r0_record = RecordBuffer(rec_size)
    r1_record = RecordBuffer(rec_size)
    # global fil_loc0_record, fil_loc1_record
    # fil_loc0_record = RecordBuffer(rec_size)
    # fil_loc1_record = RecordBuffer(rec_size)
    kf0 = []
    kf1 = []
    for i in range(4):
        kf0.append(kalman_initial(kal_q))
        kf1.append(kalman_initial(kal_q))
    means0 = [[], [], [], []]
    means1 = [[], [], [], []]
    cov0 = [[], [], [], []]
    cov1 = [[], [], [], []]
    '''
    kf0 = kalman_initial()
    kf1 = kalman_initial()
    head_loc_data0 = np.ma.masked_all((3, 4))
    m0_count = 0
    head_loc_data1 = np.ma.masked_all((3, 4))
    m1_count = 0
    '''
    if mode == WRITE_MES:
        filehandleT0 = open("./T0.csv", "w")
        filehandleT1 = open("./T1.csv", "w")
    elif mode == READ_MES:
        filehandleT0 = open("./T0.csv", "r")
        filehandleT1 = open("./T1.csv", "r")
    elif mode == REAL_TIME:
        ser = UWB_Initial(port)
    # Main loop
    i = 0
    while True:
        if mode == READ_MES:
            lin = filehandleT0.readline()
            if not lin:  # 等价于if line == "":
                break
            mes0 = UWBMessage()
            mes0.from_file(lin)
            mes0.tagID = 0

            lin = filehandleT1.readline()
            if not lin:  # 等价于if line == "":
                break
            mes1 = UWBMessage()
            mes1.from_file(lin)
            mes1.tagID = 1

        else:

            while True:
                mes0, mes1 = recv(ser, t0)
                if (not mes0.isempty) and (not mes1.isempty):
                    break

            tag0_record.append(mes0.t, mes0)
            tag1_record.append(mes1.t, mes1)
            if mode == WRITE_MES:
                mesStr0 = '%d,%d,%d,%d,%d,%d,%d\n' % (mes0.t, mes0.MASK, mes0.R0, mes0.R1, mes0.R2, mes0.R3, mes0.RSEQ)
                mesStr1 = '%d,%d,%d,%d,%d,%d,%d\n' % (mes1.t, mes1.MASK, mes1.R0, mes1.R1, mes1.R2, mes1.R3, mes1.RSEQ)
                filehandleT0.write(mesStr0)
                filehandleT1.write(mesStr1)

        r0 = measure(mes0) * 0.8675 - 377.6712
        r1 = measure(mes1) * 0.8675 - 377.6712
        filtered_r0 = np.zeros((4, 1))
        filtered_r1 = np.zeros((4, 1))

        for j in range(4):
            if len(means0[j]) == 0:
                (temp, cov0[j]) = kf0[j].filter_update(np.mat([[r0[j], ], [0, ]]), np.mat([[1, 0], [0, 1]]), r0[j])
                means0[j] = temp[:, -1]

            else:
                (means0[j], cov0[j]) = kf0[j].filter_update(means0[j], cov0[j], r0[j])
            filtered_r0[j] = means0[j][0][0]

            if len(means1[j]) == 0:
                (temp, cov1[j]) = kf1[j].filter_update(np.mat([[r1[j], ], [0, ]]), np.mat([[1, 0], [0, 1]]), r1[j])
                means1[j] = temp[:, -1]
            else:
                (means1[j], cov1[j]) = kf1[j].filter_update(means1[j], cov1[j], r1[j])
            filtered_r1[j] = means1[j][0]
        # r0_record.append(mes0.t,[r0[0][0],means0[0][0][0]])
        r0_record.append(mes0.t, r0[..., 0])
        r1_record.append(mes1.t, [r1[0][0], means1[0][0][0]])
        loc0 = triangulation(anchor_pos, 4, filtered_r0)
        loc0_record.append(mes0.t, loc0)
        loc1 = triangulation(anchor_pos, 4, filtered_r1)
        loc1_record.append(mes1.t, loc1)

        # runtime = time.perf_counter()
        # print(runtime-t0)
        if plot and i % 10 == 0:
            monitor(loc0_record)
        i = i + 1

        if i > max_loop and mode == WRITE_MES:
            break
        '''
        if m0_count == -1:
            next_mean0, next_covariance0 = kf0.filter_update(last_means0, last_covariance0, loc0)
            fil_loc0_record.append(mes0.t, next_mean0)
            last_means0 = next_mean0
            last_covariance0 = next_covariance0
            print(next_mean0)
            # monitor(fil_loc0_record.t, fil_loc0_record.record)

        elif m0_count > 3:
            means0, covariances0 = kf0.filter(head_loc_data0.transpose())
            m0_count = -1
            last_means0 = means0[0, ...].transpose()
            last_covariance0 = covariances0[0, ...].transpose()
        elif mes0.MASK == 15:
            head_loc_data0[:, m0_count] = loc0
            m0_count = m0_count + 1

        if m1_count == -1:
            next_mean1, next_covariance1 = kf1.filter_update(last_means1, last_covariance1, loc1)
            fil_loc1_record.append(mes1.t, next_mean1)
            last_means1 = next_mean1
            last_covariance1 = next_covariance1
            print(next_mean1)

        elif m1_count > 3:
            means1, covariances1 = kf1.filter(head_loc_data1.transpose())
            m1_count = -1
            last_means1 = means1[0, ...].transpose()
            last_covariance1 = covariances1[0, ...].transpose()
        elif mes1.MASK == 15:
            head_loc_data1[:, m1_count] = loc1
            m1_count = m1_count + 1
        '''
    # end main loop
    if mode == READ_MES or mode == WRITE_MES:
        filehandleT0.close()
        filehandleT1.close()
    # plt.plot(r0_record.record)
    # plt.plot(loc0_record.record)
    # plt.show()


def route_planning(arg):
    while True:
        time.sleep(2)
        print(arg, loc0_record.t[-1], '  ', loc0_record.record[-1])

'''
if __name__ == '__main__':
    REAL_TIME = 0
    WRITE_MES = 1
    READ_MES = 2
    q = 0.01  # 卡尔曼滤波的系统噪声协方差
    max_loop = 600  # WRITE_MES模式下的循环次数
    port = 'COM3'  # 串口端口
    plot = True  # 绘制XYZ曲线
    # anchor_pos = np.mat([[0, 0, 2000], [1000, 0, 2000], [1000, 1000, 2000], [0, 1000, 3800]])

    # 基站的坐标
    anchor_pos = np.mat([[0, 3110, 3110, 0],
                                [0, 0, 6920, 6920],
                                [2280, 2280, 2280, 2280]])
    mode = REAL_TIME   # UWB 工作模式



     # 车子（loc0_record）与人（loc0_record）的坐标，最后的100次记录

    global loc0_record, loc1_record
    t1 = threading.Thread(target=runUWB, args=(q, port, mode, max_loop, plot))  # UWB线程
    t2 = threading.Thread(target=route_planning, args=("t2",))  # 路径规划线程
    t1.start()
    #t2.start()

    # runUWB(q, port, mode, max_loop, plot)
'''