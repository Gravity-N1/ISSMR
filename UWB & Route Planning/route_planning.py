import serial as pyserial
import time
import numpy as np
import math
#与小车通讯buffer
class RecordBuffer2:
    def __init__(self, size):
        self.record = []
        self.size = size

    def append(self, obj):
        self.record.append(obj)
        if len(self.record) > self.size:
            del self.record[0]

    def get_record(self):
        return self.record

    def __len__(self):
        return len(self.record)


# data shift operation
def multi(distance, angle, angle2):
    d = int(round(distance))
    # a = int(abs(round((angle-angle2)*1000)))
    a = int(abs(round(angle * 10)))
    judge = 0
    if angle >= 0:
        judge = 255
        result = 255 | a << 8 | judge << 24 | d << 32 | 255 << 48 | 170 << 56
    else:
        judge = 0
        result = 255 | a << 8 | judge << 24 | d << 32 | 255 << 48 | 170 << 56

    result = result.to_bytes(length=8, byteorder='big', signed=False)
    print(result)

    return result


# 计算两向量的欧氏距离
def embedding_distance(feature_1, feature_2):
    dist = np.linalg.norm(feature_1 - feature_2)
    return dist


# record data
def calculate(start, test):
    global loc0_record, loc1_record
    car_size = 100
    people_size = 1000
    loc_car = RecordBuffer2(car_size)
    loc_people = RecordBuffer2(people_size)

    len1 = len(loc0_record)
    len2 = len(loc1_record)
    move = False
    distance = 50
    angle = 0

    if len1 != 0:
        loc_car.append(loc0_record.record[len1 - 1][0:2])
        loc_people.append(loc1_record.record[len2 - 1][0:2])
        print(loc_people.record[len(loc_people) - 1])
        len5 = len(loc_people)
        read_judge = embedding_distance(loc_people.record[len5 - 2], loc_people.record[len5 - 1])
        if read_judge <= 1 and len5 >= 40:
            del loc_people.record[len5 - 1]
        print(loc_car.record)
        print(loc_people.record)

        len3 = len(loc_car)
        len4 = len(loc_people)

        car_loc = loc_car.record[len3 - 1]  # 小车当前位置
        people_loc = loc_people.record[0]  # 小车要到达的人的位置
        people_loc2 = loc_people.record[len4 - 1]  # 人的最新位置

        # 判断车子是否运动
        distance_start = embedding_distance(car_loc, people_loc2)
        with open('test.txt', 'r') as stop:  # 读取摄像头文件
            data = stop.readline()
        if distance_start > start and data == '1':
            move = True

        # 判断是否删除目标点
        distance_judge = embedding_distance(car_loc, people_loc)
        if distance_judge <= test and move:
            del loc_people.record[0]
        angle2 = 0
        if len3 >= 2:
            angle2 = math.atan2(loc_car[len3][0] - loc_car[len3 - 1][0], loc_car[len3][1] - loc_car[len3 - 1][1])
        distance = embedding_distance(car_loc, loc_people.record[0])
        # angle = math.atan2(car_loc[0]-people_loc[0],car_loc[1]-people_loc[1])
        angle = math.atan2(people_loc[1] - car_loc[1], people_loc[0] - car_loc[0])
        # angle = math.atan2(people_loc[0]-0,people_loc[1]-0)
        angle = (angle / 3.14) * 180
        print(distance, angle)

    return distance, angle, angle2, move


# 路径规划
def route_planning(send_port, start, test):
    ser = pyserial.Serial(send_port, 115200, timeout=0.15)
    ser.flushInput()
    last_t = -1
    # 这个位置需要讨论

    while True:

        while True:
            time.sleep(0.001)
            if len(loc0_record.t) == 0 or len(loc1_record.t) == 0:
                continue
            elif last_t != loc0_record.t[-1]:
                break

        last_t = loc0_record.t[-1]
        [d, a, a2, move] = calculate(start, test)
        if move == True:
            send_data = multi(d, a, a2)
            ser.write(send_data)
        else:
            stop1 = 0xaa000000000000ff
            stop1 = stop1.to_bytes(length=8, byteorder='big', signed=False)
            ser.write(stop1)
            print(0)

        location_car = ser.readlines()
        print(location_car)
