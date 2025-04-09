# Author: Yinglei Zhu
# Version: 1.1 增加了地面参照坐标
# For robot sensor data processing
# 2025-01持续更新version，这一版的频率是120Hz

import sys
sys.path.append("./sensor")   # 添加自定义模块路径 for the NatNetClient.py
from NatNetClient import NatNetClient
import time
from collections import deque
import numpy as np
import csv

# plot
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np


class Rigidbody():
    def __init__(self,ac_id):
        self.ac_id = ac_id
        self.valid = False
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)

        self.heading = 0.
        self.quat = np.zeros(4)

        self.init_position = np.zeros(3)  # initial position
        self.init_quat = np.zeros(4)      # initial quaternion
        self.calibration = False    # calibration flag, default is False

    def set_calibration(self, init_pos, init_quat):
        self.calibration = True
        self.init_position = init_pos
        self.init_quat = init_quat


class Natnet():
    def store_track(self,ac_id, pos, t):
        if ac_id in self.rigidBodyDict.keys():    # store to-track data
            self.track[ac_id].append((pos, t))
            if len(self.track[ac_id]) > self.vel_samples:   # BUG: 为什么用vel_samples？
                self.track[ac_id].popleft()   # remove oldest sample


    def compute_velocity(self,ac_id):
        vel = [ 0., 0., 0. ]
        if len(self.track[ac_id]) >= self.vel_samples:  # BUG: 为什么用vel_samples？
            nb = -1
            for (p2, t2) in self.track[ac_id]:
                nb = nb + 1
                if nb == 0: # initialize the first frame of(p, t)
                    p1 = p2
                    t1 = t2
                else:
                    dt = t2 - t1
                    if dt < 1e-5: # remove the same frame
                        continue
                    vel[0] += (p2[0] - p1[0]) / dt
                    vel[1] += (p2[1] - p1[1]) / dt
                    vel[2] += (p2[2] - p1[2]) / dt
                    p1 = p2
                    t1 = t2
            if nb > 0:
                vel[0] /= nb  # average velocity
                vel[1] /= nb
                vel[2] /= nb
        return vel
    

    def receiveRigidBodyMarkerSetList(self, rigid_body_data, marker_set_data, stamp):
        """Receive rigid body data"""
        for rigid_body in rigid_body_data.rigid_body_list:
            # print("RigidBody ID: ", rigid_body.id_num, "  Tracking Valid: ", rigid_body.tracking_valid)
            i = rigid_body.id_num
            if i not in self.rigidBodyDict.keys():
                continue
            if not rigid_body.tracking_valid: # cannot track, then skip
                self.rigidBodyDict[i].valid = False
                continue
            # NOTE: get the data!!!
            pos = rigid_body.pos
            quat = rigid_body.rot
            self.store_track(i, pos, stamp)
            if self.timestamp[i] is None or abs(stamp - self.timestamp[i]) < self.period:
                if self.timestamp[i] is None:
                    self.timestamp[i] = stamp
                continue # too early for next message
            self.timestamp[i] = stamp
            vel = self.compute_velocity(i)
            dcm_0_0 = 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2])
            dcm_1_0 = 2.0 * (quat[0] * quat[1] - quat[3] * quat[2])
            self.rigidBodyDict[i].quat=quat
            self.rigidBodyDict[i].heading=np.arctan2(dcm_1_0, dcm_0_0)
            self.rigidBodyDict[i].position=np.array([pos[0],pos[1],pos[2]])
            self.rigidBodyDict[i].velocity=np.array([vel[0],vel[1],vel[2]])
            self.rigidBodyDict[i].valid = True

    def __init__(self,rigidBodyDict,freq=int(120),vel_samples=int(4)):   # TODO 这里似乎控制频率了 orin: freq=20

        self.runningstate = False
        self.rigidBodyDict = rigidBodyDict
        self.period = 1.0 / freq
        self.vel_samples = vel_samples
        self.timestamp = dict([(rb, None) for rb in self.rigidBodyDict])
        self.track = dict([(rb, deque()) for rb in self.rigidBodyDict])

        # run the natnet client
        self.natnet = NatNetClient()
        self.natnet.set_server_address("127.0.0.1")
        self.natnet.set_client_address('0.0.0.0')
        self.natnet.set_print_level(0)  # 1 to print all frames
        
        # use the callback function
        self.natnet.rigid_body_marker_set_list_listener = self.receiveRigidBodyMarkerSetList
        
        # run the client and check if it is connected
        if self.natnet.run():
            time.sleep(1)
            if self.natnet.connected():
                self.runningstate=True
        if not self.runningstate:
            self.stop()

    def running(self):
        return (self.runningstate)

    def stop(self):
        self.natnet.shutdown()



# --- Tools ---
def cal_rotang_quat(q1, q2):
    '''计算两个四元数之间的夹角(要求q是wxyz四元数), q2 相对于 q1'''
    '''这里计算的是绝对值, 即不考虑旋转方向, 只考虑旋转角度'''
    # 角度范围：-pi ~ pi

    q1 = np.array(q1)
    q2 = np.array(q2)
    quat2rotMat_1__ground = quat2rotMat(q1)
    quat2rotMat_2__ground = quat2rotMat(q2)
    R_2__1 = np.dot(quat2rotMat_2__ground, quat2rotMat_1__ground.T)  # 因为目标是看2的x在1下的投影，所以需要R_2^1

    # give the sign here!
    i2 = [1,0,0]
    i2__1 = np.dot(R_2__1, i2)  # 旋转之后的x轴, i_1^2

    # 计算旋转角度. atan2()返回的是[-pi, pi]之间的值
    ang = np.arctan2(-i2__1[2], i2__1[0])  # 注意，np.arctan2(y, x)是先y后x

    # 直接采用两个旋转矩阵的转动夹角计算
    # _ang = (np.trace(R) - 1) / 2
    # if _ang > 1:
    #     _ang = 1
    # elif _ang < -1:
    #     _ang = -1
    # ang_abs = np.arccos(_ang)

    return ang



# def cal_rotang_alongY(quat2rotMat_1, quat2rotMat_2):
#     z0 = [0,0,1]
#     z1 = np.dot(quat2rotMat_1, z0)  # 旋转到1坐标系下的z轴
#     z2 = np.dot(quat2rotMat_2, z0)  # 旋转到2坐标系下的z轴
#     angle = np.arccos(np.dot(z1, z2) / (np.linalg.norm(z1) * np.linalg.norm(z2)))

def quat_JPL2Hamilton(q):
    '''将JPL四元数转换为Hamilton四元数'''
    q = np.array(q)
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
    q_hamilton = np.array([q3, q0, q1, q2])
    return q_hamilton

def quat2rotMat(q):
    '''将Hamilton四元数转换为旋转矩阵(要求q是wxyz四元数)'''
    q = np.array(q)
    q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
    R = np.array([[1-2*q2**2-2*q3**2, 2*q1*q2-2*q0*q3, 2*q1*q3+2*q0*q2],
                  [2*q1*q2+2*q0*q3, 1-2*q1**2-2*q3**2, 2*q2*q3-2*q0*q1],
                  [2*q1*q3-2*q0*q2, 2*q2*q3+2*q0*q1, 1-2*q1**2-2*q2**2]])
    return R

# 计算和初始位置的距离
def distance2initpos(pos, init_pos):
    return np.linalg.norm(pos - init_pos)



# --- Sensor Class ---
class RobotSensor():
    def __init__(self, rigidBodyDict={1:Rigidbody(1), 2:Rigidbody(2), 3:Rigidbody(3), 4:Rigidbody(4)}):
        # 采用4个刚体，其中，刚体4为地面参照坐标
        self.rigidBodyDict = rigidBodyDict
        # run the natnet client
        self.natnet = Natnet(self.rigidBodyDict)

        # calibrate prepare
        self.calibrate_list = [False, False, False, False]
        # self.dist = [0,0,0]
        # self.ang = [0,0,0]

        # self.theta = np.array([0,0,0], dtype=float)

        # self.h_vec = None
        # self.h = None
        # self.h_init_measure = 0.5    # 中心高度
        # self.h_foot_measure = 0.035
        self.a_1 = 0.25
        self.a_2 = 0.25

        self.base = {
            'x': None,
            'y': None,
            # 'z': None,
            'theta10': None
        }

        self.thigh = {
            # 'x': None,
            # 'y': None,
            # 'z': None,
            'theta20': None,
            'theta21': None
        }

        self.calf = {
            # 'x': None,
            # 'y': None,
            # 'z': None,
            'theta30': None,
            'theta32': None,
        }

    def calibrate(self):
        '''校准刚体位置和四元数'''
        while self.natnet.running() and not all(self.calibrate_list):
            time.sleep(0.5)
            for i in range(4):   # 3 origin
                rigidBodyKey = i+1
                if self.calibrate_list[i] == False:
                    print(f"Calibrating body[{i}]...")
                    if not self.rigidBodyDict[rigidBodyKey].calibration:
                        if self.rigidBodyDict[rigidBodyKey].valid:
                            self.calibrate_list[i] = True
                            pass    # do nothing
                            '''
                            self.rigidBodyDict[rigidBodyKey].set_calibration(self.rigidBodyDict[rigidBodyKey].position, self.rigidBodyDict[rigidBodyKey].quat)
                            print(f"Body[{i}] init pos = {self.rigidBodyDict[rigidBodyKey].init_position}")
                            print(f"Body[{i}] init quad = {self.rigidBodyDict[rigidBodyKey].init_quat}")
                            self.h_init = self.rigidBodyDict[rigidBodyKey].init_position
                            '''


    def read_sensor_data(self):
        if self.natnet.running():
            if not all(self.calibrate_list):
                raise Exception("Please calibrate first!")
            else:
                if self.rigidBodyDict[4].valid and self.rigidBodyDict[1].valid and self.rigidBodyDict[2].valid and self.rigidBodyDict[3].valid:
                    # do post process
                    self.base['x'] = self.rigidBodyDict[4].position[0] - self.rigidBodyDict[1].position[0]  # 符号是负的
                    self.base['y'] = self.rigidBodyDict[1].position[2] - self.rigidBodyDict[4].position[2]
                    self.base['theta10'] = cal_rotang_quat(quat_JPL2Hamilton(self.rigidBodyDict[4].quat), quat_JPL2Hamilton(self.rigidBodyDict[1].quat))

                    self.thigh['theta20'] = cal_rotang_quat(quat_JPL2Hamilton(self.rigidBodyDict[4].quat), quat_JPL2Hamilton(self.rigidBodyDict[2].quat))
                    self.thigh['theta21'] = cal_rotang_quat(quat_JPL2Hamilton(self.rigidBodyDict[1].quat), quat_JPL2Hamilton(self.rigidBodyDict[2].quat))
                    # 考虑偏置：学长-0，我+pi/2
                    self.thigh['theta21'] = self.thigh['theta21'] + np.pi/2

                    self.calf['theta30'] = cal_rotang_quat(quat_JPL2Hamilton(self.rigidBodyDict[4].quat), quat_JPL2Hamilton(self.rigidBodyDict[3].quat))
                    self.calf['theta32'] = cal_rotang_quat(quat_JPL2Hamilton(self.rigidBodyDict[2].quat), quat_JPL2Hamilton(self.rigidBodyDict[3].quat))
                    # 考虑偏置
                    self.calf['theta32'] = self.calf['theta32'] + 0
                    return self.base, self.thigh, self.calf
                else:
                    if not self.rigidBodyDict[1].valid:
                        print(f"[Warning] Body[1] is not valid!")
                    if not self.rigidBodyDict[2].valid:
                        print(f"[Warning] Body[2] is not valid!")
                    if not self.rigidBodyDict[3].valid:
                        print(f"[Warning] Body[3] is not valid!")
                    if not self.rigidBodyDict[4].valid:
                        print(f"[Warning] Body[4] is not valid!")
                    # time.sleep(0.5)


    def run(self):
        while self.natnet.running():
            time.sleep(0.5)

            if not all(self.calibrate_list):
                input("Type any key to calibrate...")
                print("Calibrating...")
                self.calibrate()
                print("Calibration done!")

            for i in self.rigidBodyDict.keys():
                if self.rigidBodyDict[i].valid:
                    self.dist[i-1] = distance2initpos(self.rigidBodyDict[i].position, self.rigidBodyDict[i].init_position)
                    # print(f"Body[{i}] dist = {self.dist[i-1]}", end = " ")

                    self.ang[i-1] = cal_rotang_quat(quat_JPL2Hamilton(self.rigidBodyDict[i].init_quat), quat_JPL2Hamilton(self.rigidBodyDict[i].quat))
                    # print(f"Body[{i}] rot = {self.ang[i-1]}")

            print("-------------------")
            self.theta[0] = np.pi/2 + self.ang[1]
            self.theta[1] = self.ang[2] - self.ang[1]
            self.theta[2] = self.theta[0] + self.theta[1]
            # print(f"Theta: {self.theta*180/np.pi}")

            # 打印所有刚体的信息
            # for i in self.rigidBodyDict.keys():
            #     print(f"AC {i}: pos={self.rigidBodyDict[i].position}, quad={self.rigidBodyDict[i].quat}")
            # time.sleep(0.2)
            
        self.natnet.stop()
        print("Natnet stopped")
        sys.exit(0)        

    def shutdown(self):
        self.natnet.stop()
        print("Natnet stopped")
        sys.exit(0)


# 定义浮点数格式化函数
def format_float(value):
    return "{:.3f}".format(value)  # 设置要保留的小数位数


if __name__ == "__main__" and True:
    sensor = RobotSensor()
    # sensor.run()
    if not all(sensor.calibrate_list):
        input("Type any key to calibrate...")
        print("Calibrating...")
        sensor.calibrate()
        print("Calibration done!")

    exp_name = "静力学控制实验"
    time_rela_list = []     # 相对时间
    time_abs_list = []      # 
    
    base_x_list = []        
    base_y_list = []    
    base_theta_10 = []
    thigh_theta_21 = []
    calf_theta_32 = []

    t_start = time.time()
    run_flag = True
    last_sensor_base_y = 1000000000

    while run_flag:
        try:
            count = 0
            sensor.read_sensor_data()

            # debug: 打印所有刚体的位置
            # print(f"RigibBody[1]: {sensor.rigidBodyDict[1].position}")
            # print(f"RigibBody[2]: {sensor.rigidBodyDict[2].position}")
            # print(f"RigibBody[3]: {sensor.rigidBodyDict[3].position}")
            # print(f"RigibBody[4]: {sensor.rigidBodyDict[4].position}")
            # time.sleep(0.5)

            # if sensor.base['y'] != last_sensor_base_y:
            #     print(sensor.base['y'], last_sensor_base_y)
            #     # input()
            #     count+=1
            #     print(count)
            #     # input()
            # last_sensor_base_y = sensor.base['y']

            # debug
            if time.time() % 1 < 1/120:
                print(f"Time: {time.time()-t_start}")
                print(f"Base: x={format_float(sensor.base['x'])}, y={format_float(sensor.base['y'])}, theta10={format_float(sensor.base['theta10']*180/np.pi)}")
                print(f"Thigh: theta20={format_float(sensor.thigh['theta20']*180/np.pi)}, theta21={format_float(sensor.thigh['theta21']*180/np.pi)}")
                print(f"Calf: theta30={format_float(sensor.calf['theta30']*180/np.pi)}, theta32={format_float(sensor.calf['theta32']*180/np.pi)}")

            t_rela = time.time() - t_start
            t_abs = time.time()
            time_rela_list.append(t_rela)
            time_abs_list.append(t_abs)
            base_x_list.append(sensor.base['x'])
            base_y_list.append(sensor.base['y'])
            base_theta_10.append(sensor.base['theta10'])
            thigh_theta_21.append(sensor.thigh['theta21'])
            calf_theta_32.append(sensor.calf['theta32'])
            # time.sleep(0.2)

            # if count % 100000 == 0:
            #     print(count)
            #     print(time_rela_list[-1])
            time.sleep(1/120)

        except KeyboardInterrupt:
            print("实验结束！")
            # 记录数据
            file_name = time.strftime("./Exp_data/OptitrackRead_%Y-%m-%d_%H-%M-%S", time.localtime()) + f"_{exp_name}_data.csv"
            with open(file_name, "w", newline="") as f:
                csv_writer = csv.writer(f)
                csv_writer.writerow(["Time", "Time_abs", "base_x_list", "base_y_list", "base_theta_10", "thigh_theta_21", "calf_theta_32"])
                for i in range(len(time_rela_list)):
                    csv_writer.writerow([time_rela_list[i], time_abs_list[i], base_x_list[i], base_y_list[i], base_theta_10[i], thigh_theta_21[i], calf_theta_32[i]])
            print("Processed Finished")
            run_flag = False
            break
    sensor.shutdown()

# test and run
# if __name__ == "__main__" and False:
#     sys.exit(0)
#     rigidBodyDict = {1:Rigidbody(3), 2:Rigidbody(4), 3:Rigidbody(5)}
#     natnet = Natnet(rigidBodyDict)

#     calibrate_list = [False, False, False]
#     dist = [0,0,0]
#     ang = [0,0,0]
#     theta = np.array([0,0,0], dtype=float)
#     # h_list = [None, None, None]     # 保存三个刚体的位置矢量
#     h_vec = None
#     h = None    # 其实是个位移

#     # 测量值
#     h_init_measure = 0.5    # 中心高度
#     h_foot_measure = 0.035

#     # 腿部高度
#     a_1 = 0.25
#     a_2 = 0.25

#     while natnet.running():
#         time.sleep(0.5)

#         if calibrate_list[0] == False:
#             print("Calibrating body[1]...")
#             # calibrate
#             if not rigidBodyDict[1].calibration:
#                 if rigidBodyDict[1].valid:
#                     rigidBodyDict[1].set_calibration(rigidBodyDict[1].position, rigidBodyDict[1].quat)
#                     # print init pos and quad
#                     print(f"Body[1] init pos = {rigidBodyDict[1].init_position}")
#                     print(f"Body[1] init quad = {rigidBodyDict[1].init_quat}")
#                     h_init = rigidBodyDict[1].init_position
#                     calibrate_list[0] = True

#         if calibrate_list[1] == False:
#             print("Calibrating body[2]...")
#             if not rigidBodyDict[2].calibration:
#                 if rigidBodyDict[2].valid:
#                     rigidBodyDict[2].set_calibration(rigidBodyDict[2].position, rigidBodyDict[2].quat)
#                     # print init pos and quad
#                     print(f"Body[2] init pos = {rigidBodyDict[2].init_position}")
#                     print(f"Body[2] init quad = {rigidBodyDict[2].init_quat}")
#                     calibrate_list[1] = True

#         if calibrate_list[2] == False:
#             print("Calibrating body[3]...")
#             if not rigidBodyDict[3].calibration:
#                 if rigidBodyDict[3].valid:
#                     rigidBodyDict[3].set_calibration(rigidBodyDict[3].position, rigidBodyDict[3].quat)
#                     # print init pos and quad
#                     print(f"Body[3] init pos = {rigidBodyDict[3].init_position}")
#                     print(f"Body[3] init quad = {rigidBodyDict[3].init_quat}")
#                     calibrate_list[2] = True

#         # if calibrate_list[0] and calibrate_list[1]:
#         #     print("Calibration done!")
#         #     break

#         for i in rigidBodyDict.keys():
#             # print(f"key: {i}")
#             if rigidBodyDict[i].valid:
#                 # print(f"pos={rigidBodyDict[i].position}")
#                 # 计算和初始位置的距离
#                 dist[i-1] = distance2initpos(rigidBodyDict[i].position, rigidBodyDict[i].init_position)
#                 print(f"Body[{i}] dist = {dist[i-1]}", end = " ")

#                 # 计算和初始位姿的转动角度
#                 ang[i-1] = cal_rotang_quat(quat_JPL2Hamilton(rigidBodyDict[i].init_quat), quat_JPL2Hamilton(rigidBodyDict[i].quat))
#                 print(f"Body[{i}] rot = {ang[i-1]}")

#                 # print(f"Quad[{i}] = {rigidBodyDict[i].quat}", end = " ")
#                 # print("AC %d: pos=%s, vel=%s, quad=%s" % (i, rigidBodyDict[i].position, rigidBodyDict[i].velocity, rigidBodyDict[i].quat))
#                 # ax.scatter(rigidBodyDict[i].position[0], rigidBodyDict[i].position[1], rigidBodyDict[i].position[2], c='blue', marker='^')
#                 # plt.pause(0.01)
#         print("-------------------")
#         theta[0] = np.pi/2 + ang[1]
#         theta[1] = ang[2] - ang[1]  # ang[2]是绝对旋转，需要减掉ang[1]的绝对旋转
#         theta[2] = theta[0] + theta[1]

#         if rigidBodyDict[1] != None and calibrate_list[0] == True:
#             h_vec = rigidBodyDict[1].position - h_init
#             h_abs = np.linalg.norm(h_vec)
#             if rigidBodyDict[1].position[2] > h_init[2]:
#                 h = h_abs
#             else:
#                 h = -h_abs
            
#             print(f"去足高度: {h_init_measure+h}")
#             length = a_1*np.sin(theta[0])+a_2*np.sin(theta[2])
#             print(f"理论长度: {length}")

#             # fly_judgement
#             if (h_init_measure+h - length) > 0.03:
#                 print("fly")
#             else:
#                 print("land")
#         print(f"RES: {dist}, {theta*180/np.pi}")
#         time.sleep(0.2)
#     natnet.stop()
#     print("Natnet stopped")
#     # plt.show()
#     sys.exit(0)

    
