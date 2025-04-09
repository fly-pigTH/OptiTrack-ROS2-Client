# Author: Yinglei Zhu
# Version: 1.0
# For robot sensor data processing

import sys
# sys.path.append("D:/ZYL/Work/Zhao/SoftRobotics/Docs/Code/control/sensor")   # 添加自定义模块路径 for the NatNetClient.py
# sys.path.append("D/WQY/Data")
from NatNetClient import NatNetClient
import time
from collections import deque
import numpy as np
# import csv

# udp to vmware ubuntu
import socket
SERVER_IP = '172.16.0.1'
SERVER_PORT = '1234'
# SERVER_PORT = int(input('please input server port: '))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_address = (SERVER_IP, SERVER_PORT)

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

    def __init__(self,rigidBodyDict,freq=int(120),vel_samples=int(4)):

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
    q1 = np.array(q1)
    q2 = np.array(q2)
    quat2rotMat_1 = quat2rotMat(q1)
    quat2rotMat_2 = quat2rotMat(q2)
    R = np.dot(quat2rotMat_2, quat2rotMat_1.T)
    # give the sign here!
    z0 = [0,0,1]
    z_rot = np.dot(R, z0)  # 旋转之后的z轴
    if z_rot[0] < 0:
        ang_sign = -1
    else:
        ang_sign = 1
    temp = (np.trace(R) - 1) / 2
    if temp > 1:
        temp = 1
    elif temp < -1:
        temp = -1
    ang_abs = np.arccos(temp)
    return ang_abs*ang_sign

def cal_rotang_alongY(quat2rotMat_1, quat2rotMat_2):
    z0 = [0,0,1]
    z1 = np.dot(quat2rotMat_1, z0)  # 旋转到1坐标系下的z轴
    z2 = np.dot(quat2rotMat_2, z0)  # 旋转到2坐标系下的z轴
    angle = np.arccos(np.dot(z1, z2) / (np.linalg.norm(z1) * np.linalg.norm(z2)))

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
    # def __init__(self, rigidBodyDict={1:Rigidbody(1), 2:Rigidbody(2), 3:Rigidbody(3)}):
    def __init__(self, rigidBodyDict={1:Rigidbody(1)}):
        self.rigidBodyDict = rigidBodyDict
        # run the natnet client
        self.natnet = Natnet(self.rigidBodyDict)

        # calibrate prepare
        self.calibrate_list = [False]
        self.dist = [0]
        self.ang = [0]
        self.theta = np.array([0], dtype=float)
        self.h_vec = None
        self.h = None
        self.h_init_measure = 0.5    # 中心高度
        self.h_foot_measure = 0.035
        self.a_1 = 0.25
        self.a_2 = 0.25

    def calibrate(self):
        '''校准刚体位置和四元数'''
        while self.natnet.running() and not all(self.calibrate_list):
            time.sleep(0.5)
            for i in range(1):
                rigidBodyKey = i+1
                if self.calibrate_list[i] == False:
                    print(f"Calibrating body[{i}]...")
                    if not self.rigidBodyDict[rigidBodyKey].calibration:
                        if self.rigidBodyDict[rigidBodyKey].valid:
                            print('vbcei')
                            self.rigidBodyDict[rigidBodyKey].set_calibration(self.rigidBodyDict[rigidBodyKey].position, self.rigidBodyDict[rigidBodyKey].quat)
                            print(f"Body[{i}] init pos = {self.rigidBodyDict[rigidBodyKey].init_position}")
                            print(f"Body[{i}] init quad = {self.rigidBodyDict[rigidBodyKey].init_quat}")
                            self.h_init = self.rigidBodyDict[rigidBodyKey].init_position
                            self.calibrate_list[i] = True


    def read_sensor_data(self):
        if self.natnet.running():
            if not all(self.calibrate_list):
                raise Exception("Please calibrate first!")
            else:
                for i in self.rigidBodyDict.keys():
                    if self.rigidBodyDict[i].valid:
                        self.dist[i-1] = distance2initpos(self.rigidBodyDict[i].position, self.rigidBodyDict[i].init_position)
                        self.ang[i-1] = cal_rotang_quat(quat_JPL2Hamilton(self.rigidBodyDict[i].init_quat), quat_JPL2Hamilton(self.rigidBodyDict[i].quat))
                    else:
                        print(f"[Warning] Body[{i}] is not valid!")
                        time.sleep(0.5)

                # # update theta
                # self.theta[0] = np.pi/2 + self.ang[1]
                # self.theta[1] = self.ang[2] - self.ang[1]
                # self.theta[2] = self.theta[0] + self.theta[1]
                # # print(f"Theta: {self.theta*180/np.pi}")
                return self.dist, self.theta


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
            print(f"Theta: {self.theta*180/np.pi}")


            # if self.rigidBodyDict[1] != None and all(self.calibrate_list):
            #     self.h_vec = self.rigidBodyDict[1].position - self.h_init
            #     h_abs = np.linalg.norm(self.h_vec)
            #     if self.rigidBodyDict[1].position[2] > self.h_init[2]:
            #         self.h = h_abs
            #     else:
            #         self.h = -h_abs
                
            #     print(f"去足高度: {self.h_init_measure+self.h}")
            #     length = self.a_1*np.sin(self.theta[0])+self.a_2*np.sin(self.theta[2])
            #     print(f"理论长度: {length}")

            #     # fly_judgement
            #     if (self.h_init_measure+self.h - length) > 0.03:
            #         print("fly")
            #     else:
            #         print("land")
            # print(f"RES: {self.dist}, {self.theta*180/np.pi}")
            time.sleep(0.2)
            
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

    # exp_name = "optitrack_time_test"
    # time_rela_list = [0]     # 相对时间
    # time_abs_list = [0]      # 系统时间，作为绝对时间
    x = 0
    y = 0
    z = 0
    w = 0
    z_prev = 0

    t_start = time.time()

    run_flag = True
    record = False

    while run_flag:
        try:
            sensor.read_sensor_data()

            if z_prev != sensor.rigidBodyDict[1].quat[2]: # prevent sending repeat frame
                # t_rela = time.time() - t_start
                # t_abs = time.time()
                # time_rela_list.append(t_rela)
                # time_abs_list.append(t_abs)
                px = sensor.rigidBodyDict[1].position[0]
                py = sensor.rigidBodyDict[1].position[1]
                pz = sensor.rigidBodyDict[1].position[2]
                qx = sensor.rigidBodyDict[1].quat[0]
                qy = sensor.rigidBodyDict[1].quat[1]
                qz = sensor.rigidBodyDict[1].quat[2]
                qw = sensor.rigidBodyDict[1].quat[3]
                position_str = str(x)+','+str(y)+','+str(z)+','+str(w)
                message = position_str.encode('utf-8')
                sock.sendto(message, server_address)
                print(f"Sending: {px:.3f}, {py:.3f}, {pz:.3f}, {qx:.3f}, {qy:.3f}, {qz:.3f}, {qw:.3f}") 
                
                z_prev = z

        except:
            print("streaming shut down")
            break
    sensor.shutdown()
