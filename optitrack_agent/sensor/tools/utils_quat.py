# 四元数工具箱
# 暂时不启用
import numpy as np


# def quat2rotMat(q):
#     '''将Hamilton四元数转换为旋转矩阵(要求q是wxyz四元数)'''
#     q = np.array(q)
#     q0, q1, q2, q3 = q[0], q[1], q[2], q[3]
#     R = np.array([[1-2*q2**2-2*q3**2, 2*q1*q2-2*q0*q3, 2*q1*q3+2*q0*q2],
#                   [2*q1*q2+2*q0*q3, 1-2*q1**2-2*q3**2, 2*q2*q3-2*q0*q1],
#                   [2*q1*q3-2*q0*q2, 2*q2*q3+2*q0*q1, 1-2*q1**2-2*q2**2]])
#     return R


