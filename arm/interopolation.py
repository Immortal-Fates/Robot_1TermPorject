# encoding: utf-8

import numpy as np
import math


# 给定关节起始位置 
start_angle = np.array([15, 34, 56, 34, 66, 90])
end_angle = np.array([75, 45, 45, 56, 43, 70])
# 我用的角度，如果需要弧度可以放开下面的两行代码
# start_angle = np.deg2rad(start_angle1)
# end_angle = np.deg2rad(end_angle1)

# 设定起始时间

t_time = np.array([0, 0, 0, 0, 0, 0])

# 给定关节始末角速度与角加速度
start_vel = np.array([0, 0, 0, 0, 0, 0])
end_vel = np.array([0, 0, 0, 0, 0, 0])

start_acc = np.array([0, 0, 0, 0, 0, 0])
end_acc = np.array([0, 0, 0, 0, 0, 0])

# 初始化保存每个关节角度、速度和加速度随时间变化的列表

#对位置进行插值
def solver_interopolation(start_angle,end_angle,start_vel,end_vel,start_acc,end_acc,time=5.0):
    start_time = 0.0
    end_time = start_time+time
    qq = []
    vv = []
    aa = []
    
    for i in range(len(start_angle)):
        # i = 1
        # print(len(start_angle))
        #初始状态
        t=[t_time[0]]
        q=[start_angle[i]]
        v=[start_vel[i]]
        a=[start_acc[i]]

        a0 = start_angle[i]
        a1 = start_vel[i]
        a2 = start_acc[i]
        # print(30*start_angle[i]-30*start_angle[0])
        a3 = (20*end_angle[i]-20*start_angle[i]-(8*end_vel[i]+12*start_vel[i])*end_time-(3*end_acc[i]-start_acc[i])*math.pow(end_time, 2))/(2 * math.pow(end_time, 3))
        a4 = (30*start_angle[i]-30*end_angle[i]+(14*end_vel[i]+16*start_vel[i])*end_time+(3*end_acc[i]-2*start_acc[i])*math.pow(end_time, 2))/(2 * math.pow(end_time, 4))
        a5 = (12*end_angle[i]-12*start_angle[i]-(6*end_vel[i]+6*start_vel[i])*end_time-(end_acc[i]-start_acc[i])*math.pow(end_time, 2))/(2 * math.pow(end_time, 5))
        # print(a0,a1,a2,a3,a4,a5)
        ti=np.arange(start_time, end_time, .01)
        ti=np.append(ti,end_time)
        tt=ti
        # 求解出角度，角速度，角加速度随某个时间区间随时间变换的曲线
        qi = a0 + a1 * ti + a2 * np.power(ti, 2) + a3 * np.power(ti, 3) + a4 * np.power(ti, 4) + a5 * np.power(ti, 5)
        vi = a1 + 2 * a2 * ti  + 3 * a3 * np.power(ti , 2) + 4 * a4 * np.power(ti, 3) + 5 * a5 * np.power(ti, 4)
        ai = 2 * a2 + 6 * a3 * ti + 12 * a4 * np.power(ti, 2) + 20 * a5 * np.power(ti, 3)

        # # 将矩阵转换为List，否则进行数据整合会报错
        ti = ti.tolist()  
        qi = qi.tolist()  
        vi = vi.tolist()  
        ai = ai.tolist()  
        # print(ai)

        #进行数据整合，用来绘制函数图像
        t = t + ti[1:] 
        q = q + qi[1:] 
        v = v + vi[1:] 
        a = a + ai[1:] 
        # print(type(q))

        qq.append(q)
        vv.append(v)
        aa.append(a)
    qq.append(q)
    maxv=max(np.amax(vv),-np.amin(vv))
    maxa=max(np.amax(aa),-np.amin(aa))
    time_v=time_a=time
    inlimit=True
    
    if(maxv>95.0):
        time_v=time*(maxv / 95.0)*1.05
        inlimit=False
    if(maxa>490.0):
        time_a=time*math.sqrt(maxa / 490.0)*1.05
        inlimit=False
    print("max v=",maxv)
    print("max a=",maxa)
    time_real=max(time_v,time_a)
    if(inlimit is False):
        (qq,vv,aa,tt,maxv,maxa,time_real)=solver_interopolation(start_angle,end_angle,start_vel,end_vel,start_acc,end_acc, max(time_v,time_a))
    return (qq,vv,aa,tt,maxv,maxa,time_real)

    
# (qq,vv,aa,tt,maxv,maxa,time_all)=solver_interopolation(start_angle,end_angle,start_vel,end_vel,start_acc,end_acc,0.5)
# a=np.array(qq)
# print(len(a[0,:]))
# print(time_all)
# q0 = np.zeros(6)  # initialize q0 with all zeros
#     # angles of joint 1-6 obtained by solving the inverse kinematics
# q1 = np.array([1.35420811e+01,  7.29236025e+01,  3.34154795e+01, -
#                   1.63390821e+01,  6.82163217e-15,  1.35420811e+01]) / 180 * np.pi
# v0 = np.zeros(6)
# v1 = np.zeros(6)
# a0 = np.zeros(6)
# a1 = np.zeros(6)
# # path1 = PATH(q0, q1, v0, v1, a0, a1, 1.0,0.0)
# # path1.path_plan()
# # q = path1.angle_in_path(0, 0.22)
# (qq,vv,aa,tt,maxv,maxa,time_all)=solver_interopolation(q0, q1, v0, v1, a0, a1,0.5)
# print(time_all)
# print(vv[2])
# print(qq[2])