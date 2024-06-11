import sys
# Write the path to the IK folder here
# For example, the IK folder is in your Documents folder
from math import pi
from solver import solver2
import math
import numpy as np
import time
import enum
from interopolation import solver_interopolation
class plan_Type(enum.Enum):
    Angle_Space=1
    Cartesian_Space=2


def find_angle(pos=np.array,last_ang=np.array):
    Joint_limits = np.array([[-200, -90, -120, -150, -150, -180],
                            [200, 90, 120, 150, 150, 180]]).transpose()/180*np.pi
    # All_angles=IK.iks(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5])
    # All_angles=iks.solve(pos).T
    All_angles=solver2(pos[0],pos[1],pos[2],pos[3],pos[4],pos[5])
    print("solve",len(All_angles))
    print(pos)
    print(All_angles)
    limit=np.zeros(len(All_angles))
    cost=np.zeros(len(All_angles))
    for i in range(0,len(All_angles)):
        limit[i]=1
    weight=np.array([3.,2.,5.,2.,2.,3.])
    for i in range(0,len(All_angles)):
        for j in range(6):
            # print(All_angles[i][j],Joint_limits[j][1],Joint_limits[j][0],i,j)
            
            if(All_angles[i][j]>Joint_limits[j][1]or All_angles[i][j]<Joint_limits[j][0] or limit[i]==0):
                limit[i]=0
                cost[i]=99999.0
                break
            if(abs(All_angles[i][j]-last_ang[j])>1.0/180*pi is True):
                limit[i]=0
                cost[i]=99999.0
                break
            a=All_angles[i][j]
            b=last_ang[j]
            c=(abs(a-b)*weight[j])
            cost[i]+=c
    print(limit)
    if(np.sum(limit)==0):
        print("no Solution")
        return -1
    
    min_cost=np.argmin(cost)
    print("result")
    print(All_angles[min_cost][:]*180.0/pi)
    return All_angles[min_cost][:]*180.0/pi

class PATH:
    
    def __init__(self,
                 start_angle=np.array,
                 end_angle=np.array,
                 start_vel=np.array,
                 end_vel=np.array,
                 start_acc=np.array,
                 end_acc=np.array,
                 time_in_total=10.0,
                 start_time=0.0,
                 end_time=0.0,
                 type=plan_Type.Angle_Space,
                 maxa=0.0,
                 maxv=0.0
                 ):
        self.type=type
        self.start_angle=start_angle
        self.end_angle=end_angle
        self.start_vel=start_vel
        self.end_vel=end_vel
        self.start_acc=start_acc
        self.end_acc=end_acc
        self.time_in_total=time_in_total
        self.start_time=start_time
        self.end_time=end_time
        self.type=type
        self.maxa=maxa
        self.maxv=maxv
        self.path=np.array
        return
    def path_plan(self):
            (self.qq,self.vv,self.aa,self.tt,self.maxv,self.maxa,self.time_in_total)=solver_interopolation(self.start_angle,
                                                                                                  self.end_angle,
                                                                                                  self.start_vel,
                                                                                                  self.end_vel,
                                                                                                  self.start_acc,
                                                                                                  self.end_acc,
                                                                                                  self.time_in_total)
            self.end_time=self.start_time+self.time_in_total
            return 
    #计算当前时刻对应关节角
    def angle_in_path(self,time_start,time_now,anglenow=np.array):
        print("debug")
        Joint_limits = np.array([[-200, -90, -120, -150, -150, -180],
                            [200, 90, 120, 150, 150, 180]]).transpose()
        print(time_start)
        print(time_now)
        t=int((time_now-time_start)/0.01)
        delta_time=self.tt[t+1]-self.tt[t]
        q=np.array(self.qq)[:,t]*(1-(time_now-time_start-0.01*t)/delta_time)+np.array(self.qq)[:,t+1]*(time_now-time_start-0.01*t)/delta_time
        for i in range(0,6):
            if(q[i]>Joint_limits[i,1]):
                q[i]=(anglenow[i]+Joint_limits[i,1])/2
            if(q[i]<Joint_limits[i,0]):
                q[i]=(anglenow[i]+Joint_limits[i,1])/2
        return q
    def staright_pathplan(self,start_angle=np.array,startpos=np.array,endpos=np.array,time=2.0):
        self.time_in_total=time
        step=int(time/0.01)
        pos_list=np.linspace(startpos,endpos,step)
        path=start_angle
        last_angle=start_angle
        for i in range(1,step):
            # print(i)
            new_angle=find_angle(pos_list[i,:],last_angle)
            # print(new_angle)
            path=np.vstack((path,new_angle))
        self.qq=path
        self.end_time=self.start_time+time
        self.start_vel=self.path[1,:]-self.path[0,:]
        self.end_vel=self.path[step,:]-self.path[step-1,:]
        self.tt=np.linspace(self.start_time,self.end_time+0.01,step)





def get_pond_path(start_angle,step):
    start_pos=np.array([0.1,0.35,0.16,pi,0.0,-pi])
    end_pos=np.array([-0.13,0.35,0.16,pi,0.0,-pi])
    pos_list=np.linspace(start_pos,end_pos,step)
    path=start_angle
    last_angle=start_angle
    for i in range(1,step):
        # print(i)
        new_angle=find_angle(pos_list[i,:],last_angle)
        # print(new_angle)
        path=np.vstack((path,new_angle))
    return path
# test code
# q0 = np.zeros(6)  # initialize q0 with all zeros
#     # angles of joint 1-6 obtained by solving the inverse kinematics
# q1 = np.array([1.35420811e+01,  7.29236025e+01,  3.34154795e+01, -
#                   1.63390821e+01,  6.82163217e-15,  1.35420811e+01]) / 180 * np.pi
# v0 = np.zeros(6)
# v1 = np.zeros(6)
# a0 = np.zeros(6)
# a1 = np.zeros(6)
# path1 = PATH(q0, q1, v0, v1, a0, a1, 1.0,0.0)
# path1.path_plan()
# q = path1.angle_in_path(0, 0.22)
# print(path1.aa[2])
# angles = find_angle(np.array([0.4, 0.12, 0.15, -np.pi, 0, -47.0/180*pi]),np.zeros(6))
# print(angles)
# angles = np.array([0.4, 0.12, 0.15, -np.pi, 0, -47.0/180*pi])
# q0 = np.zeros(6)
# Cuboid1_angle=find_angle(angles,q0)
# print(Cuboid1_angle)
# step=50
# p=get_pond_path(step)
# print(p)
# pv=np.zeros(6)
# # pv=np.zeros(6)
# # pvi=np.zeros(6)
# for i in range(0,step-1):
#     pi=np.array(p[i+1,:]-p[i,:])/0.01
#     pv=np.vstack((pv, pi))
# # print(p)
# # print(pv)
# step=50
# a=find_angle(np.array([0.4, 0.12, 0.175, -np.pi, 0, -47.0/180*pi]),np.zeros(6))
# print("a=",a)
# end_angle=find_angle(np.array([0.1,0.35,0.175,180.0,0.0,-180.0]),a)
# bond_path=PATH(end_angle,np.zeros(6),np.zeros(6),np.zeros(6),np.zeros(6),np.zeros(6),1.7,0.0)
# bond_path.qq=get_pond_path(end_angle,step)
# bond_path.start_time=0
# bond_path.end_time=step*0.01
# bond_path.tt=np.linspace(bond_path.start_time,bond_path.end_time,step)
# pv=np.zeros(6)
# print(bond_path.qq)
# for i in range(0,step-1):
#     pi=np.array(bond_path.qq[i+1,:]-bond_path.qq[i,:])/0.01
#     pv=np.vstack((pv, pi))
# print(bond_path.qq)
# end_angle=np.array([1.06844651e+02 ,5.50774831e+01 ,6.70499177e+01,-3.21274008e+01,2.03328628e-15, 1.68446511e+01])
# place_end=np.array([0.35,0,0.19,-pi,0.0,90])
# place_angle=find_angle(place_end,end_angle)
# path3=PATH(end_angle, place_angle, np.array([6.93478368e+01, 4.56190109e+01,-9.36763073e+01, 4.80572964e+01,8.14296881e-15 ,6.93478368e+01]),np.zeros(6),np.zeros(6),np.zeros(6), 0.5,0.0)
# path3.path_plan()
# print(path3.qq)