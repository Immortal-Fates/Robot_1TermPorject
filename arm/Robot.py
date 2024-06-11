# python

#python
import sys
# Write the path to the IK folder here
# For example, the IK folder is in your Documents folder
sys.path.append(f"X:/Robot/arm/IK")
sys.path.append(f"X:/Robot/arm")
import IK
import numpy as np
from math import pi
from solver import solver2
from path_planning import PATH 
from path_planning import find_angle
from path_planning import plan_Type
from path_planning import get_pond_path
import getpass




#sim = client.getObject('sim')
####################################
### You Can Write Your Code Here ###
####################################


def sysCall_init():
    # initialization the simulation
    doSomeInit()    # must have

    # ------------------------------------------------------------------------
    # using the codes, you can obtain the poses and positions of four blocks
    pointHandles = []
    for i in range(2):
        pointHandles.append(sim.getObject(
            '::/Platform1/Cuboid' + str(i+1) + '/SuckPoint'))
    for i in range(2):
        pointHandles.append(sim.getObject(
            '::/Platform1/Prism' + str(i+1) + '/SuckPoint'))
    # get the pose of Cuboid/SuckPoint
    for i in range(4):
        print(sim.getObjectPose(pointHandles[i], -1))
    # -------------------------------------------------------------------------
    
    # -------------------------------------------------------------------------
    # following codes show how to call the build-in inverse kinematics solver
    # you may call the codes, or write your own IK solver
    # before you use the codes, you need to convert the above quaternions to X-Y'-Z' Euler angels
    # you may write your own codes to do the conversion, or you can use other tools (e.g. matlab)
    iks = IK.IKSolver()
    # return the joint angle vector q which belongs to [-PI, PI]
    # Position and orientation of the end-effector are defined by [x, y, z, rx, ry, rz]
    # x,y,z are in meters; rx,ry,rz are X-Y'-Z'Euler angles in radian
    #joint6 end 180 0 -90   -180 0 -90 
    angles = np.array([0.4, 0.12, 0.15, -np.pi, 0, -47.0/180*pi])
    prism2= np.array([0.4, 0.04, 0.125, -142.224/180.0*pi, 26.541/180.0*pi, -150.035/180*pi])
    prism1= np.array([0.4, -0.04, 0.125, 136.395/180.0*pi, 12.441/180.0*pi, -12.746/180*pi])
    cube2= np.array([0.4, -0.12, 0.15, pi, 0, -133.0/180*pi])
    cube2_mid1=np.array([-0.35,0.15,0.30,-pi,0.0,pi/2])
    cube2_end =np.array([-0.35,0.0,0.25,-pi,0.0,pi/2])
    pond_start=np.array([0.1,0.35,0.16,pi,0.0,-pi])
    place_end=np.array([-0.35,0.15,0.25,-pi,0.0,pi/2])
    cube1_end=np.array([-0.35,0.0,0.20,-pi,0.0,pi/2])
    prism2_mid1=np.array([-0.35,0.10,0.25,135.0/180.0*pi,0.0,0.0])
    prism2_end=np.array([-0.35,0.05,0.175,135.0/180.0*pi,0.0,0.0])
    # prism1_mid1=np.array([-0.35,-0.10,0.25,-135.0/180.0*pi,0.0,-pi])
    prism1_mid1=np.array([-0.35,-0.10,0.25,-135.0/180.0*pi,0.0,-pi])
    prism1_end=np.array([-0.35,-0.05,0.175,-135.0/180.0*pi,0.,-pi])
    global MangerStartAngle
    # ---------------------------------------------------------------------------

    """ this demo program shows a 3-postion picking task
    step1: the robot stats to run from the rest position (q0)
    step2: the robot moves to the picking position (q1) in 5s
    step3: turn on the vacumm gripper and picking in 0.5s
    step4: lift a block to position (q2) in 3s
    step5: the robot moves from q2 back to the rest positon q0
    q0 - initial joint angles of the robot
    q1 - joint angles when the robot contacts with a block
    q2 - final joint angels of the robot
    """
    global q0, q1, q2, v0, v1, a0, a1, path1,path2,bond_path,path3,path4,path5
    global path_prism21,path_prism22,path_prism23,path_prism24,path_prism25,path_prism26
    global path_prism11,path_prism12,path_prism13,path_prism14,path_prism15,path_prism16
    global path_cube21,path_cube22,path_cube23,path_cube24,path_cube25,path_cube26
    q0 = np.zeros(6)  # initialize q0 with all zeros
    # angles of joint 1-6 obtained by solving the inverse kinematics
    q1 = np.array([1.35420811e+01,  7.29236025e+01,  3.34154795e+01, -
                  1.63390821e+01,  6.82163217e-15,  1.35420811e+01]) 
    # q2 = np.array([1.35420811e+01,  6.73350208e+01,  2.79938488e+01, -
    #               5.32886951e+00,  6.82163217e-15,  1.35420811e+01])
    v0 = np.zeros(6)
    v1 = np.zeros(6)
    a0 = np.zeros(6)
    a1 = np.zeros(6)
    reset_angle=np.array([90.,45.,45.,-60.,0.,0.])
    reset_anglev=np.array([-80.,0.,0.,0.,0.,0.])
    # --------------------------------------------------------------------------
    Cuboid1_angle=find_angle(angles,q0)
    q2=find_angle(pond_start,Cuboid1_angle)
    path1 = PATH(q0, Cuboid1_angle, v0, v1, a0, a1, 1.0,0.0)
    path1.path_plan()
    
    path2=PATH(Cuboid1_angle, q2, v0, np.array([7.05626481e+01,-3.18234991e+01, 6.52886578e+01,-3.34651587e+01,8.19410463e-15,7.05626481e+01]), a0, a1, 1.0,path1.end_time)
    path2.path_plan()
    print(path1.end_time)
    bond_path=PATH(path2.end_angle,np.zeros(6),np.zeros(6),np.zeros(6),np.zeros(6),np.zeros(6),0.5,path2.end_time)
    bond_path.qq=get_pond_path(path2.end_angle,51).T
    bond_path.start_time=path2.end_time
    bond_path.end_time=path2.end_time+0.50
    bond_path.end_angle=np.array([1.06844651e+02 ,5.50774831e+01 ,6.70499177e+01,-3.21274008e+01,2.03328628e-15, 1.68446511e+01])
    bond_path.tt=np.linspace(bond_path.start_time,bond_path.end_time+0.01,51)

    
    place_angle=find_angle(place_end,bond_path.end_angle)
    print(place_angle)
    path3=PATH(bond_path.end_angle, place_angle, np.array([6.93478368e+01, 4.56190109e+01,-9.36763073e+01, 4.80572964e+01,8.14296881e-15 ,6.93478368e+01])
               ,np.array([60.,0.,0.,0.,0.,0.]), a0, a1, 0.5,bond_path.end_time)
    path3.path_plan()

    
    path4=PATH(path3.end_angle,find_angle(cube1_end,path3.end_angle),path3.end_vel,v0,a0,a0,0.1,path3.end_time)
    path4.path_plan()
    path5=PATH(path4.end_angle,reset_angle,path4.end_vel,reset_anglev,a0,a0,0.1,path4.end_time)
    path5.path_plan()
    
    #prism2
    print(find_angle(prism2,path5.end_angle))
    path_prism21=PATH(path5.end_angle,find_angle(prism2,path5.end_angle),path5.end_vel,v0,a0,a0,0.1,path5.end_time)
    path_prism21.path_plan()
    path_prism22=PATH(path_prism21.end_angle, q2, v0, np.array([7.05626481e+01,-3.18234991e+01, 6.52886578e+01,-3.34651587e+01,8.19410463e-15,7.05626481e+01]), a0, a1, 1.0,path_prism21.end_time)
    path_prism22.path_plan()
    path_prism23=PATH(path_prism22.end_angle,np.zeros(6),np.zeros(6),np.zeros(6),np.zeros(6),np.zeros(6),0.5,path_prism22.end_time)
    path_prism23.qq=get_pond_path(path_prism22.end_angle,51).T
    path_prism23.start_time=path_prism22.end_time
    path_prism23.end_time=path_prism22.end_time+0.50
    path_prism23.end_angle=np.array([1.06844651e+02 ,5.50774831e+01 ,6.70499177e+01,-3.21274008e+01,2.03328628e-15, 1.68446511e+01])
    path_prism23.tt=np.linspace(path_prism23.start_time,path_prism23.end_time+0.01,51)

    prism2_midangle=find_angle(prism2_mid1,path_prism23.end_angle)
    
    prism2_midangle[1]=10.
    prism2_midangle[4]=30.
    path_prism24=PATH(path_prism23.end_angle, prism2_midangle, np.array([6.93478368e+01, 4.56190109e+01,-9.36763073e+01, 4.80572964e+01,8.14296881e-15 ,6.93478368e+01])
               ,np.array([10.,0.,0.,0.,0.,0.]), a0, a1, 0.5,path_prism23.end_time)
    path_prism24.path_plan()

    
    path_prism25=PATH(path_prism24.end_angle,find_angle(prism2_end,path_prism24.end_angle),path_prism24.end_vel,v0,a0,a0,0.1,path_prism24.end_time)
    path_prism25.path_plan()
    path_prism26=PATH(path_prism25.end_angle,reset_angle,path_prism25.end_vel,reset_anglev,a0,a0,0.1,path_prism25.end_time)
    path_prism26.path_plan()
    
    path_prism11=PATH(path_prism26.end_angle,find_angle(prism1,path_prism26.end_angle),path_prism26.end_vel,v0,a0,a0,0.1,path_prism26.end_time)
    path_prism11.path_plan()
    path_prism12=PATH(path_prism11.end_angle, q2, v0, np.array([7.05626481e+01,-3.18234991e+01, 6.52886578e+01,-3.34651587e+01,8.19410463e-15,7.05626481e+01]), a0, a1, 1.0,path_prism11.end_time)
    path_prism12.path_plan()
    path_prism13=PATH(path_prism12.end_angle,np.zeros(6),np.zeros(6),np.zeros(6),np.zeros(6),np.zeros(6),0.5,path_prism12.end_time)
    path_prism13.qq=get_pond_path(path_prism12.end_angle,51).T
    path_prism13.start_time=path_prism12.end_time
    path_prism13.end_time=path_prism12.end_time+0.50
    path_prism13.end_angle=np.array([1.06844651e+02 ,5.50774831e+01 ,6.70499177e+01,-3.21274008e+01,2.03328628e-15, 1.68446511e+01])
    path_prism13.tt=np.linspace(path_prism13.start_time,path_prism13.end_time+0.01,51)

    prism1_midangle=find_angle(prism1_mid1,path_prism13.end_angle)
    # prism1_midangle[0]+=360.

    path_prism14=PATH(path_prism13.end_angle, prism1_midangle, np.array([6.93478368e+01, 4.56190109e+01,-9.36763073e+01, 4.80572964e+01,8.14296881e-15 ,6.93478368e+01])
               ,np.array([30.,0.,-20.,-20.,0.,0.]), a0, a1, 0.5,path_prism13.end_time)
    path_prism14.path_plan()

    prism1_endangle=find_angle(prism1_end,path_prism14.end_angle)
    # prism1_endangle[0]+=360.
    path_prism15=PATH(path_prism14.end_angle,prism1_endangle,path_prism14.end_vel,v0,a0,a0,0.1,path_prism14.end_time)
    path_prism15.path_plan()
    # reset_angle[0]=-90.
    # reset_anglev[0]=80.
    # path_prism16=PATH(path_prism15.end_angle,reset_angle,path_prism15.end_vel,reset_anglev,a0,a0,0.1,path_prism15.end_time)
    # path_prism16.path_plan()

    path_cube21=PATH(path_prism15.end_angle,find_angle(cube2,path_prism15.end_angle),path_prism15.end_vel,v0,a0,a0,0.1,path_prism15.end_time)
    path_cube21.path_plan()
    path_cube22=PATH(path_cube21.end_angle, q2, v0, np.array([7.05626481e+01,-3.18234991e+01, 6.52886578e+01,-3.34651587e+01,8.19410463e-15,7.05626481e+01]), a0, a1, 1.0,path_cube21.end_time)
    path_cube22.path_plan()
    path_cube23=PATH(path_cube22.end_angle,np.zeros(6),np.zeros(6),np.zeros(6),np.zeros(6),np.zeros(6),0.5,path_cube22.end_time)
    path_cube23.qq=get_pond_path(path_cube22.end_angle,51).T
    path_cube23.start_time=path_cube22.end_time
    path_cube23.end_time=path_cube22.end_time+0.50
    path_cube23.end_angle=np.array([1.06844651e+02 ,5.50774831e+01 ,6.70499177e+01,-3.21274008e+01,2.03328628e-15, 1.68446511e+01])
    path_cube23.tt=np.linspace(path_cube23.start_time,path_cube23.end_time+0.01,51)

    cube2_midangle=find_angle(cube2_mid1,path_cube23.end_angle)
    path_cube24=PATH(path_cube23.end_angle, cube2_midangle, np.array([6.93478368e+01, 4.56190109e+01,-9.36763073e+01, 4.80572964e+01,8.14296881e-15 ,6.93478368e+01])
               ,np.array([60.,0.,0.,0.,0.,0.]), a0, a1, 0.5,path_cube23.end_time)
    path_cube24.path_plan()

    
    path_cube25=PATH(path_cube24.end_angle,find_angle(cube2_end,path_cube24.end_angle),path_cube24.end_vel,v0,a0,a0,0.1,path_cube24.end_time)
    path_cube25.path_plan()
    path_cube26=PATH(path_cube25.end_angle,q0,v0,v0,a0,a0,0.1,path_cube25.end_time)
    path_cube26.path_plan()
def sysCall_actuation():
    # put your actuation code in this function

    # get absolute time, t
    t = sim.getSimulationTime()

    # if t>20s, pause the simulation
    # if t > 20:
    #     sim.pauseSimulation()
    # # robot takes 5s to move from q0 to q1.
    # # the vaccum gripper takes effect after wating 0.2s.
    # if t < 5.2:
    #     # call the trajactory planning funcion
    #     # return the joint angles at time t
    #     q = trajPlaningDemo(q0, q1, t, 5)
    #     state = False # vacumm gripper is off

    # # vacumm gripper takes effect from t=5.2s to 5.5s
    # elif t < 5.5:
    #     q = q1      # keeps the robot still at q1
    #     state = True  # vacumm gripper is on

    # # lift a block and move to q2
    # elif t < 8.7:
    #     q = trajPlaningDemo(q1, q2, t-5.5, 3)
    #     state = True

    # # release the vaccum gripper
    # elif t < 9:
    #     q = q2
    #     state = True
    # else:
    #     # robot moves from q2 to q0 within 5s
    #     q = trajPlaningDemo(q2, q0, t-9, 5)
    #     state = True
    q=np.zeros(6)
    state=False
    if (t <= path1.end_time):
        print(t)
        q = path1.angle_in_path(0, t,q)/180.0 *pi
        print(q)
        state=False
        
    elif(t<=path2.end_time):
        print(t)
        q =path2.angle_in_path(path2.start_time,t,q) /180.0 *pi
        print(q)
        state=True
        
    elif (t<=bond_path.end_time):
        print(t)
        q =bond_path.angle_in_path(bond_path.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    elif(t<=path3.end_time):
        print(t)
        q =path3.angle_in_path(path3.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    elif(t<=path4.end_time):
        print(t)
        q =path4.angle_in_path(path4.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    elif(t<=path5.end_time):
        print(t)
        q =path5.angle_in_path(path5.start_time,t,q) /180.0 *pi
        print(q)
        state=False 
        
    elif(t<=path_prism21.end_time):
        print(t)
        q =path_prism21.angle_in_path(path_prism21.start_time,t,q) /180.0 *pi
        print(q)
        state=False
    elif(t<=path_prism22.end_time):
        print(t)
        q =path_prism22.angle_in_path(path_prism22.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    elif(t<=path_prism23.end_time):
        print(t)
        q =path_prism23.angle_in_path(path_prism23.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    elif(t<=path_prism24.end_time):
        print(t)
        q =path_prism24.angle_in_path(path_prism24.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    elif(t<=path_prism25.end_time):
        print(t)
        q =path_prism25.angle_in_path(path_prism25.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    elif(t<=path_prism26.end_time):
        print(t)
        q =path_prism26.angle_in_path(path_prism26.start_time,t,q) /180.0 *pi
        print(q)
        state=False

    elif(t<=path_prism11.end_time):
        print(t)
        q =path_prism11.angle_in_path(path_prism11.start_time,t,q) /180.0 *pi
        print(q)
        state=False
    elif(t<=path_prism12.end_time):
        print(t)
        q =path_prism12.angle_in_path(path_prism12.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    elif(t<=path_prism13.end_time):
        print(t)
        q =path_prism13.angle_in_path(path_prism13.start_time,t,q) /180.0 *pi
        print(q)
        print("path3")
        state=True
    elif(t<=path_prism14.end_time):
        print(t)
        print("path4")
        q =path_prism14.angle_in_path(path_prism14.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    elif(t<=path_prism15.end_time):
        print("place")
        print(t)
        q =path_prism15.angle_in_path(path_prism15.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    # elif(t<=path_prism16.end_time):
    #     print("return")
    #     print(t)
    #     q =path_prism16.angle_in_path(path_prism16.start_time,t,q) /180.0 *pi
    #     print(q)
    #     state=False

    elif(t<=path_cube21.end_time):
        print(t)
        q =path_cube21.angle_in_path(path_cube21.start_time,t,q) /180.0 *pi
        print(q)
        state=False
    elif(t<=path_cube22.end_time):
        print(t)
        q =path_cube22.angle_in_path(path_cube22.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    elif(t<=path_cube23.end_time):
        print(t)
        q =path_cube23.angle_in_path(path_cube23.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    elif(t<=path_cube24.end_time):
        print(t)
        q =path_cube24.angle_in_path(path_cube24.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    elif(t<=path_cube25.end_time):
        print("place")
        print(t)
        q =path_cube25.angle_in_path(path_cube25.start_time,t,q) /180.0 *pi
        print(q)
        state=True
    elif(t<=path_cube26.end_time):
        print("return")
        print(t)
        q =path_cube26.angle_in_path(path_cube26.start_time,t,q) /180.0 *pi
        print(q)
        state=False
    # check if the joint velocities beyond limitations.
    # if they do, the simulation will stops and report errors.
    runState = move(q, state)

    if not runState:
        sim.pauseSimulation()

    """
    The following codes shows a procedure of trajectory planning using the 5th-order polynomial
    You may write your own code to replace this function, e.g. trapezoidal velocity planning
    """


def trajPlaningDemo(start, end, t, time):
    """ Quintic Polynomial: x = k5*t^5 + k4*t^4 + k3*t^3 + k2*t^2 + k1*t + k0
    :param start: Start point
    :param end: End point
    :param t: Current time
    :param time: Expected time spent
    :return: The value of the current time in this trajectory planning
    """
    if t < time:
        tMatrix = np.matrix([
            [0,           0,             0,          0,        0,   1],
            [time**5,     time**4,       time**3,    time**2,     time,   1],
            [0,           0,             0,          0,        1,   0],
            [5*time**4,   4*time**3,     3*time**2,     2*time,        1,   0],
            [0,           0,             0,          2,        0,   0],
            [20*time**3,  12*time**2,        6*time,          2,        0,   0]])

        xArray = []
        for i in range(len(start)):
            xArray.append([start[i], end[i], 0, 0, 0, 0])
        xMatrix = np.matrix(xArray).T

        kMatrix = tMatrix.I * xMatrix

        timeVector = np.matrix([t**5, t**4, t**3, t**2, t, 1]).T
        x = (kMatrix.T * timeVector).T.A[0]

    else:
        x = end

    return x


####################################################
### You Don't Have to Change the following Codes ###
####################################################

def doSomeInit():
    global Joint_limits, Vel_limits, Acc_limits
    Joint_limits = np.array([[-200, -90, -120, -150, -150, -180],
                            [200, 90, 120, 150, 150, 180]]).transpose()/180*np.pi
    Vel_limits = np.array([100, 100, 100, 100, 100, 100])/180*np.pi
    Acc_limits = np.array([500, 500, 500, 500, 500, 500])/180*np.pi

    global lastPos, lastVel, sensorVel
    lastPos = np.zeros(6)
    lastVel = np.zeros(6)
    sensorVel = np.zeros(6)

    global robotHandle, suctionHandle, jointHandles
    robotHandle = sim.getObject('.')
    suctionHandle = sim.getObject('./SuctionCup')
    jointHandles = []
    for i in range(6):
        jointHandles.append(sim.getObject('./Joint' + str(i+1)))
    sim.writeCustomDataBlock(suctionHandle, 'activity', 'off')
    sim.writeCustomDataBlock(robotHandle, 'error', '0')

    global dataPos, dataVel, dataAcc, graphPos, graphVel, graphAcc
    dataPos = []
    dataVel = []
    dataAcc = []
    graphPos = sim.getObject('./DataPos')
    graphVel = sim.getObject('./DataVel')
    graphAcc = sim.getObject('./DataAcc')
    color = [[1, 0, 0], [0, 1, 0], [0, 0, 1], [1, 1, 0], [1, 0, 1], [0, 1, 1]]
    for i in range(6):
        dataPos.append(sim.addGraphStream(
            graphPos, 'Joint'+str(i+1), 'deg', 0, color[i]))
        dataVel.append(sim.addGraphStream(
            graphVel, 'Joint'+str(i+1), 'deg/s', 0, color[i]))
        dataAcc.append(sim.addGraphStream(
            graphAcc, 'Joint'+str(i+1), 'deg/s2', 0, color[i]))


def sysCall_sensing():
    # put your sensing code here
    if sim.readCustomDataBlock(robotHandle, 'error') == '1':
        return
    global sensorVel
    for i in range(6):
        pos = sim.getJointPosition(jointHandles[i])
        if i == 0:
            if pos < -160/180*np.pi:
                pos += 2*np.pi
        vel = sim.getJointVelocity(jointHandles[i])
        acc = (vel - sensorVel[i])/sim.getSimulationTimeStep()
        if pos < Joint_limits[i, 0] or pos > Joint_limits[i, 1]:
            print("Error: Joint" + str(i+1) + " Position Out of Range!")
            sim.writeCustomDataBlock(robotHandle, 'error', '1')
            return

        if abs(vel) > Vel_limits[i]:
            print("Error: Joint" + str(i+1) + " Velocity Out of Range!")
            sim.writeCustomDataBlock(robotHandle, 'error', '1')
            return

        if abs(acc) > Acc_limits[i]:
            print("Error: Joint" + str(i+1) + " Acceleration Out of Range!")
            sim.writeCustomDataBlock(robotHandle, 'error', '1')
            return

        sim.setGraphStreamValue(graphPos, dataPos[i], pos*180/np.pi)
        sim.setGraphStreamValue(graphVel, dataVel[i], vel*180/np.pi)
        sim.setGraphStreamValue(graphAcc, dataAcc[i], acc*180/np.pi)
        sensorVel[i] = vel


def sysCall_cleanup():
    # do some clean-up here
    sim.writeCustomDataBlock(suctionHandle, 'activity', 'off')
    sim.writeCustomDataBlock(robotHandle, 'error', '0')


def move(q, state):
    if sim.readCustomDataBlock(robotHandle, 'error') == '1':
        return
    global lastPos, lastVel
    for i in range(6):
        if q[i] < Joint_limits[i, 0] or q[i] > Joint_limits[i, 1]:
            print("move(): Joint" + str(i+1) + " Position Out of Range!")
            return False
        if abs(q[i] - lastPos[i])/sim.getSimulationTimeStep() > Vel_limits[i]:
            print("move(): Joint" + str(i+1) + " Velocity Out of Range!")
            return False
        if abs(lastVel[i] - (q[i] - lastPos[i]))/sim.getSimulationTimeStep() > Acc_limits[i]:
            print("move(): Joint" + str(i+1) + " Acceleration Out of Range!")
            return False

    lastPos = q
    lastVel = q - lastPos

    for i in range(6):
        sim.setJointTargetPosition(jointHandles[i], q[i])

    if state:
        sim.writeCustomDataBlock(suctionHandle, 'activity', 'on')
    else:
        sim.writeCustomDataBlock(suctionHandle, 'activity', 'off')

    return True
