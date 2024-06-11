```python
#python
from IK.IKSolver import IKSolver
import numpy as np

from scipy.spatial.transform import Rotation
from math import sqrt, pi, acos, cos, sin
from numpy import arctan2 as atan2

global flag,t0
flag = 0
t0 = 0

def MyIKSolver(point):
    x = point[0]
    y = point[1]
    z = point[2]
    a = point[3]
    b = point[4]
    q = point[5]
    x, y, z = x*1000, y*1000, z*1000
    R = Rotation.from_euler('XYZ', [a, b, q], degrees=False).as_matrix()

    solutions = []

    for tag1 in (-1, 1):
        for tag2 in (-1, 1):
            for tag3 in (-1, 1):
                m1, n1 = 171 * R[0,2]-2 * x, 2 * y-171 * R[1,2]
                theta_1 = atan2(m1, n1) - atan2(46, tag1*sqrt(m1**2+n1**2-46**2))
                if not (-200/180*pi <= theta_1+pi/2 <= 200/180*pi):
                    temp_theta = -200/180*pi if theta_1+pi/2 < -200/180*pi else 200/180*pi
                    if(theta_1+pi/2 < -200/180*pi):
                        theta_1 = theta_1 + 2*pi*((temp_theta-(theta_1+pi/2))//(2*pi)+1)
                    else:
                        theta_1 = theta_1 - 2*pi*((theta_1+pi/2-temp_theta)//(2*pi)+1)
                    
                theta_5 = tag2*acos(R[0,2]*cos(theta_1)+R[1,2]*sin(theta_1))
                if not (-150/180*pi <= theta_5-pi/2 <= 150/180*pi):
                    temp_theta = -150/180*pi if theta_5-pi/2 < -150/180*pi else 150/180*pi
                    if(theta_5-pi/2 < -150/180*pi):
                        theta_5 = theta_5 + 2*pi*((temp_theta-(theta_5-pi/2))//(2*pi)+1)
                    else:
                        theta_5 = theta_5 - 2*pi*((theta_5-pi/2-temp_theta)//(2*pi)+1)
                    if not (-150/180*pi <= theta_5-pi/2 <= 150/180*pi):
                        continue

                m6, n6 = -R[0,0]*cos(theta_1)-R[1,0]*sin(theta_1), -R[0,1]*cos(theta_1)-R[1,1]*sin(theta_1)
                theta_6 = atan2(m6, n6)
                if sin(theta_5) > 0:
                    theta_6 -= pi/2
                else:
                    theta_6 += pi/2
                if not (-180/180*pi <= theta_6 <= 180/180*pi):
                    temp_theta = -180/180*pi if theta_6 < -180/180*pi else 180/180*pi
                    if(theta_6 < -180/180*pi):
                        theta_6 = theta_6 + 2*pi*((temp_theta-theta_6)//(2*pi)+1)
                    else:
                        theta_6 = theta_6 - 2*pi*((theta_6-temp_theta)//(2*pi)+1)

                m3 = m3 = y*cos(theta_1) - 77*sin(theta_6)*(R[1,0]*cos(theta_1)-R[0,0]*sin(theta_1)) - 171*R[1,2]*cos(theta_1)/2 - 77*cos(theta_6)*(R[1,1]*cos(theta_1)-R[0,1]*sin(theta_1)) + 171*R[0,2]*sin(theta_1)/2 - x*sin(theta_1)
                n3 = 171*R[2,2]/2 - z + 77*R[2,1]*cos(theta_6) + 77*R[2,0]*sin(theta_6) + 230
                if not (-1 <= (m3**2+n3**2-170**2-185**2)/(2*170*185) <= 1):
                    continue
                theta_3 = tag3 * acos((m3**2+n3**2-170**2-185**2)/(2*170*185))
                if not (-120/180*pi <= theta_3 <= 120/180*pi):
                    temp_theta = -120/180*pi if theta_3 < -120/180*pi else 120/180*pi
                    if(theta_3 < -120/180*pi):
                        theta_3 = theta_3 + 2*pi*((temp_theta-theta_3)//(2*pi)+1)
                    else:
                        theta_3 = theta_3 - 2*pi*((theta_3-temp_theta)//(2*pi)+1)
                    if not (-120/180*pi <= theta_3 <= 120/180*pi):
                        continue

                s2 = (n3*(170*cos(theta_3)+185)-170*sin(theta_3)*m3) / (185**2+170**2+2*170*185*cos(theta_3))
                c2 = (m3+170*sin(theta_3)*s2) / (170*cos(theta_3)+185)
                theta_2 = atan2(s2, c2)
                if not (-90/180*pi <= theta_2+pi/2 <= 90/180*pi):
                    temp_theta = -90/180*pi if theta_2+pi/2 < -90/180*pi else 90/180*pi
                    if(theta_2+pi/2 < -90/180*pi):
                        theta_2 = theta_2 + 2*pi*((temp_theta-(theta_2+pi/2))//(2*pi)+1)
                    else:
                        theta_2 = theta_2 - 2*pi*((theta_2+pi/2-temp_theta)//(2*pi)+1)
                    if not (-90/180*pi <= theta_2+pi/2 <= 90/180*pi):
                        continue

                theta_4 = atan2(cos(theta_6)*(R[1,1]*cos(theta_1)-R[0,1]*sin(theta_1))+sin(theta_6)*(R[1, 0]*cos(theta_1)-R[0,0]*sin(theta_1)), R[2,1]*cos(theta_6)+R[2,0]*sin(theta_6)) - theta_2 - theta_3
                if not (-150/180*pi <= theta_4-pi/2 <= 150/180*pi):
                    temp_theta = -150/180*pi if theta_4-pi/2 < -150/180*pi else 150/180*pi
                    if(theta_4-pi/2 < -150/180*pi):
                        theta_4 = theta_4 + 2*pi*((temp_theta-(theta_4-pi/2))//(2*pi)+1)
                    else:
                        theta_4 = theta_4 - 2*pi*((theta_4-pi/2-temp_theta)//(2*pi)+1)
                    if not (-150/180*pi <= theta_4-pi/2 <= 150/180*pi):
                        continue

                solutions.append([theta_1+pi/2, theta_2+pi/2, theta_3, theta_4-pi/2, theta_5-pi/2, theta_6])

    return solutions

def quaternion2euler(quaternion):
    r = Rotation.from_quat(quaternion)
    euler = r.as_euler('xyz', degrees=False)
    print(euler/pi*180)
    return euler

# culculate xyz-euler
def transCul(point):
    temp = []
    temp.append(point[3])
    temp.append(point[4])
    temp.append(point[5])
    temp.append(point[6])
    euler = quaternion2euler(temp)
    temp = []
    temp.append(point[0])
    temp.append(point[1])
    temp.append(point[2])
    temp.append(euler[0])
    temp.append(euler[1])
    temp.append(euler[2])
    return temp

####################################
### You Can Write Your Code Here ###
####################################

def move_nol(q1, T, state):
    global flag,t0,q_now
    if(flag==0):
        t0 = sim.getSimulationTime()
        flag = 1
    t = sim.getSimulationTime()
    q = myTrajPlaning(q_now, q1, t-t0, T)
    print("q_now")
    print(q_now)
    print(q)
    
    runState = move(q, state)

    if not runState:
        sim.pauseSimulation()
    if(t-t0<T+0.2):
        return 1
    else:
        flag = 0
        q_now = q
        return 0

def move1():
    global p1, queue1, flag1
    if(flag1):
        print("p1")
        print(p1)
        print(len(queue1))
        if(p1<len(queue1)):
            flag1 = move_nol(queue1[p1][0], queue1[p1][1], queue1[p1][2])
        else:
            sim.pauseSimulation()
    else:
        p1 = p1+1
        flag1 = 1

def sysCall_init():
    # initialization the simulation
    doSomeInit()    # must have    
    
    #------------------------------------------------------------------------
    # using the codes, you can obtain the poses and positions of four blocks
    pointHandles = []
    for i in range(2):
        pointHandles.append(sim.getObject('::/Platform1/Cuboid' + str(i+1) + '/SuckPoint'))
    for i in range(2):
        pointHandles.append(sim.getObject('::/Platform1/Prism' + str(i+1) + '/SuckPoint'))
    # get the pose of Cuboid/SuckPoint
    pointHandles_ans = []
    # for i in range(4):
    #     pointHandles_ans.append(transCul(sim.getObjectPose(pointHandles[i], -1)))
    pointHandles_ans.append([0.400, -0.120, 0.150, -180/180*pi, 0/180*pi, -46/180*pi])
    pointHandles_ans.append([0.400, 0.120, 0.150, -180/180*pi, 0/180*pi, -81/180*pi])
    pointHandles_ans.append([0.400, -0.04, 0.125, 135.065/180*pi, 2.716/180*pi, -2.721/180*pi])
    pointHandles_ans.append([0.400, 0.04, 0.125, -139.749/180*pi, 22.107/180*pi, -156.035/180*pi])
    
    #-------------------------------------------------------------------------
        
        
    #-------------------------------------------------------------------------
    # following codes show how to call the build-in inverse kinematics solver
    # you may call the codes, or write your own IK solver
    # before you use the codes, you need to convert the above quaternions to X-Y'-Z' Euler angels 
    # you may write your own codes to do the conversion, or you can use other tools (e.g. matlab)
    ##iks = IKSolver()
    # return the joint angle vector q which belongs to [-PI, PI]
    # Position and orientation of the end-effector are defined by [x, y, z, rx, ry, rz]
    # x,y,z are in meters; rx,ry,rz are X-Y'-Z'Euler angles in radian
    ##angles = iks.solve(np.array([0.320, -0.250, 0.160, 3, 0.265, -0.84]))
    # print(np.array(pointHandles_ans[0]))
    # angles = MyIKSolver(np.array(pointHandles_ans[3]))
    # angles = angles.T
    # print(f'angles:{angles}')
    #---------------------------------------------------------------------------
    
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
    global q0, q1, q2, q3, q4, q_now
    global p1,queue1,p2,queue2,p3,queue3,p4,queue4,flag1
    p1 = 0
    p2 = 0
    p3 = 0
    p4 = 0
    flag1 = 1
    queue1 = []
    queue2 = []
    queue3 = []
    queue4 = []
    q_now = np.zeros(6)
    # angles of joint 1-6 obtained by solving the inverse kinematics
    q1 = np.array(MyIKSolver(np.array(pointHandles_ans[0]))[-1])
    q2 = np.array(MyIKSolver(np.array(pointHandles_ans[1]))[-1])
    q3 = np.array(MyIKSolver(np.array(pointHandles_ans[2]))[-1])
    q4 = np.array(MyIKSolver(np.array(pointHandles_ans[3]))[-1])
    q5 = np.array(MyIKSolver(np.array([0, 0.35, 0.25, pi, 0, pi]))[-1])
    # q2 = q0
    # q2 = np.array(MyIKSolver(np.array([0.400, 0.04, 0.275, pi, 0, pi]))[-1])
    # move 4
    queue1.append([np.array(MyIKSolver(np.array([0.400, 0.04, 0.175, -139.749/180*pi, 22.107/180*pi, -156.035/180*pi]))[-1]),4,False])
    queue1.append([q4,2,False])
    queue1.append([q4,0.1,True])
    queue1.append([np.array(MyIKSolver(np.array([0.400, 0.04, 0.225, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([0.1, 0.35, 0.15, pi, 0, pi]))[-1]),4,True])
    queue1.append([np.array(MyIKSolver(np.array([0.05, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([0, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.05, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.1, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.35,0.05,0.275, 3*pi/4, 0, pi]))[-1]),4,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.35,0.05,0.175, 3*pi/4, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.35,0.05,0.175, 3*pi/4, 0, pi]))[-1]),0.1,False])
    queue1.append([q5,3,False])
    # move 2
    queue1.append([np.array(MyIKSolver(np.array([0.400, 0.120, 0.200, -180/180*pi, 0/180*pi, -81/180*pi]))[-1]),3,False])
    queue1.append([q2,2,False])
    queue1.append([q2,0.1,True])
    queue1.append([np.array(MyIKSolver(np.array([0.400, 0.120, 0.225, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([0.1, 0.35, 0.15, pi, 0, pi]))[-1]),4,True])
    queue1.append([np.array(MyIKSolver(np.array([0.05, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([0, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.05, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.1, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.35,0,0.275, pi, 0, pi]))[-1]),4,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.35,0,0.2, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.35,0,0.2, pi, 0, pi]))[-1]),0.1,False])
    queue1.append([q5,3,False])
    # move 3
    queue1.append([np.array(MyIKSolver(np.array([0.400, -0.04, 0.175, 135.065/180*pi, 2.716/180*pi, -2.721/180*pi]))[-1]),3,False])
    queue1.append([q3,2,False])
    queue1.append([q3,0.1,True])
    queue1.append([np.array(MyIKSolver(np.array([0.400, -0.04, 0.225, pi, 0, pi]))[-1]),4,True])
    queue1.append([np.array(MyIKSolver(np.array([0.1, 0.35, 0.15, pi, 0, pi]))[-1]),4,True])
    queue1.append([np.array(MyIKSolver(np.array([0.05, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([0, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.05, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.1, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.35,-0.05,0.275, -3*pi/4, 0, pi]))[-1]),4,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.35,-0.05,0.175, -3*pi/4, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.35,-0.05,0.175, -3*pi/4, 0, pi]))[-1]),0.1,False])
    queue1.append([np.array(MyIKSolver(np.array([-0.35,-0.05,0.275, -3*pi/4, 0, pi]))[-1]),2,False])
    queue1.append([q5,3,False])
    # move 1
    queue1.append([np.array(MyIKSolver(np.array([0.400, -0.120, 0.200, -180/180*pi, 0/180*pi, -46/180*pi]))[-1]),3,False])
    queue1.append([q1,2,False])
    queue1.append([q1,0.1,True])
    queue1.append([np.array(MyIKSolver(np.array([0.400, -0.120, 0.225, pi, 0, pi]))[-1]),4,True])
    queue1.append([np.array(MyIKSolver(np.array([0.1, 0.35, 0.15, pi, 0, pi]))[-1]),4,True])
    queue1.append([np.array(MyIKSolver(np.array([0.05, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([0, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.05, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.1, 0.35, 0.15, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.35,0,0.275, pi, 0, pi]))[-1]),4,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.35,0,0.25, pi, 0, pi]))[-1]),2,True])
    queue1.append([np.array(MyIKSolver(np.array([-0.35,0,0.25, pi, 0, pi]))[-1]),0.1,False])
    # move to start
    queue1.append([np.zeros(6),4,False])
    
    #--------------------------------------------------------------------------
    
def sysCall_actuation():
    # put your actuation code in this function   
    move1()
    # move3()
    # move4()
    # move2()
    
    
        
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
        [         0,           0,             0,          0,        0,   1],
        [   time**5,     time**4,       time**3,    time**2,     time,   1],
        [         0,           0,             0,          0,        1,   0],
        [ 5*time**4,   4*time**3,     3*time**2,     2*time,        1,   0],
        [         0,           0,             0,          2,        0,   0],
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
    
def ploy5(start, end, t, tf):
    start1 = 0
    start2 = 0
    end1 = 0
    end2 = 0
    a0 = start
    a1 = start1
    a2 = start2/2
    a3 = (20*end-20*start-(8*end1+12*start1)*tf-(3*start2-end2)*tf**2)/(2*tf**3)
    a4 = (30*start-30*end+(14*end1+16*start1)*tf+(3*start2-2*end2)*tf**2)/(2*tf**4)
    a5 = (12*end-12*start-(6*end1+6*start1)*tf-(start2-end2)*tf**2)/(2*tf**5)
    ans = a0+a1*t+a2*t**2+a3*t**3+a4*t**4+a5*t**5
    return ans
    

def myTrajPlaning(start, end, t, time):
    if t < time:
        ans = []
        for i in range(len(start)):
            ans.append(ploy5(start[i], end[i], t, time))
    else:
        ans = end
    # print(ans)
    return np.array(ans)


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
        dataPos.append(sim.addGraphStream(graphPos, 'Joint'+str(i+1), 'deg', 0, color[i]))
        dataVel.append(sim.addGraphStream(graphVel, 'Joint'+str(i+1), 'deg/s', 0, color[i]))
        dataAcc.append(sim.addGraphStream(graphAcc, 'Joint'+str(i+1), 'deg/s2', 0, color[i]))

def sysCall_sensing():
    # put your sensing code here
    if sim.readCustomDataBlock(robotHandle,'error') == '1':
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
        
        sim.setGraphStreamValue(graphPos,dataPos[i], pos*180/np.pi)
        sim.setGraphStreamValue(graphVel,dataVel[i], vel*180/np.pi)
        sim.setGraphStreamValue(graphAcc,dataAcc[i], acc*180/np.pi)
        sensorVel[i] = vel

def sysCall_cleanup():
    # do some clean-up here
    sim.writeCustomDataBlock(suctionHandle, 'activity', 'off')
    sim.writeCustomDataBlock(robotHandle, 'error', '0')


def move(q, state):
    if sim.readCustomDataBlock(robotHandle,'error') == '1':
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
    
```

