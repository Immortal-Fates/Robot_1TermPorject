from math import cos
from math import sin
import math
import numpy as np
from math import pi

def DH(alpha,a,d,theta):
    T=np.array([(cos(theta), -sin(theta) ,0.0 ,a),
    (sin(theta)*cos(alpha), cos(theta)*cos(alpha) ,-sin(alpha), -sin(alpha)*d),
    (sin(theta)*sin(alpha) ,cos(theta)*sin(alpha), cos(alpha) ,cos(alpha)*d),
    (0.0 ,0.0 ,0.0, 1.0)])
    return T





def limit(a):
    if(a>pi):
        return limit(a-2*pi)
    if(a<-pi):
        return limit(a+2*pi)
    return a 


def limit_mat(mat):
    for i in range(len(mat)):
        for j in range(6):
            mat[i,j]=limit(mat[i,j])
    return  mat


def solver1(theta1,theta2,theta3,theta4,theta5,theta6):
    result=np.array([0.0,0.0,0.0,0.0,0.0,0.0])
    theta2+=-pi/2
    theta4+=pi/2
    theta5+=pi/2
    r11=sin(theta4+theta2+theta3)*cos(theta1)*sin(theta6)-cos(theta6)*(sin(theta1)*sin(theta5)-cos(theta4+theta2+theta3)*cos(theta1)*cos(theta5))
    r12=sin(theta6)*(sin(theta1)*sin(theta5) - cos(theta2 + theta3 + theta4)*cos(theta1)*cos(theta5)) + sin(theta2 + theta3 + theta4)*cos(theta1)*cos(theta6)
    r13=cos(theta5)*sin(theta1) + cos(theta2 + theta3 + theta4)*cos(theta1)*sin(theta5)
    r21=cos(theta6)*(cos(theta1)*sin(theta5) + cos(theta2 + theta3 + theta4)*cos(theta5)*sin(theta1)) + sin(theta2 + theta3 + theta4)*sin(theta1)*sin(theta6)
    r22=sin(theta2 + theta3 + theta4)*cos(theta6)*sin(theta1) - sin(theta6)*(cos(theta1)*sin(theta5) + cos(theta2 + theta3 + theta4)*cos(theta5)*sin(theta1))
    r23=cos(theta2 + theta3 + theta4)*sin(theta1)*sin(theta5) - cos(theta1)*cos(theta5)
    r31=cos(theta2 + theta3 + theta4)*sin(theta6) - sin(theta2 + theta3 + theta4)*cos(theta5)*cos(theta6)
    r32=cos(theta2 + theta3 + theta4)*cos(theta6) + sin(theta2 + theta3 + theta4)*cos(theta5)*sin(theta6)
    r33=-sin(theta2 + theta3 + theta4)*sin(theta5)
    px=(185*cos(theta1)*cos(theta2) - 23*sin(theta1) + (171*cos(theta5)*sin(theta1))/2 - 170*cos(theta1)*sin(theta2)*sin(theta3) 
        + (171*cos(theta2 + theta3 + theta4)*cos(theta1)*sin(theta5))/2 + 77*cos(theta2 + theta3)*cos(theta1)*sin(theta4) 
        + 77*sin(theta2 + theta3)*cos(theta1)*cos(theta4) + 170*cos(theta1)*cos(theta2)*cos(theta3))/1000
    py=(23*cos(theta1)-(171*cos(theta1)*cos(theta5))/2 + 185*cos(theta2)*sin(theta1)
    -170*sin(theta1)*sin(theta2)*sin(theta3) + (171*cos(theta2 + theta3 + theta4)*sin(theta1)*sin(theta5))/2
    +77*cos(theta2 + theta3)*sin(theta1)*sin(theta4) + 77*sin(theta2 + theta3)*cos(theta4)*sin(theta1) 
    +170*cos(theta2)*cos(theta3)*sin(theta1))/1000
    pz=(77*cos(theta2+theta3)*cos(theta4)-185*sin(theta2)-77*sin(theta2+theta3)*sin(theta4)-sin(theta5)*((171*cos(theta2+theta3)*sin(theta4))/2
    +(171*sin(theta2+theta3)*cos(theta4))/2)-170*sin(theta2+theta3)+230)/1000
    # result[1]=math.atan2(-r31,math.sqrt(r11*r11+r21*r21))*180.00000/pi
    # result[0]=math.atan2(r21/cos(result[1]),r11/cos(result[1]))*180.0/pi
    # result[2]=math.atan2(r32/cos(result[1]),r33/cos(result[1]))*180.0/pi
    result[1]=math.asin(r13)*180.0/pi
    result[0]=math.atan2(-r23,r33)*180.0/pi
    result[2]=math.atan2(-r12,r11)*180.0/pi
    result[3]=px
    result[4]=py
    result[5]=pz
    M=np.array([(r11,r12,r13,px),(r21,r22,r23,py),(r31,r32,r33,pz),(0.0,0.0,0.0,1.0)])    
    # print(M)
    return result
def parallel(mat):
    cosfi=mat[0,0]
    sinfi=-mat[0,1]
    x=mat[0,3]
    y=-mat[2,3]
    l1=185.0
    l2=170.0
    c2=(x*x+y*y-l1*l1-l2*l2)/(2*l1*l2)
    if(abs(c2)>1):
        return np.array([(100.0 ,0, 0),(100, 0, 0)])
    s2_1=math.sqrt(1-c2*c2)
    s2_2=-math.sqrt(1-c2*c2)
    theta2_1=math.atan2(s2_1,c2)
    theta2_2=math.atan2(s2_2,c2)
    k1=l1+l2*c2
    k2_1=l2*s2_1
    k2_2=l2*s2_2
    theat1_1=math.atan2(y,x)-math.atan2(k2_1,k1)
    theat1_2=math.atan2(y,x)-math.atan2(k2_2,k1)
    theta3_1=math.atan2(sinfi,cosfi)-theat1_1-theta2_1
    theta3_2=math.atan2(sinfi,cosfi)-theat1_2-theta2_2
    return np.array([(theat1_1,theta2_1,theta3_1),(theat1_2,theta2_2,theta3_2)])

def solver2(x,y,z,alpha,beta,gamma):
    r11=cos(beta)*cos(gamma)
    r12=-cos(beta)*sin(gamma)
    r13=sin(beta)
    r21=cos(alpha)*sin(gamma)+cos(gamma)*sin(alpha)*sin(beta)
    r22=cos(alpha)*cos(gamma)-sin(alpha)*sin(beta)*sin(gamma)
    r23=-cos(beta)*sin(alpha)
    r31=sin(alpha)*sin(gamma)-cos(alpha)*cos(gamma)*sin(beta)
    r32=cos(gamma)*sin(alpha)+cos(alpha)*sin(beta)*sin(gamma)
    r33=cos(alpha)*cos(beta)
    px=x*1000.0
    py=y*1000.0
    pz=z*1000.0
# theta 1 5 5输出要-pi/2
    m=py-r23*171.0/2.0
    n=px-r13*171.0/2.0
    theta11=math.atan2(m,n)-math.atan2(23.0,-math.sqrt(n*n+m*m-23.0*23.0))
    theta12=math.atan2(m,n)-math.atan2(23.0,math.sqrt(n*n+m*m-23.0*23.0))
    theta51=math.acos(sin(theta11)*r13-cos(theta11)*r23)
    theta51_=-theta51
    theta52=math.acos(sin(theta12)*r13-cos(theta12)*r23)
    theta52_=-theta52
# theta 6
    r31_=r21*cos(theta11)-r11*sin(theta11)
    r32_=r22*cos(theta11)-r12*sin(theta11)
    theta61=math.atan2(-r32_,r31_)
    r31_=r21*cos(theta12)-r11*sin(theta12)
    r32_=r22*cos(theta12)-r12*sin(theta12)
    theta62=math.atan2(-r32_,r31_)
    if(abs((r21*cos(theta11)-r11*sin(theta11))-(cos(theta61)*sin(theta51)))>0.00001):
        theta61+=pi
    if(abs((r21*cos(theta12)-r11*sin(theta12))-(cos(theta62)*sin(theta52)))>0.00001):
        theta62+=pi
    
#theta 2 3 4
    add1_flag=False
    add2_flag=False
    M=np.array([(r11,r12,r13,px),(r21,r22,r23,py),(r31,r32,r33,pz),(0.0,0.0,0.0,1.0)])   
    T01=np.linalg.inv(DH(0.0,0.0,230.0,theta11))
    T45=np.linalg.inv(DH(pi/2,0.0,77.0,theta51))
    T56=np.linalg.inv(DH(pi/2,0.0,85.5,theta61)) 
    MM=np.dot(np.dot(T01,M),np.dot(T56,T45))
    solutionfor1=parallel(MM)
    T45=np.linalg.inv(DH(pi/2,0.0,77.0,theta51_))
    theta61_=limit(theta61+pi)
    T56=np.linalg.inv(DH(pi/2,0.0,85.5,theta61_)) 
    MM=np.dot(np.dot(T01,M),np.dot(T56,T45))
    _add1=parallel(MM)
    if(solutionfor1[0,0]==100):
        print("table1")
        
        theta51=theta51_
        theta61=theta61_
        solutionfor1=_add1
        
    elif (_add1[0,0]!=100):
        add1_flag=True
        add1=np.array([(_add1[0,0],_add1[0,1],_add1[0,2]),
                    (_add1[1,0],_add1[1,1],_add1[1,2])])
        print("add",_add1)  
    T01=np.linalg.inv(DH(0.0,0.0,230.0,theta12))
    T45=np.linalg.inv(DH(pi/2,0.0,77.0,theta52))
    T56=np.linalg.inv(DH(pi/2,0.0,85.5,theta62)) 
    MM=np.dot(np.dot(T01,M),np.dot(T56,T45))
    solutionfor2=parallel(MM)
    T45=np.linalg.inv(DH(pi/2,0.0,77.0,theta52_))
    theta62_=limit(theta62+pi)
    T56=np.linalg.inv(DH(pi/2,0.0,85.5,theta62_)) 
    MM=np.dot(np.dot(T01,M),np.dot(T56,T45))
    _add2=parallel(MM)   
    if(solutionfor2[0,0]==100):
        print("table2")
        print(theta52)
        print(theta52_)
        print(theta62)
        print(theta62_)
        theta52=theta52_
        theta62=theta62_
        solutionfor2=_add2
    elif (_add2[0,0]!=100):
        add2_flag=True
        add2=np.array([(_add2[0,0],_add2[0,1],_add2[0,2]),
                    (_add2[1,0],_add2[1,1],_add2[1,2])])  
        print("add2",add2)  
    print(add1_flag ,add2_flag )
    if(add1_flag is False and add2_flag is False):
        result=np.array([(theta11,solutionfor1[0,0]+pi/2,solutionfor1[0,1],solutionfor1[0,2]-pi/2,theta51-pi/2,theta61),
                    (theta11,solutionfor1[1,0]+pi/2,solutionfor1[1,1],solutionfor1[1,2]-pi/2,theta51-pi/2,theta61),
                    (theta12,solutionfor2[0,0]+pi/2,solutionfor2[0,1],solutionfor2[0,2]-pi/2,theta52-pi/2,theta62),
                    (theta12,solutionfor2[1,0]+pi/2,solutionfor2[1,1],solutionfor2[1,2]-pi/2,theta52-pi/2,theta62)])
    if(add1_flag is True and add2_flag is False):
        result=np.array([(theta11,solutionfor1[0,0]+pi/2,solutionfor1[0,1],solutionfor1[0,2]-pi/2,theta51-pi/2,theta61),
                    (theta11,solutionfor1[1,0]+pi/2,solutionfor1[1,1],solutionfor1[1,2]-pi/2,theta51-pi/2,theta61),
                    (theta12,solutionfor2[0,0]+pi/2,solutionfor2[0,1],solutionfor2[0,2]-pi/2,theta52-pi/2,theta62),
                    (theta12,solutionfor2[1,0]+pi/2,solutionfor2[1,1],solutionfor2[1,2]-pi/2,theta52-pi/2,theta62),
                    (theta11,add1[0,0]+pi/2,add1[0,1],add1[0,2]-pi/2,theta51_-pi/2,theta61_),
                    (theta11,add1[1,0]+pi/2,add1[1,1],add1[1,2]-pi/2,theta51_-pi/2,theta61_)])
    if(add1_flag is False and add2_flag is True):
        result=np.array([(theta11,solutionfor1[0,0]+pi/2,solutionfor1[0,1],solutionfor1[0,2]-pi/2,theta51-pi/2,theta61),
                    (theta11,solutionfor1[1,0]+pi/2,solutionfor1[1,1],solutionfor1[1,2]-pi/2,theta51-pi/2,theta61),
                    (theta12,solutionfor2[0,0]+pi/2,solutionfor2[0,1],solutionfor2[0,2]-pi/2,theta52-pi/2,theta62),
                    (theta12,solutionfor2[1,0]+pi/2,solutionfor2[1,1],solutionfor2[1,2]-pi/2,theta52-pi/2,theta62),
                    (theta12,add2[0,0]+pi/2,add2[0,1],add2[0,2]-pi/2,theta52_-pi/2,theta62_),
                    (theta12,add2[1,0]+pi/2,add2[1,1],add2[1,2]-pi/2,theta52_-pi/2,theta62_)])
    if(add1_flag is True and add2_flag is True):
        result=np.array([(theta11,solutionfor1[0,0]+pi/2,solutionfor1[0,1],solutionfor1[0,2]-pi/2,theta51-pi/2,theta61),
                    (theta11,solutionfor1[1,0]+pi/2,solutionfor1[1,1],solutionfor1[1,2]-pi/2,theta51-pi/2,theta61),
                    (theta12,solutionfor2[0,0]+pi/2,solutionfor2[0,1],solutionfor2[0,2]-pi/2,theta52-pi/2,theta62),
                    (theta12,solutionfor2[1,0]+pi/2,solutionfor2[1,1],solutionfor2[1,2]-pi/2,theta52-pi/2,theta62),
                    (theta11,add1[0,0]+pi/2,add1[0,1],add1[0,2]-pi/2,theta51_-pi/2,theta61_),
                    (theta11,add1[1,0]+pi/2,add1[1,1],add1[1,2]-pi/2,theta51_-pi/2,theta61_),
                    (theta12,add2[0,0]+pi/2,add2[0,1],add2[0,2]-pi/2,theta52_-pi/2,theta62_),
                    (theta12,add2[1,0]+pi/2,add2[1,1],add2[1,2]-pi/2,theta52_-pi/2,theta62_)])
    return limit_mat(result)



# print(solver1(pi/6,0,pi/6,0,pi/3,0))
# print(solver1(pi/6,pi/6,pi/3,0,pi/3,pi/6))
# print(solver1(pi/2,0,pi/2,-pi/3,pi/3,pi/6))
# print(solver1(-pi/6,-pi/6,-pi/3,0,pi/12,pi/2))
# print(solver1(pi/12,pi/12,pi/12,pi/12,pi/12,pi/12))


# print("逆运动学求解")
# print(solver2(0.117,0.334,0.499,-2.019,-0.058,-2.190))
# print(solver2(-0.066,0.339,0.444,-2.618,-0.524,-3.141).T)
# print(solver2(0.3,0.25,0.26,-2.64,0.59,-2.35).T)
# print(solver2(0.42,0,0.36,3.14,1,-1.57).T)
# print(solver2(0.32,-0.25,0.16,3,0.265,-0.84).T)
# print(solver2(0.4, 0.12, 0.175, -np.pi, 0, -47.0/180*pi))

# print(solver2(0.08591837 ,0.35  ,     0.16   ,    3.14159265, 0.    ,   -3.14159265))

# print(solver1(-0.06591846,
#               1.61493273,
#               -0.82452922,
#               -0.21864954,
#                0.05459678,
#               -0.03619458))
# [ 0.08591837  0.35        0.16        3.14159265  0.         -3.14159265]
# [[-1.74765446e+00 -2.13248339e+00  1.29695615e+00 -7.35269084e-01 3.14159265e+00 -1.76858130e-01]
#  [-1.74765446e+00 -8.99545315e-01 -1.29695615e+00  6.25705140e-01 3.14159265e+00 -1.76858130e-01]
#  [ 1.26621239e+00  8.99545315e-01  1.29695615e+00 -6.25705140e-01 -3.67267027e-17 -3.04583934e-01]
#  [ 1.26621239e+00  2.13248339e+00 -1.29695615e+00  7.35269084e-01 -3.67267027e-17 -3.04583934e-01]]

