# Make sure to have the add-on "ZMQ remote API"
# running in CoppeliaSim
#
# All CoppeliaSim commands will run in blocking mode (block
# until a reply from CoppeliaSim is received). For a non-
# blocking example, see simpleTest-nonBlocking.py

import time
import math
from coppeliasim_zmqremoteapi_client import RemoteAPIClient
import coppeliasim_zmqremoteapi_client
from math import pi
from solver import solver2

def setposotion(theta1,theta2,theta3,theta4,theta5,theta6):
    sim.setJointPosition(Joint1,theta1)
    sim.setJointPosition(Joint2,theta2)
    sim.setJointPosition(Joint3,theta3)
    sim.setJointPosition(Joint4,theta4)
    sim.setJointPosition(Joint5,theta5)
    sim.setJointPosition(Joint6,theta6)



print('Program started')

client = RemoteAPIClient()
sim = client.getObject('sim')

# When simulation is not running, ZMQ message handling could be a bit
# slow, since the idle loop runs at 8 Hz by default. So let's make
# sure that the idle loop runs at full speed for this program:
defaultIdleFps = sim.getInt32Param(sim.intparam_idle_fps)
sim.setInt32Param(sim.intparam_idle_fps, 0)

client.setStepping(True)
sim.startSimulation()

Joint1 = sim.getObject("./Joint1")
Joint2 = sim.getObject("./Joint2")
Joint3 = sim.getObject("./Joint3")
Joint4 = sim.getObject("./Joint4")
Joint5 = sim.getObject("./Joint5")
Joint6 = sim.getObject("./Joint6")
# sim.setJointTargetPosition(Joint1,0)
# sim.setJointTargetPosition(Joint2,0)
# sim.setJointTargetPosition(Joint3,0)
# sim.setJointTargetPosition(Joint4,math.pi/2)
# sim.setJointTargetPosition(Joint5,0)
# sim.setJointTargetPosition(Joint6,0)
# setposotion(math.pi/12,
#             math.pi/12,
#             math.pi/12,
#             math.pi/12,
#             math.pi/12,
#             math.pi/12)
# setposotion(pi/12,pi/12,pi/12,pi/12,pi/12,pi/12)
theta=solver2(0.117,0.334,0.499,-2.019,-0.058,-2.190)[1,:]
setposotion(theta[0],theta[1],theta[2],theta[3],theta[4],theta[5])

t=0

# while (t<3):
#     sim.setJointTargetPosition(Joint1,-math.pi/2*t/3)
#     sim.setJointTargetPosition(Joint2,0)
#     sim.setJointTargetPosition(Joint3,0)
#     client.step()
#     t=sim.getSimulationTime()
#     time.sleep(0.01)



#01

# Restore the original idle loop frequency:

sim.setInt32Param(sim.intparam_idle_fps, defaultIdleFps)

print('Program ended')


