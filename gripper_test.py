import pybullet as p
import pybullet_data
import time
import numpy as np
from math import *
import matplotlib.pyplot as plt
import cv2


clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
  p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

time_step = 0.001
gravity_constant = -9.81
p.resetSimulation()
p.setTimeStep(time_step)
p.setGravity(0.0, 0.0, gravity_constant)

p.loadURDF("plane.urdf", [0, 0, -0.3])

cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
boxId = p.loadURDF("table/table.urdf",cubeStartPos, cubeStartOrientation)
# appleId = p.loadURDF("apple.urdf",[0.0,0.0,0.625])
# kukaId = p.loadURDF("pybullet-playground/urdf/sisbot.urdf",[-0.6,0,0.625], cubeStartOrientation,flags=p.URDF_USE_INERTIA_FROM_FILE)
roboId = p.loadURDF("ur_description/urdf/ur10_robot_with_gripper.urdf",[-0.6,0,0.625], cubeStartOrientation,useFixedBase=0)
# roboId = kukaId
required_joints = [0,-1.9,1.9,-1.57,-1.57,0,0]
for i in range(1,7):
	p.setJointMotorControl2(roboId, i, p.POSITION_CONTROL,
	                        targetPosition=required_joints[i-1], force=500)
p.resetDebugVisualizerCamera( cameraDistance=2.2, cameraYaw=140, cameraPitch=-60, cameraTargetPosition=[0,0,0])

# numJoints = p.getNumJoints(kukaId)
# kukaEndEffectorIndex = numJoints - 1

for i in range(1000):
  p.stepSimulation()

inp1 = p.addUserDebugParameter('gripper', -0.3,0.3, 0.0)
    
while True:
	angle = p.readUserDebugParameter(inp1)
	p.resetJointState(bodyUniqueId=roboId,jointIndex=9,targetValue=angle)
	p.resetJointState(bodyUniqueId=roboId,jointIndex=10,targetValue=angle)
	p.stepSimulation()

p.disconnect()

