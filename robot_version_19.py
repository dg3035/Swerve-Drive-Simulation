import pybullet
import pybullet_data
import os
import numpy as np

pybullet.connect(pybullet.GUI)
pybullet.resetSimulation()
pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

plane = pybullet.loadURDF("plane.urdf")
pybullet.changeDynamics(plane, -1, lateralFriction=3., rollingFriction=0)

pybullet.resetDebugVisualizerCamera(6.0,0.0, -60.0, (0, 0., 0.5))
#(camera_distance,camera_yaw,camera_pitch,camera_target_position)

robot_id = pybullet.loadURDF("robot_with_4_tyres_trail_2/urdf/robot_with_4_tyres_trail_2.urdf",[0, 0, 1])
pybullet.changeDynamics(robot_id, -1, lateralFriction=3., rollingFriction=0.)
position, orientation = pybullet.getBasePositionAndOrientation(robot_id)
# Friction between joint links and surface.
for i in range(pybullet.getNumJoints(robot_id)):
    pybullet.changeDynamics(robot_id, i, lateralFriction=3., rollingFriction=0.)

print("position,orientation", position,orientation)	
joints = pybullet.getNumJoints(robot_id)
print("Number of joints swerve drive robot", joints) 

pybullet.setGravity(0,0,-9.81)

pybullet.setTimeStep(0.01)
# pybullet.resetBasePositionAndOrientation(robot_id, [0,0,1.], (3, -3.14, -1.57, 1.))
pybullet.resetBasePositionAndOrientation(robot_id, [0,0,1.], (0,0,0, 1.))

V_x = 0 # central velocity x component
V_y = 0 # central velocity y component

W = 0 # angular velocity for the robot
W_deg = 0 #since I do not want the robot to have orientation change while movement

#there is error in the line since I need to verify the exact positions of the robot with the urdf file to be exact
l = 0.1 # lenght for the robot
w = 0.1 # width for the robot

rot = 0 #complete angle for robot's orientation(also extract from the getPoseandOrientation())
euler_rot = 0 #complete angle in euler angle for the robot
pos = 0 #comple pose of the robot's position(also use getPoseanOrientation())

dt = 0.01

v1_x = 0
v1_y = 0
v2_x = 0
v2_y = 0
v3_x = 0
v3_y = 0
v4_x = 0
v4_y = 0

v1 = 0
v2 = 0
v3 = 0
v4 = 0
old_theta_1 = 0
old_theta_2 = 0
old_theta_3 = 0
old_theta_4 = 0
p_des=[1,1]
goal_orientation = 1.57
for i in range(500000):

	pos,rot = pybullet.getBasePositionAndOrientation(robot_id)
	euler = pybullet.getEulerFromQuaternion(rot)
	rot = euler[2]
	# print(rot)
 	# 
	V_x = p_des[0]-pos[0]
	V_y = p_des[1]-pos[1]
	# V_x = 1
	# V_y = 1
	# W = goal_orientation - rot
	# W = 10

	W  = goal_orientation - rot

	# if ((error) >= 0.1) | ((error) <= -0.1) : 
	# 	W =  error
	# elif ((error) < 0.1) & ((error) > -0.1):
	# 	W = 0
   


	# experimenting some stuff

	v1_x = V_x - W*l # velcity for robot 1 x component
	v1_y = V_y + W*w # velocity for robot 1 y component

	v2_x = V_x + W*l # velcity for robot 2 x component
	v2_y = V_y + W*w # velocity for robot 2 y component

	v3_x = V_x - W*l # velcity for robot 3 x component
	v3_y = V_y - W*w # velocity for robot 3 y component

	v4_x = V_x + W*l # velcity for robot 4 x component
	v4_y = V_y - W*w # velocity for robot 4 y component
	
	delta_theta_1 = np.arctan2(v1_y,v1_x) # theta for robot 1
	delta_theta_2 = np.arctan2(v2_y,v2_x) # theta for robot 2
	delta_theta_3 = np.arctan2(v3_y,v3_x) # theta for robot 3
	delta_theta_4 = np.arctan2(v4_y,v4_x) # theta for robot 4

	v1 = np.minimum((np.linalg.norm([v1_x,v1_y])),1) # maginitude for velocity of robot 1
	v2 = np.minimum((np.linalg.norm([v2_x,v2_y])),1) # maginitude for velocity of robot 2
	v3 = np.minimum((np.linalg.norm([v3_x,v3_y])),1) # maginitude for velocity of robot 3
	v4 = np.minimum((np.linalg.norm([v4_x,v4_y])),1) # maginitude for velocity of robot 4
	
	#VERTICAL TYRE CALLIBERATION
	theta_1 = 2.13 + delta_theta_1
	theta_2 = 3.23 + delta_theta_2
	theta_3 = 1.28 + delta_theta_3
	theta_4 = 3.67 + delta_theta_4

	change_theta_1 = (theta_1*theta_1 - old_theta_1*old_theta_1)/2
	change_theta_2 = (theta_2*theta_2 - old_theta_2*old_theta_2)/2
	change_theta_3 = (theta_3*theta_3 - old_theta_3*old_theta_3)/2
	change_theta_4 = (theta_4*theta_4 - old_theta_4*old_theta_4)/2

	# change_theta_1 = 0
	# change_theta_2 = 0
	# change_theta_3 = 0
	# change_theta_4 = 0

	# v1 = 0
	# v2 = 0
	# v3 = 0
	# v4 = 0	

	pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 0, controlMode = pybullet.POSITION_CONTROL,targetPosition = change_theta_1, force= 50)
	pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 2, controlMode = pybullet.POSITION_CONTROL,targetPosition = change_theta_2, force= 50)
	pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 4, controlMode = pybullet.POSITION_CONTROL,targetPosition = change_theta_3, force= 50)
	pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 6, controlMode = pybullet.POSITION_CONTROL,targetPosition = change_theta_4, force= 50)
		
	# pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 0, controlMode = pybullet.VELOCITY_CONTROL,targetVelocity = change_theta_1, force= 5)
	# pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 2, controlMode = pybullet.VELOCITY_CONTROL,targetVelocity = change_theta_2, force= 5)
	# pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 4, controlMode = pybullet.VELOCITY_CONTROL,targetVelocity = change_theta_3, force= 5)
	# pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 6, controlMode = pybullet.VELOCITY_CONTROL,targetVelocity = change_theta_4, force= 5)
	

	pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 1, controlMode = pybullet.VELOCITY_CONTROL,targetVelocity = v1, force= 100)
	pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 3, controlMode = pybullet.VELOCITY_CONTROL,targetVelocity = v2, force= 100)
	pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 5, controlMode = pybullet.VELOCITY_CONTROL,targetVelocity = v3, force= 100)
	pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 7, controlMode = pybullet.VELOCITY_CONTROL,targetVelocity = v4, force= 100)
	
	old_theta_1 = theta_1
	old_theta_2 = theta_2
	old_theta_3 = theta_3
	old_theta_4 = theta_4

	pybullet.stepSimulation()


pybullet.disconnect()
