import pybullet
import pybullet_data
import os
import numpy as np
import matplotlib.pyplot as plt
from setup import setting_up_Simulator

if __name__ == "__main__":
	
	robot_id = setting_up_Simulator()
	
	V_x = 0 
	V_y = 0 
	W = 0 
	W_deg = 0 

	l = 0.237
	w = 0.237

	rot = 0 
	euler_rot = 0 
	pos = 7
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
	
	p_des=[3,4]
	goal_orientation = 1.57
	flag = 0

	for i in range(500000):

		#Position and Orientation of the robot from the simulator
		pos,rot = pybullet.getBasePositionAndOrientation(robot_id)
		euler = pybullet.getEulerFromQuaternion(rot)
		rot = euler[2]

		#Translation and Angular Velocity vectors for the robot

		if flag ==0:
			V_x = 2*(p_des[0]-pos[0])
			V_y = 2*(p_des[1]-pos[1])
			W = 0.4*(goal_orientation - rot)

		if np.linalg.norm([np.linalg.norm([V_x,V_y]),W])<0.04:
			V_x = 0
			V_y = 0
			W = 0
			flag = 1
	

		# if np.linalg.norm(W) < 0.01:
		# 	W = 0
			# flag = 1

		# Swerve Drive Kinematics
		v1_x = V_x - W*l # velcity for tyre 1 x component
		v1_y = V_y + W*w # velocity for tyre 1 y component

		v2_x = V_x + W*l # velcity for tyre 2 x component
		v2_y = V_y + W*w # velocity for tyre 2 y component

		v3_x = V_x - W*l # velcity for tyre 3 x component
		v3_y = V_y - W*w # velocity for tyre 3 y component

		v4_x = V_x + W*l # velcity for tyre 4 x component
		v4_y = V_y - W*w # velocity for tyre 4 y component
		
		# Theta for each bracket joint
		delta_theta_1 = np.arctan2(v1_y,v1_x) # theta for bracket joint 1
		delta_theta_2 = np.arctan2(v2_y,v2_x) # theta for bracket joint 2
		delta_theta_3 = np.arctan2(v3_y,v3_x) # theta for bracket joint 3
		delta_theta_4 = np.arctan2(v4_y,v4_x) # theta for bracket joint 4

		# Velocity for each tyre joint
		v1 = np.minimum((np.linalg.norm([v1_x,v1_y])),1) # maginitude for velocity of tyre 1
		v2 = np.minimum((np.linalg.norm([v2_x,v2_y])),1) # maginitude for velocity of tyre 2
		v3 = np.minimum((np.linalg.norm([v3_x,v3_y])),1) # maginitude for velocity of tyre 3
		v4 = np.minimum((np.linalg.norm([v4_x,v4_y])),1) # maginitude for velocity of tyre 4
		
		#VERTICAL TYRE CALLIBERATION
		theta_1 = 2.13 + delta_theta_1
		theta_2 = 3.23 + delta_theta_2
		theta_3 = 1.28 + delta_theta_3
		theta_4 = 3.67 + delta_theta_4

		pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 0, controlMode = pybullet.POSITION_CONTROL,targetPosition = theta_1, force= 500)
		pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 2, controlMode = pybullet.POSITION_CONTROL,targetPosition = theta_2, force= 500)
		pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 4, controlMode = pybullet.POSITION_CONTROL,targetPosition = theta_3, force= 500)
		pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 6, controlMode = pybullet.POSITION_CONTROL,targetPosition = theta_4, force= 500)

		pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 1, controlMode = pybullet.VELOCITY_CONTROL,targetVelocity = v1, force= 500)
		pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 3, controlMode = pybullet.VELOCITY_CONTROL,targetVelocity = v2, force= 500)
		pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 5, controlMode = pybullet.VELOCITY_CONTROL,targetVelocity = v3, force= 500)
		pybullet.setJointMotorControl2(bodyUniqueId = robot_id, jointIndex = 7, controlMode = pybullet.VELOCITY_CONTROL,targetVelocity = v4, force= 500)
		

		pybullet.stepSimulation()

	pybullet.disconnect()
