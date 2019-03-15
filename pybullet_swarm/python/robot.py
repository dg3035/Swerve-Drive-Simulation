import numpy as np
import pybullet as p
import itertools

class Robot():

    index = 0
    Kf =1 # proportionality formation constant
    Kt =1 # target proportionality constant
    def __init__(self, init_pos, robot_id, dt):
        self.id = robot_id
        self.dt = dt
        # self.pybullet_id = p.loadSDF("../models/robot.sdf")[0]
        self.pybullet_id = p.loadURDF("../models/robot_with_4_tyres_trail_2/urdf/robot_with_4_tyres_trail_2.urdf",[0, 0, 1])
        self.joint_ids = list(range(p.getNumJoints(self.pybullet_id)))
        self.initial_position = init_pos
        self.velocity = [0,0]
        self.reset()
        
        p_des =  []
        goal_pose = []
        
        self.time = 0
        self.old_theta_1 = 0
        self.old_theta_2 = 0
        self.old_theta_3 = 0
        self.old_theta_4 = 0

        # No friction between body and surface.
        p.changeDynamics(self.pybullet_id, -1, lateralFriction=2, rollingFriction=0.)

        # Friction between joint links and surface.
        for i in range(p.getNumJoints(self.pybullet_id)):
            p.changeDynamics(self.pybullet_id, i, lateralFriction=2, rollingFriction=0.)

        self.messages_received = []
        self.messages_to_send = []
        self.neighbors = []


    def reset(self):
        p.resetBasePositionAndOrientation(self.pybullet_id, self.initial_position, (0., 0., 0., 1.))

    def set_wheel_velocity(self, V_x,V_y,W):
        """
        Sets the wheel velocity,expects an array containing two numbers (left and right wheel vel)
        """
        # assert len(vel) == 2, "Expect velocity to be array of size two"
        # p.setJointMotorControlArray(self.pybullet_id, self.joint_ids, p.VELOCITY_CONTROL,
        #     targetVelocities=vel)
        l = 0.23
        w = 0.23

        v1_x = V_x - W*l # velcity for robot 1 x component
        v1_y = V_y + W*w # velocity for robot 1 y component

        v2_x = V_x + W*l # velcity for robot 2 x component
        v2_y = V_y + W*w # velocity for robot 2 y component

        v3_x = V_x - W*l # velcity for robot 3 x component
        v3_y = V_y - W*w # velocity for robot 3 y component

        v4_x = V_x + W*l # velcity for robot 4 x component
        v4_y = V_y - W*w # velocity for robot 4 y component

        #add this delta_theta to the previous theta
        delta_theta_1 = np.arctan2(v1_y,v1_x) # theta for robot 1
        delta_theta_2 = np.arctan2(v2_y,v2_x) # theta for robot 2
        delta_theta_3 = np.arctan2(v3_y,v3_x) # theta for robot 3
        delta_theta_4 = np.arctan2(v4_y,v4_x) # theta for robot 4

        v1 = np.linalg.norm([v1_x,v1_y]) # maginitude for velocity of robot 1
        v2 = np.linalg.norm([v2_x,v2_y]) # maginitude for velocity of robot 2
        v3 = np.linalg.norm([v3_x,v3_y]) # maginitude for velocity of robot 3
        v4 = np.linalg.norm([v4_x,v4_y]) # maginitude for velocity of robot 4

        theta_1 = 2.13 + delta_theta_1
        theta_2 = 3.23 + delta_theta_2
        theta_3 = 1.28 + delta_theta_3
        theta_4 = 3.67 + delta_theta_4

        p.setJointMotorControl2(bodyUniqueId = self.pybullet_id, jointIndex = 0, controlMode = p.POSITION_CONTROL,targetPosition = theta_1, force= 5)
        p.setJointMotorControl2(bodyUniqueId = self.pybullet_id, jointIndex = 2, controlMode = p.POSITION_CONTROL,targetPosition = theta_2, force= 5)
        p.setJointMotorControl2(bodyUniqueId = self.pybullet_id, jointIndex = 4, controlMode = p.POSITION_CONTROL,targetPosition = theta_3, force= 5)
        p.setJointMotorControl2(bodyUniqueId = self.pybullet_id, jointIndex = 6, controlMode = p.POSITION_CONTROL,targetPosition = theta_4, force= 5)

        p.setJointMotorControl2(bodyUniqueId = self.pybullet_id, jointIndex = 1, controlMode = p.VELOCITY_CONTROL,targetVelocity = v1, force= 100)
        p.setJointMotorControl2(bodyUniqueId = self.pybullet_id, jointIndex = 3, controlMode = p.VELOCITY_CONTROL,targetVelocity = v2, force= 100)
        p.setJointMotorControl2(bodyUniqueId = self.pybullet_id, jointIndex = 5, controlMode = p.VELOCITY_CONTROL,targetVelocity = v3, force= 100)
        p.setJointMotorControl2(bodyUniqueId = self.pybullet_id, jointIndex = 7, controlMode = p.VELOCITY_CONTROL,targetVelocity = v4, force= 100)
    

    def get_pos_and_orientation(self):
        """
        Returns the position and orientation (as Yaw angle) of the robot.
        """
        pos, rot = p.getBasePositionAndOrientation(self.pybullet_id)
        euler = p.getEulerFromQuaternion(rot)
        return np.array(pos), euler[2]

    def get_messages(self):
        return self.messages_received

    def send_message(self, robot_id, message):
        self.messages_to_send.append([robot_id, message])

    def get_neighbors(self):
        return self.neighbors

        #The main formation and movement control laws are written in this function


    def compute_controller(self):
        
        neig = self.get_neighbors()#get neighbour information
        messages = self.get_messages()#get messages from the neighbors
        pos, rot = self.get_pos_and_orientation()#get pose and orientation of the robot
        # print(rot,self.id)

        #send message of positions to all neighbors indicating our position and velocity
        for n in neig:
            self.send_message(n, [pos,rot])

        # check if we received the position of our neighbors and compute desired change in position
        # as a function of the neighbors (message is composed of [neighbors id, position])
        V_x = 0.
        V_y = 0.
        W = 0
        
        #switching the waypoint based upon the value of the switching index svalue. 
        #Square formation 
        if(Robot.index==0):
            p_des = [[2.5,-1.5],[2.5,0.5],[2.5,-0.5],[2.5,1.5]]
            # p_des = [[5,-3],[5,1],[5,-1],[5,3]]
        #Line formation
        if(Robot.index ==1):
            p_des = [[2.5,4.5],[1,4.5],[2.5,6],[1,6]]
            # p_des = [[5,9],[2,9],[5,12],[2,12]]
        #first square foramtion
        elif(Robot.index ==2):
            p_des = [[-1.5,4.5],[-3,4.5],[-1.5,6],[-3,6]]
        # #x direction motion
        elif(Robot.index ==3):
            p_des = [[-1.5,-2.5],[-3,-2.5],[-1.5,-1],[-3,-1]]
        # #y direction motion
        elif(Robot.index ==4): 
            p_des = [[2,-0.5],[0.5,-0.5],[2,1],[0.5,1]]
        #diagonal motion
      
        if messages:
            for m in messages:

                #Second order formation control law
                V_x += Robot.Kf*(m[1][0][0] - pos[0] -p_des[m[0]][0] + p_des[self.id][0]) 
                V_y += Robot.Kf*(m[1][0][1] - pos[1] -p_des[m[0]][1] + p_des[self.id][1])

                # V_x += Robot.Kf*(m[1][0][0] - pos[0]) 
                # V_y += Robot.Kf*(m[1][0][1] - pos[1])

                

                #Second order Reach- Goal target control law
                residual_x = np.minimum((p_des[self.id][0]-pos[0]),1)
                residual_y = np.minimum((p_des[self.id][1]-pos[1]),1)
                # residual_x = 0
                # residual_y =0
                
                if ((self.id)==0):
                    if ((rot) >= 0.01) | ((rot) <= -0.01) :
                        W = - 2*rot
                    elif (((rot) < 0.01) & ((rot) > -0.01)):
                        W = 0
                else:
                    W +=1*(m[1][1] - rot)

                #Calculating the stopping condition for contr
                stopping_condition = np.linalg.norm([residual_x,residual_y])

                #LINE FORMATION

                if((stopping_condition<0.3)&(Robot.index==0)):
                    Robot.index=1 #SQURE FORAMTION
                    Robot.Kf = 0.4
                    Robot.Kt = 0.4
                                      
                if((stopping_condition<0.1)&(Robot.index==1)):
                    Robot.index=2 #MOVE IN X DIRECTION
                    Robot.Kf = 0.6
                    Robot.Kt = 0.6

                if((stopping_condition<0.08)&(Robot.index==2)):
                    Robot.index=3 #MOVE IN Y DIRECTION
                    Robot.Kf = 0.5
                    Robot.Kt = 0.5
 
                if((stopping_condition<0.06)&(Robot.index==3)):
                    Robot.index=4 # MOVE DIAGONALLY
                    Robot.Kf = 0.6
                    Robot.Kt = 0.6

                #Second order motion control law
                V_x += Robot.Kt*residual_x
                V_y += Robot.Kt*residual_y


            self.set_wheel_velocity(V_x,V_y,W)


