#!/usr/bin/env python3

import pybullet_data
import pybullet
import time
import numpy as np
import rospy
#from simple_robot_sim.msg import joint_angle , joint_state
from std_msgs.msg import Float64MultiArray
from std_msgs.msg import MultiArrayDimension


class robot_sim:
    def __init__(self, real_time):
        
        
        self.real_time = real_time

        rospy.init_node('robot_sim')
        self.rate = rospy.Rate(100)

        phisycsClient = pybullet.connect(pybullet.GUI)
        pybullet.setAdditionalSearchPath(pybullet_data.getDataPath())

        self.robotID = None
        self.reset()

        self.joint_publisher = rospy.Publisher('joint_state',Float64MultiArray,queue_size=1000)
        rospy.Subscriber('joint_command', Float64MultiArray, self.set_joint_positions)
        pass

    def set_joint_positions(self,msg):
        config = msg.data
        if len(config) != 6:
                rospy.loginfo("Invalid joint angles")
        else:
            for index in range(pybullet.getNumJoints(self.robotID)):
                pybullet.setJointMotorControl2(bodyUniqueId=self.robotID, 
                                        jointIndex=index, 
                                        controlMode=pybullet.POSITION_CONTROL, 
                                        targetPosition = config[index]) 
        pass

    def run(self):
        
        while not rospy.is_shutdown():
            state = [0] * 6
            for index in range(pybullet.getNumJoints(self.robotID)):
                state[index] = float(pybullet.getJointState(self.robotID,pybullet.getJointInfo(self.robotID,index)[0])[0])

            msg = Float64MultiArray(data=state)
            msg.layout.dim.append(MultiArrayDimension())
            msg.layout.dim[0].label = "theta"
            msg.layout.dim[0].size = 6
            msg.layout.dim[0].stride = 1
            msg.layout.data_offset = 0
            msg.data = state

            self.joint_publisher.publish(msg)
            #rospy.loginfo(state)
            
            #rospy.spin()
            self.rate.sleep()

        pass
    
    def reset(self):
        
        pybullet.resetSimulation()
        planeID = pybullet.loadURDF("plane.urdf")
        pybullet.setGravity(0,0,-9.81)
        self.robotID = pybullet.loadURDF("src/simple_robot_sim/urdf/simple_robot.urdf",
                                [0.0,0.0,0.6],pybullet.getQuaternionFromEuler([0.0,0.0,0.0]),useFixedBase = 1)
        
        if self.real_time:
            pybullet.setRealTimeSimulation(1)
        else:
            pybullet.setRealTimeSimulation(0)
        pass

    def close():
        pybullet.disconnect()


if __name__ == "__main__":
    robot = robot_sim(real_time=True)
    robot.run()
    pass