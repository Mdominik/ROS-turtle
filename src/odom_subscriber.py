#!/usr/bin/python
from tf.transformations import euler_from_quaternion
from gazebo_msgs.msg import ModelStates
import rospy


# Author: Dominik Mazniak
# Date: 10.08.2018
# Course: Autonomous Mobile Robots

# Description: Odom subscriber class which deals with extracting data from the
# /gazebo/model_states topic
class OdomSubscriber:
    def __init__(self):
        self.x = 0.0 # current robot's X position
        self.y = 0.0 # current robot's Y position
        self.theta = 0.0 # current robot's  theta position
        self.odom_topic = '/gazebo/model_states' # topic
        self.previous_state = "init"
        # subscriber - message of type ModelStates
        rospy.Subscriber(self.odom_topic, ModelStates, self.callback)


    # callback for subscriber to '/gazebo/model_states'
    def callback(self,msg):
        x = msg.name.index("mobile_base") # get the index of mobile_base object in the list
        self.x = msg.pose[x].position.x # retrieve the position x of the x-th element
        self.y = msg.pose[x].position.y # retrieve the position y of the x-th element
        rot = msg.pose[x].orientation

        # calculate roll pitch and theta based on the library function
        (roll, pitch, self.theta) = euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
