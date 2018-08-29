#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist

# Author: Dominik Mazniak
# Date: 10.08.2018
# Course: Autonomous Mobile Robots

# Description:
# The class which deals with publishing the velocity to the topic

class VelocityPublisher:
    def __init__(self):
        self.linear_speed = 0.8 # max linear velocity (+/-)
        self.angular_speed = 0.3 # max angular velocity (+/-)
        self.speed = Twist() # object of type Twist, same as message type
        self.speed.linear.x = 0.0 # current linear speed value
        self.speed.angular.z = 0.0 # current angular speed value
        self.vel_topic = '/mobile_base/commands/velocity' # topic
        self.pub = rospy.Publisher(self.vel_topic, Twist, queue_size=1)
    # controls movement of the robot
    # range of linear_power = (-100, 100) - resulting in linear speed = (-0.8, 0.8)
    # range of angular_power = (-100, 100) - resulting in angular_speed = (-0.3, 0.3)
    # negative values of angular_power -> clockwise
    # positive values of angular_power -> counterclockwise
    def move_linear(self, linear_power):
        self.speed.linear.x = self.linear_speed * float(linear_power) / 100
        self.pub.publish(self.speed)
    def move_angular(self, angular_power):
        self.speed.angular.z = self.angular_speed * float(angular_power) / 100
        self.pub.publish(self.speed)

    # stops the whole robot
    def stop_robot(self):
        self.speed.linear.x = 0
        self.speed.angular.z = 0
