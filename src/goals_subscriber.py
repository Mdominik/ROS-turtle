#!/usr/bin/python
import rospy
from std_msgs.msg import Float32MultiArray
# Author: Dominik Mazniak
# Date: 10.08.2018
# Course: Autonomous Mobile Robots

# This class is extracting goals coordinates data. There are 6 coordinates (3 points) published to the topic "/goals"
# To run the topic, you should first launch ublisher package

class GoalsSubscriber:
    def __init__(self):
        self.goals = [] # list of goals
        self.goals_topic = '/goals' # topic
        self.is_started = False # is the list populated?
        rospy.Subscriber(self.goals_topic, Float32MultiArray, self.callback)


    # callback for subscriber to /goals topic
    def callback(self, msg):
        self.goals = msg.data

    # return goals list
    def get_goals(self):
        return self.goals
    # did the subscriber start?
    def started(self):
        if self.goals == []:
            self.is_started = False
        else:
            self.is_started = True
