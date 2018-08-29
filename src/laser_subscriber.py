#!/usr/bin/python
import rospy
from sensor_msgs.msg import LaserScan
from state_machine import StateMachine
# Author: Dominik Mazniak
# Date: 10.08.2018
# Course: Autonomous Mobile Robots

# Description: Laser subscriber class which deals with extracting data from the
# /laserscan topic. It also creates sectors and checks for obstacles

class LaserSubscriber:
    def __init__(self):
        self.second_threshold = 1.5 # changes to found_obs_state if anything.
        # in front in the given distance seen
        self.laser_topic = '/laserscan' # topic to subscribe laser data
        self.ranges = None # laser data measurement
        self.max_range = 0.0 # max range
        self.samples = 0 # amount of samples
        self.is_obstacle = False # is obstacle seen within self.threshold_distance?

        #2nd state machine (sm = state machine)
        self.sm_avoid =       StateMachine()
        self.sm_avoid.state = {      "find_obstacle" :                    True,
                                     "turn_left" :                      False,
                                     "follow_obstacle" :                 False,
                                     }

        self.sectors = {} # divided data into 5 sectors

        # subscriber initialization
        rospy.Subscriber(self.laser_topic, LaserScan, self.callback)

    # callback for subscriber to /laserscan
    def callback(self, msg):
        self.ranges = msg.ranges
        self.max_range = msg.range_max

    # magic method to get len() method. Returns amount of samples of the laser
    def __len__(self):
        return len(self.samples)
    # assigns sectors based on data from the callback
    def get_sectors(self):
        # if callback hasn't started sending data yet
        if self.ranges == None:
            self.sectors = {
                "right": 0,
                "front_right": 0,
                "front": 0,
                "front_left": 0,
                "left": 0
            }
        # if callback started sending data
        else:
            # length of data
            self.samples = len(self.ranges)
            self.sectors = {
                # divide into 5 sectors
                "right" :       min(min(self.ranges[0:self.samples/5-1]), self.max_range),
                "front_right":  min(min(self.ranges[1*self.samples/5:2*self.samples/5-1]), self.max_range),
                "front":        min(min(self.ranges[2*self.samples/5:3*self.samples/5-1]), self.max_range),
                "front_left":   min(min(self.ranges[3*self.samples/5:4*self.samples/5-1]), self.max_range),
                "left":         min(min(self.ranges[4*self.samples/5:1*self.samples-1]), self.max_range)
            }

    # algorithm of avoiding the obstacle. Detects obstacles in the given sectors
    def check_obstacle(self):
        self.get_sectors()

        # nothing
        if self.sectors["front_left"] > self.second_threshold and \
         self.sectors["front"] > self.second_threshold \
        and self.sectors["front_right"] > self.second_threshold:
            self.sm_avoid.set_state("find_obstacle")

        # sth in front
        elif self.sectors["front_left"] > self.second_threshold and \
        self.sectors["front"] < self.second_threshold \
         and self.sectors["front_right"] > self.second_threshold:
            self.sm_avoid.set_state("turn_left")

        # sth in front_left
        elif self.sectors["front_left"] < self.second_threshold and \
        self.sectors["front"] > self.second_threshold \
         and self.sectors["front_right"] > self.second_threshold:
            self.sm_avoid.set_state("find_obstacle")


        # sth in front_right
        elif self.sectors["front_left"] > self.second_threshold and \
        self.sectors["front"] > self.second_threshold \
         and self.sectors["front_right"] < self.second_threshold:
            self.sm_avoid.set_state("follow_obstacle")
        # sth in front_left and front
        elif self.sectors["front_left"] < self.second_threshold and \
        self.sectors["front"] < self.second_threshold \
         and self.sectors["front_right"] > self.second_threshold:
            self.sm_avoid.set_state("turn_left")

        # sth in front front_right
        elif self.sectors["front_left"] > self.second_threshold and \
        self.sectors["front"] < self.second_threshold \
         and self.sectors["front_right"] < self.second_threshold:
            self.sm_avoid.set_state("turn_left")

        # sth in front_left and front_right
        elif self.sectors["front_left"] < self.second_threshold and \
        self.sectors["front"] > self.second_threshold \
         and self.sectors["front_right"] < self.second_threshold:
            self.sm_avoid.set_state("find_obstacle")

        # sth in front everywhere
        elif self.sectors["front_left"] < self.second_threshold and \
        self.sectors["front"] < self.second_threshold \
         and self.sectors["front_right"] < self.second_threshold:
            self.sm_avoid.set_state("turn_left")
