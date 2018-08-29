#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist

# Author: Dominik Mazniak
# Date: 10.08.2018
# Course: Autonomous Mobile Robots

# Description:
# State machine which is universal - the object which utilizes this state machine
# decides about the states

class StateMachine():
    def __init__(self):
        self.state = {}

    # change state
    def set_state(self, state_name):
        # check if the state given in an argument exists
        if state_name in self.state:
            # iterate through the whole dictionary
            for key,value in self.state.items():
                # if it sees the given key, set the state to True
                if key == state_name:
                    self.state[key] = True
                else:
                # set all other states to False
                    self.state[key] = False
        else:
            rospy.loginfo("WRONG STATE!")
