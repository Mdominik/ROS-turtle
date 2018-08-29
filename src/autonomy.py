#!/usr/bin/python
from tf.transformations import euler_from_quaternion
import math
import rospy
# Author: Dominik Mazniak
# Date: 10.08.2018
# Course: Autonomous Mobile Robots

# Description:
# This class wraps up the whole functionality of the robot.

from laser_subscriber import LaserSubscriber
from goals_subscriber import GoalsSubscriber
from odom_subscriber import OdomSubscriber
from velocity_publisher import VelocityPublisher
from state_machine import StateMachine


class Autonomy:
    def __init__(self):
        # composition of the classes
        self.vel_pub =          VelocityPublisher()
        self.odom_sub =         OdomSubscriber()
        self.goals_sub =        GoalsSubscriber()
        self.laser_sub =        LaserSubscriber()

        #1st state machine (sm = state machine). Keeps track of the program
        self.sm_general =         StateMachine()
        self.sm_general.state = {   "init" :         True,
                                    "odom" :         False,
                                    "obstacle" :     False,
                                    "stop"      :    False}

        self.goals_points = [] # list which stores the goals
        self.index = 0 # field which keeps track of the goals from 0 to 2
        self.distance = None # distance to a point

        self.angle_to_goal = 0 # current angle calculated from atan
        self.angle_diff = 0 # current angle error of the angle
        self.prev_angle_diff = 0 # previous angle error for derivative part of PID
        self.distance_to_stop = 0.2 # distance from a point to stop the robot
        self.distance_to_slower = 0.7 # distance from a point to slower the robot more
        self.distance_to_deccelerate = 1 # distance from a point to slower the robot
        self.angular_offset_velocity = 0.1 # angle offset

        self.kp = rospy.get_param('proportional_gain') # Proportional gain
        self.ki = rospy.get_param('integral_gain')  # Integral gain
        self.kd = rospy.get_param('derivative_gain') # derivative gain
        self.integral = 0 # integral part
        self.derivative = 0 # derivative part

        self.second_threshold = 2 # second threshold from bug0
        self.third_threshold = 2.5 # third threshold from bug0

        self.rate = rospy.get_param('rate') # rate with which the loop operates
    # checks if the whole task is finished and switches to stop state
    def is_done(self):
        if self.index >= 3:
            self.sm_general.set_state("stop")

    # computes the angle error from the goal
    def get_error_angle(self):
        if self.index < 3: # double checking the index
            # differences in X and Y coordinates
            inc_x = self.goals_points[2*self.index] - self.odom_sub.x
            inc_y = self.goals_points[2*self.index+1] - self.odom_sub.y

            # arc tg  and distance
            self.angle_to_goal = math.atan2(inc_y, inc_x)
            self.distance = math.sqrt(inc_x*inc_x+inc_y*inc_y)

            # assign previous angle for PID controller
            self.prev_angle_diff = self.angle_diff

            # angle error
            self.angle_diff = self.angle_to_goal - self.odom_sub.theta

            #normalize the angle
            if(math.fabs(self.angle_diff) > math.pi):
                self.angle_diff = self.angle_diff - (2 * math.pi * self.angle_diff) /\
                 (math.fabs(self.angle_diff))

    # robot driving to a goal based on the get_error_angle method
    def go_to_point(self):
        self.get_error_angle()

        # PID controller,
        self.integral += self.angle_diff
        self.derivative = self.angle_diff - self.prev_angle_diff

        # if the robot is close to the goal, deccelerate it and proceed to the next goal
        if self.distance < self.distance_to_stop:
            self.index += 1
            self.vel_pub.stop_robot()
        elif self.distance < self.distance_to_slower:
            self.vel_pub.move_linear(25)
        elif self.distance < self.distance_to_deccelerate:
            self.vel_pub.move_linear(50)

        # drive if the robot within the offset
        elif abs(self.angle_diff) < self.angular_offset_velocity:
            if round(self.angle_diff, 2) == 0:
                self.integral = 0
            self.vel_pub.move_linear(100)

        # PID controller setting up speed
        self.vel_pub.move_angular(100*(self.kp * self.angle_diff + self.ki \
         * self.derivative + self.ki * self.integral))

    # 3-state machine used for obstacle following
    def set_velocity(self):
        if self.laser_sub.sm_avoid.state["turn_left"]: # turn right
            self.vel_pub.move_linear(0)
            self.vel_pub.move_angular(80)
        if self.laser_sub.sm_avoid.state["find_obstacle"]: # move the other direction
            self.vel_pub.move_linear(20)
            self.vel_pub.move_angular(-80)
        if self.laser_sub.sm_avoid.state["follow_obstacle"]:# keep the obstacle on the left
            self.vel_pub.move_linear(80)
            self.vel_pub.move_angular(0)

    # bug0 algorithm is a simple obstacle avoidance algorithm.
    def bug0(self):
        self.get_error_angle()
        error_yaw = self.angle_diff
        self.laser_sub.get_sectors()

        # variables to make the conditions shorter
        fl = self.laser_sub.sectors['front_left']
        f = self.laser_sub.sectors['front']
        fr = self.laser_sub.sectors['front_right']
        l =  self.laser_sub.sectors['left']
        r =  self.laser_sub.sectors['right']
        s = self.second_threshold
        t = self.third_threshold


        if self.sm_general.state["odom"]:
            # if there is an obstacle in front, change the state
            if self.laser_sub.sectors["front"] < self.second_threshold:
                self.sm_general.set_state("obstacle")

        # !!!
        if self.sm_general.state["obstacle"]:
            if math.fabs(error_yaw) < (math.pi / 6) and f > t and fr > s and fl > s:
                self.sm_general.set_state("odom")
            if error_yaw < 0 and math.fabs(error_yaw) > (math.pi / 6) and \
                math.fabs(error_yaw) < (math.pi / 2)  and r > t and fr > s:
                self.sm_general.set_state("odom")
            if error_yaw > 0 and math.fabs(error_yaw) > (math.pi / 6) and \
                math.fabs(error_yaw) < (math.pi / 2)  and l > t and fl > s:
                self.sm_general.set_state("odom")


def main():
    rospy.init_node('autonomy') # initialize the node
    autonomy = Autonomy()
    r = rospy.Rate(autonomy.rate)
    while not rospy.is_shutdown():
        autonomy.goals_sub.started() # check if the list of goals is populated
        autonomy.goals_points = autonomy.goals_sub.get_goals()

        # 0) Init state, wait for the goals
        if autonomy.sm_general.state["init"]:
            if autonomy.goals_sub.is_started:
                autonomy.sm_general.set_state("odom")

        # 1) Odom state, navigate from point to point
        if autonomy.sm_general.state["odom"]:
            autonomy.bug0()
            autonomy.go_to_point()

        # 2) Obstacle state, follow an obstacle and switch to odom when bug0 conditions are met
        if autonomy.sm_general.state["obstacle"]:
            autonomy.bug0()
            autonomy.laser_sub.check_obstacle()
            autonomy.set_velocity()

        # 3) Stop state after reaching the last goal
        if autonomy.sm_general.state["stop"]:
            autonomy.vel_pub.stop_robot()

        autonomy.is_done() # check if the index is more than 2
        r.sleep()

if __name__ == '__main__':
    main()
