#!/usr/bin/env python

import math
import time
import random

import rospy
from geometry_msgs.msg import Twist, Pose, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from interfearence_msgs.msg import EdgeDetection
from interfearence_controllers_generic_controller.generic_controller import GenericController


class RandomController(GenericController):
    """
    A controller that dodges when the enemy rush at us,
    and go around and push them off
    """
    def __init__(self):
        GenericController.__init__(self)

        # create class variables
        self.__max_vel = 2.5
        self.a = 1.3963
        self.b = 2.0944
        self.vel = 0
        self.theta = 0
        # # pose of enemy in our body frame
        # self.p_eb = Pose()
        # # pose of body in starting frame
        # self.p_bs = Pose()
        # switch state: FORWARDS, BACKWARDS,TURNING_LEFT, TURNING_RIGHT
        self.state = ""

        # if at edge
        self.edges = [False,False,False,False]

        self.tf_prefix = rospy.get_param('tf_prefix')
        if self.tf_prefix != '':
            self.tf_prefix = '/' + self.tf_prefix + '/'

        # ros subscribers
        # rospy.Subscriber('enemy_vo', Odometry, self.enemy_odom_cb)
        # rospy.Subscriber('odometry/measured', Odometry, self.self_odom_cb)
        rospy.Subscriber('edge_sensors/front_left', EdgeDetection, self.edge_cb)
        rospy.Subscriber('edge_sensors/front_right', EdgeDetection, self.edge_cb)
        rospy.Subscriber('edge_sensors/rear_left', EdgeDetection, self.edge_cb)
        rospy.Subscriber('edge_sensors/rear_right', EdgeDetection, self.edge_cb)
        # ros publisher
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=2)

    # # distance from enemy to us(body)
    # def get_d_eb(self):
    #   position = p_bs.position
    #   return math.sqrt(position.x**2 + position.y**2 + position.z**2)

    def reset(self):
        cmd_vel = Twist()
        self.publisher.publish(cmd_vel)

    def forward(self):
    		if self.vel < self.__max_vel:
    				self.vel += 0.1
    		else:
    				self.vel = self.__max_vel
    		rospy.loginfo("self.vel: {}".format(self.vel))
    		return self.vel

    def backward(self):
    		if self.vel > -self.__max_vel:
    				self.vel -= 0.1
    		else:
    				self.vel = -self.__max_vel
    		rospy.loginfo("self.vel: {}".format(self.vel))
    		return self.vel

    def turn(self,left):
    		



    def update(self):
        cmd_vel = Twist()
        self.theta = self.a * random.random() + self.b
        if self.edges[0] and self.edges[1]:
        		# cmd_vel.linear.x = - self.__max_vel
        		cmd_vel.linear.x = self.backward()
        elif self.edges[0] and not self.edges[1]:
        		cmd_vel.angular.z = random.random() * self.a + self.b
        elif self.edges[1] and not self.edges[0]:
        		cmd_vel.angular.z = -random.random() * self.a - self.b
        elif self.edges[2] or self.edges[3]:
            cmd_vel.linear.x = self.forward()
            # cmd_vel.linear.x = self.__max_vel
        else:
        		rospy.loginfo("going forward...")
        		cmd_vel.linear.x = self.forward()
        		# cmd_vel.linear.x = self.__max_vel
        self.publisher.publish(cmd_vel)


    def get_name(self):
        return "random"

    # def enemy_odom_cb(self, msg):
    #   self.p_eb = msg.pose.pose 

    # def self_odom_cb(self, msg):
    #   self.p_bs = msg.pose.pose

    def edge_cb(self, msg):
        if msg.header.frame_id == self.tf_prefix + 'front_left_line_sensor':
            self.edges[0] = msg.at_edge
        elif msg.header.frame_id == self.tf_prefix + 'front_right_line_sensor':
            self.edges[1] = msg.at_edge
        elif msg.header.frame_id == self.tf_prefix + 'rear_left_line_sensor':
            self.edges[2] = msg.at_edge
        elif msg.header.frame_id == self.tf_prefix + 'rear_right_line_sensor':
            self.edges[3] = msg.at_edge
        else:
            raise RuntimeError(msg.header.frame_id + ' is not a valid option!')

if __name__ == '__main__':
    controller = RandomController()
    controller.main()
