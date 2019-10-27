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

FINDING = 2 # find where the enemy robot is
MOVE = 3 # move so that we face towards enemy's side
ATTACK = 4 # sprint at the enemy robot
TURN_AROUND = 5

class SidePushController(GenericController):
"""
A controller that push from the side
"""
	def __init__(self):
		GenericController.__init__(self)

		self.edges = [False, False, False, False]
		self.turned_cycle = 0
		self.linear_vel = 0

		self.p_eb = Pose()
		self.v_eb = Twist()
		self.last_enemy_vo_time = rospy.Time()
		self.max_vel = 2.5

		# ros subscribers
        rospy.Subscriber('enemy_vo', Odometry, self.enemy_odom_cb)
        # rospy.Subscriber('odometry/measured', Odometry, self.self_odom_cb)
        rospy.Subscriber('edge_sensors/front_left', EdgeDetection, self.edge_cb)
        rospy.Subscriber('edge_sensors/front_right', EdgeDetection, self.edge_cb)
        rospy.Subscriber('edge_sensors/rear_left', EdgeDetection, self.edge_cb)
        rospy.Subscriber('edge_sensors/rear_right', EdgeDetection, self.edge_cb)
        # ros publisher
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=2)

        # TODO: tune parameter
        self.turning_cycle = 25


	def reset(self):
		cmd_vel = Twist()
		self.edges = [False, False, False, False]
		self.linear_vel = 0
		self.turned_cycle = 0
		self.p_eb = Pose()
		self.publisher.publish(cmd_vel)
		self.state = FINDING

	def check_edge_sensors(self):
        if self.edges[0] or self.edges[1]:
            self.linear_vel = 0
            self.state = TURN_AROUND
        else:
            cmd_vel = Twist()
            cmd_vel.linear_vel.x = 1.0
            self.publisher.publish(cmd_vel)
            self.state = FINDING

	def finding(self):
		current_time = rospy.Time.now()
		if (current_time - self.last_enemy_vo_time).nsec < 2e8:
			self.state = MOVE
			return
		cmd_vel = Twist()
		cmd_vel.angular.z = 10
		self.publisher.publish(cmd_vel)
		
	def turn_around(self):
		if self.turned_cycle < self.turning_cycle:
			cmd_vel = Twist()
			cmd_vel.angular.z = 10
			self.publisher.publish(cmd_vel)
			self.turned_cycle += 1
		else:
			self.state = FINDING

	def move(self):
		self.p_eb
		self.v_eb

	def attack(self):
		cmd_vel = Twist()
		if self.p_eb.y > 0.05 or self.p_eb.y < 0.05:
			cmd_vel.angular.z = 40 * self.p_eb
		else:
			if self.linear_vel < self.max_vel:
				self.linear_vel += 0.5
			cmd_vel.linear.x = self.linear_vel
		self.publisher.publish(cmd_vel)


	def update(self):
		cmd_vel = Twist()
		if self.state == FINDING:
			self.finding()
			self.check_edge_sensors()
		elif self.state == TURN_AROUND:
			self.turn_around()

	def getname(self):
		return "side_push"

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

    def enemy_odom_cb(self,msg):
    	self.p_eb = msg.pose.pose
    	self.v_eb = msg.twist.twist
    	self.last_enemy_vo_time = msg.header.stamp 

if __name__ == '__main__':
controller = SidePushController()
controller.main()
