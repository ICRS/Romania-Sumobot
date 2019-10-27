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
        self.turned_1 = 0
        self.forward_route = 0
        self.turned_2 = 0

        self.p_eb = Pose()
        # self.v_eb = Twist()
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
        self.hard_code_turning_1 = 4
        self.hard_code_forward = 20
        self.hard_code_turning_2 = 14


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
            
        cmd_vel = Twist()
        cmd_vel.angular.z = 10
        self.publisher.publish(cmd_vel)
        
    def turn_around(self):
        self.linear_vel = 0
        if self.turned_cycle < self.turning_cycle:
            cmd_vel = Twist()
            cmd_vel.angular.z = 10
            self.publisher.publish(cmd_vel)
            self.turned_cycle += 1
        else:
            self.state = FINDING

    def hard_code_route(self):
        cmd_vel = Twist()
        if self.turned_1 < self.hard_code_turning_1:
            cmd_vel.angular.z = 10
            self.turned_1 += 1
        elif (self.turned_1 >= self.hard_code_turning_1 
            and self.forward_route < self.hard_code_forward):
            if self.linear_vel < self.max_vel:
                self.linear_vel += 0.5
            self.check_edge_sensors()
            cmd_vel.linear.x = self.linear_vel
            self.forward_route += 1
        elif (self.turned_1 >= self.hard_code_turning_1 
            and self.forward_route >= self.hard_code_forward
            and self.turned_2 < self.hard_code_turning_2):
            self.linear_vel = 0
            cmd_vel.angular.z = -10
            self.turned_2 += 1 
        else:
            self.turned_1 = 0
            self.forward_route = 0
            self.turned_2 = 0
            self.hard_code_route()
        self.publisher.publish(cmd_vel)
        self.check_if_attack()
        


    def check_if_attack(self):
        enemy_theta = math.acos(self.p_eb.orientation.w)
        if enemy_theta > 1.0472 and enemy_theta < 2.0944:
            self.state = ATTACK


    def attack(self):
        cmd_vel = Twist()
        if self.p_eb.position.y > 0.05 or self.p_eb.position.y < 0.05:
            cmd_vel.angular.z = 40 * self.p_eb.position.y
        if self.linear_vel < self.max_vel:
            self.linear_vel += 0.5
        cmd_vel.linear.x = self.linear_vel
        self.publisher.publish(cmd_vel)


    def update(self):
        if self.state == FINDING:
            rospy.loginfo("state: FINDING")
            self.finding()
            self.check_edge_sensors()
        elif self.state == TURN_AROUND:
            rospy.loginfo("state: TURN AROUND")
            self.turn_around()
        elif self.state == MOVE:
            rospy.loginfo("state: MOVE")
            self.hard_code_route()
        elif self.state == ATTACK:
            rospy.loginfo("state: ATTACK")
            self.attack()
        else:
            rospy.logerr("State invalid!")

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
        # self.v_eb = msg.twist.twist
        self.last_enemy_vo_time = msg.header.stamp 

if __name__ == '__main__':
    controller = SidePushController()
    controller.main()
