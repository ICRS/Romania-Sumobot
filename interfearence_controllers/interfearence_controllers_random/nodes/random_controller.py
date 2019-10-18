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

FORWARDS = 2
BACKWARDS = 3
TURNING_LEFT = 4
TURNING_RIGHT = 5

class RandomController(GenericController):
    """
    A controller that dodges when the enemy rush at us,
    and go around and push them off
    """
    def __init__(self):
        GenericController.__init__(self)

        # create class variables
        self.a = 1.3963
        self.b = 2.0944
        self.__max_linear_vel = 2.5
        # linear velocity
        self.linear_vel = 0
        # angular 
        self.angle_turned = 0
        self.angular_vel = 3
        self.theta = self.a * random.random() + self.b
       # switch state: FORWARDS, BACKWARDS,TURNING_LEFT, TURNING_RIGHT
        self.state = FORWARDS
        # # pose of enemy in our body frame
        # self.p_eb = Pose()
        # # pose of body in starting frame
        # self.p_bs = Pose()
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
        cmd_vel = Twist()
        if self.linear_vel < self.__max_linear_vel:
            self.linear_vel += 0.5
        else:
            self.linear_vel = self.__max_linear_vel
        cmd_vel.linear.x = self.linear_vel
        self.publisher.publish(cmd_vel)

    def backward(self):
        cmd_vel = Twist()
        if self.linear_vel > -self.__max_linear_vel:
            self.linear_vel -= 0.5
        else:
            self.linear_vel = -self.__max_linear_vel
        cmd_vel.linear.x = self.linear_vel
        self.publisher.publish(cmd_vel)

    def turn_left(self,turn_angle):
        if self.angle_turned < turn_angle:
            start_time = rospy.Time.now()
            cmd_vel = Twist()
            cmd_vel.angular.z = -self.angular_vel
            self.publisher.publish(cmd_vel)
            time_elapsed = rospy.Time.now() - start_time
            self.angle_turned += self.angular_vel * (
                time_elapsed.secs + 1e-9 * time_elapsed.nsecs)
        else:
            self.angle_turned = 0
            self.theta = self.a * random.random() + self.b
            self.state = FORWARDS

    def turn_right(self,turn_angle):
        if self.angle_turned < turn_angle:
            start_time = rospy.Time.now()
            cmd_vel = Twist()
            cmd_vel.angular.z = self.angular_vel
            self.publisher.publish(cmd_vel)
            time_elapsed = rospy.Time.now() - start_time
            self.angle_turned += self.angular_vel * (
                time_elapsed.secs + 1e-9 * time_elapsed.nsecs)
        else:
            self.angle_turned = 0
            self.theta = self.a * random.random() + self.b
            self.state = FORWARDS

    def check_edge_sensors(self):
        if self.edges[0] and self.edges[1]:
            self.linear_vel = 0
            self.state = BACKWARDS
            rospy.loginfo("BOTH SENSORS ARE AT EDGE")
        elif self.edges[0] and not self.edges[1]:
            self.state = TURNING_RIGHT
            rospy.loginfo("LEFT SENSOR IS AT EDGE")
        elif self.edges[1] and not self.edges[0]:
            self.state = TURNING_LEFT
            rospy.loginfo("RIGHT SENSOR IS AT EDGE")
        else:
            self.state = FORWARDS

    def update(self):
        cmd_vel = Twist()
        if self.state == FORWARDS:
            self.forward()
            # self.check_edge_sensors()
            # rospy.loginfo("FORWARDS")
        elif self.state == BACKWARDS:
            self.backward()
            self.check_edge_sensors()
            # rospy.loginfo("BACKWARDS")
        elif self.state == TURNING_LEFT:
            turn_angle = self.a * random.random() + self.b
            self.turn_left(self.theta)
            # rospy.loginfo("LEFT")
        elif self.state == TURNING_RIGHT:
            self.turn_right(self.theta)
            # rospy.loginfo("RIGHT")
        else:
            rospy.logerr("State invalid!")


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
