#!/usr/bin/env python

import math
import time

import rospy
from geometry_msgs.msg import Twist, Pose, Vector3
from nav_msgs.msg import Odometry
import tf2_ros

BASE_FRAME = 'base_link'
MAX_VEL = 1.5
MAX_THETA_DOT = 6.28318530718
LASER_SCAN_TIMEOUT = 100000

enemy_pose = Pose()
enemy_updated = False
our_pose = Pose()
our_velocity = Twist()
us_updated = False
last_message = None

tf_buffer = None
tf_listener = None

publisher = None

def mag(vec):
    return math.sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z)

def dot(a, b):
    return a.x*b.x + a.y*b.y + a.z*b.z

def publish_cmd_vel():
    global publisher
    global enemy_pose
    global enemy_updated
    global our_pose
    global our_velocity
    global us_updated

    global MAX_VEL
    global MAX_THETA_DOT

    if us_updated and enemy_updated:
        # Calculate vector between us and them
        vec = Vector3()
        vec.x = enemy_pose.position.x# - our_pose.position.x
        vec.y = enemy_pose.position.y# - our_pose.position.y
        vec.z = enemy_pose.position.z# - our_pose.position.z

        # Normalise
        m = mag(vec)
        if m != 0:
            vec.x = vec.x / m
            vec.y = vec.y / m
            vec.z = vec.z / m

        # Calculate the angle we need to turn
        forwards = Vector3()
        forwards.x = 1
        # acos ranges from 0 to pi
        theta = math.acos(dot(forwards, vec))

        # Maximum turning speed when the angle is greater than 20 degrees
        theta_dot = 0.35 * theta / MAX_THETA_DOT 
        if theta > 0.35:
            theta_dot = MAX_THETA_DOT

        # The linear velocity scales proportionally inversely with the
        # angular velocity
        linear_vel = MAX_VEL*(1 - theta_dot/MAX_THETA_DOT)

        # Account for negative theta, we want to turn TOWARDS them
        if vec.y < 0:
            theta_dot = -theta_dot

        cmd_vel = Twist()
        cmd_vel.angular.z = theta_dot
        cmd_vel.linear.x = linear_vel

        publisher.publish(cmd_vel)

        us_updated = False
        enemy_updated = False

def enemy_odom_callback(msg):
    global enemy_pose
    global enemy_updated
    global tf_buffer
    global last_message
    last_message = rospy.Time.now().secs + rospy.Time.now().nsecs / 1000000000

    # Transform enemy pose into our co-ordinate frame
    #try:
    #    trans = tf_buffer.lookup_transform(msg.child_frame_id,
    #                                       BASE_FRAME,
    #                                       rospy.Time(0))
    #except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
    #    rospy.logerr(e)
    #    return

    enemy_pose = msg.pose.pose
    #enemy_pose = trans.transform * msg.pose.pose
    enemy_updated = True

    publish_cmd_vel()

def self_odom_callback(msg):
    global our_pose
    global our_twist
    global us_updated

    our_pose = msg.pose.pose
    our_twist = msg.twist.twist
    us_updated = True

    publish_cmd_vel()

def main():
    global tf_buffer
    global tf_listener
    global publisher
    global last_message
    global LASER_SCAN_TIMEOUT
    global MAX_THETA_DOT

    rospy.init_node("master")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
    rospy.Subscriber('/odom', Odometry, self_odom_callback)
    rospy.Subscriber('/enemy_vo', Odometry, enemy_odom_callback)

    last_message = rospy.Time.now().secs + rospy.Time.now().nsecs / 1000000000
    
    while not rospy.is_shutdown():
        if rospy.Time.now().secs + rospy.Time.now().nsecs / 1000000000 - last_message > LASER_SCAN_TIMEOUT:
            cmd_vel = Twist()
            cmd_vel.angular.z = MAX_THETA_DOT
            cmd_vel.linear.x = 0

            publisher.publish(cmd_vel)
        rospy.sleep(rospy.Duration(0.02))

    rospy.spin()

if __name__ == '__main__':
    main()
