#!/usr/bin/env python

import math
import time

import rospy
from geometry_msgs.msg import Twist, Pose, Vector3
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from interfearence_controllers_generic_controller.generic_controller import GenericController
import tf2_ros
import tf2_geometry_msgs


class RushEnemyController(GenericController):
    """
    A controller which aims to rush down the enemy robot as fast as
    possible by driving straight at them.
    """
    def __init__(self):
        """
        Standard init function creating necessary variables and subscribers.
        """
        # First we have to initialise the super class
        GenericController.__init__(self)
        
        # Now create class variables
        try:
            tf_prefix = rospy.get_param("tf_prefix")
            self.BASE_FRAME = tf_prefix + "/base_link"
        except KeyError:
            self.BASE_FRAME = "base_link"

        self.MAX_VEL = 0.5
        self.MAX_THETA_DOT = 9.425
        self.LASER_SCAN_TIMEOUT = 100000

        self.enemy_pose = Pose()
        self.enemy_updated = False
        self.our_pose = Pose()
        self.our_velocity = Twist()
        self.us_updated = False

        # Init tf and subscribers
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=2)
        rospy.Subscriber('odometry/measured', Odometry, self.self_odom_callback)
        rospy.Subscriber('enemy_vo', Odometry, self.enemy_odom_callback)

        self.last_message = rospy.Time.now().secs + rospy.Time.now().nsecs / 1000000000

    def reset(self):
        """
        Reset variables and time-stuff for a new simulation.
        """
        # Create a target of 0 as our output
        cmd_vel = Twist()
        self.publisher.publish(cmd_vel)

        # Reset timer
        self.last_message = rospy.Time.now().secs + rospy.Time.now().nsecs / 1000000000

        # Reset tf
        self.tf_buffer.clear()


    def update(self):
        """
        Non-blocking control loop, called at regular intervals.

        Overloading base class
        """
        if rospy.Time.now().secs + rospy.Time.now().nsecs / 1000000000 - self.last_message > self.LASER_SCAN_TIMEOUT:
            cmd_vel = Twist()
            cmd_vel.angular.z = MAX_THETA_DOT
            cmd_vel.linear.x = 0

            publisher.publish(cmd_vel)


    def get_name(self):
        """
        Default name of the node, if no name is specified.

        Overloading base class.
        """
        return "rush_enemy"


    def mag(self, vec):
        return math.sqrt(vec.x*vec.x + vec.y*vec.y + vec.z*vec.z)


    def dot(self, a, b):
        return a.x*b.x + a.y*b.y + a.z*b.z


    def publish_cmd_vel(self):
        if self.RESET:
            return

        if self.us_updated and self.enemy_updated:
            # Calculate vector between us and them
            vec = Vector3()
            vec.x = self.enemy_pose.position.x# - our_pose.position.x
            vec.y = self.enemy_pose.position.y# - our_pose.position.y
            vec.z = self.enemy_pose.position.z# - our_pose.position.z

            # Normalise
            m = self.mag(vec)
            if m != 0:
                vec.x = vec.x / m
                vec.y = vec.y / m
                vec.z = vec.z / m

            # Calculate the angle we need to turn
            forwards = Vector3()
            forwards.x = 1
            # acos ranges from 0 to pi
            theta = math.acos(self.dot(forwards, vec))

            # Maximum turning speed when the angle is greater than 20 degrees
            theta_dot = 0.35 * theta / self.MAX_THETA_DOT 
            if theta > 0.35:
                theta_dot = self.MAX_THETA_DOT

            # The linear velocity scales proportionally inversely with the
            # angular velocity
            linear_vel = self.MAX_VEL*(1 - theta_dot/self.MAX_THETA_DOT)

            # Account for negative theta, we want to turn TOWARDS them
            if vec.y < 0:
                theta_dot = -theta_dot

            cmd_vel = Twist()
            cmd_vel.angular.z = theta_dot
            cmd_vel.linear.x = linear_vel

            self.publisher.publish(cmd_vel)

            self.us_updated = False
            self.enemy_updated = False


    def enemy_odom_callback(self, msg):
        self.last_message = rospy.Time.now().secs + rospy.Time.now().nsecs / 1000000000

        self.enemy_pose = msg.pose.pose

        if msg.header.frame_id != self.BASE_FRAME:
            # Transform enemy pose into our co-ordinate frame
            try:
                trans = self.tf_buffer.lookup_transform(msg.header.frame_id,
                                                   self.BASE_FRAME,
                                                   rospy.Time(0))
                self.enemy_pose = tf2_geometry_msgs.do_transform_pose(
                    msg.pose, trans).pose
            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logerr(e)
                return

        self.enemy_updated = True

        self.publish_cmd_vel()


    def self_odom_callback(self, msg):
        self.our_pose = msg.pose.pose
        self.our_twist = msg.twist.twist
        self.us_updated = True

        self.publish_cmd_vel()


if __name__ == '__main__':
    controller = RushEnemyController()
    controller.main()
