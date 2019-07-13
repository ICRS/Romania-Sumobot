#! /usr/bin/python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

LEFT_CONTROLLER_COMMAND_TOPIC = "/interfearence/controller/velocity/left_velocity_controller/command"
RIGHT_CONTROLLER_COMMAND_TOPIC = "/interfearence/controller/velocity/right_velocity_controller/command"


left_pub = rospy.Publisher(LEFT_CONTROLLER_COMMAND_TOPIC, 
                           Float64, 
                           queue_size=1)

right_pub = rospy.Publisher(RIGHT_CONTROLLER_COMMAND_TOPIC, 
                            Float64, 
                            queue_size=1)

# Parameters to be filled from the parameter server
wheel_radius = 0
wheel_separation = 0

def cmd_vel_cb(msg):
    global wheel_radius
    global wheel_separation

    # Ignore any commands other than x-translation and z-rotation
    Vl = (msg.linear.x-msg.angular.z*wheel_separation/2.0) / wheel_radius
    Vr = (msg.linear.x+msg.angular.z*wheel_separation/2.0) / wheel_radius

    left_msg = Float64()
    left_msg.data = Vl
    right_msg = Float64()
    right_msg.data = Vr

    left_pub.publish(left_msg)
    right_pub.publish(right_msg)

if __name__ == '__main__':
    global wheel_radius
    global wheel_separation

    rospy.init_node("velocity_control")

    wheel_radius = rospy.get_param("~wheel_radius")
    wheel_separation = rospy.get_param("~wheel_separation")

    rospy.Subscriber("/cmd_vel", Twist, cmd_vel_cb)

    rospy.spin()

