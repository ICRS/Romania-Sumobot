#! /usr/bin/python

import math

import rospy
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Float64, Bool
from control_msgs.msg import JointControllerState
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler as quat_from_eul
import tf2_ros

LEFT_CONTROLLER_COMMAND_TOPIC = "interfearence/controller/velocity/left_velocity_controller/command"
RIGHT_CONTROLLER_COMMAND_TOPIC = "interfearence/controller/velocity/right_velocity_controller/command"

LEFT_CONTROLLER_STATE_TOPIC = "interfearence/controller/velocity/left_velocity_controller/state"
RIGHT_CONTROLLER_STATE_TOPIC = "interfearence/controller/velocity/right_velocity_controller/state"

left_pub = None
right_pub = None

odom_pub = rospy.Publisher("odometry/measured", Odometry, queue_size=1)

# Parameters to be filled from the parameter server
wheel_radius = 0
wheel_separation = 0
tf_prefix = ""
publish_tf = True

# Used for odometry
x_pos = 0
y_pos = 0
yaw = 0

old_x_pos = 0
old_y_pos = 0
old_yaw = 0

last_odom_time = None

RESET = True

def cmd_vel_cb(msg):
    global wheel_radius
    global wheel_separation

    global RESET
    
    if RESET == True:
        return

    # Ignore any commands other than x-translation and z-rotation
    Vl = (msg.linear.x-msg.angular.z*wheel_separation/2.0) / wheel_radius
    Vr = (msg.linear.x+msg.angular.z*wheel_separation/2.0) / wheel_radius

    left_msg = Float64()
    left_msg.data = Vl
    right_msg = Float64()
    right_msg.data = Vr

    left_pub.publish(left_msg)
    right_pub.publish(right_msg)


def left_state_cb(msg):
    global x_pos
    global y_pos
    global yaw
    global wheel_radius
    global wheel_separation

    global RESET
    
    if RESET == True:
        return

    # We assume that the wheel has moved by v*dt and that the other has not
    # As such, the robot has moved on an arc centred about the other wheel
    motion = msg.process_value * msg.time_step * wheel_radius

    # As motion increases, yaw decreases
    dyaw = -math.atan2(motion, wheel_separation)
    
    # Use average yaw
    avg_yaw = yaw + dyaw / 2.0

    # Calculated delta x and delta y
    dcenter = motion / 2.0
    dx = dcenter * math.cos(avg_yaw)
    dy = -dcenter * math.sin(avg_yaw) # yaw is counter clockwise

    x_pos += dx
    y_pos += dy
    yaw += dyaw


def right_state_cb(msg):
    global x_pos
    global y_pos
    global yaw
    global wheel_radius
    global wheel_separation

    global RESET
    
    if RESET == True:
        return

    # We assume that the wheel has moved by v*dt and that the other has not
    # As such, the robot has moved on an arc centred about the other wheel
    motion = msg.process_value * msg.time_step * wheel_radius

    # As motion increases, yaw increases
    dyaw = math.atan2(motion, wheel_separation)
    
    # Use average yaw
    avg_yaw = yaw + dyaw / 2.0

    # Calculated delta x and delta y
    dcenter = motion / 2.0
    dx = dcenter * math.cos(avg_yaw)
    dy = -dcenter * math.sin(avg_yaw) # yaw is counter clockwise

    x_pos += dx
    y_pos += dy
    yaw += dyaw


def odom_cb():
    global publish_tf
    global tf_prefix
    global odom_pub
    global x_pos
    global y_pos
    global yaw
    global old_x_pos
    global old_y_pos
    global old_yaw
    global last_odom_time

    global RESET
    
    if RESET == True:
        return
    
    # First run through we have no dt
    if last_odom_time == None:
        last_odom_time = rospy.Time.now()
        return
    
    now = rospy.Time.now()

    dt = now - last_odom_time
    dt = dt.to_sec()

    if dt < 0.001:
        return

    dx = x_pos - old_x_pos
    dy = y_pos - old_y_pos
    dyaw = yaw - old_yaw

    old_x_pos += dx
    old_y_pos += dy
    old_yaw += dyaw

    dx /= dt
    dy /= dt
    dyaw /= dt
    
    odom = Odometry()
    odom.header.stamp = now
    odom.header.frame_id = tf_prefix + "/odom"
    odom.child_frame_id = tf_prefix + "/base_link"

    odom.pose.pose.position.x = old_x_pos
    odom.pose.pose.position.y = old_y_pos

    q = quat_from_eul(0, 0, old_yaw)

    odom.pose.pose.orientation.x = q[0]
    odom.pose.pose.orientation.y = q[1]
    odom.pose.pose.orientation.z = q[2]
    odom.pose.pose.orientation.w = q[3]

    odom.twist.twist.linear.x = dx
    odom.twist.twist.linear.y = dy
    odom.twist.twist.angular.z = dyaw

    odom.pose.covariance[0] = 1e-4
    odom.pose.covariance[7] = 1e-4
    odom.pose.covariance[35] = 1e-3

    odom.twist.covariance[0] = 1e-3
    odom.twist.covariance[7] = 1e-3
    odom.twist.covariance[35] = 1e-2

    odom_pub.publish(odom)

    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()

    t.header = odom.header
    t.child_frame_id = odom.child_frame_id
    t.transform.translation.x = odom.pose.pose.position.x
    t.transform.translation.y = odom.pose.pose.position.y
    t.transform.translation.z = odom.pose.pose.position.z
    t.transform.rotation = odom.pose.pose.orientation

    br.sendTransform(t)

def reset_cb(msg):
    global x_pos
    global y_pos
    global yaw
    global old_x_pos
    global old_y_pos
    global old_yaw
    global last_odom_time

    global RESET

    RESET = msg.data
    
    if msg.data == True:
        last_odom_time = None
        x_pos = 0
        y_pos = 0
        yaw = 0
        old_x_pos = 0
        old_y_pos = 0
        old_yaw = 0


if __name__ == '__main__':
    global wheel_radius
    global wheel_separation
    global publish_tf
    global tf_prefix
    global LEFT_CONTROLLER_STATE_TOPIC
    global RIGHT_CONTROLLER_STATE_TOPIC
    global LEFT_CONTROLLER_COMMAND_TOPIC
    global RIGHT_CONTROLLER_COMMAND_TOPIC
    global left_pub
    global right_pub

    rospy.init_node("velocity_control")

    wheel_radius = rospy.get_param("~wheel_radius")
    wheel_separation = rospy.get_param("~wheel_separation")
    publish_tf = rospy.get_param("~publish_tf")
    try:
        tf_prefix = rospy.get_param("tf_prefix")
    except KeyError:
        tf_prefix = ""

    try:
        sim = rospy.get_param("~simulation")
        if sim:
            LEFT_CONTROLLER_COMMAND_TOPIC = "interfearence/controller/sim/left_velocity_controller/command"
            RIGHT_CONTROLLER_COMMAND_TOPIC = "interfearence/controller/sim/right_velocity_controller/command"

            LEFT_CONTROLLER_STATE_TOPIC = "interfearence/controller/sim/left_velocity_controller/state"
            RIGHT_CONTROLLER_STATE_TOPIC = "interfearence/controller/sim/right_velocity_controller/state"

    except KeyError:
        pass

    left_pub = rospy.Publisher(LEFT_CONTROLLER_COMMAND_TOPIC, 
                               Float64, 
                               queue_size=1)
    right_pub = rospy.Publisher(RIGHT_CONTROLLER_COMMAND_TOPIC, 
                                Float64, 
                                queue_size=1)

    rospy.Subscriber("cmd_vel", Twist, cmd_vel_cb)
    rospy.Subscriber("/reset", Bool, reset_cb)

    # Listen to the controller measurements
    rospy.Subscriber(
        LEFT_CONTROLLER_STATE_TOPIC, JointControllerState, left_state_cb)
    rospy.Subscriber(
        RIGHT_CONTROLLER_STATE_TOPIC, JointControllerState, right_state_cb)

    # Odometry regular callback at 50 Hz
    #rospy.Timer(rospy.Duration(1.0/50.0), odom_cb)
    rate = rospy.Rate(50)
    
    while not rospy.is_shutdown():
        odom_cb()
        try:
            rate.sleep()
        except rospy.exceptions.ROSTimeMovedBackwardsException:
            pass
