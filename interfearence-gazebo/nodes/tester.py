#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Bool
from std_srvs.srv import Empty as SrvEmpty
from gazebo_msgs.srv import GetModelState, GetLinkState, SetLinkState
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node("sumobot tester")

    iterations = rospy.get_param("~iterations")
    robot_1 = rospy.get_param("~robot_ns_1")
    robot_2 = rospy.get_param("~robot_ns_2")

    rospy.wait_for_service("gazebo/reset_simulation")
    rospy.wait_for_service("gazebo/get_model_state")
    rospy.wait_for_service("gazebo/unpause_physics")
    rospy.wait_for_service("gazebo/pause_physics")
    rospy.wait_for_service("gazebo/get_link_state")
    rospy.wait_for_service("gazebo/set_link_state")
    gazebo_reset = rospy.ServiceProxy("gazebo/reset_simulation", SrvEmpty)
    robot_pos = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
    gazebo_unpause = rospy.ServiceProxy("gazebo/unpause_physics", SrvEmpty)
    gazebo_pause = rospy.ServiceProxy("gazebo/pause_physics", SrvEmpty)
    get_link = rospy.ServiceProxy("gazebo/get_link_state", GetLinkState)
    set_link = rospy.ServiceProxy("gazebo/set_link_state", SetLinkState)

    def reset_link_speed(robot, link):
        link_state = get_link(robot+"::"+link, robot+"::base_link").link_state
        link_state.twist = Twist() # Reset speed to 0
        set_link(link_state)

    def reset_sim():
        # Reset controllers, pause the sim, reset the world, 
        # unpause the sim and then release the controllers
        reset_msg.data = True
        reset_pub.publish(reset_msg)
        rospy.sleep(rospy.Duration(0.1))
#        reset_link_speed(robot_1, "left_wheel")
#        reset_link_speed(robot_1, "right_wheel")
#        reset_link_speed(robot_2, "left_wheel")
#        reset_link_speed(robot_2, "right_wheel")
        gazebo_pause()
        gazebo_reset()
        #time.sleep(0.1) # Using wall time as ROS time is paused
        gazebo_unpause()
        reset_msg.data = False
        reset_pub.publish(reset_msg)

    reset_pub = rospy.Publisher("/reset", Bool)
    reset_msg = Bool()
    # Make sure all controllers are in the reset state
    reset_msg.data = True
    reset_pub.publish(reset_msg)

    rospy.sleep(rospy.Duration(0.5))

    sleeper = rospy.Rate(20)
    robot_1_wins = 0
    robot_2_wins = 0
    draws = 0
    it = 0

    # The initial paused state can be set off by the user and then we take
    # control
    reset_msg.data = False
    reset_pub.publish(reset_msg)
    try:
        while not rospy.is_shutdown() and it < iterations:
            # Get the poses of both robots
            robot_1_pose = robot_pos(robot_1, "world")
            robot_2_pose = robot_pos(robot_2, "world")

            if robot_1_pose.pose.position.z <= 0:
                if robot_1_pose.pose.position.z < robot_2_pose.pose.position.z:
                    robot_2_wins += 1
                    rospy.loginfo("{0} wins".format(robot_2))
                elif robot_1_pose.pose.position.z > robot_2_pose.pose.position.z:
                    robot_1_wins += 1
                    rospy.loginfo("{0} wins".format(robot_1))
                else:
                    draws += 1
                    rospy.loginfo("draw")
                it += 1
                reset_sim()
                
            elif robot_2_pose.pose.position.z <= 0:
                robot_1_wins += 1
                it += 1
                rospy.loginfo("{0} wins".format(robot_1))
                reset_sim()

            sleeper.sleep()
    except rospy.exceptions.ROSInterruptException:
        pass
    rospy.logfatal(
        "\n{5}\n{0}: {1} wins. {2}: {3} wins. {4} draws.\n{5}\n".format(
            robot_1, robot_1_wins, robot_2, robot_2_wins, draws,
            "=================================================="))
