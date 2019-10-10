#!/usr/bin/env python

import time
import rospy
from std_msgs.msg import Bool, String
from std_srvs.srv import Empty as SrvEmpty
from gazebo_msgs.srv import GetModelState, SetModelState, GetLinkState, SetLinkState
from gazebo_msgs.msg import ModelState
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import Twist

if __name__ == '__main__':
    rospy.init_node("sumobot_tester")

    iterations = rospy.get_param("~iterations")
    robot_1 = rospy.get_param("~robot_ns_1")
    robot_2 = rospy.get_param("~robot_ns_2")

    # Wait for Gazebo to be started
    rospy.sleep(0.01)

    rospy.wait_for_service("gazebo/reset_world")
    rospy.wait_for_service("gazebo/get_model_state")
    rospy.wait_for_service("gazebo/set_model_state")
    rospy.wait_for_service(robot_1+"/controller_manager/switch_controller")
    rospy.wait_for_service(robot_2+"/controller_manager/switch_controller")
    gazebo_reset = rospy.ServiceProxy("gazebo/reset_world", SrvEmpty)
    robot_pos = rospy.ServiceProxy("gazebo/get_model_state", GetModelState)
    set_robot_pos = rospy.ServiceProxy("gazebo/set_model_state", SetModelState)
    reset_1 = rospy.ServiceProxy(
        robot_1+"/controller_manager/switch_controller", SwitchController)
    reset_2 = rospy.ServiceProxy(
        robot_2+"/controller_manager/switch_controller", SwitchController)

    winner_pub = rospy.Publisher("/winner", String, queue_size=2)

    def reset_controllers():
        reset_1([], 
                ["interfearence/controller/state/joint_state_controller",
                 "interfearence/controller/velocity/left_velocity_controller",
                 "interfearence/controller/velocity/right_velocity_controller"],
                1)
        reset_2([], 
                ["interfearence/controller/state/joint_state_controller",
                 "interfearence/controller/velocity/left_velocity_controller",
                 "interfearence/controller/velocity/right_velocity_controller"],
                1)

        reset_1(["interfearence/controller/state/joint_state_controller",
                 "interfearence/controller/velocity/left_velocity_controller",
                 "interfearence/controller/velocity/right_velocity_controller"],
                [],
                1)
        reset_2(["interfearence/controller/state/joint_state_controller",
                 "interfearence/controller/velocity/left_velocity_controller",
                 "interfearence/controller/velocity/right_velocity_controller"],
                [],
                1)

    def reset_model(robot, pose):
        model_state = ModelState()
        model_state.model_name = robot
        model_state.pose = pose
        model_state.twist = Twist()
        model_state.reference_frame = "world"
        set_robot_pos(model_state)

    global start_time
    start_time = rospy.Time.now()

    def reset_sim():
        global start_time
        # Reset controllers, pause the sim, reset the world, 
        # unpause the sim and then release the controllers
        winner_pub.publish(win_msg)
        reset_msg.data = True
        reset_pub.publish(reset_msg)
        gazebo_reset()
        reset_model(robot_1, robot_1_initial_pose)
        reset_controllers()
        rospy.sleep(rospy.Duration(0.1))
        reset_msg.data = False
        reset_pub.publish(reset_msg)
        start_time = rospy.Time.now()

    reset_pub = rospy.Publisher("/reset", Bool, queue_size=1)
    reset_msg = Bool()
    # Make sure all controllers are in the reset state
    reset_msg.data = True
    reset_pub.publish(reset_msg)

    rospy.sleep(rospy.Duration(0.5))

    robot_1_initial_pose = robot_pos(robot_1, "world").pose
    robot_2_initial_pose = robot_pos(robot_2, "world").pose

    sleeper = rospy.Rate(20)
    robot_1_wins = 0
    robot_2_wins = 0
    draws = 0
    it = 0
    win_msg = String()

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
                    win_msg.data = robot_2
                    rospy.loginfo("{0} wins".format(robot_2))
                elif robot_1_pose.pose.position.z > robot_2_pose.pose.position.z:
                    robot_1_wins += 1
                    win_msg.data = robot_1
                    rospy.loginfo("{0} wins".format(robot_1))
                else:
                    draws += 1
                    win_msg.data = 'draw'
                    rospy.loginfo("draw")
                it += 1
                reset_sim()
                
            elif robot_2_pose.pose.position.z <= 0:
                robot_1_wins += 1
                win_msg.data = robot_1
                it += 1
                rospy.loginfo("{0} wins".format(robot_1))
                reset_sim()
            
            # 20s timeout
            elif (rospy.Time.now() - start_time).secs >= 20:
                draws += 1
                win_msg.data = 'draw'
                rospy.loginfo('draw')
                it += 1
                reset_sim()

            sleeper.sleep()
    except rospy.exceptions.ROSInterruptException:
        pass
    rospy.logfatal(
        "\n{5}\n{0}: {1} wins. {2}: {3} wins. {4} draws.\n{5}\n".format(
            robot_1, robot_1_wins, robot_2, robot_2_wins, draws,
            "=================================================="))
