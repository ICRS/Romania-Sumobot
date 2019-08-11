import numpy
import math
import time

from gym import spaces, envs
from interfearence_openai.robot_envs import interfearence_env
from gym.envs.registration import register

import rospy
from geometry_msgs.msg import Pose, TransformStamped


class InterfearencePushEnemyEnv(interfearence_env.InterfearenceEnv):
    def __init__(self):
        """Make Interfearence learn how to push an enemy out of a ring
        """

        """We consider 5 potential actions
        1-2) Increment/Decrement x velocity
        3-4) Increment/Decrement theta velocity
        5) Continue at current velocity
        """
        rospy.logdebug("Start InterfearencePushEnemyEnv INIT...")
        self.action_space = spaces.Discrete(4)

        self.reward_range = (-numpy.inf, numpy.inf)

        self.vel_x = 0
        self.vel_theta = 0

        super(InterfearencePushEnemyEnv, self).__init__()

        # Params
        self.max_x_vel = 1.5
        self.max_theta_vel = 5
        self.x_vel_increment_value = 0.01#0.05
        self.theta_vel_increment_value = 0.1# 0.1

        self.enemy_z_height_threshold = 0.02
        self.z_height_threshold = 0.02

        self.enemy_name = 'box'
        self.dec_obs = 4

        # Observation states
        # 1) (measured) x_velocity
        # 2) (measured) theta_velocity
        # 3) (measured) x
        # 4) (measured) y
        # 5) (measured) yaw
        # 6...) (measured) raw laser scan data

        laser_scan = self.get_laser_scan()
        rospy.logdebug("laser_scan len===>"+str(len(laser_scan.ranges)))

        arr = []
        arr.append(self.max_x_vel)
        arr.append(self.max_theta_vel)
        arr.append(100)
        arr.append(100)
        arr.append(100)

        for i in range(len(laser_scan.ranges)):
            arr.append(laser_scan.range_max)

        high = numpy.array(arr)
        
        arr = []
        arr.append(-self.max_x_vel)
        arr.append(-self.max_theta_vel)
        arr.append(-100)
        arr.append(-100)
        arr.append(-100)

        for i in range(len(laser_scan.ranges)):
            arr.append(laser_scan.range_min)

        low = numpy.array(arr)

        self.observation_space = spaces.Box(low, high)

        rospy.logdebug("ACTION SPACES TYPE===>"+str(self.action_space))
        rospy.logdebug("OBSERVATION SPACES TYPE===>"+str(self.observation_space))

        rospy.logdebug("END InterfearencePushEnemyEnv INIT...")

    def _set_init_pose(self):
        """
        Sets the robot in it's initial state. Preparing to reset the world.
        """

        self.move(0, 0)
        self.vel_x = 0
        self.vel_theta = 0

        now = rospy.Time.now()
        self.last_move_time = rospy.Time.now().secs
        self.last_move_x = 0
        self.last_move_y = 0
        self.last_move_yaw = 0

        self.start_time = now.secs + now.nsecs / 1000000000.0

        return True

    def _init_env_variables(self):
        """
        Inits variables that need to be initialised for each episode.
        """

        self.cumulated_reward = 0.0
        self._episode_done= False

        time.sleep(0.5)
    
    def _set_action(self, action):
        """
        Sets the velocities of the robot based on the action integer given.
        """
        rospy.logdebug("Start Set Action ==>"+str(action))

        self.last_action = action

        if action == 0: # Increment x velocity
            self.vel_x += self.x_vel_increment_value
            rospy.logdebug("Incrementing x velocity to " + str(self.vel_x))
        elif action == 1: # Decrement x velocity
            self.vel_x -= self.x_vel_increment_value
            rospy.logdebug("Decrementing x velocity to " + str(self.vel_x))
        elif action == 2: # Increment theta velocity
            self.vel_theta += self.theta_vel_increment_value
            rospy.logdebug("Incrementing theta velocity to " + str(self.vel_theta))
        elif action == 3: # Decrement theta velocity
            self.vel_theta -= self.theta_vel_increment_value
            rospy.logdebug("Decrementing theta velocity to " + str(self.vel_theta))
        elif action == 4: # Dont change velocities
            rospy.logdebug("Maintaining current velocities")

        rospy.logdebug("START ACTION EXECUTE>>>"+str(action))
        self.move(self.vel_x, self.vel_theta)
        rospy.logdebug("END ACTION EXECUTE>>>"+str(action))

        rospy.logdebug("END Set Action ==>"+str(action))

    def _get_obs(self):
        """
        Here we define what sensor data defines our robot's observations.
        From this we then need to calculate the state of the robot in order
        to learn. This is an array of:
        1) (measured) x_velocity
        2) (measured) theta_velocity
        3) (measured) x
        4) (measured) y
        5) (measured) yaw
        6...) (measured) raw laser scan data
        """

        odom = self.get_odom()
        x_vel = math.sqrt(pow(odom.twist.twist.linear.x, 2) +
                          pow(odom.twist.twist.linear.y, 2))
        theta_vel = odom.twist.twist.linear.z

        position = odom.pose.pose.position
        yaw = 2*math.acos(odom.pose.pose.orientation.w)

        laser_scan = self.get_laser_scan()

        observation = []
        observation.append(round(x_vel, self.dec_obs))
        observation.append(round(theta_vel, self.dec_obs))
        observation.append(round(position.x, self.dec_obs))
        observation.append(round(position.y, self.dec_obs))
        observation.append(round(yaw, self.dec_obs))
        for point in laser_scan.ranges:
            if point < laser_scan.range_min:
                observation.append(
                    round(laser_scan.range_max, self.dec_obs))
            elif point > laser_scan.range_max:
                observation.append(
                    round(laser_scan.range_max, self.dec_obs))
            else:
                observation.append(round(point, self.dec_obs))
#
        return observation

    def _is_done(self, observations):
        """
        We are done if:
        1) The enemy has fallen out of the ring
        2) We have fallen out of the ring
        3) Wehaven't moved from where we were for the last 5 seconds
        """

        current_pose = self.get_own_state().pose
        if current_pose.position.z < self.z_height_threshold:
            rospy.logerr("We fell out of the ring.")
            self.is_dead = True
            return True
        if self.get_enemy_state(self.enemy_name).pose.position.z < self.enemy_z_height_threshold:
            rospy.logerr("We pushed the enemy out of the ring!")
            self.is_dead = False
            return True
        now = rospy.Time.now()
        if now.secs + now.nsecs / 1000000000.0 - self.start_time > 40:
            rospy.logerr("Time ran out!")
            self.is_dead = True
            return True

        EPSILON = 0.01
        EPSILON_YAW = 0.05
        yaw = 2.0*math.acos(current_pose.orientation.w)
        if(abs(current_pose.position.x - self.last_move_x) < EPSILON and
           abs(current_pose.position.y - self.last_move_y) < EPSILON and
           abs(yaw - self.last_move_yaw) < EPSILON):
            if rospy.Time.now().secs - self.last_move_time >= 5:
                rospy.logdebug("We didn't move enough!")
                self.is_dead = True
                return True
        else:
            self.last_move_x = current_pose.position.x
            self.last_move_y = current_pose.position.y
            self.last_move_yaw = yaw
            self.last_move_time = rospy.Time.now().secs

        return False

    def _compute_reward(self, observations, done):
        """
        TODO: Work out how to calculate the reward - it's quite arbitrary
        """
        # Decrease reward the longer it takes to push the enemy out of the 
        # ring
        reward = 0
        # If we died then the penalty is large
        if done and self.is_dead:
            reward = -1000
            return reward
        # If we won then the reward is large
        elif done and not self.is_dead:
            now = rospy.Time.now()
            reward = 1000 * (40 - now.secs + now.nsecs / 1000000000.0 - self.start_time)
            return reward

        enemy_pos = self.get_enemy_state(self.enemy_name).pose.position
        our_state = self.get_own_state()

        dist = math.sqrt(pow(enemy_pos.x-our_state.pose.position.x, 2) +
                         pow(enemy_pos.y-our_state.pose.position.y, 2))
        reward = 1.5-dist

        reward = reward * (1+our_state.twist.linear.x)

        return reward
