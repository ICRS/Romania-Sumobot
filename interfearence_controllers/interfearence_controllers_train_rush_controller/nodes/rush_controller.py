#!/usr/bin/env python3
import math

import rospy
import rospkg

from std_msgs.msg import String, Float64
from geometry_msgs.msg import Twist
from gazebo_msgs.srv import GetModelState

from interfearence_controllers_generic_controller.basic_controller import BasicController

from tensorforce.agents import Agent

class RushEnemyController(BasicController):
    """
    A controller which aims to rush down the enemy robot as fast as 
    possible by driving straight at them. Using reinforcement learning
    to develop a strategy to follow.
    """
    def __init__(self):
        """
        Init function
        """
        # First initialise the super class
        BasicController.__init__(self)

        # Check if it's a simulation
        try:
            if rospy.get_param("/use_sim_time"):
                self.in_simulation = True
                self.pose_reader = rospy.ServiceProxy(
                    "/gazebo/get_model_state", GetModelState)
            else:
                self.in_simulation = False
        except KeyError:
            self.in_simulation = False

        self.won = None

        rospack = rospkg.RosPack()
        path = rospack.get_path(
            'interfearence_controllers_train_rush_controller') + '/results'

        self.agent = Agent.create(
            agent='ppo',
            states=dict(type='float', shape=(10,)),
            actions=dict(type='int', num_values=9),
            max_episode_timesteps=30*50,
            #memory=10000,
            saver=dict(directory=path, filename='rush_controller'),
            network='auto',
            estimate_terminal=True
        )

        self.agent.initialize()

        rospy.Subscriber('/winner', String, self.winner_cb)
        self.score_pub = rospy.Publisher('score', Float64, queue_size=5)
        self.reward_pub = rospy.Publisher('reward', Float64, queue_size=10)
        self.iterations = 0
        self.total_score = 0

    def reset(self):
        """
        Reset variables and time-stuff for a new simulation.
        """
        if self.won != None:
            state = self.get_current_state()
            action = self.agent.act(states=state)
            self.execute_decision(action)
            reward, terminal = self.calculate_reward()
            self.agent.observe(reward=reward, terminal=terminal)
            score_msg = Float64()
            score_msg.data = self.total_score
            self.score_pub.publish(score_msg)
            self.total_score = 0
            self.iterations = 0
            self.agent.reset()

        # Run the reset function of the super class
        BasicController.reset(self)

        cmd_vel = Twist()
        self.publisher.publish(cmd_vel)

    def update(self):
        """
        Non-blocking control loop, called at regular intervals.
        """
        self.iterations += 1

        # Learn
        state = self.get_current_state()
        action = self.agent.act(states=state)
        self.execute_decision(action)
        reward, terminal = self.calculate_reward()
        self.agent.observe(reward=reward, terminal=terminal)

        # Run the update function of the super class
        BasicController.update(self)

    def get_name(self):
        return "deep_rush_enemy"

    def get_current_state(self):
        state = []
        state.append(self.our_velocity.linear.x)
        state.append(self.our_velocity.angular.z)
        state.append(self.enemy_pose.position.x)
        state.append(self.enemy_pose.position.y)
        state.append(self.enemy_velocity.linear.x)
        state.append(self.enemy_velocity.linear.y)
        state.append(float(self.front_left_edge))
        state.append(float(self.front_right_edge))
        state.append(float(self.rear_left_edge))
        state.append(float(self.rear_right_edge))
        return state

    def execute_decision(self, action):
        if action == 0:
            self.cmd_vel.linear.x += 0.1
        elif action == 1:
            self.cmd_vel.linear.x += 0.5
        elif action == 2:
            self.cmd_vel.linear.x -= 0.1
        elif action == 3:
            self.cmd_vel.linear.x -= 0.5
        elif action == 4:
            self.cmd_vel.angular.z += 0.1
        elif action == 5:
            self.cmd_vel.angular.z += 0.5
        elif action == 6:
            self.cmd_vel.angular.z -= 0.1
        elif action == 7:
            self.cmd_vel.angular.z -= 0.5
        elif action == 8:
            # Do nothing action
            pass

    def calculate_reward(self):
        if not self.in_simulation:
            return 0
        
        terminal = False

        if self.won != None:
            terminal = True
            reward = 0
            if self.won == True:
                # Big reward for winning
                reward = 1000 / self.iterations
            #else:
                # Small penalty for losing
            #    reward = -100
            rospy.loginfo("iterations: {}".format(self.iterations))
            self.won = None
            self.total_score += reward
            return reward, terminal
        
        enemy_prefix = ''
        our_prefix = ''
        if self.tf_prefix == 'test_bot_1/':
            enemy_prefix = 'test_bot_2'
            our_prefix = 'test_bot_1'
        elif self.tf_prefix == 'test_bot_2/':
            enemy_prefix = 'test_bot_1'
            our_prefix = 'test_bot_2'
        else:
            raise RuntimeError(
                "Robot names need to be 'test_bot_1' and 'test_bot_2'")
        
        our_real_pose = self.pose_reader(our_prefix, 'world').pose.position
        their_real_pose = self.pose_reader(enemy_prefix, 'world').pose.position

        # Reward based on how close we are to the enemy and how far the
        # enemy is from 0,0
        our_dist = math.sqrt(math.pow(our_real_pose.x-their_real_pose.x,2) +
                             math.pow(our_real_pose.y-their_real_pose.y,2))

        their_dist = math.sqrt(math.pow(their_real_pose.x, 2) +
                               math.pow(their_real_pose.y, 2))

        reward = (2*their_dist-our_dist)/self.iterations
        self.total_score += reward

        reward_msg = Float64()
        reward_msg.data = reward
        self.reward_pub.publish(reward_msg)

        return reward, terminal

    def winner_cb(self, msg):
        if msg.data + '/' == self.tf_prefix:
            self.won = True
        else:
            self.won = False
    
    def save_learning(self):
        self.agent.close()

if __name__ == '__main__':
    controller = RushEnemyController()
    try:
        controller.main()
    except:
        pass
    controller.save_learning()
