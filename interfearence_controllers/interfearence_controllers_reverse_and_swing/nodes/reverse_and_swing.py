#!/usr/bin/env python

import rospy # for logerr
import math

from interfearence_controllers_generic_controller.basic_controller import BasicController

# Different states the overall controller can be in
REVERSING = 0
SWEEPING = 1
ATTACKING = 2


class ReverseAndSwingController(BasicController):
    """ 
    A controller which follows the edge of the arena to attempt to swing
    behind the enemy robot before attacking it from behind.
    """
    def __init__(self):
        """
        Init function
        """
        # First initialise the super class
        BasicController.__init__(self)

        # Variables used by us
        self.state = REVERSING
        self.first_sweep = True

        # When the yaw of the enemy robot is greater than this we switch to
        # attack mode
        self.YAW_THRESHOLD = 1.4
        # Project enemy pose 0.2s into the future
        self.PROJECTION_TIME = 0.2
        # When the projected enemy pose is closer than this to us then
        # we switch to attack mode no matter what state we're in.
        # This lets us account for aggressive enemy robots, or bad yaw
        # estimates.
        self.ENEMY_PROXIMITY_THRESHOLD = 0.3
        # Reversing velocity in m/s. Needs to be negative or we'll go
        # forwards...
        self.REVERSE_VELOCITY = -0.5

        # Speed at which we turn when sweeping. radians/s
        self.SWEEPING_TURNING_SPEED = 3.1415926
        # Velocity at which we move forwards when sweeping. meters/s
        self.SWEEPING_FORWARDS_SPEED = 0.5

        # Proportional controller for yaw velocity when attacking
        self.ATTACK_YAW_P_CONSTANT = 70
        # Further than this distance from the enemy we move at minimum speed
        self.ATTACK_MIN_SPEED_DISTANCE = 0.7
        # Minimum speed for movement
        self.ATTACK_MIN_SPEED = 0.1
        # Maximum speed for movement
        self.ATTACK_MAX_SPEED = 2.0

    def reset(self):
        """
        Reset everything
        """
        self.state = REVERSING
        self.first_sweep = True

        # Reset the super class
        BasicController.reset(self)
    
    def update(self):
        """
        Update loop called at ~ 50 Hz
        """
        # Check whether the state has changed
        self.process_state_machine_changes()
        
        # Run the update for whichever state we're in
        if self.state == REVERSING:
            self.process_reversing_state()
        elif self.state == SWEEPING:
            self.process_sweeping_state()
        elif self.state == ATTACKING:
            self.process_attacking_state()
        else:
            raise RuntimeError("Invalid controller state: %d", self.state)

        # Update the super class
        BasicController.update(self)

    def get_name(self):
        return "reverse_and_swing"

    def process_reversing_state(self):
        self.cmd_vel.linear.x = self.REVERSE_VELOCITY
        self.cmd_vel.angular.z = 0

    def process_sweeping_state(self):
        # If we're moving backwards then decelerate quick and move forwards
        # slightly
        if self.our_velocity.linear.x < 0:
            self.cmd_vel.linear.x = 0.1
            return
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0
        # If they're on our left we want to move right
        if self.enemy_pose.position.y > 0:
            if self.first_sweep:
                if self.front_right_edge:
                    self.cmd_vel.angular.z = self.SWEEPING_TURNING_SPEED
                    self.first_sweep = False
                else:
                    self.cmd_vel.angular.z = -self.TURNING_SPEED
            else:
                if self.front_right_edge:
                    self.cmd_vel.angular.z = self.TURNING_SPEED
                else:
                    self.cmd_vel.linear.x = self.SWEEPING_FORWARDS_SPEED
        # Otherwise we move left
        else:
            if self.first_sweep:
                if self.front_left_edge:
                    self.cmd_vel.angular.z = -self.TURNING_SPEED
                    self.first_sweep = False
                else:
                    self.cmd_vel.angular.z = self.TURNING_SPEED
            else:
                if self.front_left_edge:
                    self.cmd_vel.angular.z = -self.TURNING_SPEED
                else:
                    self.cmd_vel.linear.x = self.SWEEPING_FORWARDS_SPEED
            
    def process_attacking_state(self):
        # The goal now is to turn and accurately drive into the enemy robot
        # at maximum speed.

        # We want to then be able to keep the enemy robot in our centre
        # so that they cannot slip away from our grasp

        # Enemy pose is relative to us, so the goal is to have y be 0 and
        # then have x become 0
        self.cmd_vel.angular.z = (self.ATTACK_YAW_P_CONSTANT
                                * self.enemy_pose.position.y)

        # Forwards velocity is proportional to how close the enemy is
        # from us
        dist = self.project_enemy_distance(project_time=False)
        self.cmd_vel.linear.x = (self.ATTACK_MAX_SPEED 
                               * (self.ATTACK_MIN_SPEED_DISTANCE - dist))
        if self.cmd_vel.linear.x < self.ATTACK_MIN_SPEED:
            self.cmd_vel.linear.x = self.ATTACK_MIN_SPEED

    def process_state_machine_changes(self):
        # Once in the attacking state there is no return
        if self.state == ATTACKING:
            return
        # Standard state machine options
        if self.state == REVERSING:
            if self.rear_left_edge:
                self.state = SWEEPING
                rospy.logerr("State changed to SWEEPING")
            elif self.rear_right_edge:
                self.state = SWEEPING
                rospy.logerr("State changed to SWEEPING")
        elif self.state == SWEEPING:
            yaw = 2 * math.acos(self.enemy_pose.orientation.w)
            if abs(yaw) < self.YAW_THRESHOLD:
                self.state = ATTACKING
                rospy.logerr("State changed to ATTACKING")
        else:
            raise RuntimeError("Invalid controller state: %d", self.state)
        # If we're not in the attacking state there are some direct
        # jumps to attack state
        if self.enemy_pose.position.x < 0:
            self.state = ATTACKING
            rospy.logerr("State changed to ATTACKING")
        if self.project_enemy_distance() < self.ENEMY_PROXIMITY_THRESHOLD:
            self.state = ATTACKING
            rospy.logerr("State changed to ATTACKING")

    def project_enemy_distance(self, project_time=True):
        enemy_yaw = 2 * math.acos(self.enemy_pose.orientation.w)
        dx = self.enemy_velocity.linear.x * math.cos(enemy_yaw)
        dy = self.enemy_velocity.linear.x * math.sin(enemy_yaw)
        enemy_pos = self.enemy_pose.position
        if project_time:
            enemy_pos.x += dx / self.PROJECTION_TIME
            enemy_pos.y += dy / self.PROJECTION_TIME
        return math.sqrt(enemy_pos.x ** 2 + enemy_pos.y ** 2)

if __name__ == '__main__':
    controller = ReverseAndSwingController()
    controller.main()
