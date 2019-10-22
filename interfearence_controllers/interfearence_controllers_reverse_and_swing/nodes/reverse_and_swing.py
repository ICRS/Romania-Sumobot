#!/usr/bin/env python

from interfearence_controllers_generic_controller.basic_controller import BasicController

# Different states the overall controller can be in
REVERSING = 0
SWEEPING = 1
ATTACKING = 2

# States the motion controller can be in
FORWARDS = 10
TURNING_LEFT = 11
TURNING_RIGHT = 12
REVERSING = 13


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

    def reset(self):
        """
        Reset everything
        """
        self.state = REVERSING

        # Reset the super class
        BasicController.reset(self)
    
    def update(self):
        """
        Update loop called at ~ 50 Hz
        """
        # TODO: update
        
        self.cmd_vel.linear.x = 0
        self.cmd_vel.angular.z = 0

        # Update the super class
        BasicController.update(self)

    def get_name(self):
        return "reverse_and_swing"

if __name__ == '__main__':
    ReverseAndSwingController controller
    controller.main()
