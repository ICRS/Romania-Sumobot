import rospy
from std_msgs.msg import Bool


class GenericController(object):
    """
    An abstract, generic controller which other controllers can inherit from
    Just fill in the NotImplemented methods and add methods of your own
    """
    def __init__(self):
        """
        Initialise ROS etc. init this before doing the rest of your own 
        initialisation with GenericController().__init__(self)
        """
        # Start in the reset state
        self.RESET = True

        rospy.init_node(self.get_name())

        self._rate = rospy.Rate(50)

        # Subscribe to the reset topic
        rospy.Subscriber("/reset", Bool, self._reset_callback)


    def main(self):
        """
        The main rospy loop
        Call this in your main function
        """
        # Run the main control loop
        while not rospy.is_shutdown():
            if self.RESET:
                self.reset()
            else:
                self.update()
            # Handle simulation reset
            try:
                self._rate.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                pass


    def reset(self):
        """
        Fill this in with initialisation / reset stuff.
        Remember to reset any time-based things (e.g. TF) as Gazebo will
        break you if you don't!
        """
        raise NotImplementedError


    def update(self):
        """
        Fill this in with your (non-blocking) control loop.
        E.G. cmd_vel publishing, etc...
        """
        raise NotImplementedError


    def get_name(self):
        """
        Whatever you want your node to be called goes here.
        Returns a string.
        """
        raise NotImplementedError
    
    
    def _reset_callback(self, msg):
        """
        When we're in the reset state nothing else should be allowed to
        happen.
        """

        self.RESET = msg.data
