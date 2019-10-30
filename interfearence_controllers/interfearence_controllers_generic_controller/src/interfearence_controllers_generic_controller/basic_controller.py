import rospy

from geometry_msgs.msg import Twist, Pose
from interfearence_msgs.msg import EdgeDetection
from nav_msgs.msg import Odometry

from interfearence_controllers_generic_controller.generic_controller import GenericController

class BasicController(GenericController):
    """
    A controller which abstracts ROS away from the user, such that they only
    need to focus on modifying the cmd_vel and using the measured data.
    """
    def __init__(self):
        """
        Init function
        """
        # First initialise the super class
        GenericController.__init__(self)

        # Create variables
        self.cmd_vel = Twist() # Our target cmd velocity

        # Things we've read from sensors
        self.enemy_pose = Pose() # Relative to us
        self.enemy_velocity = Twist() # Not accounting for our own speed
        self.our_velocity = Twist()
        self.front_left_edge = False
        self.front_right_edge = False
        self.rear_left_edge = False
        self.rear_right_edge = False

        # Create ROS subs and pubs
        self.publisher = rospy.Publisher('cmd_vel', Twist, queue_size=2)
        rospy.Subscriber(
            'odometry/measured', Odometry, self.self_odom_callback)
        rospy.Subscriber('enemy_vo', Odometry, self.enemy_odom_callback)
        rospy.Subscriber(
            'edge_sensors/front_left', EdgeDetection, self.edge_callback)
        rospy.Subscriber(
            'edge_sensors/front_right', EdgeDetection, self.edge_callback)
        rospy.Subscriber(
            'edge_sensors/rear_left', EdgeDetection, self.edge_callback)
        rospy.Subscriber(
            'edge_sensors/rear_right', EdgeDetection, self.edge_callback)

        try:
            self.tf_prefix = rospy.get_param('tf_prefix') + '/'
        except KeyError:
            self.tf_prefix = ''

    def reset(self):
        """
        Reset everything
        """
        self.cmd_vel = Twist()
        self.publisher.publish(self.cmd_vel)
        self.enemy_pose = Pose()
        self.enemy_velocity = Twist()
        self.our_velocity = Twist()
        self.front_left_edge = False
        self.front_right_edge = False
        self.rear_left_edge = False
        self.rear_right_edge = False

    def update(self):
        self.publisher.publish(self.cmd_vel)

    def enemy_odom_callback(self, msg):
        self.enemy_pose = msg.pose.pose
        self.enemy_velocity = msg.twist.twist

    def self_odom_callback(self, msg):
        self.our_pose = msg.pose.pose
        self.our_velocity = msg.twist.twist

    def edge_callback(self, msg):
        frame = msg.header.frame_id
        if frame == '/' + self.tf_prefix + 'front_right_line_sensor':
            self.front_right_edge = msg.at_edge
        elif frame == '/' + self.tf_prefix + 'front_left_line_sensor':
            self.front_left_edge = msg.at_edge
        elif frame == '/' + self.tf_prefix + 'rear_right_line_sensor':
            self.rear_right_edge = msg.at_edge
        elif frame == '/' + self.tf_prefix + 'rear_left_line_sensor':
            self.rear_left_edge = msg.at_edge
        else:
            raise RuntimeError(
                "{} not a valid edge callback option!".format(frame))
