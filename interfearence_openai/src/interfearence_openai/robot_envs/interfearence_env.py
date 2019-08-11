import rospy

from openai_ros import robot_gazebo_env
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from gazebo_msgs.srv import GetModelStateRequest, GetModelState

class InterfearenceEnv(robot_gazebo_env.RobotGazeboEnv):
    """Superclass for all InterfearenceEnv environments
    """
    def __init__(self):
        """
        Initialises a new InterfearenceEnv environment.

        To check any topic we need to have the simulations running, we need to do two things:
        1) Unpause the simulation: without that the stream of data doesn't 
           flow. This is for simulations that are paused for whatever the 
           reason.
        2) If the simulation was running already for some reason we need to
           reset the controllers. This has to do with the fact that some 
           plugins with tf don't understand the reset of the simulation and
           need to be reset to work properly.

        The Sensors: The sensors accessible are the ones considered useful
                     for AI learning

        Sensor Topic List:
        * /interfearence/laser_scan: Laser scan data from a Hokuyo to detect
                                     the enemy robot
        * /interfearence/odom: odometry readings of where the robot is

        Actuators Topic List:
        * /interfearence/cmd_vel: Move the robot around
        """
        rospy.logdebug("Start InterfearenceEnv INIT...")

        controller_prefix = 'controller/differential'
        self.controllers_list = [
            '{}/differential_controller'.format(controller_prefix)
        ]
        
        # No namespace yet
        self.robot_name_space = "interfearence"

        # Launch the init function of the parent class RobotGazeboEnv
        super(InterfearenceEnv, self).__init__(
            controllers_list=self.controllers_list,
            robot_name_space='', # Controller manager isn't prefixed atm
            reset_controls=True,
            reset_world_or_sim="WORLD")
        
        rospy.logdebug("InterfearenceEnv unpause1...")
        self.gazebo.unpauseSim()
        self.controllers_object.reset_controllers()

        self._check_all_systems_ready()

        # Start all the ROS related subs and pubs
        rospy.Subscriber("{}/{}/{}/odom".format(self.robot_name_space,
                                                controller_prefix,
                                                "differential_controller"), 
                         Odometry, 
                         self._odom_callback)
        rospy.Subscriber("/laser_scan", LaserScan, self._laser_callback)
        
        self.publishers_array = []
        self._cmd_vel_pub = rospy.Publisher(
            '{}/{}/{}/cmd_vel'.format(self.robot_name_space,
                                      controller_prefix,
                                      "differential_controller"),
            Twist,
            queue_size=1)

        self.publishers_array.append(self._cmd_vel_pub)

        self._check_all_publishers_ready()

        self.model_state_client = rospy.ServiceProxy(
            '/gazebo/get_model_state', GetModelState)

        self.gazebo.pauseSim()

        self.cmd_vel = Twist()

        rospy.logdebug("Finished InterfearenceEnv INIT...")
        
    # Methods needed by the RobotGazeboEnv
    # ----------------------------

    def _check_all_systems_ready(self):
        """
        Checks that all the sensors, publishers and other systems are
        operational.
        """
        rospy.logdebug("InterfearenceEnv check_all_systems_ready...")
        self._check_all_sensors_ready()
        rospy.logdebug("END InterfearenceEnv check_all_systems_ready...")
        return True

    # CubeSingleDiskEnv virtual methods
    # ----------------------------
    
    def _check_all_sensors_ready(self):
        rospy.logdebug("START ALL SENSIRS READY")
        self._check_odom_ready()
        self._check_laser_ready()
        rospy.logdebug("ALL SENSORS READY")

    def _check_odom_ready(self):
        self.odom = None
        while self.odom is None and not rospy.is_shutdown():
            try:
                self.odom = rospy.wait_for_message(
                    "/interfearence/controller/differential/differential_controller/odom", Odometry, timeout=1.0)
                rospy.logdebug("Current /interfearence/controller/differential/differential_controller/odom READY=>")
            except:
                rospy.logerr("Current /interfearence/controller/differential/differential_controller/odom not ready yet, retrying...")
        return self.odom

    def _check_laser_ready(self):
        self.laser = None
        while self.laser is None and not rospy.is_shutdown():
            try:
                self.laser = rospy.wait_for_message(
                    "/laser_scan", LaserScan, timeout=1.0)
                rospy.logdebug("Current /laser_scan READY=>")
            except:
                rospy.logerr("Current /laser_scan not ready yet, retrying...")
        return self.laser


    def _odom_callback(self, data):
        self.odom = data

    def _laser_callback(self, data):
        self.laser = data


    def _check_all_publishers_ready(self):
        """
        Checks that all the publishers are working
        """
        rospy.logdebug("START ALL PUBLISHERS READY")
        for publisher_object in self.publishers_array:
            self._check_pub_connection(publisher_object)
        rospy.logdebug("ALL PUBLISHERS READY")

    def _check_pub_connection(self, publisher_object):
        rate = rospy.Rate(10)
        while(publisher_object.get_num_connections() == 0 and not rospy.is_shutdown()):
            rospy.logdebug("No subscribers to publisher_object yet")
            try:
                rate.sleep()
            except rospy.ROSInterruptException:
                # Avoid worl restart errors
                pass
        rospy.logdebug("publisher_object Publisher Connected")


    # Methods that the TrainingEnvironment will need to define here as
    # virtual as they will be used in RobotGazeboEnv
    # ---------------------------

    def _set_init_pose(self):
        """Sets the robot in it's initial pose
        """
        raise NotImplementedError()

    def _init_env_variables(self):
        """Inits variables needed to be initialised each time we reset at
        the start of an episode.
        """
        raise NotImplementedError()

    def _compute_reward(self, observations, done):
        """Calculates the reward to give based on the observations given.
        """
        raise NotImplementedError()

    def _set_action(self, action):
        """Applies the given action to the simulation.
        """
        raise NotImplementedError()

    def _get_obs(self):
        raise NotImplementedError()

    def _is_done(self, observations):
        """Checks if episode is done based on observations given.
        """
        raise NotImplementedError()


    # Methods that the TrainingEnvironment will need
    # ---------------------------
    
    def move(self, x_dot, theta_dot):
        """Publish the command velocity to move the robot.
        """
        self.cmd_vel = Twist()
        self.cmd_vel.linear.x = x_dot
        self.cmd_vel.linear.y = 0
        self.cmd_vel.linear.z = 0
        self.cmd_vel.angular.x = 0
        self.cmd_vel.angular.y = 0
        self.cmd_vel.angular.z = theta_dot
        self._cmd_vel_pub.publish(self.cmd_vel)

    def get_odom(self):
        return self.odom

    def get_laser_scan(self):
        return self.laser

    def get_enemy_state(self, enemy_name):
        srv = GetModelStateRequest()
        srv.model_name = enemy_name
        srv.relative_entity_name = "world"
        resp = self.model_state_client(srv)
        return resp

    def get_own_state(self):
        srv = GetModelStateRequest()
        srv.model_name = self.robot_name_space
        srv.relative_entity_name = "world"
        resp = self.model_state_client(srv)
        return resp
