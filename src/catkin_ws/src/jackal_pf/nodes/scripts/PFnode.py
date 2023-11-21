#!/usr/bin/env python3


import numpy as np
import rospy
from std_msgs.msg import String
from PFlib import ParticleFilter as PF
from PFutils.stats import calculate_mean_and_cov
from PFlib.stateManger import *
import sys
from jackal_range.msg import RD_recap
from geometry_msgs.msg import PointStamped, Twist, Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from PFutils.pfMetry import pfmetry

# global vars
print(sys.argv)
# sampling method
RESAMPLE_METHOD = 'systematic'
# namespace
NS = sys.argv[1]
# spin rate
RATE = 10
HEIGHT = 2
# variance for the initial spread of the particles
INITIAL_PARTICLES_VARIANCE = 4

class PFnode:
    
    # init namespace
    namespace = None

    # range init 
    rangesub = None
    current_range_msg = RD_recap()
    last_range_msg_time = 0.              
    ctrl_odom_msg_curr_range_time = None
    
    # cmd_vel init 
    # !! NOT IN USE when i will be brave enough, i will get rid of this
    cmdvelsub = None
    current_theta = 0
    dpos = np.zeros(STATE_SIZE_2D)
    current_cmd_vel_time = None
    last_cmd_vel_time = None
    
    # odometry init
    ctrlodomsub = None
    current_controller_odom_msg : Odometry = None
    last_controller_odom_msg = Odometry()

    # PF init
    x = np.zeros(TOTAL_STATE_SIZE)
    z = np.zeros(RANGE_MEASUREMENT_SIZE)
    u = np.zeros(TOTAL_STATE_SIZE)
    particles = None
    beacons_position = list()
    
    # assign odometry message
    op = Odometry()
    

    
    # ? what we want do with the metry?
    metry_obj = None
    
    # class constructor
    def __init__(self):   
        
        NS_name = rospy.get_param('~NS_name', '')   # ? why we call this here and not in the handler?
        self.NUM_OF_PARTICLES = int(rospy.get_param('~N_particles', '')) 
        
        # Initialize the ROS node
        rospy.logwarn_once('PF starting init')
        self.init_metry()
        self.init_variables()
        self.namespace_handler(NS_name)
        self.subscribers()
        self.publishers()
        self.define_models()
        self.init_timers()
        rospy.logwarn_once('PF finished init')

    # init metry
    def init_metry(self):
        rospy.loginfo('starting init metry')
        self.metry_obj = pfmetry()
    
    # init Odometry variables
    def init_variables(self):
        self.last_controller_odom_msg.header.stamp = rospy.Time.now()
        self.last_controller_odom_msg.pose.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        self.last_controller_odom_msg.twist.twist = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

    # init timers
    def init_timers(self):
        rospy.loginfo('starting set timers')
        self.main_timer = rospy.Timer(rospy.Duration(1/RATE), self.timer_callback)
        rospy.loginfo('finished set timers')
        
    # PF model definition
    def define_models(self):
        
        # log
        rospy.loginfo('defining models')  
        # define covariance matrix of the transition model: agents and beacons
        cov_transition_agent = [SIGMA_TRANSITION_AGENT**2 for i in range(NUM_OF_AGENTS*STATE_SIZE_2D)]        
        cov_transition_beacon = [SIGMA_TRANSITION_BEACON**2 for i in range(NUM_OF_BEACONS*STATE_SIZE_2D)]
        cov_transition_beacon = [0 for i in range(NUM_OF_BEACONS*STATE_SIZE_2D)]
        self.cov_measurement = np.diag([SIGMA_MEASUREMENT**2 for i in range(RANGE_MEASUREMENT_SIZE)])
        self.cov_transition = np.diag(cov_transition_agent + cov_transition_beacon)
        
        # define the likelihood function
        self.measurements_likelihood = lambda z, x: PF.normal_model_pdf(z, calculate_true_range_meas(x), self.cov_measurement)
        
        # define the transition function
        self.propagate_state_function = lambda x, u: x + PF.sample_normal_model(u, self.cov_transition)
        
        rospy.loginfo('finished defining models')

    # get the namespace
    def namespace_handler(self, NS):                 
        
        # get the namespace        
        if NS is None:
            self.namespace = rospy.get_namespace()
            rospy.loginfo('got namespace from ros: {}'.format(self.namespace))
        elif NS.startswith('_'):            
            NS = None
            rospy.loginfo('namespace is private, using ROS namespace')
        else:
            self.namespace = NS
            rospy.loginfo('got namespace from args: {}'.format(self.namespace))                    
                
        # remove all leading and trailing slashes
        self.namespace = self.namespace.strip('/')
        rospy.loginfo('final name space: {}'.format(self.namespace))

    # define subscribers
    def subscribers(self):
        
        # define topic names
        rospy.loginfo('starting subscribers')
        range_topic = f'/{self.namespace}/Anchors/range'
        cmdveltopic = f'/{self.namespace}/cmd_vel'
        ctrlodomtopic = f'/{self.namespace}/jackal_velocity_controller/odom'
        
        # log
        rospy.loginfo('range_topic: {}'.format(range_topic))
        rospy.loginfo('cmdveltopic: {}'.format(cmdveltopic))
        rospy.loginfo('ctrlodomtopic: {}'.format(ctrlodomtopic))
        
        # init subscribers
        self.rangesub = rospy.Subscriber(range_topic, RD_recap, self.rangeMsgHandler)
        self.cmdvelsub = rospy.Subscriber(cmdveltopic, Twist, self.velCmdHandler) #NOT IN USE
        self.ctrlodomsub = rospy.Subscriber(ctrlodomtopic, Odometry, self.controllerOdomHandler)
        
        # log
        rospy.loginfo('finished subscribers')
        
    # define publishers
    def publishers(self):
        
        # define topic names
        rospy.loginfo('starting subscribers')
        pfest_topic = f'/{self.namespace}/PFest'
        rospy.loginfo('pfest_topic: {}'.format(pfest_topic))
        
        # init publishers
        self.publisher = rospy.Publisher(pfest_topic, Odometry, queue_size=10)
        rospy.loginfo('finished subscribers')
        
    # handle the RD_recap message
    def rangeMsgHandler(self, data):
        
        # log
        rospy.loginfo_once('got first range message')
        rospy.loginfo_once('range message: %s', data)
        
        # set the msg
        self.current_range_msg = data
        self.ctrl_odom_msg_curr_range_time = self.current_controller_odom_msg
        
        # init particles
        # ? where is particles initialized? is there a chance that particles is not None?
        # ? Ok I see, it's a one time initialization
        if self.particles is None:
            self.initialize_particles()
    
    # handle the Twist message from velocity command
    # !! NOT IN USE
    def velCmdHandler(self, data): 
        
        # log
        rospy.loginfo_once('got first velocity command')
        return
    
        # not in use right now
        time_now = rospy.get_time()
        self.last_cmd_vel_time = self.current_cmd_vel_time
        self.current_cmd_vel_time = time_now
        self.current_omega = data.angular.z
        omega = data.angular.z
        if self.last_cmd_vel_time is None:
            self.last_cmd_vel_time = time_now
        dt = self.current_cmd_vel_time - self.last_cmd_vel_time
        self.current_theta += omega*dt
        self.dpos += np.array([data.linear.x*np.cos(self.current_theta), data.linear.x*np.sin(self.current_theta)]) * dt
        rospy.loginfo('dpos: %s', self.dpos)
        rospy.loginfo_once('velocity command: %s', data)

    # handle the Odometry message from the controller
    def controllerOdomHandler(self, data):
        
        # log
        rospy.loginfo_once('got first controller odom message')
        rospy.loginfo_once('controller odom message: %s', data)
        
        # set data
        self.current_controller_odom_msg = data

    # PF init particles
    def initialize_particles(self):
        
        # if I get no odom I don't initialize particles
        # remember that here er're talking about the odometry from the integration of the cmd_vel command
        # so basically, if I don't get an update on the position of the robot, I don't initialize the particles
        if self.current_controller_odom_msg is None:
            return
        
        # init state
        initial_state = np.zeros(TOTAL_STATE_SIZE)
        # the agents positions are initialized with the current controller odometry (integrated)
        initial_state[get_agent_index(0)] = np.array([self.current_controller_odom_msg.pose.pose.position.x, self.current_controller_odom_msg.pose.pose.position.y])
        self.particles = np.zeros((self.NUM_OF_PARTICLES, TOTAL_STATE_SIZE))
        
        # here I update the anchors from the range measurements
        for i in range(NUM_OF_BEACONS):
            initial_state[get_beacon_index(i)] = np.array([self.current_range_msg.A_POS[i*3 + 0], self.current_range_msg.A_POS[i*3 + 1]])
            set_beacon_height(self.current_range_msg.A_POS[i*3 + HEIGHT])
            rospy.logwarn(self.namespace + ': beacon height read: ' + str(self.current_range_msg.A_POS[i*3 + 2]))

            self.particles[:,get_beacon_index(i)] = initial_state[get_beacon_index(i)]
        
        # add gaussian noise to the particles (process noise)
        # ? how shuld we initialize the particles? shoud it really be a gaussian noise? or maybe a uniform noise? 
        self.particles[:,get_agent_index(0)] += np.random.normal(initial_state[get_agent_index(0)], INITIAL_PARTICLES_VARIANCE, (self.NUM_OF_PARTICLES, STATE_SIZE_2D))
        
        # log
        rospy.loginfo('initial state: %s', initial_state)        
        rospy.loginfo('first 5 particles: \n %s', self.particles[:5])
        
    
    def is_particle_filter_time(self):
        if self.current_range_msg.header.stamp.to_sec() <= self.last_range_msg_time:
            rospy.loginfo(self.namespace +  ': timer issue in the PF!')
            return False
        if np.any(np.array(self.current_range_msg.D) < 0):
            rospy.logfatal('measurement rejected: range to one of the beacons is negative')
            return False
        if np.any(np.array(self.current_range_msg.A_POS) == -999.):
            rospy.logfatal('measurement rejected: one of the beacons is not in the map')
            return False
        return True
    # timer callback
    def timer_callback(self, event):
        
        # log first timer callback
        rospy.logwarn_once('first timer callback, dt: %s', self.current_range_msg.header.stamp.to_sec() - self.last_range_msg_time)
        
        # if the range message is newer than the last range message time
        if self.is_particle_filter_time():
            
            # log
            rospy.loginfo_once('starting first PF step')
            
            # set particles
            self.metry_obj.write('particles', self.particles)
            self.metry_obj.write('u', self.u)
            self.metry_obj.write('z', self.z)
            self.last_range_msg_time = self.current_range_msg.header.stamp.to_sec()
            
            # get last XY position and current one
            last_odom_xy = np.array([self.last_controller_odom_msg.pose.pose.position.x, self.last_controller_odom_msg.pose.pose.position.y])
            current_odom_xy = np.array([self.ctrl_odom_msg_curr_range_time.pose.pose.position.x, self.ctrl_odom_msg_curr_range_time.pose.pose.position.y])
            
            # set odom time as the range time
            self.last_controller_odom_msg = self.ctrl_odom_msg_curr_range_time
            # control is difference between current and last odom
            self.u[get_agent_index(0)] = np.array([current_odom_xy - last_odom_xy])
            # self.u = np.nan_to_num(self.u, nan=0.0)
            # measure is the range measurement
            self.z = np.array(self.current_range_msg.D).T
            
            # init mean anc cov
            self.mean = np.zeros(TOTAL_STATE_SIZE)
            self.cov = np.zeros((TOTAL_STATE_SIZE, TOTAL_STATE_SIZE))
            
            # call the PF
            self.particles = PF.single_step_particle_filter(self.particles,
                                                            self.u,
                                                            self.z,
                                                            self.propagate_state_function,
                                                            self.measurements_likelihood,
                                                            resample_method = RESAMPLE_METHOD)
            
            # average the particles
            self.mean, self.cov = calculate_mean_and_cov(self.particles)            
            
            # log
            self.metry_obj.write('mean', self.mean)
            self.metry_obj.write('cov', self.cov)
            
            # set data
            self.op.header.stamp = rospy.Time.now()
            self.op.header.frame_id = self.namespace + '/odom'
            self.op.child_frame_id = self.namespace + '/base_link'
            self.op.pose.pose.position = Point(self.mean[0], self.mean[1], 0.)
            self.op.pose.pose.orientation = Quaternion(0., 0., 0., 1.)
            self.op.pose.covariance[0:2] = self.cov[0:2].tolist()
            self.op.pose.covariance[2:4] = self.cov[TOTAL_STATE_SIZE:TOTAL_STATE_SIZE + 2].tolist()    
            self.op.pose.covariance[0] += 0.5*0.5          
            self.op.pose.covariance[3] += 0.5*0.5  
            self.op.twist.twist.linear = Vector3(self.u[0], self.u[1], 0.)
            self.op.twist.twist.angular = Vector3(0., 0., 0.)
            
            # publish and log
            self.publisher.publish(self.op)
            rospy.loginfo_once('finished first PF step')
            
    # run the node
    def run(self):
        rospy.spin()

# default exec
if __name__ == '__main__':
    
    try:
        rospy.init_node('pf')
        rospy.loginfo('+++++++++++++++ node started +++++++++++++++++')
        
        # init
        node = PFnode()
        node.run()
        
    except rospy.ROSInterruptException:
        pass
