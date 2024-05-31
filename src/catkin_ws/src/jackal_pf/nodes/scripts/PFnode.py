#!/usr/bin/env python

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

RESAMPLE_METHOD = 'systematic'
NS = sys.argv[1]
RATE = 5
NUM_OF_PARTICLES = 100
class PFnode:
    namespace = None
    
    rangesub = None
    current_range_msg = RD_recap()
    last_range_msg_time = 0.
    cntrl_odom_msg_curr_range_time = None
    
    cmdvelsub = None
    current_theta = 0
    dpos = np.zeros(STATE_SIZE_2D)
    current_cmd_vel_time = None
    last_cmd_vel_time = None
    
    cntrlodomsub = None
    current_controller_odom_msg : Odometry = None
    last_controller_odom_msg = Odometry()

    x = np.zeros(TOTAL_STATE_SIZE)
    z = np.zeros(RANGE_MEASUREMENT_SIZE)
    u = np.zeros(TOTAL_STATE_SIZE)
    particals = None
    beacons_position = list()
    
    op = Odometry()
    
    # state and measurement init
    x = np.zeros(TOTAL_STATE_SIZE)
    z = np.zeros(RANGE_MEASUREMENT_SIZE)        
    
    # class constructor
    def __init__(self):
        
        # Initialize the ROS node
        rospy.loginfo('starting init')
        self.init_variables()
        self.namespace_handler()
        self.subscribers()
        self.publishers()
        self.define_models()
        self.init_timers()
        rospy.loginfo('finished init')

        
    def init_variables(self):
        self.last_controller_odom_msg.header.stamp = rospy.Time.now()
        self.last_controller_odom_msg.pose.pose = Pose(Point(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0))
        self.last_controller_odom_msg.twist.twist = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        
    def init_timers(self):
        rospy.loginfo('starting set timers')
        self.main_timer = rospy.Timer(rospy.Duration(1/RATE), self.timer_callback)
        rospy.loginfo('finished set timers')
        
    def define_models(self):
        rospy.loginfo('defining models')
        cov_transition_agent = [SIGMA_TRANSITION_AGENT**2 for i in range(NUM_OF_AGENTS*STATE_SIZE_2D)]
        # cov_transition_agent = [0 for i in range(NUM_OF_AGENTS*STATE_SIZE_2D)]
        cov_transition_beacon = [SIGMA_TRANSITION_BEACON**2 for i in range(NUM_OF_BEACONS*STATE_SIZE_2D)]
        cov_transition_beacon = [0 for i in range(NUM_OF_BEACONS*STATE_SIZE_2D)]
        self.cov_measurement = np.diag([SIGMA_MEASUREMENT**2 for i in range(RANGE_MEASUREMENT_SIZE)])
        self.cov_transition = np.diag(cov_transition_agent + cov_transition_beacon)
        
        self.measurements_likelihood = lambda z, x: PF.normal_model_pdf(z, calculate_true_range_meas(x), self.cov_measurement)
        self.propagate_state_function = lambda x, u: x + PF.sample_normal_model(u, self.cov_transition)
        rospy.loginfo('finished defining models')

    def namespace_handler(self):
        if len(sys.argv) > 1:
            NS = sys.argv[1]
        if NS.startswith('_'):
            NS = None
            rospy.loginfo('namespace is private, using ROS namespace')
        if NS is None:
            self.namsepace = rospy.get_namespace()
            rospy.loginfo('got namespace from ros: {}'.format(self.namsepace))
        else:
            self.namsepace = NS
            rospy.loginfo('got namespace from args: {}'.format(self.namsepace))
        # remove all leading and trailing slashes
        self.namsepace = self.namsepace.strip('/')
        rospy.loginfo('final name space: {}'.format(self.namsepace))

    
    def subscribers(self):
        rospy.loginfo('startimg subscribers')
        range_topic = f'/{self.namsepace}/range'
        cmdveltopic = f'/{self.namsepace}/cmd_vel'
        cntrlodomtopic = f'/{self.namsepace}/jackal_velocity_controller/odom'
        rospy.loginfo('range_topic: {}'.format(range_topic))
        rospy.loginfo('cmdveltopic: {}'.format(cmdveltopic))
        rospy.loginfo('cntrlodomtopic: {}'.format(cntrlodomtopic))
        self.rangesub = rospy.Subscriber(range_topic, RD_recap, self.rangeMsgHandler)
        self.cmdvelsub = rospy.Subscriber(cmdveltopic, Twist, self.velCmdHandler) #NOT IN USE
        self.cntrlodomsub = rospy.Subscriber(cntrlodomtopic, Odometry, self.controllerOdomHandler)
        rospy.loginfo('finished subscribers')
        
    def publishers(self):
        rospy.loginfo('startimg subscribers')
        pfest_topic = f'/{self.namsepace}/PFest'
        rospy.loginfo('pfest_topic: {}'.format(pfest_topic))
        self.publisher = rospy.Publisher(pfest_topic, Odometry, queue_size=10)
        rospy.loginfo('finished subscribers')
        
    def rangeMsgHandler(self, data):
        rospy.loginfo_once('got first range message')
        rospy.loginfo_once('range message: %s', data)
        self.current_range_msg = data
        self.cntrl_odom_msg_curr_range_time = self.current_controller_odom_msg
        if self.particals is None:
            self.initialize_particles()
    
    def velCmdHandler(self, data): #NOT IN USE
        rospy.loginfo_once('got first velocity command')
        return
        time_now = rospy.get_time()
        self.last_cmd_vel_time = self.current_cmd_vel_time
        self.current_cmd_vel_time = time_now
        self.current_omega = data.angular.z
        omege = data.angular.z
        if self.last_cmd_vel_time is None:
            self.last_cmd_vel_time = time_now
        dt = self.current_cmd_vel_time - self.last_cmd_vel_time
        self.current_theta += omege*dt
        self.dpos += np.array([data.linear.x*np.cos(self.current_theta), data.linear.x*np.sin(self.current_theta)]) * dt
        rospy.loginfo('dpos: %s', self.dpos)
        rospy.loginfo_once('velocity command: %s', data)

    def controllerOdomHandler(self, data):
        rospy.loginfo_once('got first controller odom message')
        rospy.loginfo_once('controller odom message: %s', data)
        self.current_controller_odom_msg = data

    
    def initialize_particles(self):
        if self.current_controller_odom_msg is None:
            return
        initial_state = np.zeros(TOTAL_STATE_SIZE)
        initial_state[get_agent_index(0)] = np.array([self.current_controller_odom_msg.pose.pose.position.x, self.current_controller_odom_msg.pose.pose.position.y])
        self.particals = np.zeros((NUM_OF_PARTICLES, TOTAL_STATE_SIZE))

        for i in range(NUM_OF_BEACONS):
            initial_state[get_beacon_index(i)] = np.array([self.current_range_msg.A_POS[i*3 + 0], self.current_range_msg.A_POS[i*3 + 1]])
            self.particals[:,get_beacon_index(i)] = initial_state[get_beacon_index(i)]
        self.particals[:,get_agent_index(0)] += np.random.normal(initial_state[get_agent_index(0)], 4, (NUM_OF_PARTICLES, STATE_SIZE_2D))
        
        rospy.loginfo('initial state: %s', initial_state)
        # log the first 5 particles
        rospy.loginfo('first 5 particles: \n %s', self.particals[:5])
        
        
    def timer_callback(self, event):
        rospy.loginfo_once('first timer callback, dt: %s', self.current_range_msg.header.stamp.to_sec() - self.last_range_msg_time)
        if self.current_range_msg.header.stamp.to_sec() > self.last_range_msg_time:
            rospy.loginfo_once('starting first PF step')
            self.last_range_msg_time = self.current_range_msg.header.stamp.to_sec()
            last_odom_xy = np.array([self.last_controller_odom_msg.pose.pose.position.x, self.last_controller_odom_msg.pose.pose.position.y])
            current_odom_xy = np.array([self.cntrl_odom_msg_curr_range_time.pose.pose.position.x, self.cntrl_odom_msg_curr_range_time.pose.pose.position.y])
            self.last_controller_odom_msg = self.cntrl_odom_msg_curr_range_time
            self.u[get_agent_index(0)] = np.array([current_odom_xy - last_odom_xy])
            rospy.loginfo('current u: %s', self.u)
            self.z = np.array(self.current_range_msg.D).T
            rospy.loginfo('current z: %s', self.z)
            self.mean = np.zeros(TOTAL_STATE_SIZE)
            self.cov = np.zeros((TOTAL_STATE_SIZE, TOTAL_STATE_SIZE))
            self.particals = PF.single_step_particle_filter(self.particals,
                                                            self.u,
                                                            self.z,
                                                            self.propagate_state_function,
                                                            self.measurements_likelihood,
                                                            resample_method = RESAMPLE_METHOD)
            self.mean, self.cov = calculate_mean_and_cov(self.particals)
            self.op.header.stamp = rospy.Time.now()
            self.op.pose.pose.position = Point(self.mean[0], self.mean[1], 0.)
            self.op.pose.covariance[0:2] = self.cov[0:2].tolist()
            self.op.pose.covariance[2:4] = self.cov[TOTAL_STATE_SIZE:TOTAL_STATE_SIZE + 2].tolist()
            self.publisher.publish(self.op)
            rospy.loginfo_once('finished first PF step')
            # rospy.loginfo('current op: \n %s', self.op)
            # rospy.loginfo('first 5 particles: \n %s', self.particals[:5])
    def PFstep(self):
        pass




    # run the node
    def run(self):
        rospy.spin()

# default exec
if __name__ == '__main__':
    
    try:
        rospy.init_node('pf')
        rospy.loginfo('+++++++++++++++node started+++++++++++++++++')
        node = PFnode()
        node.run()
        
    except rospy.ROSInterruptException:
        pass
