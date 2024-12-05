#!/usr/bin/env python3


import numpy as np
import rospy
from std_msgs.msg import String
from PFlib import ParticleFilter as PF
from PFutils.stats import calculate_mean_and_cov
from PFlib.stateManger import *
from PFlib.GradientResampling import GradientResamplingUtiles
import sys
from jackal_range.msg import RD_recap
from geometry_msgs.msg import PointStamped, Twist, Pose, Point, Quaternion, Vector3
from nav_msgs.msg import Odometry
from PFutils.pfMetry import pfmetry
import tf2_ros
import tf_conversions

# point cloud
from sensor_msgs.msg import PointCloud2
import std_msgs.msg
import sensor_msgs.point_cloud2 as pcl2

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
    tf_available = False

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
    particles_msg = PointCloud2()
    
    pf_busy = False

    
    # ? what we want do with the metry?
    metry_obj = None
    
    ambiguity_flag = False
    pending_ambiguity_resolve = False
    pending_ambiguity_raised = False
    static = False
    static_counter = 0
    last_nonstatic_time = 0
    # class constructor
    def __init__(self):   
        
        NS_name = rospy.get_param('~NS_name', '')   # ? why we call this here and not in the handler?
        self.NUM_OF_PARTICLES = int(rospy.get_param('~N_particles', '')) 
        
        # indices for the invalid measurements
        self._invalid_indices = []
        
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
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))  # 10 seconds cache time
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.main_timer = rospy.Timer(rospy.Duration(1/RATE), self.timer_callback)
        self.tf_timer = rospy.Timer(rospy.Duration(1/RATE), self.timer_callback_tf)
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
        self.cmdvelsub = rospy.Subscriber(cmdveltopic, Twist, self.velCmdHandler) # NOT IN USE
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
        
        particles_topic = f'/{self.namespace}/PFparticles'
        self.publisher_particles = rospy.Publisher(particles_topic, PointCloud2, queue_size=10)
        
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
        # get the current time
        time_now = rospy.get_time()
        # check if we are static using velocity
        if np.abs(self.current_controller_odom_msg.twist.twist.linear.x) < 0.1 and np.abs(self.current_controller_odom_msg.twist.twist.linear.y) < 0.1:
            self.static_counter += 1
            # rospy.logwarn('static counter: %s', self.static_counter)
        else:
            if self.static:
                rospy.logwarn('resetting static counter')
            self.static_counter = 0
            self.static = False
            self.last_nonstatic_time = time_now
        
        if self.static_counter > 10 and time_now - self.last_nonstatic_time > 5:
            if not self.static:
                rospy.logwarn('static detected')
            self.static = True
        


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
        # ? how should we initialize the particles? should it really be a gaussian noise? or maybe a uniform noise? 
        self.particles[:,get_agent_index(0)] += np.random.normal(initial_state[get_agent_index(0)], INITIAL_PARTICLES_VARIANCE, (self.NUM_OF_PARTICLES, STATE_SIZE_2D))
        
        # log
        rospy.loginfo('initial state: %s', initial_state)        
        rospy.loginfo('first 5 particles: \n %s', self.particles[:5])
        
    
    def is_particle_filter_time(self):
        if self.pf_busy:
            rospy.logwarn_once('PF is busy')
            return False
        if self.current_range_msg.header.stamp.to_sec() <= self.last_range_msg_time:
            rospy.logwarn_once(self.namespace +  ': timer issue in the PF!')
            return False
        if np.any(np.array(self.current_range_msg.D) < 0):
            rospy.logwarn_once('measurement rejected: range to one of the beacons is negative')
            # find indices of the negative measurements
            self._invalid_indices = [i for i, x in enumerate(self.current_range_msg.D) if x < 0]
            return True
        if np.any(np.array(self.current_range_msg.A_POS) == -999.):
            rospy.logfatal('measurement rejected: one of the beacons is not in the map')
            return False
        # when all is good, there are no indices
        self._invalid_indices = []
        return True
    
    # timer callback
    def timer_callback(self, event):
        
        # log first timer callback
        rospy.logwarn_once('first timer callback, dt: %s', self.current_range_msg.header.stamp.to_sec() - self.last_range_msg_time)
        self.ambiguity_handler()
        # if the range message is newer than the last range message time
        if self.is_particle_filter_time():
            self.pf_busy = True
            
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
            
            # replace the cov of _invalid_indices with a high value
            self.cov_measurement = np.diag([SIGMA_MEASUREMENT**2 for i in range(RANGE_MEASUREMENT_SIZE)])
            for i in self._invalid_indices:
                self.cov_measurement[i,i] = 1000
            
            # log the covariance matrix
            # rospy.logwarn('covariance matrix: %s', self.cov_measurement)
            
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
            # local position
            tag_pos_local = np.array((self.mean[0], self.mean[1], 0.))
            agent_pos_local = self.DCM_local_tag@(tag_pos_local + self.trans_local)
            self.op.pose.pose.position = Point(agent_pos_local[0], agent_pos_local[1], np.abs(agent_pos_local[2]))
            self.op.pose.pose.orientation = Quaternion(0., 0., 0., 1.)
            self.op.pose.covariance[0:2] = self.cov[0:2].tolist()
            self.op.pose.covariance[2:4] = self.cov[TOTAL_STATE_SIZE:TOTAL_STATE_SIZE + 2].tolist()    
            self.op.pose.covariance[0] += 0.5*0.5          
            self.op.pose.covariance[3] += 0.5*0.5  
            self.op.twist.twist.linear = Vector3(self.u[0], self.u[1], 0.)
            self.op.twist.twist.angular = Vector3(0., 0., 0.)
            
            # publish and log
            self.publisher.publish(self.op)
            
            # publish the particles as PintCloud
            head = std_msgs.msg.Header()
            head.stamp = rospy.Time.now()
            head.frame_id = self.namespace + '/odom'
            particles_2d = np.zeros((self.NUM_OF_PARTICLES, 3))
            particles_2d[:,:2] = self.particles[:,:2]          
            particles_2d[:,2] = np.abs(agent_pos_local[2])  
            self.particles_msg = pcl2.create_cloud_xyz32(head, particles_2d.tolist())               
            self.publisher_particles.publish(self.particles_msg)

            if self.pending_ambiguity_raised and self.static:
                rospy.logwarn('ambiguity raised, trying to run gradient resampling')
                #get the first beacon the is valid
                for i in range(NUM_OF_BEACONS):
                    if i not in self._invalid_indices:
                        beacon_id = i
                        break
                self.particles = GradientResamplingUtiles.main(  x_orig = self.mean,
                                                z = self.z,
                                                agent_id = 0,
                                                beacon_id = beacon_id,
                                                particles = self.particles,
                                                measurements_likelihood_function = self.measurements_likelihood,
                                                sigma_measurement = SIGMA_MEASUREMENT,
                                                step_size = 0.05,
                                                do_debug = False)
                self.pending_ambiguity_raised = False
                rospy.logwarn('finished gradient resampling')
            
            self.pf_busy = False
            rospy.loginfo_once('finished first PF step')
            
    # callback for the transform
    def timer_callback_tf(self, event):
            
        # define frames of the transformation for the local frame
        target_frame = f"{self.namespace}/right_tag"
        source_frame = f"{self.namespace}/base_link"
        
        #  log once
        rospy.logwarn_once('source_frame: {}'.format(source_frame))
        rospy.logwarn_once('target_frame: {}'.format(target_frame))
        
        # wait for the transformation
        try:          
            
            # Define target time for the transformation
            target_time = rospy.Time.now() - rospy.Duration(1/RATE)  # 1 seconds ago  
            
            # Lookup the transform, allowing extrapolation
            transform = self.tf_buffer.lookup_transform(
                target_frame,  # target frame
                source_frame,  # source frame
                target_time,     # time at which you want the transform
                rospy.Duration(1/RATE)  # timeout for looking up the transform
            )    
            
            # assign the transformation
            trans = np.array((  transform.transform.translation.x, 
                                transform.transform.translation.y, 
                                transform.transform.translation.z))
            rot = np.array((    transform.transform.rotation.x,
                                transform.transform.rotation.y,
                                transform.transform.rotation.z,
                                transform.transform.rotation.w))
            
            # log
            rospy.loginfo_once("Transform found: translation %s, rotation %s", str(trans), str(rot))
            
            # correct formatting
            matrix = tf_conversions.transformations.quaternion_matrix(rot)
            self.DCM_local_tag = matrix[:3,:3]
            self.trans_local = np.array(trans).T
            self.tf_available = True
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:            
            rospy.sleep(0.1)
            self.tf_available = False
            rospy.logwarn("Transformation error raised: %s", e)
    
    
    def ambiguity_handler(self):
        last_ambiguity_flag = self.ambiguity_flag
        self.ambiguity_flag = False
        # check how many beacons are invalid
        n_invalid = len(self._invalid_indices)
        #get the number of valid beacons
        n_valid = NUM_OF_BEACONS - n_invalid
        if n_valid < 3:
            self.ambiguity_flag = True
            rospy.logwarn_once('ambiguity detected')
        if last_ambiguity_flag != self.ambiguity_flag:
            rospy.logwarn('ambiguity flag changed: %s', self.ambiguity_flag)
        if last_ambiguity_flag == True and self.ambiguity_flag == False:
            self.pending_ambiguity_resolve = True
            rospy.logwarn('ambiguity resolved')
        if last_ambiguity_flag == False and self.ambiguity_flag == True:
            self.pending_ambiguity_raised = True
            rospy.logwarn('ambiguity detected')
            
    
        
    
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
