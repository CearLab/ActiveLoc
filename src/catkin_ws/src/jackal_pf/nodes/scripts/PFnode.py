#!/usr/bin/env python

import numpy as np
import rospy
from std_msgs.msg import String
from PFlib import ParticleFilter as PF
from PFutils import stats
from PFlib.stateManger import *

from jackal_range.msg import RD_recap
from geometry_msgs.msg import Twist

RESAMPLE_METHOD = 'systematic'

class PFnode:
    rangesub = None
    beaconMsg = list()
    beaconData = list()
    tagData = list()
    x = np.zeros(TOTAL_STATE_SIZE)
    z = np.zeros(RANGE_MEASUREMENT_SIZE)
    LockMsg = False
    vel_cmd_msg = None
    odom_msg = None
    
    def __init__(self):
        # Initialize the ROS node
        rospy.loginfo("hello!.")
        self.UWBSubscribers()
        self.cmdMsgSubscriber()
        
        # Publisher
        self.publisher = rospy.Publisher('/my_topic', String, queue_size=10)
        tstmsg = RD_recap()

        # Timer (for periodic tasks)
        self.timer = rospy.Timer(rospy.Duration(1), self.timer_callback)

    def defineModels(self):
        cov_transition_agent = [SIGMA_TRANSITION_AGENT**2 for i in range(NUM_OF_AGENTS*STATE_SIZE_2D)]
        cov_transition_agent = [0 for i in range(NUM_OF_AGENTS*STATE_SIZE_2D)]
        cov_transition_beacon = [SIGMA_TRANSITION_BEACON**2 for i in range(NUM_OF_BEACONS*STATE_SIZE_2D)]
        self.cov_measurement = np.diag([SIGMA_MEASUREMENT**2 for i in range(RANGE_MEASUREMENT_SIZE)])
        self.cov_transition = np.diag(cov_transition_agent + cov_transition_beacon)
        
        self.measurements_likelihood = lambda z, x: PF.normal_model_pdf(z, calculate_true_range_meas(x), PF.cov_measurement)
        self.propagate_state_function = lambda x, u: x + PF.sample_normal_model(u, self.cov_transition)

        

    def Subscribers(self):
        # define lambda function for tag message handler
        self.rangesub = rospy.Subscriber('/UWB/range', RD_recap, lambda data: self.rangeMsgHandler(data, 'range'))
        
    def tagMsgHandler(self, data, tag):
        pass
    
    def beaconMsgHandler(self, data, tag):
        pass

    def timer_callback(self, event):
        pass
    
    def PFstep(self):

        particles = PF.single_step_particle_filter(particles, self.u, self.z, self.propagate_state_function, self.measurements_likelihood_function, resample_method = RESAMPLE_METHOD)
        mean, cov = calculate_mean_and_cov(particles)



    def run(self):
        rospy.init_node('PF', anonymous=True)
        rospy.spin()

if __name__ == '__main__':
    try:
        node = PFnode()
        node.run()
    except rospy.ROSInterruptException:
        pass
