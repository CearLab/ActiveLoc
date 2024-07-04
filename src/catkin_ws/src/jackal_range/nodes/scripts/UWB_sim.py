#!/usr/bin/env python

import rospy
import range.jackal_range as jr
from jackal_range.msg import RD_recap as RD
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np
import sys
import tf_conversions

# global vars
# node rate
RATE = 10
# namespace
NS = None

# method to calculate the range measurement from the agent to the beacon
def calc_range_meas(agent_loc,beacon_loc):    
    return np.linalg.norm(agent_loc - beacon_loc)

class UWB:
    
    def __init__(self):
        
        # define jr instance
        self.jr_instance = jr.JackalRange()
        
        # get namespace
        NS_name = rospy.get_param('~NS_name', '')
        
        # get anchors params
        params_name = rospy.get_param('~params_name', '')
        
        # get the range meas covariance
        self.range_cov = rospy.get_param('~range_meas_cov', 0.0)
        
        
        # class constructor
        rospy.loginfo('starting init')
        self.tf_available = False
        self.current_gt = None
        self.namespace_handler(NS_name)
        self.initop(params_name)
        self.pubsub()
        self.timers()
        rospy.loginfo('finished init')
        

    # init the message
    def initop(self,params_name):
        
        # init message
        self.op = RD()
        
        # We should get here in the Anchors Estimate node, so we gather the anchors from the params_name
        # if not available, we use the default params, otherwise we would have a AvailableKey error                
        rospy.logwarn('AnchorsInfoReal: ' + str(params_name))
        try:
            anchors_params = rospy.get_param(params_name)  
        except:
            anchors_params = self.jr_instance.anchors_params
            rospy.logwarn('No params found, using default params')
            
        # now extract the coordinates from anchors_params
        items = np.array([coord for anchor in anchors_params for coord in anchor[2:]])            
        
        # number of anchors
        N_A = int(len(items)/3)     
        
        # get IDs
        self.op.A_ID = [f'AN{i}' for i in range(N_A)]
        
        # set anchors position
        self.op.A_POS = items
        self.op.N_ID = 'A'
        self.op.T_ID = 'TAG1'
        self.op.NUM_A = N_A

    # publish message
    def pubsub(self):
        
        # get the ground truth topic and the publish topic
        # gt_topic = f'/{self.namespace}/ground_truth/state'
        gt_topic = f'/{self.namespace}/odometry/filtered'
        pub_topic = f'/{self.namespace}/range'
        
        # logs
        rospy.loginfo('gt_topic: {}'.format(gt_topic))
        rospy.loginfo('pub_topic: {}'.format(pub_topic))
        
        # subscribe and publish
        self.ground_truth_sub = rospy.Subscriber(gt_topic, Odometry, self.ground_truth_callback)
        self.uwb_pub = rospy.Publisher(pub_topic, RD, queue_size=10)

    # define timers
    def timers(self):
        self.main_timer = rospy.Timer(rospy.Duration(1/RATE), self.timer_callback)
        # Create a buffer and a listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))  # 10 seconds cache time
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback_tf)


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

    # callback for the ground truth
    def ground_truth_callback(self, data: Odometry):
        rospy.loginfo_once('GT received')
        self.current_gt = data

    # callback for the transform
    def timer_callback_tf(self, event):
        
        # ?? why do we do so
        # if self.tf_available:
        #     return
            
        # define frames of the transformation
        target_frame = f"{self.namespace}/base_link"
        source_frame = f"{self.namespace}/right_tag"        
        rospy.loginfo_once('source_frame: {}'.format(source_frame))
        rospy.loginfo_once('target_frame: {}'.format(target_frame))
        try:          
            
            # Define target time for the transformation
            target_time = rospy.Time.now() - rospy.Duration(1.0)  # 1 seconds ago  
            
            # Lookup the transform, allowing extrapolation
            transform = self.tf_buffer.lookup_transform(
                target_frame,  # target frame
                source_frame,  # source frame
                target_time,     # time at which you want the transform
                rospy.Duration(1.0)  # timeout for looking up the transform
            )               
            trans = np.array((  transform.transform.translation.x, 
                                transform.transform.translation.y, 
                                transform.transform.translation.z))
            rot = np.array((    transform.transform.rotation.x,
                                transform.transform.rotation.y,
                                transform.transform.rotation.z,
                                transform.transform.rotation.w))
            rospy.loginfo_once("Transform found: translation %s, rotation %s", str(trans), str(rot))
            matrix = tf_conversions.transformations.quaternion_matrix(rot)
            self.DCM = matrix[:3,:3]
            self.trans = np.array(trans).T
            rospy.loginfo_once("Transform available")
            rospy.loginfo_once("DCM: \n {}".format(self.DCM))
            rospy.loginfo_once("trans: {}".format(self.trans))            
            self.tf_available = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:            
            rospy.sleep(0.1)
            self.tf_available = False
            rospy.logwarn("Transformation error raised: %s", e)

    # callback for the timer
    def timer_callback(self, event):
        rospy.loginfo_once('first timer callback')
        rospy.loginfo_once('current_gt: {}'.format(self.current_gt))
        if self.current_gt is not None and self.tf_available:
            rospy.loginfo_once('publishing')
            self.op.header.stamp = rospy.Time.now()
            agent_pos = np.array([
                                self.current_gt.pose.pose.position.x,
                                self.current_gt.pose.pose.position.y,
                                self.current_gt.pose.pose.position.z
                                ]).T
            agent_pos = self.DCM@(agent_pos + self.trans)
            agent_pos = agent_pos
            self.op.D = [calc_range_meas(agent_pos, np.array(self.op.A_POS[i*3:i*3+3])) for i in range(4)]
            
            # here the gaussian noise is added
            self.op.D = [d + np.random.normal(0, self.range_cov) for d in self.op.D]
            
            # publish 
            self.uwb_pub.publish(self.op)
            rospy.loginfo_once('first published')

    # run the node
    def run(self):
        rospy.loginfo('running')
        rospy.spin()

# wrapper
if __name__ == '__main__':
    try:
        rospy.init_node('UWB_SIM', anonymous=True)
        rospy.loginfo('++++++++++++++++++starting node+++++++++++++++++')
        uwb = UWB()
        uwb.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("UWB failed")
        pass
