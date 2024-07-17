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
import rosgraph
import message_filters 

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
        self.params_name = params_name
        
        # get the range meas covariance
        self.range_cov = rospy.get_param('~range_meas_cov', 0.0)
        
        # range message        
        self.op = RD()
        
        # class constructor
        rospy.loginfo('starting init')
        self.tf_available = False
        self.current_gt_local = None
        self.current_gt_agents = []
        self.namespace_handler(NS_name)
        self.get_ros_namespaces()        
        self.initop(params_name)
        self.pubsub()
        self.timers()
        rospy.loginfo('finished init')
        

    # init the message
    def initop(self,params_name):
        
        # We should get here in the Anchors Estimate node, so we gather the anchors from the params_name
        # if not available, we use the default params, otherwise we would have a AvailableKey error                
        rospy.logwarn_once('AnchorsInfoReal: ' + str(params_name))
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
        gt_topic_local = f'/{self.namespace}/ground_truth/state'
        pub_topic_anchors = f'/{self.namespace}/Anchors/range'

        # Publisher and subscriber local
        self.uwb_pub_anchors = rospy.Publisher(pub_topic_anchors, RD, queue_size=10)
        self.ground_truth_sub_local = message_filters.Subscriber(gt_topic_local, Odometry)
        
        # get the publishers and subscriber topics for the agents
        if self.namespaces:
            
            # reset lists
            gt_topic_agents = []
            pub_topic_agents = []
            self.uwb_pub_agents = []
            self.uwb_sub_agents = []
            
            # cycle namespaces
            for i in range(len(self.namespaces)):
                # Subscribers
                gt_topic_agents.append('/' + self.namespaces[i] + '/ground_truth/state')
                self.uwb_sub_agents.append(message_filters.Subscriber(gt_topic_agents[i], Odometry))
                # Publishers
                pub_topic_agents.append('/' + self.namespace + '/Agents/' + self.namespaces[i] + '/range')
                self.uwb_pub_agents.append(rospy.Publisher(pub_topic_agents[i], RD, queue_size=10))
        
        # subscribe and publish
        if self.namespaces:
            list_subscribers = [self.ground_truth_sub_local] + self.uwb_sub_agents
        else:
            list_subscribers = [self.ground_truth_sub_local]
        ts = message_filters.ApproximateTimeSynchronizer(list_subscribers, queue_size=10, slop=0.1)
        ts.registerCallback(self.ground_truth_callback)
        
    # callback for the ground truth
    def ground_truth_callback(self, *args):
        
        # check number of arguments
        if self.namespaces:
            expected_num_inputs = len(self.namespaces) + 1
        else:
            expected_num_inputs = 1
            
        if len(args) != expected_num_inputs:
            raise ValueError(f"Expected {expected_num_inputs} inputs, got {len(args)}")
        else:
            rospy.logwarn_once('GT received!')
            
        # now handle the subscriptions
        # local one
        self.current_gt_local = args[0]
        # agents
        if self.namespaces:
            for i in range(len(self.namespaces)):
                self.current_gt_agents.append(args[i+1])
        

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
        
    # get all the namespaces
    def get_ros_namespaces(self):        
        
        # Get the list of all active nodes
        master = rosgraph.Master('/rostopic')
        node_names = master.getSystemState()[0]
        node_names = [node[0] for sublist in node_names for node in sublist]
        
        # Extract namespaces from node names
        namespaces = set()
        for name in node_names:
            # Split the node name by slashes to get all namespace levels            
            parts = name.split('/')            
            # Reconstruct namespace from parts and add to the set
            if len(parts) > 1:
                namespaces.add(parts[1])
    
        # return
        namespaces = list({name for name in namespaces if name.startswith('UGV')})
        if str(self.namespace) in namespaces:
            namespaces.remove(str(self.namespace))
        self.namespaces = namespaces
        rospy.logwarn_once('Namespaces found: ' + str(namespaces))

    # callback for the transform
    def timer_callback_tf(self, event):
            
        # define frames of the transformation for the local frame
        target_frame = f"{self.namespace}/base_link"
        source_frame = f"{self.namespace}/right_tag"
        
        #  log once
        rospy.loginfo_once('source_frame: {}'.format(source_frame))
        rospy.loginfo_once('target_frame: {}'.format(target_frame))
        
        # get current namespaces
        self.get_ros_namespaces()
        
        # wait for the transformation
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
            self.DCM_local = matrix[:3,:3]
            self.trans_local = np.array(trans).T                        
            
            # now I need to check the same for all the self.namespaces                                    
            if self.namespaces:
                
                self.DCM_MAS = [np.zeros((3, 3)) for _ in range(len(self.namespaces))]
                self.trans_MAS = [np.zeros((3, 1)) for _ in range(len(self.namespaces))]
            
                for i in range(len(self.namespaces)):

                    # define frames of the transformation for the local frame
                    target_frame = f"{self.namespaces[i]}/right_tag"
                    source_frame = f"{self.namespaces[i]}/base_link"
                    
                    # Define target time for the transformation
                    target_time = rospy.Time.now() - rospy.Duration(1.0)  # 1 seconds ago  
                    
                    # Lookup the transform, allowing extrapolation
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,  # target frame
                        source_frame,  # source frame
                        target_time,     # time at which you want the transform
                        rospy.Duration(1.0)  # timeout for looking up the transform
                    )    

                    # assign the transformation
                    trans = np.array((  transform.transform.translation.x, 
                                        transform.transform.translation.y, 
                                        transform.transform.translation.z))
                    rot = np.array((    transform.transform.rotation.x,
                                        transform.transform.rotation.y,
                                        transform.transform.rotation.z,
                                        transform.transform.rotation.w))
                    
                    # correct formatting
                    matrix = tf_conversions.transformations.quaternion_matrix(rot)
                    self.DCM_MAS[i] = matrix[:3,:3]
                    self.trans_MAS[i] = np.array(trans).T
                    
            # if you get here then you have all the transformations available
            self.tf_available = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:            
            rospy.sleep(0.1)
            self.tf_available = False
            rospy.logwarn("Transformation error raised: %s", e)

    # callback for the timer
    def timer_callback(self, event):
        
        # logs
        rospy.loginfo_once('first timer callback')
        rospy.loginfo_once('current_gt_local: {}'.format(self.current_gt_local))
        
        # get current namespaces
        self.get_ros_namespaces()
        self.initop(self.params_name)
        
        # tf available
        if self.current_gt_local is not None and self.tf_available:
            
            # info
            rospy.loginfo_once('publishing')
            self.op.header.stamp = rospy.Time.now()
            agent_pos_local = np.array([
                                self.current_gt_local.pose.pose.position.x,
                                self.current_gt_local.pose.pose.position.y,
                                self.current_gt_local.pose.pose.position.z
                                ]).T
            
            # local position
            tag_pos_local = self.DCM_local@(agent_pos_local + self.trans_local)
            
            # distance from the anchors
            self.op.D = [calc_range_meas(tag_pos_local, np.array(self.op.A_POS[i*3:i*3+3])) for i in range(4)]
            
            # here the gaussian noise is added
            self.op.D = [d + np.random.normal(0, self.range_cov) for d in self.op.D]
            
            # manage the out of range: any element of D > RANGE is set to -1.0
            for i in range(len(self.op.D)):
                if self.op.D[i] > jr.RANGE:
                    self.op.D[i] = -1.0
                                
            # publish 
            self.uwb_pub_anchors.publish(self.op)
            
            # now we do the same with the MAS distances
            if self.namespaces:
                for i in range(len(self.namespaces)):
                    
                    # reset
                    self.op.D = []
                    self.op.A_POS = []
                    self.op.N_ID = []
                    self.op.A_ID = []
                    self.op.T_ID = []
                    self.op.NUM_A = []
                    
                    # set agent as anchor
                    self.op.A_POS = agent_pos_local
                    self.op.A_ID = str(self.namespace)
                    self.op.N_ID = str(self.namespaces[i])
                    self.op.T_ID = 'TAG1'
                    self.op.NUM_A = 1
                    
                    agent_pos_MAS = np.array([
                                self.current_gt_agents[i].pose.pose.position.x,
                                self.current_gt_agents[i].pose.pose.position.y,
                                self.current_gt_agents[i].pose.pose.position.z
                                ]).T
                    
                    # compute transformation
                    try:
                        tag_pos = self.DCM_MAS[i]@(agent_pos_MAS + self.trans_MAS[i])
                    except:
                        rospy.log_fatal('DCM_MAS: ' + self.DCM_MAS)
                    
                    # distance from the anchors
                    d = calc_range_meas(agent_pos_local, agent_pos_MAS)
                    
                    # here the gaussian noise is added
                    d_noise = d + np.random.normal(0, self.range_cov)
                    
                    # manage the out of range: any element of D > RANGE is set to -1.0
                    if d_noise > jr.RANGE:
                            d_noise = -1.0
                        
                    # set to list
                    self.op.D.append(d)
                    
                    # publish 
                    self.uwb_pub_agents[i].publish(self.op)
            
            
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
