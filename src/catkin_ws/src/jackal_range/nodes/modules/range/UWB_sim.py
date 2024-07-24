#!/usr/bin/env python

import rospy
from range.jackal_range import JackalRange
from jackal_range.msg import RD_recap as RD
from jackal_range.msg import AnchorsInfo
from nav_msgs.msg import Odometry
import tf2_ros
import numpy as np
import tf_conversions
import message_filters 

class UWB_sim(JackalRange):
    
    tf_available = False
    current_gt_local = None
    current_gt_agents = []
    range_cov = 0.0
    
    def __init__(self, anchors_topic_name, odometry_topic_name, range_meas_cov):
        
        # Inherit
        super().__init__()
        
        # range message                
        self.range_cov = range_meas_cov
        self.op = RD()
        self.sub_anchors_name = anchors_topic_name
        self.sub_odometry_name = odometry_topic_name
        
        # class constructor        
        self.namespace_handler(None)
        
        # other stuff
        rospy.sleep(2)
        self.get_ros_namespaces()
        self.pubsub()

    # publish message
    def pubsub(self):
        
        # get the ground truth topic and the publish topic
        odometry_topic_local = '/' + self.namespace + '/' + self.sub_odometry_name
        self.ground_truth_sub_local = message_filters.Subscriber(odometry_topic_local, Odometry)
        anchors_topic_local =  '/' + self.namespace + '/' + self.sub_anchors_name
        self.anchors_sub_local = message_filters.Subscriber(anchors_topic_local, AnchorsInfo)        
        
        # Publisher and subscriber local
        pub_topic_anchors = '/' + self.namespace + '/Anchors/range'
        self.uwb_pub_anchors = rospy.Publisher(pub_topic_anchors, RD, queue_size=10)        
        
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
                gt_topic_agents.append('/' + self.namespaces[i] + '/' + self.sub_odometry_name)
                self.uwb_sub_agents.append(message_filters.Subscriber(gt_topic_agents[i], Odometry))
                
                # Publishers
                pub_topic_agents.append('/' + self.namespace + '/Agents/' + self.namespaces[i] + '/range')
                self.uwb_pub_agents.append(rospy.Publisher(pub_topic_agents[i], RD, queue_size=10))
        
        # subscribe and publish
        if self.namespaces:
            list_subscribers = [self.ground_truth_sub_local] + [self.anchors_sub_local] + self.uwb_sub_agents
        else:
            list_subscribers = [self.ground_truth_sub_local] + [self.anchors_sub_local]
        
        rospy.logwarn([str(tmp.topic) for tmp in list_subscribers])
        ts = message_filters.ApproximateTimeSynchronizer(list_subscribers, queue_size=10, slop=0.1)
        ts.registerCallback(self.distance_callback)
        
    # callback for the ground truth
    def distance_callback(self, *args):
        
        # local one
        self.current_gt_local = args[0]
        self.op.A_POS = args[1].A_POS
        
        # agents
        if self.namespaces:
            self.current_gt_agents = []
            for i in range(len(self.namespaces)):
                self.current_gt_agents.append(args[2+i])
                
        # tf available
        if self.current_gt_local is not None and self.tf_available:
            
            # info
            rospy.logwarn_once('publishing')
            self.op.header.stamp = rospy.Time.now()
            agent_pos_local = np.array([
                                self.current_gt_local.pose.pose.position.x,
                                self.current_gt_local.pose.pose.position.y,
                                self.current_gt_local.pose.pose.position.z
                                ]).T
            
            # local position
            tag_pos_local = self.DCM_local_tag@(agent_pos_local + self.trans_local)
            # distance from the anchors
            self.op.D = [self.calc_range_meas(tag_pos_local, np.array(self.op.A_POS[i*3:i*3+3])) for i in range(self.N_A)]
            
            # here the gaussian noise is added
            self.op.D = [d + np.random.normal(0, self.range_cov) for d in self.op.D]
            
            # manage the out of range: any element of D > RANGE is set to -1.0
            for i in range(len(self.op.D)):
                if self.op.D[i] > self.RANGE:
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
                        tag_pos_MAS = self.DCM_local_tag_agents[i]@(agent_pos_MAS + self.trans_MAS[i])
                    except Exception as e:
                        rospy.logfatal('DCM_local_tag_agents: ' + str(e))
                    
                    # distance from the anchors
                    d = self.calc_range_meas(tag_pos_local, tag_pos_MAS)
                    
                    # here the gaussian noise is added
                    d_noise = d + np.random.normal(0, self.range_cov)
                    
                    # manage the out of range: any element of D > RANGE is set to -1.0
                    if d_noise > self.RANGE:
                            d_noise = -1.0
                        
                    # set to list
                    self.op.D.append(d)
                    
                    # publish 
                    self.uwb_pub_agents[i].publish(self.op)
            
            
            rospy.logwarn_once('first published')

    # callback for the transform
    def timer_callback_tf(self, event):
            
        # define frames of the transformation for the local frame
        target_frame = f"{self.namespace}/base_link"
        source_frame = f"{self.namespace}/right_tag"
        
        #  log once
        rospy.logwarn_once('source_frame: {}'.format(source_frame))
        rospy.logwarn_once('target_frame: {}'.format(target_frame))
        
        # get current namespaces
        self.get_ros_namespaces()
        
        # wait for the transformation
        try:          
            
            # Define target time for the transformation
            target_time = rospy.Time.now() - rospy.Duration(1/self.RATE)  # 1 seconds ago  
            
            # Lookup the transform, allowing extrapolation
            transform = self.tf_buffer.lookup_transform(
                target_frame,  # target frame
                source_frame,  # source frame
                target_time,     # time at which you want the transform
                rospy.Duration(1/self.RATE)  # timeout for looking up the transform
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
            
            # now I need to check the same for all the self.namespaces                                    
            if self.namespaces:
                
                self.DCM_local_tag_agents = [np.zeros((3, 3)) for _ in range(len(self.namespaces))]
                self.trans_MAS = [np.zeros((3, 1)) for _ in range(len(self.namespaces))]
            
                for i in range(len(self.namespaces)):

                    # define frames of the transformation for the local frame
                    target_frame = f"{self.namespaces[i]}/right_tag"
                    source_frame = f"{self.namespaces[i]}/base_link"
                    
                    # Define target time for the transformation
                    target_time = rospy.Time.now() - rospy.Duration(1/self.RATE)  # 1 seconds ago  
                    
                    # Lookup the transform, allowing extrapolation
                    transform = self.tf_buffer.lookup_transform(
                        target_frame,  # target frame
                        source_frame,  # source frame
                        target_time,     # time at which you want the transform
                        rospy.Duration(1/self.RATE)  # timeout for looking up the transform
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
                    self.DCM_local_tag_agents[i] = matrix[:3,:3]
                    self.trans_MAS[i] = np.array(trans).T
                    
            # if you get here then you have all the transformations available
            self.tf_available = True

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:            
            rospy.sleep(0.1)
            self.tf_available = False
            rospy.logwarn("Transformation error raised: %s", e)

    # callback for the timer
    def timer_callback(self, event):
        pass
        
        