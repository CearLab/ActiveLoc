#!/usr/bin/env python
# license removed for brevity

# general import
import rospy
import message_filters 
import numpy as np
import colorsys

# message import
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
import message_filters
import tf2_ros
import tf_conversions

# custom message import
from jackal_range.msg import RD_recap as RD
from jackal_range.msg import AnchorsInfo
from general.jackal_general import JackalGeneral
from range.jackal_range import JackalRange

class JackalVis(JackalRange):
    
    marker_array_line = MarkerArray()
    marker_array_anchor = MarkerArray()
    
    def __init__(self, anchors_topic_name, odometry_topic_name, pub_topic, color):
        
        super().__init__()
        
        self.sub_anchors_name = anchors_topic_name
        self.sub_odometry_name = odometry_topic_name
        self.pub_topic = pub_topic
        self.color = np.asarray([float(x) for x in color.split()]) 
        
        # class constructor        
        self.namespace_handler(None)
        
        # other stuff
        rospy.sleep(2)
        self.get_ros_namespaces()

    def publish_line_marker(self):
        
        # Publisher and subscriber local
        self.uwb_pub_topic_markers = rospy.Publisher(self.pub_topic, MarkerArray, queue_size=10)
        
        # get the ground truth topic and the publish topic
        odometry_topic_local = self.sub_odometry_name
        self.ground_truth_sub_local = message_filters.Subscriber(odometry_topic_local, Odometry)
        anchors_topic_local =  self.sub_anchors_name
        self.anchors_sub_local = message_filters.Subscriber(anchors_topic_local, AnchorsInfo)
        
        list_subscribers = [self.ground_truth_sub_local] + [self.anchors_sub_local]
        
        rospy.logwarn(['Publish line anchors: ' + str(tmp.topic) for tmp in list_subscribers])
        ts = message_filters.ApproximateTimeSynchronizer(list_subscribers, queue_size=10, slop=0.1)
        ts.registerCallback(self.vis_line_callback)
        
    def vis_line_callback(self, *args):
        
        odom = args[0]
        anchors = args[1]
        
        if self.color[0] == -1:
            color_gen = 1
        else:
            color_gen = 0
            color = tuple(self.color)   
            
        self.marker_array_line = MarkerArray()
        
        for i in range(self.N_A):
            
            if color_gen == 1:
                color = self.generate_color(i, self.N_A)     
                
            # set marker
            marker_line = Marker()    
            marker_line.header.frame_id = "world"
            marker_line.header.stamp = rospy.Time.now()
            marker_line.ns = "AN" + str(i) + "/line_markers"
            marker_line.id = i
            marker_line.type = Marker.LINE_STRIP
            marker_line.action = Marker.ADD            
            marker_line.scale.x = 0.01  # Line width
            marker_line.color.r = color[0]
            marker_line.color.g = color[1]
            marker_line.color.b = color[2]
            marker_line.color.a = color[3]
            marker_line.pose.orientation.x = 0
            marker_line.pose.orientation.y = 0
            marker_line.pose.orientation.z = 0
            marker_line.pose.orientation.w = 1
            
            pointANC = Point()
            pointANC.x = float(anchors.A_POS[3*i+0])
            pointANC.y = float(anchors.A_POS[3*i+1])
            pointANC.z = float(anchors.A_POS[3*i+2])
            
            pointUGV = Point()
            pos = np.array([odom.pose.pose.position.x, 
                            odom.pose.pose.position.y, 
                            odom.pose.pose.position.z])
            pos_tmp = self.DCM_local_tag@(pos + self.trans_local)
            pointUGV.x = pos_tmp[0]
            pointUGV.y = pos_tmp[1]
            pointUGV.z = pos_tmp[2]
            
            x1, y1, z1 = pointANC.x, pointANC.y, pointANC.z
            x2, y2, z2 = pointUGV.x, pointUGV.y, pointUGV.z
            d = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
            
            if d > self.RANGE:
                pointANC.x = 0.0
                pointANC.y = 0.0
                pointANC.z = 0.0
                pointUGV.x = 0.0
                pointUGV.y = 0.0
                pointUGV.z = 0.0
                
            marker_line.points.append(pointANC)
            marker_line.points.append(pointUGV)                               
            self.marker_array_line.markers.append(marker_line)  
        
        self.uwb_pub_topic_markers.publish(self.marker_array_line)
        
    def publish_line_agents(self):
        
        # Publisher and subscriber local
        odometry_topic_local = '/' + self.namespace + self.sub_odometry_name
        self.ground_truth_sub_local = message_filters.Subscriber(odometry_topic_local, Odometry)
        
        # get other agents
        self.get_ros_namespaces()
        
        if self.namespaces:  
            
            inter_agents_sub = []
            pub_name = '/' + self.namespace + self.pub_topic
            rospy.logwarn(self.namespace + ' MAS line publisher: ' + pub_name)
            self.inter_agents_pub = rospy.Publisher(pub_name, MarkerArray, queue_size=10)
            
            for i in range(len(self.namespaces)):                
                sub_name = '/' + self.namespaces[i] + self.sub_odometry_name
                inter_agents_sub.append(message_filters.Subscriber(sub_name, Odometry))
                
            list_subscribers = [self.ground_truth_sub_local] + inter_agents_sub
            rospy.logwarn(['Subscribe line MAS: ' + str(tmp.topic) for tmp in list_subscribers])
            
            ts = message_filters.ApproximateTimeSynchronizer(list_subscribers, queue_size=10, slop=0.1)
            ts.registerCallback(self.vis_agents_callback)
        
    def vis_agents_callback(self, *args):
        
        if self.color[0] == -1:
            color_gen = 1
        else:
            color_gen = 0
            color = tuple(self.color)
        
        self.marker_array_line = MarkerArray()    
        pos_local = args[0]
        pos_local_arr = np.array((pos_local.pose.pose.position.x, pos_local.pose.pose.position.y, pos_local.pose.pose.position.z))
        pos_local_tmp = self.DCM_local_tag@(pos_local_arr + self.trans_local)
        pos_local.pose.pose.position.x = pos_local_tmp[0]
        pos_local.pose.pose.position.y = pos_local_tmp[1]
        pos_local.pose.pose.position.z = pos_local_tmp[2]
        
        if self.namespaces:
            for i in range(len(self.namespaces)):   
                
                pos_MAS = args[i+1]
                pos_MAS_arr = np.array((pos_MAS.pose.pose.position.x, pos_MAS.pose.pose.position.y, pos_MAS.pose.pose.position.z))
                pos_MAS_tmp = self.DCM_local_tag_agents[i]@(pos_MAS_arr + self.trans_MAS[i])
                pos_MAS.pose.pose.position.x = pos_MAS_tmp[0]
                pos_MAS.pose.pose.position.y = pos_MAS_tmp[1]
                pos_MAS.pose.pose.position.z = pos_MAS_tmp[2]
                
                if color_gen == 1:
                    color = self.generate_color(i, len(self.namespaces))                    
                    
                marker_line = Marker()    
                marker_line.header.frame_id = "world"
                marker_line.header.stamp = rospy.Time.now()
                marker_line.ns = self.namespaces[i] + "/line_markers"
                marker_line.id = i
                marker_line.type = Marker.LINE_STRIP
                marker_line.action = Marker.ADD            
                marker_line.scale.x = 0.01  # Line width
                marker_line.color.r = color[0]
                marker_line.color.g = color[1]
                marker_line.color.b = color[2]
                marker_line.color.a = color[3]
                marker_line.pose.orientation.x = 0
                marker_line.pose.orientation.y = 0
                marker_line.pose.orientation.z = 0
                marker_line.pose.orientation.w = 1
                
                pointANC = Point()
                pointANC.x = pos_local.pose.pose.position.x
                pointANC.y = pos_local.pose.pose.position.y
                pointANC.z = pos_local.pose.pose.position.z
                
                pointUGV = Point()                
                pointUGV.x = pos_MAS.pose.pose.position.x
                pointUGV.y = pos_MAS.pose.pose.position.y
                pointUGV.z = pos_MAS.pose.pose.position.z
                
                x1, y1, z1 = pointANC.x, pointANC.y, pointANC.z
                x2, y2, z2 = pointUGV.x, pointUGV.y, pointUGV.z
                d = np.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
                
                if d > self.RANGE:
                    pointANC.x = 0.0
                    pointANC.y = 0.0
                    pointANC.z = 0.0
                    pointUGV.x = 0.0
                    pointUGV.y = 0.0
                    pointUGV.z = 0.0
                
                marker_line.points.append(pointANC)
                marker_line.points.append(pointUGV)
                self.marker_array_line.markers.append(marker_line)

            self.inter_agents_pub.publish(self.marker_array_line)
    
    def publish_anchors_marker(self):
        
        # Publisher and subscriber local
        self.uwb_pub_topic_markers = rospy.Publisher(self.pub_topic, MarkerArray, queue_size=10)
        
        # get the ground truth topic and the publish topic
        anchors_topic_local =  self.sub_anchors_name
        self.anchors_sub_local = message_filters.Subscriber(anchors_topic_local, AnchorsInfo)
        
        list_subscribers = [self.anchors_sub_local]
        
        rospy.logwarn(['Publish anchor server:' + str(tmp.topic) for tmp in list_subscribers])
        ts = message_filters.ApproximateTimeSynchronizer(list_subscribers, queue_size=10, slop=0.1)
        ts.registerCallback(self.vis_anchor_callback)
    
    def vis_anchor_callback(self, *args):
        
        anchors = args[0]
        # rospy.logwarn(anchors)
        
        if self.color[0] == -1:
            color_gen = 1
        else:
            color_gen = 0
            color = tuple(self.color)   
            
        self.marker_array_anchor = MarkerArray()
        
        for i in range(self.N_A):
            
            if color_gen == 1:
                color = self.generate_color(i, self.N_A)     
                
            # set marker
            marker_anchor = Marker()    
            marker_anchor.header.frame_id = "world"
            marker_anchor.header.stamp = rospy.Time.now()
            marker_anchor.ns = "AN" + str(i) + "/anchor_markers"
            marker_anchor.id = i
            marker_anchor.type = Marker.CUBE
            marker_anchor.action = Marker.ADD            
            marker_anchor.scale.x = 0.25
            marker_anchor.scale.y = 0.25
            marker_anchor.scale.z = 0.25
            marker_anchor.color.r = color[0]
            marker_anchor.color.g = color[1]
            marker_anchor.color.b = color[2]
            marker_anchor.color.a = color[3]
            marker_anchor.pose.position.x = anchors.A_POS[3*i+0]
            marker_anchor.pose.position.y = anchors.A_POS[3*i+1]
            marker_anchor.pose.position.z = anchors.A_POS[3*i+2]
            marker_anchor.pose.orientation.x = 0
            marker_anchor.pose.orientation.y = 0
            marker_anchor.pose.orientation.z = 0
            marker_anchor.pose.orientation.w = 1
            self.marker_array_anchor.markers.append(marker_anchor)  
        
        self.uwb_pub_topic_markers.publish(self.marker_array_anchor)
    
    def generate_color(self,index, total_markers):
            hue = index / float(total_markers)  # Vary the hue between 0 and 1
            saturation = 1.0  # Full saturation
            value = 1.0  # Full brightness
            rgb = colorsys.hsv_to_rgb(hue, saturation, value)
            return rgb + (1.0,)  # Return as (r, g, b, a) tuple with full opacity
        
    # callback for the transform
    def timer_callback_tf(self, event):
            
        # define frames of the transformation for the local frame
        target_frame = f"{self.namespace}/base_link"
        source_frame = f"{self.namespace}/right_tag"
        
        #  log once
        rospy.logwarn_once('source_frame: {}'.format(source_frame))
        rospy.logwarn_once('target_frame: {}'.format(target_frame))
        
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
                    target_frame = f"{self.namespaces[i]}/base_link"
                    source_frame = f"{self.namespaces[i]}/right_tag"
                    
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
    
    def timer_callback(self, event):
        pass