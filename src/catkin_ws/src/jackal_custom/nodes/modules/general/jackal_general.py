#!/usr/bin/env python
# license removed for brevity

# general import
import rospy
import numpy as np
import rosgraph
import tf_conversions
import tf2_ros

class JackalGeneral:
    
    # rate
    RATE = 10 #(Hz)
    
    def __init__(self):                
                
        # number of anchors
        self.NUM_A = len(self.ID)
        
        # get namespace
        self.namespace_handler(None)
        self.get_ros_namespaces()
        
        # define timers
        self.timers()
        
        # define transform
        tmp_trans = np.array([0.0, 0.0, 0.0])
        tmp_rot = np.array([0.0, 0.0, 0.0, 1.0])   
        tmp_matrix = tf_conversions.transformations.quaternion_matrix(tmp_rot)
        self.DCM_local_tag = tmp_matrix[:3,:3]
        self.trans_local = np.array(tmp_trans).T

    # define timers
    def timers(self):
        
        # Create a buffer and a listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))  # 10 seconds cache time
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.event_timer_tf = rospy.Timer(rospy.Duration(1/self.RATE), self.timer_callback_tf) 
        self.event_timer = rospy.Timer(rospy.Duration(1/self.RATE), self.timer_callback) 
        
    def timer_callback_tf(self, event):
        pass
    
    def timer_callback(self, event):
        pass
    
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
        
        # logs
        if self.namespaces:
            rospy.loginfo_once('Namespaces found: ' + str(namespaces))
        else:
            rospy.loginfo_once('No namespaces found')
        
    # method to calculate the range measurement from the agent to the beacon
    def calc_range_meas(self, agent_loc, beacon_loc):    
        return np.linalg.norm(agent_loc - beacon_loc)
    
    # callback for the transform
    def sort_points_clockwise(self,points):
        
        # Compute the distances from the origin
        distances = np.sqrt(points[:,0]**2 + points[:,1]**2)

        # Find the point closest to the origin        
        origin = np.mean(points, axis=0)
        points_shifted = points - origin

        # Compute the angles from the origin
        angles = np.arctan2(points_shifted[:,1], points_shifted[:,0])

        # Sort the points by the angles in clockwise order
        sorted_indices = np.argsort(-angles)
        sorted_indices = np.roll(sorted_indices, shift=1, axis=0)
        sorted_points = points[sorted_indices]

        return sorted_points, sorted_indices