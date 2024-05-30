import rospy
from jackal_range.msg import RD_recap as RD
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf
import numpy as np

import tf.transformations
RATE = 10

def calc_range_meas(agent_loc,beacon_loc):
    return np.linalg.norm(agent_loc - beacon_loc)
    

class UWB:
    def __init__(self):
        rospy.loginfo('starting init')
        self.tf_available = False
        self.uwb_pub = rospy.Publisher('range', RD, queue_size=10)
        self.op = RD()
        self.op.A_ID = [f'AN{i}' for i in range(3)]
        self.op.A_POS = [-5.,-5.,0., -5.,5.,0., 5.,5.,0., 5.,-5.,0.]
        self.op.N_ID = 'A'
        self.op.T_ID = 'TAG1'
        self.op.NUM_A = 4
        self.current_gt = None
        self.timer = rospy.Timer(rospy.Duration(1/RATE), self.timer_callback)
        self.grond_truth_sub = rospy.Subscriber('/UGV01/ground_truth/state', Odometry, self.ground_truth_callback)
        self.tf_listener = tf.TransformListener()
        rospy.Timer(rospy.Duration(1.0), self.timer_callback_tf)
        
        rospy.loginfo('finished init')

    def ground_truth_callback(self, data: Odometry):
        rospy.logdebug('GT received')
        self.current_gt = data
    
    def timer_callback_tf(self, event):
        try:
            # Wait for the transform to be available
            self.tf_listener.waitForTransform('/UGV01/odom', '/UGV01/right_tag', rospy.Time(), rospy.Duration(4.0))
            
            try:
                (trans, rot) = self.tf_listener.lookupTransform('/UGV01/odom', '/UGV01/right_tag', rospy.Time(0))
                rospy.logdebug("Transform found: translation %s, rotation %s", str(trans), str(rot))
                self.DCM = tf.transformations.quaternion_matrix(rot)[:3,:3]
                self.trans = np.array(trans).T
                if not self.tf_available:
                    rospy.loginfo("Transform available")
                    rospy.loginfo("DCM: {}".format(self.DCM))
                    rospy.loginfo("trans: {}".format(self.trans))
                self.tf_available = True

            except tf.ExtrapolationException as e:
                rospy.logdebug("Extrapolation error: %s", e)
                # Attempt to use the latest available transform
                latest_common_time = self.tf_listener.getLatestCommonTime('/UGV01/odom', '/UGV01/right_tag')
                (trans, rot) = self.tf_listener.lookupTransform('/UGV01/odom', '/UGV01/right_tag', latest_common_time)
                rospy.logdebug("Using latest available transform: translation %s, rotation %s", str(trans), str(rot))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transformation error: %s", e)
    
    def timer_callback(self, event):
        rospy.logdebug('timer callback')
        rospy.logdebug('current_gt: {}'.format(self.current_gt))
        if self.current_gt is not None and self.tf_available:
            self.op.header.stamp = rospy.Time.now()
            agent_pos = np.array([self.current_gt.pose.pose.position.x,
                                self.current_gt.pose.pose.position.y,
                                self.current_gt.pose.pose.position.z]).T
            agent_pos = self.DCM@(agent_pos + self.trans)
            rospy.logdebug('agent_pos: {}'.format(agent_pos))
            self.op.D = [calc_range_meas(agent_pos, np.array(self.op.A_POS[i*3:i*3+3])) for i in range(4)]
            self.uwb_pub.publish(self.op)
            rospy.logdebug('published')
    def run(self):
        rospy.loginfo('running')
        rospy.spin()
        
if __name__ == '__main__':
    try:
        
        rospy.init_node('UWB_SIM', anonymous=True)
        uwb = UWB()
        # uwb.tf_listener = tf.TransformListener()
        # uwb.tf_listener.waitForTransform('/UGV01/odom', '/UGV01/right_tag', rospy.Time(0), rospy.Duration(4.0))
        # (trans,rot) = uwb.tf_listener.lookupTransform('/UGV01/odom', '/UGV01/right_tag', rospy.Time(0))

        uwb.run()
        
    except rospy.ROSInterruptException:
        rospy.loginfo("UWB failed")
        pass