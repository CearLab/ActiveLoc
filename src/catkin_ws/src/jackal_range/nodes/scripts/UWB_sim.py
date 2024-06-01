import rospy
from jackal_range.msg import RD_recap as RD
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf
import numpy as np
import sys
import tf.transformations

RATE = 10
NS = None

def calc_range_meas(agent_loc,beacon_loc):
    return np.linalg.norm(agent_loc - beacon_loc)

if len(sys.argv) > 1:
    NS = sys.argv[1]

class UWB:
    
    def __init__(self):
        rospy.loginfo('starting init')
        self.tf_available = False
        self.current_gt = None
        self.namespace_handler()
        self.initop()
        self.pubsub()
        self.timers()
        rospy.loginfo('finished init')


    def pubsub(self):
        gt_topic = f'/{self.namsepace}/ground_truth/state'
        pub_topic = f'/{self.namsepace}/range'
        rospy.loginfo('gt_topic: {}'.format(gt_topic))
        rospy.loginfo('pub_topic: {}'.format(pub_topic))
        self.grond_truth_sub = rospy.Subscriber(gt_topic, Odometry, self.ground_truth_callback)
        self.uwb_pub = rospy.Publisher(pub_topic, RD, queue_size=10)


    def timers(self):
        self.main_timer = rospy.Timer(rospy.Duration(1/RATE), self.timer_callback)
        self.tf_listener = tf.TransformListener()
        self.tf_timer = rospy.Timer(rospy.Duration(1.0), self.timer_callback_tf)


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


    def initop(self):
        self.op = RD()
        self.op.A_ID = [f'AN{i}' for i in range(3)]
        self.op.A_POS = [-5.,-5.,0., -5.,5.,0., 5.,5.,0., 5.,-5.,0.]
        self.op.N_ID = 'A'
        self.op.T_ID = 'TAG1'
        self.op.NUM_A = 4


    def ground_truth_callback(self, data: Odometry):
        rospy.loginfo_once('GT received')
        self.current_gt = data


    def timer_callback_tf(self, event):
        odom_frame = f"/{self.namsepace}/odom"
        right_tag_frame = f"/{self.namsepace}/right_tag"
        rospy.loginfo_once('odom_frame: {}'.format(odom_frame))
        rospy.loginfo_once('right_tag_frame: {}'.format(right_tag_frame))
        try:
            # Wait for the transform to be available
            self.tf_listener.waitForTransform(odom_frame, right_tag_frame, rospy.Time(), rospy.Duration(4.0))

            try:
                (trans, rot) = self.tf_listener.lookupTransform(odom_frame, right_tag_frame, rospy.Time(0))
                rospy.loginfo_once("Transform found: translation %s, rotation %s", str(trans), str(rot))
                self.DCM = tf.transformations.quaternion_matrix(rot)[:3,:3]
                self.trans = np.array(trans).T
                rospy.loginfo_once("Transform available")
                rospy.loginfo_once("DCM: \n {}".format(self.DCM))
                rospy.loginfo_once("trans: {}".format(self.trans))
                self.tf_available = True

            except tf.ExtrapolationException as e:
                rospy.loginfo_once("Extrapolation error: %s", e)
                # Attempt to use the latest available transform
                latest_common_time = self.tf_listener.getLatestCommonTime(odom_frame, right_tag_frame)
                (trans, rot) = self.tf_listener.lookupTransform(odom_frame, right_tag_frame, latest_common_time)
                rospy.loginfo_once("Using latest available transform: translation %s, rotation %s", str(trans), str(rot))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("Transformation error: %s", e)


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
            self.op.D = [calc_range_meas(agent_pos, np.array(self.op.A_POS[i*3:i*3+3])) for i in range(4)]
            self.uwb_pub.publish(self.op)
            rospy.loginfo_once('first published')


    def run(self):
        rospy.loginfo('running')
        rospy.spin()


if __name__ == '__main__':
    try:
        rospy.init_node('UWB_SIM', anonymous=True)
        rospy.loginfo('++++++++++++++++++starting node+++++++++++++++++')
        uwb = UWB()
        uwb.run()

    except rospy.ROSInterruptException:
        rospy.loginfo("UWB failed")
        pass
