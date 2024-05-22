#!/usr/bin/env python
# license removed for brevity

# imports
import rospy
import jackal_custom as jc

if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster')        
    name = rospy.get_param('~TF2name')
    rospy.loginfo(name)
    rospy.Subscriber('/%sodometry/filtered' % name,
                    jc.nav_msgs.msg.Odometry,
                    jc.handle_pose,
                    '%sodom' %name)
    rospy.spin()