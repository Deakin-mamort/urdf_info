#!/usr/bin/env python
import roslib
#roslib.load_manifest('cs225b')
import rospy
import tf
import math

if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')

    br = tf.TransformBroadcaster()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        br.sendTransform((0.5, 1.0, 0),
                         tf.transformations.quaternion_from_euler(0, 0, 30*math.pi/180),
                         rospy.Time.now(),
                         "A",     # child
                         "W"      # parent
                         )
        rate.sleep()
