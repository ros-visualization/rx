#!/usr/bin/env python
import roslib
roslib.load_manifest('rxtools')
# maybe not so kosher but then again we probably don't want geometry_msgs in the manifest
# just for these tests, nor do we want to create a whole new package just for the tests:
roslib.load_manifest('geometry_msgs') 
import rospy

from geometry_msgs.msg import Vector3Stamped

import numpy as np

def main():
    rospy.init_node('rxplot_stress_test_datagen')
    r = rospy.Rate(50)
    pub = rospy.Publisher('data', Vector3Stamped)
    t_start = rospy.Time.now()
    T = 5.0
    msg = Vector3Stamped()
    while not rospy.is_shutdown():
        msg.header.stamp = rospy.Time.now()
        t = (msg.header.stamp - t_start).to_sec()
        msg.vector.x = np.sin(t/T*2*np.pi)
        msg.vector.y = np.cos(t/T*2*np.pi)
        msg.vector.z = msg.vector.x**2 - msg.vector.y**2
        pub.publish(msg)
        r.sleep()
        
if __name__ == "__main__":
    main()