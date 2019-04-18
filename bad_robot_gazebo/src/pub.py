#!/usr/bin/env python

import rospy
from std_msgs.msg import Time
import time

if __name__ == "__main__":
    global pub1
    # Init der Note
    rospy.init_node("Pub", anonymous=False)
    rospy.sleep(0.1)
    # Setup Subscribing
    pub1 = rospy.Publisher(("/listen"), Time, queue_size=10)
    rospy.sleep(0.1)
    pub1.publish(rospy.Time.now())

    rospy.spin()
