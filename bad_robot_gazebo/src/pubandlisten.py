#!/usr/bin/env python

import rospy
from std_msgs.msg import Time
import time





def repeater(data):
    
    k = Time(rospy.Time.now())
    b = Time()
    b.data.nsecs = k.data.nsecs-data.data.nsecs
    b.data.secs = k.data.secs-data.data.secs
    print(b)
    rospy.sleep(0.1)
    pub1.publish(rospy.Time.now())

if __name__ == "__main__":
    global pub1
    # Init der Note
    rospy.init_node("PubandListen", anonymous=False)
    rospy.sleep(0.1)
    # Setup Subscribing
    pub1 = rospy.Publisher(("/listen"), Time, queue_size=10)
    rospy.sleep(0.1)
    rospy.Subscriber("/listen", Time, repeater)
    rospy.sleep(0.1)
    pub1.publish(rospy.Time.now())

    rospy.spin()
