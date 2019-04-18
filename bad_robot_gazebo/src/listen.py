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


if __name__ == "__main__":
    global pub1
    # Init der Note
    rospy.init_node("Listen", anonymous=False)
    rospy.sleep(0.1)
    rospy.Subscriber("/listen", Time, repeater)
    rospy.sleep(0.1)

    rospy.spin()
