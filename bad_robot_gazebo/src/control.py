#!/usr/bin/env python

#Bibliotheken einbinden
import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Joy 


cmd_vel = Twist()

# Ermittlung der Distanz
def send_vel(data):
    if data.buttons[0] == 1:
        cmd_vel.linear.x  = 1
        # cmd_vel.angular.z = data.axes[0]
    elif data.buttons[1] == 1:
        cmd_vel.linear.x  = -1
        # cmd_vel.angular.z = data.axes[0]
    else:
        cmd_vel.linear.x = 0
        # cmd_vel.angular.z = 0


    if data.axes[0] > 0.2:
        cmd_vel.angular.z = 0.5
    elif data.axes[0] < -0.2:
        cmd_vel.angular.z = -0.5
    else:
        cmd_vel.angular.z = 0

    # cmd_vel.angular.z = data.axes[0]
    pub1.publish(cmd_vel)

if __name__ == '__main__':
    global pub1
    # Init der Note
    rospy.init_node("U_N_data", anonymous=False)

    # Setup Subscribing
    rospy.Subscriber("/joy", Joy, send_vel)
    pub1 = rospy.Publisher(("/cmd_vel"), Twist, queue_size=10)
    rospy.spin()
