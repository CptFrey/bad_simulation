#!/usr/bin/env python

#Bibliotheken einbinden
import rospy
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Joy 
from ackermann_msgs.msg import AckermannDrive


cmd_vel = AckermannDrive()
cmd_vel.steering_angle_velocity = 0
cmd_vel.acceleration = 0
cmd_vel.jerk = 0

# Ermittlung der Distanz
def send_vel(data):
    if data.buttons[0] == 1:
        cmd_vel.speed  = 1
    elif data.buttons[1] == 1:
        cmd_vel.speed  = -1
    else:
        cmd_vel.speed = 0
    # Rotation
    if data.axes[0] > 0.07:
        cmd_vel.steering_angle = data.axes[0]
    elif data.axes[0] < -0.07:
        cmd_vel.steering_angle = data.axes[0]
    else:
        cmd_vel.steering_angle = 0
    # cmd_vel.steering_angle = data.axes[0]
    pub1.publish(cmd_vel)


if __name__ == '__main__':
    global pub1
    # Init der Note
    rospy.init_node("Joy_to_Ackermann", anonymous=False)

    # Setup Subscribing
    rospy.Subscriber("/joy", Joy, send_vel)
    pub1 = rospy.Publisher(("/ackermann_cmd"), AckermannDrive, queue_size=10)

    
    rospy.spin()