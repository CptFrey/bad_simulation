#include "ros/ros.h"
#include "std_msgs/Time.h"

#include <sstream>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::Time>("chatter", 1000);
  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::Time t_message;

    t_message = ros::Time::now();
    chatter_pub.publish(t_message);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}