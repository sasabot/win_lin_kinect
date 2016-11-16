#include <ros/ros.h>
#include "std_msgs/String.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tts_sample");
  ros::NodeHandle nh;

  ros::Publisher text_publisher =
    nh.advertise<std_msgs::String>("/windows/voice", 1);

  // wait for publisher to be ready
  usleep(1000 * 1000);

  // english speech
  std_msgs::String msg;
  msg.data = "speaking in English";
  text_publisher.publish(msg);

  // wait till speech finish
  usleep(5000 * 1000);

  // japanese speech
  msg.data = "日本語で喋ります";
  text_publisher.publish(msg);
}
