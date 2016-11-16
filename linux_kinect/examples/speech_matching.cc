#include <ros/ros.h>
#include <chrono>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

void Callback(const std_msgs::String::ConstPtr& _msg)
{
  ROS_INFO("speech: %s", _msg->data.c_str());
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "speech_matching_sample");
  ros::NodeHandle nh;

  ros::Publisher speech_detection_settings_publisher =
    nh.advertise<std_msgs::Bool>("/stt/trigger/manual", 1);

  ros::Subscriber speech_listener =
    nh.subscribe("/kinect/voice", 1000, Callback);

  // wait for publisher to be ready
  usleep(1000 * 1000);

  // start listening
  std_msgs::Bool flag;
  flag.data = true;
  speech_detection_settings_publisher.publish(flag);

  // listen 10 seconds
  auto start = std::chrono::high_resolution_clock::now();
  ros::Rate r(10);
  while (std::chrono::duration_cast<std::chrono::milliseconds>
         (std::chrono::high_resolution_clock::now() - start).count()
         < 10000) {
    ros::spinOnce();
    r.sleep();
  }

  // end listening
  flag.data = false;
  speech_detection_settings_publisher.publish(flag);
}
