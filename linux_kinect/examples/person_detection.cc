#include <ros/ros.h>
#include <chrono>
#include "KinectInterface.hh"
#include "linux_kinect/People.h"

void Callback(const linux_kinect::People::ConstPtr& _msg)
{
  ROS_WARN("got face!");
  ROS_INFO("face3d: %f, %f, %f", _msg->data[0].face3d.x,
           _msg->data[0].face3d.y, _msg->data[0].face3d.z);
  ROS_INFO("face6d: %f, %f, %f", _msg->data[0].face6d.x,
           _msg->data[0].face6d.y, _msg->data[0].face6d.z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "person_detection_sample");
  ros::NodeHandle nh;

  kinect::interface::KinectInterfacePtr kinect
    (new kinect::interface::KinectInterface(nh));

  ros::Subscriber person_watcher =
    nh.subscribe("/kinect/person/targets", 1000, Callback);

  kinect->StartPersonStream();

  // watch 30 seconds
  auto start = std::chrono::high_resolution_clock::now();
  ros::Rate r(10);
  while (std::chrono::duration_cast<std::chrono::milliseconds>
         (std::chrono::high_resolution_clock::now() - start).count()
         < 30000) {
    ros::spinOnce();
    r.sleep();
  }

  kinect->StopPersonStream();
}
