#include <ros/ros.h>
#include "sensor_msgs/CameraInfo.h"
#include "linux_kinect/KinectCameraInfo.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_info_sample");
  ros::NodeHandle nh;

  ros::Publisher info_publisher =
    nh.advertise<sensor_msgs::CameraInfo>("/kinect/camera_info", 1);

  ros::ServiceClient info_client =
    nh.serviceClient<linux_kinect::KinectCameraInfo>
    ("/kinect/request/camera_info");

  linux_kinect::KinectCameraInfo ci;
  while (!info_client.call(ci)) {
    ROS_WARN("failed camera info");
    return -1;
  }

  sensor_msgs::CameraInfo info;
  info.K[0] = ci.response.fx;
  info.K[1] = 0;
  info.K[2] = ci.response.cx;
  info.K[3] = 0;
  info.K[4] = ci.response.fy;
  info.K[5] = ci.response.cy;
  info.K[6] = 0;
  info.K[7] = 0;
  info.K[8] = 1;

  ros::Rate r(10);
  while (ros::ok()) {
    info_publisher.publish(info);
    r.sleep();
  }
}
