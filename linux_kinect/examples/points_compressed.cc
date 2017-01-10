#include <ros/ros.h>
#include <chrono>
#include "linux_kinect/KinectInterface.hh"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_points_sample");
  ros::NodeHandle nh;

  kinect::interface::KinectInterfacePtr kinect
    (new kinect::interface::KinectInterface(nh));

  ros::Publisher points_publisher =
    nh.advertise<sensor_msgs::PointCloud2>("/kinect/pointstream/compressed", 1);

  ros::Rate r(1);
  while (ros::ok()) {
    auto start = std::chrono::high_resolution_clock::now();

    auto points = kinect->ReadPoints(0.25, 0.334);

    ROS_INFO("finished read points %f",
             static_cast<float>
             (std::chrono::duration_cast<std::chrono::milliseconds>
              (std::chrono::high_resolution_clock::now() - start).count()));
    points_publisher.publish(points);

    r.sleep();
  }
}
