#include <ros/ros.h>
#include <chrono>
#include "KinectInterface.hh"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kinect_image_sample");
  ros::NodeHandle nh;

  kinect::interface::KinectInterfacePtr kinect
    (new kinect::interface::KinectInterface(nh));

  ros::Publisher image_publisher =
    nh.advertise<sensor_msgs::Image>("/kinect/pixelstream", 1);

  ros::Rate r(1);
  while (ros::ok()) {
    auto start = std::chrono::high_resolution_clock::now();

    auto image = kinect->ReadImage();

    ROS_INFO("finished read image %f",
             static_cast<float>
             (std::chrono::duration_cast<std::chrono::milliseconds>
              (std::chrono::high_resolution_clock::now() - start).count()));

    image_publisher.publish(image);
    r.sleep();
  }
}
