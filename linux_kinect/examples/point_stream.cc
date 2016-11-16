#include <ros/ros.h>
#include <chrono>
#include "KinectInterface.hh"

std::chrono::high_resolution_clock::time_point t;

void Callback(const sensor_msgs::PointCloud2::ConstPtr& _msg)
{
  ROS_INFO("got points %f",
           static_cast<float>
           (std::chrono::duration_cast<std::chrono::milliseconds>
            (std::chrono::high_resolution_clock::now() - t).count()));
  t = std::chrono::high_resolution_clock::now();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "point_stream_sample");
  ros::NodeHandle nh;

  kinect::interface::KinectInterfacePtr kinect
    (new kinect::interface::KinectInterface(nh));

  ros::Subscriber stream =
    nh.subscribe("/kinect/stream", 1000, Callback);

  kinect->StartPointStream();

  // get points for 30 seconds
  auto start = std::chrono::high_resolution_clock::now();
  ros::Rate r(10);
  t = std::chrono::high_resolution_clock::now();
  while (std::chrono::duration_cast<std::chrono::milliseconds>
         (std::chrono::high_resolution_clock::now() - start).count()
         < 30000) {
    ros::spinOnce();
    r.sleep();
  }

  kinect->StopPointStream();
}
