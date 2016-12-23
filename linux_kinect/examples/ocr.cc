#include <ros/ros.h>
#include "WindowsInterface.hh"

#include "sensor_msgs/Image.h"
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ocr_sample");
  ros::NodeHandle nh;

  if (argc != 2)
    ROS_ERROR("please enter a filename to conduct OCR!");

  windows::interface::WindowsInterfacePtr windows
    (new windows::interface::WindowsInterface(nh));

  auto image = cv::imread(std::string(argv[1]));

  // get OCR result
  auto result = windows->OCR({image});

  for (auto it = result.begin(); it != result.end(); ++it) {
    for (auto p = it->begin(); p != it->end(); ++p)
      std::cout << *p << std::endl;
    std::cout << "----" << std::endl;
  }
}
