#include "linux_kinect/WindowsInterface.hh"

using namespace windows;
using namespace interface;

//////////////////////////////////////////////////
WindowsInterface::WindowsInterface(ros::NodeHandle _nh) : nh_(_nh)
{
  call_ocr_ =
    nh_.serviceClient<linux_kinect::Ocr>("/request/ocr");
}

//////////////////////////////////////////////////
WindowsInterface::~WindowsInterface()
{
}

//////////////////////////////////////////////////
std::vector<std::vector<std::string> > WindowsInterface::OCR
(std::vector<cv::Mat> _images)
{
  linux_kinect::Ocr srv;
  srv.request.images.reserve(_images.size());

  for (auto it = _images.begin(); it != _images.end(); ++it) {
    sensor_msgs::Image image;
    image.width = it->cols;
    image.height = it->rows;
    image.data.reserve(it->rows * it->cols * 3);
    for (unsigned int i = 0; i < it->rows; ++i)
      for (unsigned int j = 0; j < it->cols; ++j) {
        cv::Vec3b color = it->at<cv::Vec3b>(i, j);
        image.data.push_back(color[2]);
        image.data.push_back(color[1]);
        image.data.push_back(color[0]);
      }
    srv.request.images.push_back(image);
  }

  if (!call_ocr_.call(srv)) {
    ROS_WARN("OCR: service call failed");
    std::vector<std::vector<std::string> > null;
    return null;
  }

  std::vector<std::vector<std::string> > result;
  int at = 0;
  for (auto it = srv.response.counts.begin();
       it != srv.response.counts.end(); ++it) {
    int to = at + *it;
    std::vector<std::string> texts(srv.response.texts.begin() + at,
                                   srv.response.texts.begin() + to);
    result.push_back(texts);
    at = to;
  }

  return result;
}
