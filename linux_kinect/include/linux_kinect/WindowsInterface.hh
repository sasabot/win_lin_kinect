#ifndef _LINUX_KINECT_WINDOWS_INTERFACE_
#define _LINUX_KINECT_WINDOWS_INTERFACE_

#include <ros/ros.h>

#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "sensor_msgs/Image.h"

#include "linux_kinect/Ocr.h"

namespace windows
{
  namespace interface
  {

    class WindowsInterface
    {
    public: explicit WindowsInterface(ros::NodeHandle _nh);

    public: ~WindowsInterface();

    public: std::vector<std::vector<std::string> > OCR
    (std::vector<cv::Mat> _images);

    private: ros::NodeHandle nh_;

    private: ros::ServiceClient call_ocr_;
    };

    typedef std::shared_ptr<WindowsInterface> WindowsInterfacePtr;

  }
}

#endif
