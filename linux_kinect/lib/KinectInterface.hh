#ifndef _LINUX_KINECT_KINECT_INTERFACE_
#define _LINUX_KINECT_KINECT_INTERFACE_

#include <ros/ros.h>

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Point.h"

#include "linux_kinect/KinectPoints.h"
#include "linux_kinect/KinectImage.h"
#include "linux_kinect/KinectRequest.h"
#include "linux_kinect/Bit.h"

namespace kinect
{
  namespace interface
  {

    class KinectInterface
    {
    public: explicit KinectInterface(ros::NodeHandle _nh);

    public: ~KinectInterface();

    public: sensor_msgs::PointCloud2 ReadPoints();

    public: sensor_msgs::Image ReadImage();

    public: std::vector<linux_kinect::Bit> ImageBounds
    (std::vector<std::array<int, 4> > _depth_indicies);

    public: std::vector<geometry_msgs::Point> ImageCenters
    (std::vector<linux_kinect::Bit> _image_bounds);

    private: ros::NodeHandle nh_;

    private: ros::ServiceClient call_points_;

    private: ros::ServiceClient call_image_;

    private: ros::ServiceClient call_centers_;

    private: const int depth_width_;

    private: const int w_stride_;

    private: const int h_stride_;
    };

    typedef std::shared_ptr<KinectInterface> KinectInterfacePtr;

  }
}

#endif
