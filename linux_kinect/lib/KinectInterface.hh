#ifndef _LINUX_KINECT_KINECT_INTERFACE_
#define _LINUX_KINECT_KINECT_INTERFACE_

#include <ros/ros.h>

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/Image.h"

#include "linux_kinect/KinectPoints.h"
#include "linux_kinect/KinectImage.h"
#include "linux_kinect/KinectRequest.h"
#include "linux_kinect/Bit.h"
#include "linux_kinect/Cognition.h"
#include "linux_kinect/Tag.h"

namespace kinect
{
  namespace interface
  {

    class Option
    {
    public: Option();

    public: ~Option();

    public: inline void SetCognitionMinImageSize(int _size) {
        cognition_min_image_size_ = _size;
      };

    public: inline void SetCognitionMaxImageSize(int _size) {
        cognition_max_image_size_ = _size;
      };

    public: inline int GetCognitionMinImageSize() {
        return cognition_min_image_size_;
      };

    public: inline int GetCognitionMaxImageSize() {
        return cognition_max_image_size_;
      };

    private: int cognition_min_image_size_;

    private: int cognition_max_image_size_;
    };

    typedef std::shared_ptr<Option> optionptr;


    class KinectInterface
    {
    public: explicit KinectInterface(ros::NodeHandle _nh);

    public: ~KinectInterface();

    public: void ReadKey();

    public: sensor_msgs::PointCloud2 ReadPoints();

    public: sensor_msgs::Image ReadImage();

    public: std::vector<linux_kinect::Bit> ImageBounds
    (std::vector<std::array<int, 4> > _depth_indicies);

    public: linux_kinect::KinectRequest::Response Cognition
    (std::vector<linux_kinect::Bit> _image_bounds, std::vector<int>& _valid_cluster_ids,
     kinect::interface::optionptr opt);

    private: ros::NodeHandle nh_;

    private: ros::ServiceClient call_points_;

    private: ros::ServiceClient call_image_;

    private: ros::ServiceClient call_bounds_;

    private: ros::ServiceClient call_cognition_;

    private: std::string key_;
    };

    typedef std::shared_ptr<KinectInterface> KinectInterfacePtr;

  }
}

#endif
