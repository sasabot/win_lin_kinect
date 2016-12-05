#include "KinectInterface.hh"

using namespace kinect;
using namespace interface;

//////////////////////////////////////////////////
KinectInterface::KinectInterface(ros::NodeHandle _nh)
  : nh_(_nh), depth_width_(640), depth_height_(360), w_stride_(3), h_stride_(3),
    color_width_(1920), color_height_(1080)
{
  call_points_ =
    nh_.serviceClient<linux_kinect::KinectPoints>("/kinect/request/points");
  call_image_ =
    nh_.serviceClient<linux_kinect::KinectImage>("/kinect/request/image");
  call_centers_ =
    nh_.serviceClient<linux_kinect::KinectRequest>("/kinect/request/centers");
}

//////////////////////////////////////////////////
KinectInterface::~KinectInterface()
{
}

//////////////////////////////////////////////////
sensor_msgs::PointCloud2 KinectInterface::ReadPoints()
{
  linux_kinect::KinectPoints srv;

  if (!call_points_.call(srv)) {
    ROS_WARN("readpoints: service call failed");
    sensor_msgs::PointCloud2 null;
    return null;
  }

  return srv.response.points;
}

//////////////////////////////////////////////////
sensor_msgs::Image KinectInterface::ReadImage()
{
  linux_kinect::KinectImage srv;

  if (!call_image_.call(srv)) {
    ROS_WARN("readimage: service call failed");
    sensor_msgs::Image null;
    return null;
  }

  return srv.response.image;
}

//////////////////////////////////////////////////
std::vector<linux_kinect::Bit> KinectInterface::ImageBounds
(std::vector<std::array<int, 4> > _depth_indicies)
{
  std::vector<linux_kinect::Bit> result;
  result.reserve(_depth_indicies.size());

  for (auto it = _depth_indicies.begin(); it != _depth_indicies.end(); ++it) {
    int idx0 = it->at(0) / depth_height_ * h_stride_ + it->at(0) * w_stride_;
    int pixel0_y = idx0 / color_width_;
    int pixel0_x = idx0 - pixel0_y * color_width_;

    int idx1 = it->at(1) / depth_height_ * h_stride_ + it->at(1) * w_stride_;
    int pixel1_y = idx1 / color_width_;
    int pixel1_x = idx1 - pixel1_y * color_width_;

    int idx2 = it->at(2) / depth_height_ * h_stride_ + it->at(2) * w_stride_;
    int pixel2_y = idx2 / color_width_;
    int pixel2_x = idx2 - pixel2_y * color_width_;

    int idx3 = it->at(3) / depth_height_ * h_stride_ + it->at(3) * w_stride_;
    int pixel3_y = idx3 / color_width_;
    int pixel3_x = idx3 - pixel3_y * color_width_;

    linux_kinect::Bit image_bounds;
    image_bounds.x = std::min({pixel0_x, pixel1_x, pixel2_x, pixel3_x});
    image_bounds.y = std::min({pixel0_y, pixel1_y, pixel2_y, pixel3_y});
    image_bounds.width =
      std::max({pixel0_x, pixel1_x, pixel2_x, pixel3_x}) - image_bounds.x;
    image_bounds.height =
      std::max({pixel0_y, pixel1_y, pixel2_y, pixel3_y}) - image_bounds.y;

    result.push_back(image_bounds);
  }

  return result;
}

//////////////////////////////////////////////////
std::vector<geometry_msgs::Point> KinectInterface::ImageCenters
(std::vector<linux_kinect::Bit> _image_bounds)
{
  linux_kinect::KinectRequest srv;
  srv.request.data.reserve(_image_bounds.size());

  for (auto it = _image_bounds.begin(); it != _image_bounds.end(); ++it)
    srv.request.data.push_back(*it);

  if (!call_centers_.call(srv)) {
    ROS_WARN("imagecenters: service call failed");
    std::vector<geometry_msgs::Point> null;
    return null;
  }

  return srv.response.points;
}
