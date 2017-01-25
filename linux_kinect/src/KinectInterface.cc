#include "linux_kinect/KinectInterface.hh"

using namespace kinect;
using namespace interface;

//////////////////////////////////////////////////
KinectInterface::KinectInterface(ros::NodeHandle _nh)
  : nh_(_nh), depth_width_(640), w_stride_(3), h_stride_(3)
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
sensor_msgs::PointCloud2 KinectInterface::ReadPoints(std::string _filename)
{
  linux_kinect::KinectPoints srv;
  srv.request.filename = _filename; // in case read from file

  if (!call_points_.call(srv)) {
    ROS_WARN("readpoints: service call failed");
    sensor_msgs::PointCloud2 null;
    return null;
  }

  return srv.response.points;
}

//////////////////////////////////////////////////
sensor_msgs::PointCloud2 KinectInterface::ReadPoints
(float _scale_x, float _scale_y, std::string _filename)
{
  linux_kinect::KinectPoints srv;
  srv.request.filename = _filename;

  if (!call_points_.call(srv)) {
    ROS_WARN("readpoints: service call failed");
    sensor_msgs::PointCloud2 null;
    return null;
  }

  auto depth = srv.response.points;
  auto res = sensor_msgs::PointCloud2();
  res.header.frame_id = depth.header.frame_id;
  res.header.stamp = depth.header.stamp;
  res.fields.assign(depth.fields.begin(), depth.fields.end());
  res.height = static_cast<int>(depth.height * _scale_y);
  res.width = static_cast<int>(depth.width * _scale_x);
  res.point_step = depth.point_step;
  res.row_step = res.point_step * res.width;
  res.is_dense = depth.is_dense;
  res.is_bigendian = depth.is_bigendian;

  float stride_x;
  float stride_y;

  if (_scale_x > 0.5 || _scale_y > 0.5) {
    ROS_WARN("scale larger than 0.5 may cause some errors.");
    stride_x = 1.0 / _scale_x;
    stride_y = 1.0 / _scale_y;
  } else {
    // 1.01 for scale 0.334, note, points should not be compressed with scale ~0.01
    stride_x = static_cast<int>(1.01 / _scale_x);
    stride_y = static_cast<int>(1.01 / _scale_y);
  }

  // compress point cloud
  res.data.resize(res.height * res.width * res.point_step);
  float row = 0.0f;
  float at = 0.0f;
  int head = 0;
  int j = 0;
  while (j < res.data.size()) {
    int tl = static_cast<int>(head + at) * depth.point_step; // top left
    int tr = static_cast<int>(head + at + stride_x - 1) * depth.point_step; // top right
    int bl = static_cast<int>(row + stride_y - 1) * depth.row_step
      + static_cast<int>(at) * depth.point_step; // bottom left
    int br = static_cast<int>(row + stride_y - 1) * depth.row_step
      + static_cast<int>(at + stride_x - 1) * depth.point_step; // bottom right

    // get xyz
    float val[3] = {std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN(),
                    std::numeric_limits<float>::quiet_NaN()};
    for (int i = 0; i < 3; ++i) {
      uint8_t tl_bytes[4] =
        {depth.data[tl++], depth.data[tl++], depth.data[tl++], depth.data[tl++]};
      float tl_float;
      std::memcpy(&tl_float, &tl_bytes, 4);

      if (std::isnan(tl_float)) {
        tr += 4; bl += 4; br += 4;
        continue;
      }

      uint8_t tr_bytes[4] =
        {depth.data[tr++], depth.data[tr++], depth.data[tr++], depth.data[tr++]};
      float tr_float;
      std::memcpy(&tr_float, &tr_bytes, 4);

      if (std::isnan(tr_float)) {
        bl += 4; br += 4;
        continue;
      }
      uint8_t bl_bytes[4] =
        {depth.data[bl++], depth.data[bl++], depth.data[bl++], depth.data[bl++]};
      float bl_float;
      std::memcpy(&bl_float, &bl_bytes, 4);

      if (std::isnan(bl_float)) {
        br += 4;
        continue;
      }

      uint8_t br_bytes[4] =
        {depth.data[br++], depth.data[br++], depth.data[br++], depth.data[br++]};
      float br_float;
      std::memcpy(&br_float, &br_bytes, 4);

      if (std::isnan(br_float))
        continue;

      val[i] = (tl_float + tr_float + bl_float + br_float) * 0.25;
    }

    auto x = reinterpret_cast<uint8_t*>(&val[0]);
    auto y = reinterpret_cast<uint8_t*>(&val[1]);
    auto z = reinterpret_cast<uint8_t*>(&val[2]);

    res.data[j++] = x[0]; res.data[j++] = x[1]; res.data[j++] = x[2]; res.data[j++] = x[3];
    res.data[j++] = y[0]; res.data[j++] = y[1]; res.data[j++] = y[2]; res.data[j++] = y[3];
    res.data[j++] = z[0]; res.data[j++] = z[1]; res.data[j++] = z[2]; res.data[j++] = z[3];

    // get rgb
    int rgb[3] = {0, 0, 0};
    for (int i = 0; i < 3; ++i)
      rgb[i] = static_cast<int>((static_cast<int>(depth.data[tl++])
                                 + static_cast<int>(depth.data[tr++])
                                 + static_cast<int>(depth.data[bl++])
                                 + static_cast<int>(depth.data[br++])) * 0.25);

    res.data[j++] = reinterpret_cast<uint8_t*>(&rgb[0])[0];
    res.data[j++] = reinterpret_cast<uint8_t*>(&rgb[1])[0];
    res.data[j++] = reinterpret_cast<uint8_t*>(&rgb[2])[0];
    res.data[j++] = 0;

    at += stride_x;
    if (at > depth.width - stride_x + 0.01) { // +0.01 to avoid float error
      row += stride_y;
      head = static_cast<int>(row) * depth.width;
      at = 0.0f;
    }
  }

  return res;
}

//////////////////////////////////////////////////
sensor_msgs::Image KinectInterface::ReadImage(std::string _filename)
{
  linux_kinect::KinectImage srv;
  srv.request.filename = _filename; // in case read from file

  if (!call_image_.call(srv)) {
    ROS_WARN("readimage: service call failed");
    sensor_msgs::Image null;
    return null;
  }

  return srv.response.image;
}

//////////////////////////////////////////////////
std::vector<sensor_msgs::RegionOfInterest> KinectInterface::ImageBounds
(std::vector<std::array<int, 4> > _depth_indicies)
{
  std::vector<sensor_msgs::RegionOfInterest> result;
  result.reserve(_depth_indicies.size());

  for (auto it = _depth_indicies.begin(); it != _depth_indicies.end(); ++it) {
    // min x
    int depth0_y = it->at(0) / depth_width_;
    int depth0_x = it->at(0) - depth0_y * depth_width_;
    int pixel0_x = depth0_x * w_stride_;

    // min y
    int depth1_y = it->at(1) / depth_width_;
    int pixel1_y = depth1_y * h_stride_;

    // max_x
    int depth2_y = it->at(2) / depth_width_;
    int depth2_x = it->at(2) - depth2_y * depth_width_;
    int pixel2_x = depth2_x * w_stride_;

    // max_y
    int depth3_y = it->at(3) / depth_width_;
    int pixel3_y = depth3_y * h_stride_;

    sensor_msgs::RegionOfInterest image_bounds;
    image_bounds.x_offset = pixel0_x;
    image_bounds.y_offset = pixel1_y;
    image_bounds.width = pixel2_x - image_bounds.x_offset;
    image_bounds.height = pixel3_y - image_bounds.y_offset;

    result.push_back(image_bounds);
  }

  return result;
}

//////////////////////////////////////////////////
std::vector<geometry_msgs::Point> KinectInterface::ImageCenters
(std::vector<sensor_msgs::RegionOfInterest> _image_bounds)
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
