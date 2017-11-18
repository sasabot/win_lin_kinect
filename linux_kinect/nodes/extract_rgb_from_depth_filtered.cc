// This file has been modified since version 1.1.0.
// Instead of static depth filtering, current version filters from local radius.
// simple_axis_threshold is now deprecated. Global map filters are out of development scope.
// local_piecewise_linear may be added in future.

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

ros::Publisher pub_;

int queue_size_;
double time_thre_;
std::function<bool(float, float)> f_;

float localradius_param_r_;

bool LocalRadius(float _x, float _z) {
  // condition: z < r * cos(theta)
  return (_z < (localradius_param_r_ * _z / sqrt(_x*_x + _z*_z)));
}

void ExtractRgbFromDepth(const sensor_msgs::PointCloud2::ConstPtr &_msg) {
  sensor_msgs::Image msg;
  msg.data.resize(_msg->width * _msg->height * 3);

  for (size_t y = 0; y < _msg->height; ++y)
    for (size_t x = 0; x < _msg->width; ++x) {
      int dst = (y * _msg->width + x) * 3;
      int src = (y * _msg->width + x) * 16;
      uint8_t bytes_x[4] =
        {_msg->data[src], _msg->data[src + 1], _msg->data[src + 2], _msg->data[src + 3]};
      uint8_t bytes_z[4] =
        {_msg->data[src + 8], _msg->data[src + 9], _msg->data[src + 10], _msg->data[src + 11]};
      float p_x;
      float p_z;
      std::memcpy(&p_x, &bytes_x, 4);
      std::memcpy(&p_z, &bytes_z, 4);
      if (!f_(p_x, p_z) || std::isnan(p_z)) {
        msg.data[dst] = 255;
        msg.data[dst + 1] = 255;
        msg.data[dst + 2] = 255;
      } else {
        msg.data[dst] = _msg->data[src + 12];
        msg.data[dst + 1] = _msg->data[src + 13];
        msg.data[dst + 2] = _msg->data[src + 14];
      }
    }

  msg.header = _msg->header;
  msg.width = _msg->width;
  msg.height = _msg->height;
  msg.step = 3 * msg.width;
  msg.encoding = "bgr8";
  msg.is_bigendian = true;
  pub_.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "extract_rgb_from_depth_filtered");
  ros::NodeHandle nh("~");

  std::string ns("kinect");
  nh.getParam("ns", ns);

  queue_size_ = 10;
  nh.getParam("queue_size", queue_size_);

  time_thre_ = 0.1;
  nh.getParam("timestamp", time_thre_);

  localradius_param_r_ = 3.0; // ~1.2 personal space ~3.7 social space
  nh.getParam("localradius_r", localradius_param_r_);

  pub_ 
    = nh.advertise<sensor_msgs::Image>("/" + ns + "/rgb", 1);

  ros::Subscriber sub =
    nh.subscribe("/" + ns + "/stream", 1, &ExtractRgbFromDepth);

  f_ = LocalRadius;

  ros::spin();
}
