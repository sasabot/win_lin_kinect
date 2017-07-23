#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

ros::Publisher pub_;

int queue_size_;
double time_thre_;
std::vector<geometry_msgs::PointStamped> v_dthre_;

void ExtractRgbFromDepth(const sensor_msgs::PointCloud2::ConstPtr &_msg) {
  // find threshold w/ closest time frame
  double secs = _msg->header.stamp.toSec();
  int found = -1;
  double time_diff = std::numeric_limits<double>::max();
  for (auto p = v_dthre_.begin(); p != v_dthre_.end(); ++p) {
    double diff = fabs(secs - p->header.stamp.toSec());
    if (diff < time_diff) {
      time_diff = diff;
      found = static_cast<int>(p - v_dthre_.begin());
    }
  }
  float depth_threshold;
  if (found >= 0 && time_diff < time_thre_) {
    ROS_INFO("found %f == %f", v_dthre_.at(found).header.stamp.toSec(), secs);
    depth_threshold = v_dthre_.erase(v_dthre_.begin(), v_dthre_.begin() + found)->point.z;
  } else {
    ROS_WARN("threshold and depth frame cannot be aligned! %f > %f", time_diff, time_thre_);
    return;
  }

  sensor_msgs::Image msg;
  msg.data.resize(_msg->width * _msg->height * 3);

  for (size_t y = 0; y < _msg->height; ++y)
    for (size_t x = 0; x < _msg->width; ++x) {
      int dst = (y * _msg->width + x) * 3;
      int src = (y * _msg->width + x) * 16;
      uint8_t bytes[4] =
        {_msg->data[src + 8], _msg->data[src + 9], _msg->data[src + 10], _msg->data[src + 11]};
      float z;
      std::memcpy(&z, &bytes, 4);
      if (z > depth_threshold) {
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

void DepthThresholdCallback(const geometry_msgs::PointStamped::ConstPtr &_msg) {
  if (v_dthre_.size() >= queue_size_)
    v_dthre_.erase(v_dthre_.begin());
  v_dthre_.push_back(*_msg);
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

  pub_ 
    = nh.advertise<sensor_msgs::Image>("/" + ns + "/rgb", 1);

  ros::Subscriber sub =
    nh.subscribe("/" + ns + "/stream", 1, &ExtractRgbFromDepth);
  ros::Subscriber depthre =
    nh.subscribe("/" + ns + "/rgb/filter", 1, &DepthThresholdCallback);

  ros::spin();
}
