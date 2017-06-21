#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

ros::Publisher pub_;

void ExtractRgbFromDepth(const sensor_msgs::PointCloud2::ConstPtr &_msg) {
  sensor_msgs::Image msg;
  msg.data.resize(_msg->width * _msg->height * 3);

  for (size_t y = 0; y < _msg->height; ++y)
    for (size_t x = 0; x < _msg->width; ++x) {
      int dst = (y * _msg->width + x) * 3;
      int src = (y * _msg->width + x) * 16;
      msg.data[dst] = _msg->data[src + 12];
      msg.data[dst + 1] = _msg->data[src + 13];
      msg.data[dst + 2] = _msg->data[src + 14];
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
  ros::init(argc, argv, "extract_rgb_from_depth");
  ros::NodeHandle nh("~");

  std::string ns("kinect");
  nh.getParam("ns", ns);

  pub_ 
    = nh.advertise<sensor_msgs::Image>("/" + ns + "/rgb", 1);

  ros::Subscriber sub =
    nh.subscribe("/" + ns + "/stream", 100, &ExtractRgbFromDepth);

  ros::spin();
}
