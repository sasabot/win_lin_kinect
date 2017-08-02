#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

ros::Publisher pub_;
double timespan_;
double last_pub_time_;

void CompressDepth(const sensor_msgs::PointCloud2::ConstPtr &_msg) {
  if (_msg->header.stamp.toSec() - last_pub_time_ < timespan_) {
    return; // stream only every 2 seconds
  }

  sensor_msgs::PointCloud2 msg;
  msg.data.resize((_msg->width >> 1) * (_msg->height >> 1) * 16);

  int src = 0;
  int dst = 0;
  for (size_t y = 0; y < _msg->height; ) {
    for (size_t x = 0; x < _msg->width; ) {
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      msg.data[dst++] = _msg->data[src++];
      src += 16;
      x += 2;
    }
    src += _msg->width * 16;
    y += 2;
  }

  msg.header = _msg->header;
  msg.fields.assign(_msg->fields.begin(), _msg->fields.end());
  msg.width = (_msg->width >> 1);
  msg.height = (_msg->height >> 1);
  msg.point_step = _msg->point_step;
  msg.row_step = msg.width * msg.point_step;
  msg.is_bigendian = _msg->is_bigendian;
  msg.is_dense = _msg->is_dense;
  pub_.publish(msg);

  last_pub_time_ = msg.header.stamp.toSec();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "compress_depth");
  ros::NodeHandle nh("~");

  std::string ns("kinect");
  nh.getParam("ns", ns);

  timespan_ = 2.0;
  nh.getParam("timespan", timespan_);

  last_pub_time_ = 0.0;

  pub_
    = nh.advertise<sensor_msgs::PointCloud2>("/" + ns + "/compressed/points", 1);

  ros::Subscriber sub =
    nh.subscribe("/" + ns + "/stream", 1, &CompressDepth);

  ros::spin();
}
