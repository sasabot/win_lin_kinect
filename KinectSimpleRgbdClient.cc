#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"

#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <thread>

#include <grpc/grpc.h>
#include <grpc++/channel.h>
#include <grpc++/client_context.h>
#include <grpc++/create_channel.h>
#include <grpc++/security/credentials.h>
#include "kinect_rgbd.grpc.pb.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;

using kinectrgbd::Point;
using kinectrgbd::Request;
using kinectrgbd::KinectRgbd;

class KinectClient {
public:
  KinectClient(std::shared_ptr<Channel> channel, ros::NodeHandle _nh)
    : stub_(KinectRgbd::NewStub(channel)), nh_(_nh)
  {
    pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/kinect/points", 1);

    field_.resize(4);
    sensor_msgs::PointField x;
    x.name = "x";
    x.offset = 0;
    x.datatype = 7;
    x.count = 1;
    field_[0] = x;
    sensor_msgs::PointField y;
    y.name = "y";
    y.offset = 4;
    y.datatype = 7;
    y.count = 1;
    field_[1] = y;
    sensor_msgs::PointField z;
    z.name = "z";
    z.offset = 8;
    z.datatype = 7;
    z.count = 1;
    field_[2] = z;
    sensor_msgs::PointField rgb;
    rgb.name = "rgb";
    rgb.offset = 12;
    rgb.datatype = 7;
    rgb.count = 1;
    field_[3] = rgb;
  }

  void GetPoints()
  {
    ClientContext context;
    Point point;
    Request req;
    req.set_header(0);

    std::unique_ptr<ClientReader<Point> > reader(
	stub_->GetPoints(&context, req));
    std::vector<uint8_t> data;
    data.reserve(16 * 512 * 424); // 12

    int point_index = 0;
    int point_count = 0;
    while (reader->Read(&point))
    {
      float d = point.z();
      float y = point.y();
      float x = point.x();
      // float d = (point.position() & 0x00000000000ffff) * 0.00001;
      // float y = ((point.position() >> 16) & 0x00000000000ffff) * 0.00001;
      // float x = ((point.position() >> 32) & 0x00000000000ffff) * 0.00001;
      uint8_t r = (point.color() & 0x00000ff);
      uint8_t g = ((point.color() >> 8) & 0x00000ff);
      uint8_t b = ((point.color() >> 16) & 0x00000ff);
      // ROS_INFO("read X:%f, Y:%f, D:%f, R:%d, G:%d, B:%d",
      // 	       x, y, d, r, g, b);

      uint8_t *x_bytes;
      x_bytes = reinterpret_cast<uint8_t*>(&x);
      data.push_back(x_bytes[0]);
      data.push_back(x_bytes[1]);
      data.push_back(x_bytes[2]);
      data.push_back(x_bytes[3]);

      uint8_t *y_bytes;
      y_bytes = reinterpret_cast<uint8_t*>(&y);
      data.push_back(y_bytes[0]);
      data.push_back(y_bytes[1]);
      data.push_back(y_bytes[2]);
      data.push_back(y_bytes[3]);

      uint8_t *d_bytes;
      d_bytes = reinterpret_cast<uint8_t*>(&d);
      data.push_back(d_bytes[0]);
      data.push_back(d_bytes[1]);
      data.push_back(d_bytes[2]);
      data.push_back(d_bytes[3]);

      uint8_t dummy = 0;
      data.push_back(b);
      data.push_back(g);
      data.push_back(r);
      data.push_back(dummy);

      ++point_count;
    }
    data.resize(16 * point_count); //12

    ROS_INFO("read %d points", point_count);

    sensor_msgs::PointCloud2 msg;
    msg.header.frame_id = "ps4eye_frame";
    msg.header.stamp = ros::Time(0);
    msg.height = 1;
    msg.width = point_count;
    msg.fields.assign(field_.begin(), field_.end());
    msg.point_step = 16; //12
    msg.row_step = point_count;
    msg.is_dense = true;
    msg.is_bigendian = true;
    msg.data.assign(data.begin(), data.end());
    pub_.publish(msg);
  }


private:

  std::unique_ptr<KinectRgbd::Stub> stub_;

  ros::NodeHandle nh_;

  ros::Publisher pub_;

  std::vector<sensor_msgs::PointField> field_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "kinect_client");

  ros::NodeHandle nh;

  KinectClient client(
      grpc::CreateChannel("192.168.101.190:50052", grpc::InsecureCredentials()),
      nh);

  ros::Rate loop_rate(1);

  while(ros::ok)
  {
    client.GetPoints();
    loop_rate.sleep();
  }
}
