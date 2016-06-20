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
// #include <grpc++/channel.h>
// #include <grpc++/client_context.h>
// #include <grpc++/create_channel.h>
#include <grpc++/server.h>
#include <grpc++/server_builder.h>
#include <grpc++/server_context.h>
#include <grpc++/security/credentials.h>
#include "kinect_rgbd.grpc.pb.h"


using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;
// using grpc::Channel;
// using grpc::ClientContext;
// using grpc::ClientReader;
// using grpc::ClientReaderWriter;
// using grpc::ClientWriter;
using grpc::Status;

using kinectrgbd::Point;
using kinectrgbd::Response;
using kinectrgbd::KinectRgbd;


class KinectRgbdImpl final : public KinectRgbd::Service
{
public:
  explicit KinectRgbdImpl(ros::NodeHandle nh) : nh_(nh)
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

  Status SendPoints(ServerContext* context, ServerReader<Point>* reader,
		    Response* res) override
  {
    std::vector<uint8_t> data;
    data.reserve(16 * 512 * 424); // 12

    int point_index = 0;
    int point_count = 0;
    kinectrgbd::Point point;
    while (reader->Read(&point))
    {
      float d = point.z();
      float y = point.y();
      float x = point.x();
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
    // msg.height = 1; // unorganized
    // msg.width = point_count; // unorganized
    msg.height = 240;
    msg.width = 320;
    msg.fields.assign(field_.begin(), field_.end());
    msg.point_step = 16; //12
    msg.row_step = point_count;
    msg.is_dense = false;
    msg.is_bigendian = true;
    msg.data.assign(data.begin(), data.end());
    pub_.publish(msg);

    res->set_x(95);
    res->set_y(91);
    res->set_width(320);
    res->set_height(240);

    return Status::OK;
  }

private:

  ros::NodeHandle nh_;

  ros::Publisher pub_;

  std::vector<sensor_msgs::PointField> field_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_rgbd");

  ros::NodeHandle nh;

  KinectRgbdImpl service(nh);

  ServerBuilder builder;
  builder.AddListeningPort("192.168.101.192:50052", grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  server->Wait();
}
