#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Point.h"
#include <aero_application/KinectRequest.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <random>
#include <string>
#include <thread>

#include <grpc/grpc.h>
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
using grpc::Status;

using kinectrgbd::Pixels;
using kinectrgbd::Point;
using kinectrgbd::Points;
using kinectrgbd::Positions;
using kinectrgbd::Header;
using kinectrgbd::Response;
using kinectrgbd::Request;
using kinectrgbd::KinectRgbd;

/*
  @define srv
  int32 mode
  float32[] x
  float32[] y
  int32 width
  int32 height
  bool once
  ---
  geometry_msgs/Point[] data
  int32[] rgb
*/

enum KinectModes
{
  MODES = 4,
  WAIT = 0,
  RGBD = 1,
  IMAGE = 2,
  IMAGE_SPACE_POSITIONS = 3
};


class KinectRgbdImpl final : public KinectRgbd::Service
{
public:
  explicit KinectRgbdImpl(ros::NodeHandle nh) : nh_(nh)
  {
    pub_points_ = nh_.advertise<sensor_msgs::PointCloud2>("/kinect/points", 1);
    pub_pixels_ = nh_.advertise<sensor_msgs::Image>("/kinect/image", 1);
    ros_to_grpc_ = nh_.advertiseService(
        "/kinect/request", &KinectRgbdImpl::KinectRequest, this);

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

    request_.set_mode(static_cast<int>(KinectModes::WAIT));
    request_.set_width(320);
    request_.set_height(240);
    request_.set_once(true);

    request_status_finished_.resize(static_cast<int>(KinectModes::MODES));
    finish_stream_ = true;
  }

  bool KinectRequest(aero_application::KinectRequest::Request &req,
		     aero_application::KinectRequest::Response &res)
  {
    ROS_WARN("received request from ROS");

    // RGBD desired params
    // x[0] = 95
    // y[0] = 91
    // width = 320
    // height = 240

    // IMAGE desired params
    // x[0] = 0
    // y[0] = 0
    // width = 1920
    // height = 1080

    // stop stream / set stream
    if (!req.once)
      finish_stream_ = false;
    else
      if (!finish_stream_)
      {
	finish_stream_ = true;
	request_status_finished_[static_cast<int>(KinectModes::WAIT)] = false;
	// wait till streaming has finished
	while (!request_status_finished_[static_cast<int>(KinectModes::WAIT)])
	{
	}
      }

    // setup request
    request_.clear_x();
    request_.clear_y();
    for (unsigned int i = 0; i < req.x.size(); ++i)
      request_.add_x(req.x[i]);
    for (unsigned int i = 0; i < req.y.size(); ++i)
      request_.add_y(req.y[i]);
    request_.set_width(req.width);
    request_.set_height(req.height);
    request_.set_once(req.once);
    request_.set_mode(req.mode);
    request_status_finished_[req.mode] = false;
    
    // wait till windows response
    while (!request_status_finished_[req.mode])
    {
    }

    ROS_WARN("finished request");

    // handle type request
    if (req.mode == static_cast<int>(KinectModes::IMAGE_SPACE_POSITIONS))
    {
      res.data.reserve(image_space_values_.size());
      for (unsigned int i = 0; i < image_space_values_.size(); ++i)
	res.data.push_back(image_space_values_[i]);
    }

    return true;
  }

  Status CheckRequest(ServerContext* context, const Header* header,
		      Request* request) override
  {
    request->set_mode(request_.mode());
    for (unsigned int i = 0; i < request_.x_size(); ++i)
      request->add_x(request_.x(i));
    for (unsigned int i = 0; i < request_.y_size(); ++i)
      request->add_y(request_.y(i));
    request->set_width(request_.width());
    request->set_height(request_.height());
    request->set_once(request_.once());

    request_.set_mode(static_cast<int>(KinectModes::WAIT));
    request_status_finished_[static_cast<int>(KinectModes::WAIT)] = true;

    ROS_INFO("communicated once");

    return Status::OK;
  }

  // Publish point cloud XYZRGB.
  Status SendPoints(ServerContext* context, const Points* points,
		    Response* res) override
  {
    std::vector<uint8_t> data;
    data.reserve(16 * 512 * 424);

    int point_index = 0;
    int point_count = 0;
    for (unsigned int i = 0; i < points->data_size(); ++i)
    {
      float d = points->data(i).z();
      float y = points->data(i).y();
      float x = points->data(i).x();
      uint8_t r = (points->data(i).color() & 0x00000ff);
      uint8_t g = ((points->data(i).color() >> 8) & 0x00000ff);
      uint8_t b = ((points->data(i).color() >> 16) & 0x00000ff);

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
    data.resize(16 * point_count);

    ROS_INFO("read %d points", point_count);

    sensor_msgs::PointCloud2 msg;
    msg.header.frame_id = "kinect_frame";
    msg.header.stamp = ros::Time(0);
    // msg.height = 1; // unorganized
    // msg.width = point_count; // unorganized
    msg.height = request_.height();
    msg.width = request_.width();
    msg.fields.assign(field_.begin(), field_.end());
    msg.point_step = 16;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = false;
    msg.is_bigendian = true;
    msg.data.assign(data.begin(), data.end());
    pub_points_.publish(msg);

    request_status_finished_[static_cast<int>(KinectModes::RGBD)] = true;

    res->set_finish(finish_stream_);
    return Status::OK;
  }

  // Publish image rgb.
  Status SendImage(ServerContext* context, const Pixels* pixels,
		   Response* res) override
  {
    std::vector<uint8_t> data;
    data.reserve(3 * 1920 * 1080);

    int pixel_count = 0;
    for (unsigned int i = 0; i < pixels->color_size(); ++i)
    {
      uint8_t r = (pixels->color(i) & 0x00000ff);
      uint8_t g = ((pixels->color(i) >> 8) & 0x00000ff);
      uint8_t b = ((pixels->color(i) >> 16) & 0x00000ff);

      data.push_back(b);
      data.push_back(g);
      data.push_back(r);

      ++pixel_count;
    }
    data.resize(3 * pixel_count);

    ROS_INFO("read %d pixels", pixel_count);

    sensor_msgs::Image msg;
    msg.header.frame_id = "kinect_frame";
    msg.header.stamp = ros::Time(0);
    msg.height = request_.height();
    msg.width = request_.width();
    msg.step = 3 * request_.width();
    msg.encoding = "bgr8";
    msg.is_bigendian = true;
    msg.data.assign(data.begin(), data.end());
    pub_pixels_.publish(msg);

    request_status_finished_[static_cast<int>(KinectModes::IMAGE)] = true;

    res->set_finish(finish_stream_);
    return Status::OK;
  }

  // Return physical position of image pixel.
  Status SendPosition(ServerContext* context, const Positions* positions,
		      Response* res) override
  {
    image_space_values_.clear();

    if (positions->status())
    {
      image_space_values_.reserve(positions->x().size());
      for (unsigned int i = 0; i < positions->x().size(); ++i)
      {
	geometry_msgs::Point space_val;
	space_val.x = positions->x(i);
	space_val.y = positions->y(i);
	space_val.z = positions->z(i);
	image_space_values_.push_back(space_val);
	ROS_INFO("got value %f %f %f", space_val.x, space_val.y, space_val.z);
      }
    }

    request_status_finished_[static_cast<int>(
        KinectModes::IMAGE_SPACE_POSITIONS)] = true;

    res->set_finish(finish_stream_);
    return Status::OK;
  }

private:

  ros::NodeHandle nh_;

  ros::Publisher pub_points_;

  ros::Publisher pub_pixels_;

  ros::ServiceServer ros_to_grpc_;

  std::vector<sensor_msgs::PointField> field_;

  std::vector<geometry_msgs::Point> image_space_values_;

  std::vector<bool> request_status_finished_;

  Request request_;

  bool finish_stream_;
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_rgbd");

  ros::NodeHandle nh;

  KinectRgbdImpl service(nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  ServerBuilder builder;
  builder.AddListeningPort("192.168.101.192:50052", grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  server->Wait();
}
