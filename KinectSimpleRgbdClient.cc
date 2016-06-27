#include <ros/ros.h>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/Image.h"
#include "geometry_msgs/Point.h"
#include <aero_application/KinectRequest.h>
#include <aero_application/Bit.h>
#include <aero_application/Tag.h>
#include <aero_application/Cognition.h>

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

using kinectrgbd::KinectRgbd;

/*
  @define srv KinectRequest
  int32 mode
  aero_application/Bit[] data
  bool once
  string args
  ---
  geometry_msgs/Point[] points
  aero_application/Cognition[] cognitions
  aero_application/Bit[] bits
  bool status
*/

/*
  @define msg Bit
  string name
  float32 x
  float32 y
  float32 width
  float32 height
*/

/*
  @define msg Tag
  string tag
  float32 confidence
*/

/*
  @define msg Cognition
  aero_application/Tag[] captions
  aero_application/Tag[] tags
  string[] texts
  bool status
*/

enum KinectModes
{
  MODES = 6,
  WAIT = 0,
  RGBD = 1,
  IMAGE = 2,
  IMAGE_SPACE_POSITIONS = 3,
  SPACE2PIXEL_BOUNDINGS = 4,
  COGNITION = 5
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
    request_.set_once(true);

    request_status_finished_.resize(static_cast<int>(KinectModes::MODES));
    finish_stream_ = true;
    status_ = false;
  }

  bool KinectRequest(aero_application::KinectRequest::Request &req,
		     aero_application::KinectRequest::Response &res)
  {
    ROS_WARN("received request from ROS");

    // RGBD params
    // width <= 512
    // height <= 414

    // IMAGE params
    // width <= 1920
    // height <= 1080

    // COGNITION args
    // args = subscription key

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
    request_.clear_data();
    request_.set_mode(req.mode);
    for (unsigned int i = 0; i < req.data.size(); ++i)
    {
      auto bit = request_.add_data();
      bit->set_x(req.data[i].x);
      bit->set_y(req.data[i].y);
      bit->set_width(req.data[i].width);
      bit->set_height(req.data[i].height);
      if (req.data[i].name == "")
	bit->set_name("image" + std::to_string(i));
    }
    request_.set_once(req.once);
    request_.set_args(req.args);
    request_status_finished_[req.mode] = false;
    
    // wait till windows response
    while (!request_status_finished_[req.mode])
    {
    }

    ROS_WARN("finished request");

    // handle type request
    if (req.mode == static_cast<int>(KinectModes::IMAGE_SPACE_POSITIONS))
    {
      res.points.reserve(image_space_values_.size());
      for (unsigned int i = 0; i < image_space_values_.size(); ++i)
	res.points.push_back(image_space_values_[i]);
      res.status = status_;
    }
    else if (req.mode == static_cast<int>(KinectModes::SPACE2PIXEL_BOUNDINGS))
    {
      res.bits.reserve(boundings_.size());
      for (unsigned int i = 0; i < boundings_.size(); ++i)
	res.bits.push_back(boundings_[i]);
      res.status = status_;
    }
    else if (req.mode == static_cast<int>(KinectModes::COGNITION))
    {
      res.cognitions.reserve(cognitions_.size());
      for (unsigned int i = 0; i < cognitions_.size(); ++i)
	res.cognitions.push_back(cognitions_[i]);
      res.points.reserve(image_space_values_.size());
      for (unsigned int i = 0; i < image_space_values_.size(); ++i)
	res.points.push_back(image_space_values_[i]);
      res.status = status_;
    }

    return true;
  }

  Status CheckRequest(ServerContext* context, const kinectrgbd::Header* header,
		      kinectrgbd::Request* request) override
  {
    request->set_mode(request_.mode());
    for (unsigned int i = 0; i < request_.data_size(); ++i)
    {
      auto bit = request->add_data();
      bit->set_x(request_.data(i).x());
      bit->set_y(request_.data(i).y());
      bit->set_width(request_.data(i).width());
      bit->set_height(request_.data(i).height());
      bit->set_name(request_.data(i).name());
    }
    request->set_once(request_.once());
    request->set_args(request_.args());

    request_.set_mode(static_cast<int>(KinectModes::WAIT));
    request_status_finished_[static_cast<int>(KinectModes::WAIT)] = true;

    ROS_INFO("communicated once");

    return Status::OK;
  }

  // Publish point cloud XYZRGB.
  Status SendPoints(ServerContext* context, const kinectrgbd::Points* points,
		    kinectrgbd::Response* res) override
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
    msg.height = request_.data(0).height();
    msg.width = request_.data(0).width();
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
  Status SendImage(ServerContext* context, const kinectrgbd::Pixels* pixels,
		   kinectrgbd::Response* res) override
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
    msg.height = request_.data(0).height();
    msg.width = request_.data(0).width();
    msg.step = 3 * msg.width;
    msg.encoding = "bgr8";
    msg.is_bigendian = true;
    msg.data.assign(data.begin(), data.end());
    pub_pixels_.publish(msg);

    request_status_finished_[static_cast<int>(KinectModes::IMAGE)] = true;

    res->set_finish(finish_stream_);
    return Status::OK;
  }

  // Return physical position of image pixel.
  Status ReturnPositionsFromPixels(
      ServerContext* context, const kinectrgbd::DataStream* positions,
      kinectrgbd::Response* res) override
  {
    image_space_values_.clear();

    if (positions->status())
    {
      image_space_values_.reserve(positions->data().size());
      for (unsigned int i = 0; i < positions->data().size(); ++i)
      {
	geometry_msgs::Point space_val;
	space_val.x = positions->data(i).x();
	space_val.y = positions->data(i).y();
	space_val.z = positions->data(i).z();
	image_space_values_.push_back(space_val);
	ROS_INFO("got value %f %f %f", space_val.x, space_val.y, space_val.z);
      }
    }

    status_ = positions->status();
    request_status_finished_[static_cast<int>(
        KinectModes::IMAGE_SPACE_POSITIONS)] = true;

    res->set_finish(finish_stream_);
    return Status::OK;
  }

  // Return bounding box of image pixel.
  Status ReturnPixelBoundsFromSpaceBounds(
      ServerContext* context, const kinectrgbd::BitStream* boundings,
      kinectrgbd::Response* res) override
  {
    boundings_.clear();

    if (boundings->status())
    {
      boundings_.reserve(boundings->data().size());
      for (unsigned int i = 0; i < boundings->data().size(); ++i)
      {
	aero_application::Bit bit;
	bit.name = boundings->data(i).name();
	bit.x = boundings->data(i).x();
	bit.y = boundings->data(i).y();
	bit.width = boundings->data(i).width();
	bit.height = boundings->data(i).height();
	boundings_.push_back(bit);
	ROS_INFO("got bounding [%d, %d], width: %d, height: %d",
		 bit.x, bit.y, bit.width, bit.height);
      }
    }

    status_ = boundings->status();
    request_status_finished_[static_cast<int>(
        KinectModes::SPACE2PIXEL_BOUNDINGS)] = true;

    res->set_finish(finish_stream_);
    return Status::OK;
  }

  // Return physical position and cognition results of images.
  Status ReturnCognition(
      ServerContext* context, const kinectrgbd::DataStream* stream,
      kinectrgbd::Response* res) override
  {
    cognitions_.clear();
    image_space_values_.clear();

    cognitions_.reserve(stream->data_size());
    image_space_values_.reserve(stream->data_size());
    for (unsigned int i = 0; i < stream->data_size(); ++i)
    {
      geometry_msgs::Point image_i_pos;
      image_i_pos.x = stream->data(i).x();
      image_i_pos.y = stream->data(i).y();
      image_i_pos.z = stream->data(i).z();
      image_space_values_.push_back(image_i_pos);
      aero_application::Cognition image_i;
      image_i.status = stream->data(i).status();
      if (stream->data(i).captions_size() > 0)
      {
	aero_application::Tag image_i_cap;
	image_i_cap.tag = stream->data(i).captions(0).tag();
	image_i_cap.confidence = stream->data(i).captions(0).confidence();
	image_i.captions.push_back(image_i_cap);
      }
      image_i.tags.reserve(stream->data(i).tags_size());
      for (unsigned int j = 0; j < stream->data(i).tags_size(); ++j)
      {
	aero_application::Tag image_i_tag_j;
	image_i_tag_j.tag = stream->data(i).tags(j).tag();
	image_i_tag_j.confidence = stream->data(i).tags(j).confidence();
	image_i.tags.push_back(image_i_tag_j);
      }
      image_i.texts.reserve(stream->data(i).texts_size());
      for (unsigned int j = 0; j < stream->data(i).texts_size(); ++j)
	image_i.texts.push_back(stream->data(i).texts(j));
      cognitions_.push_back(image_i);
    }

    status_ = stream->status();
    request_status_finished_[static_cast<int>(
        KinectModes::COGNITION)] = true;

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

  std::vector<aero_application::Bit> boundings_;

  std::vector<bool> request_status_finished_;

  kinectrgbd::Request request_;

  std::vector<aero_application::Cognition> cognitions_;

  bool finish_stream_;

  bool status_;
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
