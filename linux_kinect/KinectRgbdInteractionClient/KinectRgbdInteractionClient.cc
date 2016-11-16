#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/PointField.h"
#include "sensor_msgs/Image.h"
#include "linux_kinect/KinectRequest.h"
#include "linux_kinect/KinectPoints.h"
#include "linux_kinect/KinectImage.h"
#include "linux_kinect/KinectSettings.h"
#include "linux_kinect/Tag.h"
#include "linux_kinect/Cognition.h"
#include "linux_kinect/Bit.h"
#include "linux_kinect/Person.h"
#include "linux_kinect/People.h"
#include "linux_kinect/KinectCameraInfo.h"
#include "linux_kinect/UrlInfo.h"

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
#include <grpc++/channel.h>
#include <grpc++/client_context.h>
#include <grpc++/create_channel.h>
#include "kinect_person.grpc.pb.h"
#include "kinect_robot.grpc.pb.h"

using grpc::Server;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::ServerReader;
using grpc::ServerReaderWriter;
using grpc::ServerWriter;
using grpc::Status;

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;

using kinectperson::KinectPerson;
using kinectrobot::KinectRobot;

class KinectRobotClient
{
public:

  KinectRobotClient(std::shared_ptr<Channel> channel, ros::NodeHandle nh)
    : stub_(KinectRobot::NewStub(channel)), nh_(nh)
  {
    // rgbd streaming

    frame_name_ = "";
    nh_.getParam("/kinect_rgbd_interaction_client/frame", frame_name_);

    // pub_points_ = nh_.advertise<sensor_msgs::PointCloud2>("/kinect/points", 1);
    // pub_pixels_ = nh_.advertise<sensor_msgs::Image>("/kinect/image", 1);

    srv_timestamp_ = nh_.advertiseService(
        "/kinect/request/updatets", &KinectRobotClient::UpdateTimeStamp, this);
    srv_points_ = nh_.advertiseService(
	"/kinect/request/points", &KinectRobotClient::RequestPoints, this);
    srv_image_ = nh_.advertiseService(
	"/kinect/request/image", &KinectRobotClient::RequestImage, this);
    srv_bounds_ = nh_.advertiseService(
	"/kinect/request/bounds", &KinectRobotClient::RequestBounds, this);
    srv_cognition_ = nh_.advertiseService(
	"/kinect/request/cognition", &KinectRobotClient::RequestCognition, this);
    srv_camera_info_ = nh_.advertiseService(
        "/kinect/request/camera_info", &KinectRobotClient::RequestCameraInfo, this);

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

    // stream settings

    srv_stream_settings_ = nh_.advertiseService(
        "/kinect/stream/settings", &KinectRobotClient::StreamSettings, this);

    // interaction

    sub_robot_speech_ = nh_.subscribe(
        "/windows/voice", 1, &KinectRobotClient::SpeechSubscriber, this);
    sub_stt_manual_trigger_ = nh_.subscribe(
        "/stt/trigger/manual", 1,
	&KinectRobotClient::STTManualSubscriber, this);
    sub_stt_auto_trigger_ = nh_.subscribe(
        "/stt/trigger/auto/robot", 1,
	&KinectRobotClient::UseRobotSpeechSubscriber, this);
    sub_web_agent_ = nh_.subscribe(
        "/windows/web", 1,
	&KinectRobotClient::UrlInfoSubscriber, this);
  }

  void SetupRequest(kinectrobot::Request &request,
		    linux_kinect::KinectRequest::Request &req)
  {
    ROS_WARN("received request from ROS");
    for (unsigned int i = 0; i < req.data.size(); ++i) {
      auto bit = request.add_data();
      bit->set_x(req.data[i].x);
      bit->set_y(req.data[i].y);
      bit->set_width(req.data[i].width);
      bit->set_height(req.data[i].height);
      if (req.data[i].name == "")
	bit->set_name("image" + std::to_string(i));
    }
    request.set_args(req.args);
  }

  bool UpdateTimeStamp(linux_kinect::KinectRequest::Request &req,
                       linux_kinect::KinectRequest::Response &res)
  {
    ROS_WARN("received request from ROS");

    ClientContext context;
    kinectrobot::Request update;
    update.set_args(req.args);
    kinectrobot::Response response;

    Status status = stub_->UpdateTimeStamp(&context, update, &response);
    res.status = response.status();

    ROS_WARN("finished request");
    return true;
  }

  bool RequestCameraInfo(linux_kinect::KinectCameraInfo::Request &req,
                         linux_kinect::KinectCameraInfo::Response &res)
  {
    kinectrobot::Request request;
    ClientContext context;
    kinectrobot::CameraInfo info;

    Status status = stub_->ReturnCameraInfo(
        &context, request, &info);

    res.fx = info.fx();
    res.fy = info.fy();
    res.cx = info.cx();
    res.cy = info.cy();

    ROS_WARN("finished request");
    return true;
  }

  bool RequestPoints(linux_kinect::KinectPoints::Request &req,
                     linux_kinect::KinectPoints::Response &res)
  {
    kinectrobot::Request request;
    ROS_WARN("received request from ROS");
    auto bit = request.add_data();
    bit->set_x(req.data.x);
    bit->set_y(req.data.y);
    bit->set_width(req.data.width);
    bit->set_height(req.data.height);
    if (req.data.name == "")
      bit->set_name("image0");

    ClientContext context;
    kinectrobot::Points points;
    std::unique_ptr<ClientReader<kinectrobot::Points> > reader(
	stub_->ReturnPoints(&context, request));

    bool has_delay;
    std::vector<uint8_t> data;
    data.reserve(16 * 512 * 424);
    int point_count = 0;

    while (reader->Read(&points)) {
      has_delay = points.delay();
      for (unsigned int i = 0; i < points.data_size(); ++i) {
	float d = points.data(i).z();
	float y = points.data(i).y();
	float x = points.data(i).x();
	uint8_t r = (points.data(i).color() & 0x00000ff);
	uint8_t g = ((points.data(i).color() >> 8) & 0x00000ff);
	uint8_t b = ((points.data(i).color() >> 16) & 0x00000ff);

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
    }
    Status status = reader->Finish();
    data.resize(16 * point_count);

    ROS_INFO("read %d points", point_count);

    res.time_delay = has_delay;
    res.points.header.frame_id = frame_name_;
    res.points.header.stamp = ros::Time(0);
    res.points.height = request.data(0).height();
    res.points.width = request.data(0).width();
    res.points.fields.assign(field_.begin(), field_.end());
    res.points.point_step = 16;
    res.points.row_step = res.points.point_step * res.points.width;
    res.points.is_dense = false;
    res.points.is_bigendian = true;
    res.points.data.assign(data.begin(), data.end());

    ROS_WARN("finished request");
    return true;
  }

  bool RequestImage(linux_kinect::KinectImage::Request &req,
		    linux_kinect::KinectImage::Response &res)
  {
    kinectrobot::Request request;
    ROS_WARN("received request from ROS");
    auto bit = request.add_data();
    bit->set_x(req.data.x);
    bit->set_y(req.data.y);
    bit->set_width(req.data.width);
    bit->set_height(req.data.height);
    if (req.data.name == "")
      bit->set_name("image0");

    ClientContext context;
    kinectrobot::Pixels pixels;
    std::unique_ptr<ClientReader<kinectrobot::Pixels> > reader(
	stub_->ReturnImage(&context, request));

    bool has_delay;
    std::vector<uint8_t> data;
    data.reserve(3 * 1920 * 1080);
    int pixel_count = 0;

    while (reader->Read(&pixels)) {
      has_delay = pixels.delay();
      for (unsigned int i = 0; i < pixels.color_size(); ++i) {
	uint8_t r = (pixels.color(i) & 0x00000ff);
	uint8_t g = ((pixels.color(i) >> 8) & 0x00000ff);
	uint8_t b = ((pixels.color(i) >> 16) & 0x00000ff);

	data.push_back(b);
	data.push_back(g);
	data.push_back(r);

	++pixel_count;
      }
    }
    data.resize(3 * pixel_count);

    ROS_INFO("read %d pixels", pixel_count);

    res.time_delay = has_delay;
    res.image.header.frame_id = frame_name_;
    res.image.header.stamp = ros::Time(0);
    res.image.height = request.data(0).height();
    res.image.width = request.data(0).width();
    res.image.step = 3 * res.image.width;
    res.image.encoding = "bgr8";
    res.image.is_bigendian = true;
    res.image.data.assign(data.begin(), data.end());

    ROS_WARN("finished request");
    return true;
  }

  bool RequestBounds(linux_kinect::KinectRequest::Request &req,
                     linux_kinect::KinectRequest::Response &res)
  {
    kinectrobot::Request request;
    SetupRequest(request, req);

    ClientContext context;
    kinectrobot::BitStream boundings;

    Status status = stub_->ReturnPixelBoundsFromSpaceBounds(
        &context, request, &boundings);

    res.status = boundings.status();
    if (!boundings.status()) {
      ROS_WARN("finished request");
      return true;
    }

    res.bits.reserve(boundings.data().size());
    for (unsigned int i = 0; i < boundings.data().size(); ++i) {
      linux_kinect::Bit bit;
      bit.name = boundings.data(i).name();
      bit.x = boundings.data(i).x();
      bit.y = boundings.data(i).y();
      bit.width = boundings.data(i).width();
      bit.height = boundings.data(i).height();
      res.bits.push_back(bit);
      ROS_INFO("got bounding [%f, %f], width: %f, height: %f",
	       bit.x, bit.y, bit.width, bit.height);
    }

    ROS_WARN("finished request");
    return true;
  }

  bool RequestCognition(linux_kinect::KinectRequest::Request &req,
			linux_kinect::KinectRequest::Response &res)
  {
    kinectrobot::Request request;
    SetupRequest(request, req);

    ClientContext context;
    kinectrobot::DataStream stream;

    Status status = stub_->ReturnCognition(&context, request, &stream);

    res.status = stream.status();
    res.cognitions.reserve(stream.data_size());
    res.points.reserve(stream.data_size());
    for (unsigned int i = 0; i < stream.data_size(); ++i) {
      geometry_msgs::Point image_i_pos;
      image_i_pos.x = stream.data(i).x();
      image_i_pos.y = stream.data(i).y();
      image_i_pos.z = stream.data(i).z();
      res.points.push_back(image_i_pos);
      linux_kinect::Cognition image_i;
      image_i.status = stream.data(i).status();
      if (stream.data(i).captions_size() > 0) {
	linux_kinect::Tag image_i_cap;
	image_i_cap.tag = stream.data(i).captions(0).tag();
	image_i_cap.confidence = stream.data(i).captions(0).confidence();
	image_i.captions.push_back(image_i_cap);
      }
      image_i.tags.reserve(stream.data(i).tags_size());
      for (unsigned int j = 0; j < stream.data(i).tags_size(); ++j) {
	linux_kinect::Tag image_i_tag_j;
	image_i_tag_j.tag = stream.data(i).tags(j).tag();
	image_i_tag_j.confidence = stream.data(i).tags(j).confidence();
	image_i.tags.push_back(image_i_tag_j);
      }
      image_i.texts.reserve(stream.data(i).texts_size());
      for (unsigned int j = 0; j < stream.data(i).texts_size(); ++j)
	image_i.texts.push_back(stream.data(i).texts(j));
      res.cognitions.push_back(image_i);
    }

    ROS_WARN("finished request");
    return true;
  }

  bool StreamSettings(linux_kinect::KinectSettings::Request &req,
                      linux_kinect::KinectSettings::Response &res)
  {
    kinectrobot::StreamSettings settings;
    ROS_WARN("received settings from ROS");
    for (unsigned int i = 0; i < req.streams.size(); ++i) {
      settings.add_streams(req.streams[i]);
      settings.add_settings(req.settings[i]);
    }

    ClientContext context;
    kinectrobot::Response response;

    Status status = stub_->SetStreamSettings(&context, settings, &response);
    res.status = response.status();

    ROS_WARN("updateded settings");
    return true;
  }

  void LinkSpeechOutputSettings(bool* setting)
  {
    // speech output setting must exist before KinectRobotClient initialization
    // therefore, pointer is used to set output settings from KinectRobotClient
    use_robot_speech_engine_ = setting;
  }

  void SpeechSubscriber(const std_msgs::String& msg)
  {
    ClientContext context;
    kinectrobot::Speech speech;
    kinectrobot::Response response;
    if (msg.data == "quit") {
      speech.set_command("quit");
      Status status = stub_->SendSpeech(&context, speech, &response);
      return;
    }

    speech.set_speech(msg.data);
    Status status = stub_->SendSpeech(&context, speech, &response);
  }

  void STTManualSubscriber(const std_msgs::Bool& msg)
  {
    ClientContext context;
    kinectrobot::VoiceTriggers triggers;
    kinectrobot::Response response;
    if (msg.data) triggers.set_manualtriggeron(true);
    else triggers.set_manualtriggeroff(true);
    Status status = stub_->SetSTTBehavior(&context, triggers, &response);
  }

  void UseRobotSpeechSubscriber(const std_msgs::Bool& msg)
  {
    ClientContext context;
    kinectrobot::VoiceTriggers triggers;
    kinectrobot::Response response;
    if (msg.data) {
      *use_robot_speech_engine_ = true;
      triggers.set_autotriggerafterrecognition(true);
    } else {
      *use_robot_speech_engine_ = false;
      triggers.set_autotriggerduringspeech(true);
    }
    Status status = stub_->SetSTTBehavior(&context, triggers, &response);
  }

  void UrlInfoSubscriber(const linux_kinect::UrlInfo& msg)
  {
    ClientContext context;
    kinectrobot::UrlInfo info;
    kinectrobot::Response response;
    info.set_url(msg.url);
    info.set_linkhead(msg.linkhead);
    info.set_key(msg.key);
    info.set_style(msg.style);
    Status status = stub_->WebAgent(&context, info, &response);
  }
  
private:

  std::unique_ptr<KinectRobot::Stub> stub_;

  ros::NodeHandle nh_;

  // ros::Publisher pub_points_;

  // ros::Publisher pub_pixels_;

  ros::ServiceServer srv_timestamp_;

  ros::ServiceServer srv_points_;

  ros::ServiceServer srv_image_;

  ros::ServiceServer srv_bounds_;

  ros::ServiceServer srv_cognition_;

  ros::ServiceServer srv_stream_settings_;

  ros::ServiceServer srv_camera_info_;

  std::vector<sensor_msgs::PointField> field_;

  std::string frame_name_;

  ros::Subscriber sub_robot_speech_;

  ros::Subscriber sub_stt_manual_trigger_;

  ros::Subscriber sub_stt_auto_trigger_;
  
  ros::Subscriber sub_web_agent_;

  bool* use_robot_speech_engine_;
};


class KinectPersonImpl final : public KinectPerson::Service
{
public:
  
  explicit KinectPersonImpl(ros::NodeHandle nh) : nh_(nh)
  {
    pub_person_speaker_ = nh_.advertise<linux_kinect::Person>("/kinect/person/speaker", 1);
    pub_person_targets_ = nh_.advertise<linux_kinect::People>("/kinect/person/targets", 1);
    pub_voice_recognition_ = nh_.advertise<std_msgs::String>("/kinect/voice", 1);
    pub_console_command_ = nh_.advertise<std_msgs::String>("/console/command", 1);
    pub_points_ = nh_.advertise<sensor_msgs::PointCloud2>("/kinect/stream", 1);
    previous_target_ = -1;
    use_robot_speech_engine_ = false; // must be false
    // set to true using /stt/trigger/auto/robot, after constructing robot speech server
    // default voice will link to windows engine
    //to_speech_engine_ = nh_.serviceClient<>(""); //  todo

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

  Status SendPersonState(ServerContext* context, const kinectperson::PersonStream* persons,
			 kinectperson::Response* res) override
  {
    int this_target_id = -1;
    linux_kinect::People targets;

    if (persons->status() > 0) {
      int target = -1;
      int candidate_from_previous = -1;
      std::map<int, float> candidate_farness;

      targets.data.resize(persons->data_size());

      // find speaker
      for (unsigned int i = 0; i < persons->data_size(); ++i) {
	// save data to targets
	targets.data[i].verified_id = i; // todo
	linux_kinect::Bit bounds;
	bounds.x = persons->data(i).face().x();
	bounds.y = persons->data(i).face().y();
	bounds.width = persons->data(i).face().width();
	bounds.height = persons->data(i).face().height();
	targets.data[i].face2d = bounds;
	geometry_msgs::Point position;
	position.x = persons->data(i).position().x();
	position.y = persons->data(i).position().y();
	position.z = persons->data(i).distance();
	targets.data[i].face3d = position;
	geometry_msgs::Vector3 rpy;
	rpy.x = persons->data(i).face().roll();
	rpy.y = persons->data(i).face().pitch();
	rpy.z = persons->data(i).face().yaw();
	targets.data[i].face6d = rpy;
	targets.data[i].looking = persons->data(i).looking();
	targets.data[i].speaking = persons->data(i).speaking();

	if (persons->data(i).speaking()) {
	  target = i; // set target to speaker
	  break;
	}
	if (persons->data(i).id() == previous_target_)
	  candidate_from_previous = i;
	if (persons->data(i).looking())
	  candidate_farness[i] = persons->data(i).distance();
      }

      // if no speaker found and last speaker is in scene
      if (target == -1 && candidate_from_previous >= 0) {
	target = candidate_from_previous; // set target to last speaker
      }

      // if no speaker and last speaker is not in scene
      if (target == -1) {
	float distance = std::numeric_limits<float>::max();
	for (auto it = candidate_farness.begin(); it != candidate_farness.end(); ++it)
	  if (it->second < distance)
	    target = it->first; // set target to nearest face
      }

      if (target >= 0) {
	linux_kinect::Person data = targets.data[target];
	pub_person_speaker_.publish(data);
	this_target_id = persons->data(target).id();
      }

      pub_person_targets_.publish(targets);
    }
    previous_target_ = this_target_id;

    res->set_status(true);
    return Status::OK;
  }

  Status SendVoiceRecognition(ServerContext* context, const kinectperson::Text* voice,
			      kinectperson::Response* res) override
  {
    ROS_INFO("%s", voice->text().c_str());

    if (use_robot_speech_engine_) {
      // todo
    } else {
      std_msgs::String result;
      result.data = voice->text();
      pub_voice_recognition_.publish(result);
    }
    
    return Status::OK;
  }

  Status SendConsoleCommand(ServerContext* context, const kinectperson::Text* command,
			    kinectperson::Response* res) override
  {
    std_msgs::String data;
    data.data = command->text();
    pub_console_command_.publish(data);
    return Status::OK;
  }

  Status SendPointStream(ServerContext* context,
                         ServerReader<kinectperson::PointStream>* reader,
                         kinectperson::Response* res) override
  {
    std::vector<uint8_t> data;
    data.reserve(16 * 512 * 424);

    int point_count = 0;
    kinectperson::PointStream points;
    while (reader->Read(&points))
      for (unsigned int i = 0; i < points.data_size(); ++i) {
          float d = points.data(i).z();
          float y = points.data(i).y();
          float x = points.data(i).x();
          uint8_t r = (points.data(i).color() & 0x00000ff);
          uint8_t g = ((points.data(i).color() >> 8) & 0x00000ff);
          uint8_t b = ((points.data(i).color() >> 16) & 0x00000ff);

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
    msg.header.frame_id = "dynamic_kinect_frame";
    msg.header.stamp = ros::Time(0);
    msg.height = 424;
    msg.width = 512;
    msg.fields.assign(field_.begin(), field_.end());
    msg.point_step = 16;
    msg.row_step = msg.point_step * msg.width;
    msg.is_dense = false;
    msg.is_bigendian = true;
    msg.data.assign(data.begin(), data.end());
    pub_points_.publish(msg);

    return Status::OK;
  }
  
  Status CreateRobotClient(ServerContext* context, const kinectperson::Text* ip,
			   kinectperson::Response* res) override
  {
    ROS_INFO("creating robot client: %s", ip->text().c_str());
    client.reset(new KinectRobotClient
		 (grpc::CreateChannel(ip->text(), grpc::InsecureChannelCredentials()), nh_));
    client->LinkSpeechOutputSettings(&use_robot_speech_engine_);
    return Status::OK;
  }

private:

  std::shared_ptr<KinectRobotClient> client;
  
  ros::NodeHandle nh_;

  ros::Publisher pub_person_speaker_;

  ros::Publisher pub_person_targets_;

  ros::Publisher pub_voice_recognition_;

  ros::Publisher pub_console_command_;

  ros::Publisher pub_points_;

  std::vector<sensor_msgs::PointField> field_;

  //ros::ServiceClient to_speech_engine_; // todo

  int previous_target_;

  bool use_robot_speech_engine_;
};




int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_rgbd_interaction_client");

  ros::NodeHandle nh;

  KinectPersonImpl service(nh);
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
 
  ServerBuilder builder;
  builder.AddListeningPort("0.0.0.0:50052", grpc::InsecureServerCredentials());
  builder.RegisterService(&service);
  std::unique_ptr<Server> server(builder.BuildAndStart());
  server->Wait();

  return 0;
}
