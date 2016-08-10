#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "linux_kinect/Bit.h"
#include "linux_kinect/Person.h"
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
//using grpc::ClientAsyncResponseReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
//using grpc::CompletionQueue;

using kinectperson::KinectPerson;
using kinectrobot::KinectRobot;

class KinectRobotClient
{
public:

  KinectRobotClient(std::shared_ptr<Channel> channel, ros::NodeHandle nh)
    : stub_(KinectRobot::NewStub(channel)), nh_(nh)
  {
    sub_robot_speech_ = nh_.subscribe("/windows/voice", 1, &KinectRobotClient::SpeechSubscriber, this);
    sub_stt_manual_trigger_ = nh_.subscribe("/stt/trigger/manual", 1, &KinectRobotClient::STTManualSubscriber, this);
    sub_stt_auto_trigger_ = nh_.subscribe("/stt/trigger/auto/robot", 1, &KinectRobotClient::UseRobotSpeechSubscriber, this);
    // sub_log_ = nh_.subscribe("/windows/log", 1, &KinectRobotClient::LogSubscriber, this);
    sub_web_agent_ = nh_.subscribe("/windows/web", 1, &KinectRobotClient::UrlInfoSubscriber, this);
  }

  void LinkSpeechOutputSettings(bool* setting)
  {
    // speech output settings must exist before KinectRobotClient initialization
    // therefore, pointer is used to set output settings from KinectRobotClient
    use_robot_speech_engine_ = setting;
  }
  
  void SpeechSubscriber(const std_msgs::String& msg)
  {
    ClientContext context;
    kinectrobot::Speech speech;
    kinectrobot::Response response;
    if (msg.data == "quit")
    {
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
    if (msg.data)
    {
      *use_robot_speech_engine_ = true;
      triggers.set_autotriggerafterrecognition(true);
    }
    else
    {
      *use_robot_speech_engine_ = false;
      triggers.set_autotriggerduringspeech(true);
    }
    Status status = stub_->SetSTTBehavior(&context, triggers, &response);
  }

  // void LogSubscriber(const linux_kinect::Log& msg)
  // {
  //   ClientContext context;
  //   kinectrobot::Log log;
  //   kinectrobot::Response response;
  //   log.set_id(msg.id);
  //   log.set_message(msg.message);
  //   Status status = stub_->WriteLogOnWindows(&context, log, &response);
  // }
  
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

  ros::Subscriber sub_robot_speech_;

  ros::Subscriber sub_stt_manual_trigger_;

  ros::Subscriber sub_stt_auto_trigger_;

  // ros::Subscriber sub_log_;
  
  ros::Subscriber sub_web_agent_;

  bool* use_robot_speech_engine_;
};


class KinectPersonImpl final : public KinectPerson::Service
{
public:
  
  explicit KinectPersonImpl(ros::NodeHandle nh) : nh_(nh)
  {
    pub_person_speaker_ = nh_.advertise<linux_kinect::Person>("/kinect/person/speaker", 1);
    pub_person_target_ = nh_.advertise<linux_kinect::Person>("/kinect/person/target", 1); // todo
    pub_voice_recognition_ = nh_.advertise<std_msgs::String>("/kinect/voice", 1);
    pub_console_command_ = nh_.advertise<std_msgs::String>("/console/command", 1);
    previous_target_ = -1;
    use_robot_speech_engine_ = false; // must be false
    // set to true using /stt/trigger/auto/robot, after constructing robot speech server
    // default voice will link to windows engine
    //to_speech_engine_ = nh_.serviceClient<>(""); //  todo
  }

  Status SendPersonState(ServerContext* context, const kinectperson::PersonStream* persons,
			 kinectperson::Response* res) override
  {
    int this_target_id = -1;
    if (persons->status() > 0)
    {
      int target = -1;
      int candidate_from_previous = -1;
      std::map<int, float> candidate_farness;

      // find speaker
      for (unsigned int i = 0; i < persons->data_size(); ++i)
      {
	if (persons->data(i).speaking())
	{
	  target = i; // set target to speaker
	  break;
	}
	if (persons->data(i).id() == previous_target_)
	  candidate_from_previous = i;
	if (persons->data(i).looking())
	  candidate_farness[i] = persons->data(i).distance();
      }

      // if no speaker found and last speaker is in scene
      if (target == -1 && candidate_from_previous >= 0)
      {
	target = candidate_from_previous; // set target to last speaker
      }

      // if no speaker and last speaker is not in scene
      if (target == -1)
      {
	float distance = 100000;
	for (auto it = candidate_farness.begin(); it != candidate_farness.end(); ++it)
	  if (it->second < distance)
	    target = it->first; // set target to nearest face
      }
      
      if (target >= 0)
      {
	int face_center_x = persons->data(target).face().x()
	  + persons->data(target).face().width() * 0.5;
	linux_kinect::Person data;
	linux_kinect::Bit bounds;
	bounds.x = persons->data(target).face().x();
	bounds.y = persons->data(target).face().y();
	bounds.width = persons->data(target).face().width();
	bounds.height = persons->data(target).face().height();
	data.face2d = bounds;
	geometry_msgs::Point position;
	position.x = 0.0; // todo
	position.y = 0.0; // todo
	position.z = persons->data(target).distance();
	data.face3d = position;
	geometry_msgs::Vector3 rpy;
	rpy.x = persons->data(target).face().roll();
	rpy.y = persons->data(target).face().pitch();
	rpy.z = persons->data(target).face().yaw();
	data.face6d = rpy;
	data.looking = persons->data(target).looking();
	data.speaking = persons->data(target).speaking();
	pub_person_speaker_.publish(data);
	this_target_id = persons->data(target).id();
      }
    }
    previous_target_ = this_target_id;

    res->set_status(true);
    return Status::OK;
  }

  Status SendVoiceRecognition(ServerContext* context, const kinectperson::Text* voice,
			      kinectperson::Response* res) override
  {
    ROS_INFO("%s", voice->text().c_str());

    if (use_robot_speech_engine_)
    {
      // todo
    }
    else
    {
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

  ros::Publisher pub_person_target_; // todo

  ros::Publisher pub_voice_recognition_;

  ros::Publisher pub_console_command_;

  //ros::ServiceClient to_speech_engine_; // todo

  int previous_target_;

  bool use_robot_speech_engine_;
};




int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_rgbd");

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
