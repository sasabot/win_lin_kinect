#include <ros/ros.h>
#include "std_msgs/String.h"

bool speech_finished = false;

void Callback(const std_msgs::String::ConstPtr& _msg) {
  speech_finished = true;
}

void WaitForSpeechFinish() {
  while (!speech_finished)
    ros::spinOnce();
  speech_finished = false;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tts_sample");
  ros::NodeHandle nh;

  ros::Publisher text_publisher =
    nh.advertise<std_msgs::String>("/windows/voice", 1);

  ros::Subscriber flag_listener =
    nh.subscribe("/windows/voice/finished", 1000, Callback);

  std_msgs::String msg;
  // wait for publisher to be ready
  usleep(1000 * 1000);

  // english speech
  msg.data = "speaking in English";
  text_publisher.publish(msg);
  WaitForSpeechFinish();

  // japanese speech
  msg.data = "日本語で喋ります";
  text_publisher.publish(msg);
  WaitForSpeechFinish();

  // special speech
  msg.data = "u--m... yeah?";
  text_publisher.publish(msg);

  // wait or code will finish before publish
  usleep(1000 * 1000);
}
