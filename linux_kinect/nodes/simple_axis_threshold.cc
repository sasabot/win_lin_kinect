// brief: creates a depth filter threshold using distance from sensor position to axis

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

ros::Publisher pub_;
std::string axis_;
float point_;

void RobotPositionCallback(const geometry_msgs::PoseStamped::ConstPtr &_msg) {
  geometry_msgs::PointStamped msg;
  msg.header = _msg->header;

  if (axis_ == "x") {
    msg.point.z = fabs(_msg->pose.position.x - point_);
  } else if (axis_ == "y") {
    msg.point.z = fabs(_msg->pose.position.y - point_);
  }

  pub_.publish(msg);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "simple_axis_threshold");
  ros::NodeHandle nh("~");

  std::string ns("kinect");
  nh.getParam("ns", ns);

  axis_ = "x";
  nh.getParam("axis", axis_);

  point_ = 0.0;
  nh.getParam("point", point_);

  pub_ = nh.advertise<geometry_msgs::PointStamped>("/" + ns + "/rgb/filter", 1);

  ros::Subscriber subscriber =
    nh.subscribe("/tf_msg/sensor", 100, RobotPositionCallback);

  ros::Rate r(30);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }
}
