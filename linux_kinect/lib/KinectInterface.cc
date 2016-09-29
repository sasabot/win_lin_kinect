#include "KinectInterface.hh"

using namespace kinect;
using namespace interface;

//////////////////////////////////////////////////
Option::Option()
{
  // default parameter values
  cognition_min_image_size_ = 5000;
  cognition_max_image_size_ = 75000;
}

//////////////////////////////////////////////////
Option::~Option()
{
}

//////////////////////////////////////////////////
KinectInterface::KinectInterface(ros::NodeHandle _nh) : nh_(_nh)
{
  call_points_ =
    nh_.serviceClient<linux_kinect::KinectPoints>("/kinect/request/points");
  call_bounds_ =
    nh_.serviceClient<linux_kinect::KinectRequest>("/kinect/request/bounds");
  call_cognition_ =
    nh_.serviceClient<linux_kinect::KinectRequest>("/kinect/request/cognition");
}

//////////////////////////////////////////////////
std::string KinectInterface::ReadKey()
{
 // read setup key for Microsoft Cognitive Service
  std::string cmd = "grep key= $(rospack find linux_kinect)/key.txt | cut -d= -f2";
  char buffer[128];
  key_ = "";
  std::shared_ptr<FILE> pipe(popen(cmd.c_str(), "r"), pclose);
  if (!pipe) throw std::runtime_error("popen() failed!");
  while (!feof(pipe.get()))
    if (fgets(buffer, 128, pipe.get()) != NULL)
      key_ += buffer;
  key_ = key_.substr(0, key_.size() - 1);
  if (key_ == "")
    throw std::runtime_error("please add a key in linux_kinect/key.txt!");
  ROS_INFO("key is %s.", key_.c_str());
}

//////////////////////////////////////////////////
KinectInterface::~KinectInterface()
{
}

//////////////////////////////////////////////////
sensor_msgs::PointCloud2 KinectInterface::ReadPoints()
{
  linux_kinect::Bit bounds;
  bounds.x = 0;
  bounds.y = 0;
  bounds.width = 512;
  bounds.height = 424;

  // request point clouds
  linux_kinect::KinectPoints srv;
  srv.request.data.push_back(bounds);

  if (!call_points_.call(srv))
  {
    ROS_WARN("service call failed");
    sensor_msgs::PointCloud2 null;
    return null;
  }

  return srv.response.points;
}

//////////////////////////////////////////////////
std::vector<linux_kinect::Bit> KinectInterface::ImageBounds
(std::vector<std::array<int, 4> > _depth_indicies)
{
  // request bounds in 2D image pixels
  // depth index -> image bounds
  linux_kinect::KinectRequest srv;
  srv.request.data.reserve(_depth_indicies.size());
  for (auto it = _depth_indicies.begin(); it != _depth_indicies.end(); ++it) {
    linux_kinect::Bit depth_indicies;
    depth_indicies.x = it->at(0);
    depth_indicies.y = it->at(1);
    depth_indicies.width = it->at(2);
    depth_indicies.height = it->at(3);
    srv.request.data.push_back(depth_indicies);
  }
  // try until succeed
  while (!call_bounds_.call(srv)) {
  }

  if (!srv.response.status) {
    ROS_ERROR("invalid cluster region detected");
    return {};
  }

  return srv.response.bits;
}

//////////////////////////////////////////////////
linux_kinect::KinectRequest::Response KinectInterface::Cognition
(std::vector<linux_kinect::Bit> _image_bounds, std::vector<int>& _valid_cluster_ids,
 kinect::interface::optionptr opt)
{
  _valid_cluster_ids.clear();
  _valid_cluster_ids.reserve(_image_bounds.size());

  linux_kinect::KinectRequest cog;
  cog.request.data.reserve(_image_bounds.size());
  for (unsigned int i = 0; i < _image_bounds.size(); ++i) {
    linux_kinect::Bit cluster = _image_bounds[i];

    // check whether cluster is valid or not
    int size = cluster.width * cluster.height;
    if (size < opt->GetCognitionMinImageSize() ||
        size > opt->GetCognitionMaxImageSize()) continue;

    // save below for calculating grasp position
    int cluster_id = std::stoi(cluster.name);
    _valid_cluster_ids.push_back(cluster_id);

    cluster.name = ""; // images will be overwritten unless name is cleaned
    cog.request.data.push_back(cluster);
  }
  cog.request.args = key_;

  if (cog.request.data.size() == 0) {
    ROS_WARN("no image bounds provided!");
    linux_kinect::KinectRequest::Response null;
    null.status = false;
    return null;
  }

  // try until succeed
  while (!call_cognition_.call(cog)) {
  }

  return cog.response;
}
