## about

win_lin_kinect is a ROS implemented Windows/Linux bridge for sending Kinect sensor data from Windows to Linux. (Tested on ROS Indigo, Ubuntu 14.04.)
- Required(Linux): Grpc 0.15, C++11 or above, ROS indigo or above  
- Required(Windows): Visual Studio 2015  
- Optional: Azure key for voice recognition etc.  

## features

Service call and receive organized point clouds:
- [input] rosservice linux_kinect::KinectRequest  /kinect/request/points
- [output] rostopic sensor_msgs::PointCloud2 /kinect/points

Service call and receive image:
- [input] rosservice linux_kinect::KinectRequest /kinect/request/image
- [output] rostopic sensor_msgs::Image /kinect/image

Service call and receive bounding box in image from point cloud index
- [input] rosservice linux_kinect::KinectRequest /kinect/request/bounds
- [return] linux_kinect::Bit[] bits

Service call and receive cognition results of image regions
- [input] rosservice linux_kinect::KinectRequest /kinect/request/cognition
- [return] linux_kinect::Cognition[] cognitions

Streaming person detection
- [output] rostopic linux_kinect::People /kinect/person/targets

Streaming speaker detection
- [output] rostopic linux_kinect::Person /kinect/person/speaker

Streaming voice detection
- [output] rostopic std_msgs::String /kinect/voice

Publish voice on Windows
- [input] rostopic std_msgs::String /windows/voice

## install

Windows
- download Visual Studio 2015.
- open windows_kinect/KinectRgbdInteractionServer/KinectRgbdInteractionServer.sln
- Build > Build Solution

Linux
- Install grpc version 0.15
```
[sudo] apt-get install build-essential autoconf libtool
git clone https://github.com/grpc/grpc.git
cd grpc
git submodule update --init
cd third_party/protobuf
[sudo] apt-get install autoconf automake libtool curl make g++ unzip
./autogen.sh
./configure
make
make check
[sudo] make install
[sudo] ldconfig
cd ../..
make
[sudo] make install
```
- catkin build linux_kinect

## startup

Linux (must run first)
```
rosparam set /kinect_rgbd_interaction_client/frame "kinect_frame"
rosrun linux_kinect kinect_rgbd_interaction_client
```

Windows
- Double click windows_kinect/KinectRgbdInteractionServer/KinectRgbdInteractionServer/bin/Debug/KinectRgbdInteractionServer.exe.
- Once the program starts, the console will ask whether to use voice recognition or not. The voice recognition will require an Azure key. If you do not have one, type 'y' and disable voice recognition. By disabling, speaker recognition will also be disabled. If you do have a key, the console will ask to type in the key.
- The console will ask for number of clients (which should be 1) and then ask to enter an IP address. If you are using previous settings, you can skip this process by directly pressing enter twice.

## easy examples

Getting point clouds on Linux
```
rosrun linux_kinect points.sh
```

Getting image pixels on Linux
```
rosrun linux_kinect image.sh
```
