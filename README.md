## about

win_lin_kinect is a ROS implemented Windows/Linux bridge for sending Kinect sensor data from Windows to Linux. (Tested on ROS Indigo, Python 2.7.6, Ubuntu 14.04 and Windows 10.)
- Required(Linux): mosquitto, python2.7, ROS indigo or above  
- Required(Windows): Visual Studio 2015  
- Optional(Linux): chainer, C++11 or above.  

## updates

The current master branch is working on a major update. Please use the 0.2.1 branch for a working version.

The next stable version (0.3.0) will no longer be compatible with previous versions (before 0.2.1). The package will have completely different install dependencies. The changes and reason for the changes are as below:

- Switching Windows code from WPF to UWP
 - In order to use the latest API features provided by Microsoft, we plan to switch to a UWP application.
- Switching from Grpc to MQTT
 - Grpc does not support UWP applications.
 - Beside the reason above, Grpc had some disadvantages on speed (too slow for streaming point clouds) and installation on Linux (takes time to install and also had a damaging affect on other protobuf reliant applications)
 - On the other hand, MQTT is light, fast, and easy to install.
 - As a concequence of using MQTT, serialization of protocols will no longer be managed.
- Swithing to full offline
 - Previously, there was an option to use Microsoft Cognitive Services. However, in practical situations, network access may not be possible or may be too slow.
 - To keep our codes simple and compact, online features will be removed.
- Switching to 2D face tracking
 - Previously, the Kinect's skeleton tracking was used for face tracking. However, faces close to the camera could not be captured.
 - Facial features will now be handled on Linux side using local deep learned models and networks.
 - To keep our codes simple and compact, skeleton tracking features will be removed.
- Switching from C++ to Python
 - The reason C++ was used in previous versions, was due to the lack of performance in Grpc python.
 - Now that performance issue is not much of a worry, Python will be used to make installation easier and quick.
 - C++ code to wrap ROS interface will be provided but not essential.

## features

To be updated.

## install

Windows
- download Visual Studio 2015.  
- open windows_kinect/KinectRgbdInteraction/KinectRgbdInteraction.sln
- Build > Build Solution  

Linux
- Install mosquitto
```
[sudo] apt-get install mosquitto
[sudo] pip-install paho-mqtt
```
- Install chainer (optional)
```
[sudo] pip-install chainer
```
- catkin make win_lin_kinect/linux_kinect

## startup

Linux
```
roslaunch linux_kinect kinect_rgbd_interaction.launch
```
This starts the application with localhost. For specified hosts use
```
roslaunch linux_kinect kinect_rgbd_interaction.launch host:="host ip address"
```

## easy examples

To be updated.
