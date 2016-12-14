## about

win_lin_kinect is a ROS implemented Windows/Linux bridge for sending Kinect sensor data from Windows to Linux. (Tested on ROS Indigo, Python 2.7.6, Ubuntu 14.04 and Windows 10.)
- Required(Linux): mosquitto, python2.7, ROS indigo or above  
- Required(Windows): Visual Studio 2015  
- Optional(Linux): chainer, C++11 or above.  

For non-ROS users, please directly use mqtt mosquitto and receive data from Windows.

## expected users

Those who use ROS on Linux but also wants to use Windows API. Those who don't want to go through calibration and device managing but want to receive Kinect point clouds on Linux.

Performance depends on your machine. Codes on Windows run at minimum 15 FPS when used on NUC6i7KYK.

## features

To be updated.

## prerequisites

Windows
- download Visual Studio 2015.  

Linux
- Install mosquitto
```
[sudo] apt-get install mosquitto
[sudo] pip-install paho-mqtt
```
- Install ROS Indigo or above (If ROS is not an option, please use mqtt directly)
- Install chainer (optional)
```
[sudo] pip-install chainer
```

## build

Windows
- Install WindowsKinectLaunch etc.
  - open *windows_kinect/WindowsKinectLaunch/WindowsKinectLaunch.sln*
  - make sure build option is set to *Debug/x64*
  - Build > Build Solution
  - right click *KinectRgbdInteraction* > Deploy (this will deploy app and add to HKEY_CLASSES_ROOT)
  - right click *KinectWindowsInteraction* > Deploy (this will deploy app and add to HKEY_CLASSES_ROOT)
  - right click *WindowsKinectLaunch* > Deploy (this will deploy app and add to HKEY_CLASSES_ROOT)
- Install KinectMicrophoneInteraction
  - open *windows_kinect/KinectMicrophoneInteraction/KinectMicrophoneInteraction.sln*
  - make sure build options is set to *Debug/Any CPU*
  - Build > Build Solution
  - right click KinectMicrophoneInteraction.exe under *windows_kinect/KinectMicrophoneInteraction/KinectMicrophoneInteraction/bin/Debug* and select "Run as administrator"
  - click "register this app" button (this will add app to HKEY_CLASSES_ROOT)
  - close app window
- Install KinectFaceInteraction (optional)
  - open *windows_kinect/KinectFaceInteraction/KinectFaceInteraction.sln*
  - right click *HaarRuntimeComponent* > Build
  - right click *KinectFaceInteraction* > Build
  - right click *kinectFaceInteraction* > Deploy
  - in order to run KinectFaceInteraction, *cascades\haarcascade_mcs_upperbody.xml* must be located under *Documents* (please copy file from an installed OpenCV 3.1.0 contrib)


Linux
- easy install (for ROS users)
  - ```cd win_lin_kinect/linux_kinect```
  - ```./setup.sh --lib```
  - catkin make win_lin_kinect/linux_kinect
- build examples (for ROS users)
  - ```./setup.sh --ex```
  - if you have OpenCV3 and PCL1.8 installed, you can build all examples with ```./setup.sh --cvex```

## startup

**Windows**

```
start windowskinectlaunch:
```
Type the above in command prompt and all applications will start (to open command prompt, press windows key, and then type "cmd"). Or you can press windows key and type "WindowsKinectLaunch" to find the launching app.  

All apps will pop through the launch. Note that the apps will not start unless you type in configurations (e.g. ip of host) and press the "send" button.

**Linux**

```
roslaunch linux_kinect kinect_rgbd_interaction.launch
```
This starts the application with localhost. For specified hosts use
```
roslaunch linux_kinect kinect_rgbd_interaction.launch host:="host ip address"
```
To enable face detection, turn the *optionals* option on
```
roslaunch linux_kinect kinect_rgbd_interaction.launch optionals:=1
```

## closing apps

Please press the "close" button for any windows app you have started with "send". For, Linux, just ctrl-C and wait a while till the app stops.  
Make sure to close apps on windows first before stopping apps on Linux.

## easy examples

To be updated.

## updates

The current version (master branch) is no longer compatible with previous versions (before 0.2.1). The package will have completely different install dependencies. The changes and reason for the changes are as below:

- Switching Windows code from WPF to UWP
 - ~~In order to use the latest API features provided by Microsoft, we plan to switch to a UWP application.~~
 - As some features are still only available for WPF, the microphone related codes will be a WPF app. We plan to switch all apps to UWP when possible.
- Switching from Grpc to MQTT
 - Grpc does not support UWP applications.
 - Beside the reason above, Grpc had some disadvantages on speed (too slow for streaming point clouds), installation on Linux (takes time to install and also had a damaging affect on other protobuf reliant applications), and stability (when one of the connection dies, all codes have to be restarted).
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
