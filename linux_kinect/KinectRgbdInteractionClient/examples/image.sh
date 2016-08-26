#!/bin/bash
rosservice call /kinect/request/image '{data: [{x: 0, y: 0, width: 1920, height: 1080}]}'
