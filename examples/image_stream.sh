#!/bin/bash
rosservice call /kinect/request '{mode: 2, data: [{x: 0, y: 0, width: 1920, height: 1080}], once: false}'
