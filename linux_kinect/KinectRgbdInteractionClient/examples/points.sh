#!/bin/bash
rosservice call /kinect/request/points '{data: [{x: 0, y: 0, width: 512, height: 424}]}'
