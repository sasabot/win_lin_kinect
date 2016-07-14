#!/bin/bash
rosservice call /kinect/request '{mode: 1, data: [{x: 0, y: 0, width: 512, height: 414}], once: false}'
