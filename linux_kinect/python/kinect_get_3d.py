#!/usr/bin/env python
import paho.mqtt.client as mqtt
import struct
import rospy
import time
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from linux_kinect.srv import *

port = 1883
topic = '/kinect/stream/centers'

def on_rossubscribe(msg):
    global points
    global requested
    if requested and points is None:
        points = msg
        requested = False

def on_rosservice_points(req):
    global requested
    global points
    requested = True
    while points is None:
        time.sleep(0.1)
    res = KinectPointsResponse(points)
    points = None
    return(res)

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)

def on_message(client, userdata, mqttmsg):
    global centers
    num_points = struct.unpack('i', mqttmsg.payload[0:2] + '\x00\x00')[0]
    cbuffer = [Point() for x in range(num_points)]
    at = 2
    for i in range(0, num_points):
        cbuffer[i].x = struct.unpack('f', mqttmsg.payload[at:at+4])[0]
        cbuffer[i].y = struct.unpack('f', mqttmsg.payload[at+4:at+8])[0]
        cbuffer[i].z = struct.unpack('f', mqttmsg.payload[at+8:at+12])[0]
        at += 12
    centers = cbuffer[0:num_points]

def on_rosservice_centers(req):
    global centers
    bbuffer = struct.pack('i', len(req.data))[0:2]
    for dat in req.data:
        bbuffer += struct.pack('i', dat.x_offset)[0:2]
        bbuffer += struct.pack('i', dat.y_offset)[0:2]
        bbuffer += struct.pack('i', dat.width)[0:2]
        bbuffer += struct.pack('i', dat.height)[0:2]
    client.publish('/kinect/request/centers', bytearray(bbuffer))
    while centers is None:
        time.sleep(0.1)
    res = KinectRequestResponse(centers)
    centers = None
    return(res)

if __name__ == '__main__':
    rospy.init_node('kinect_get_points')
    s1 = rospy.Service('/kinect/request/points', KinectPoints, on_rosservice_points)
    s2 = rospy.Service('/kinect/request/centers', KinectRequest, on_rosservice_centers)
    sub = rospy.Subscriber('/kinect/stream', PointCloud2, on_rossubscribe)

    host = rospy.get_param('~ip')
    points = None
    centers = None
    requested = False

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)
    client.loop_start()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        r.sleep()
