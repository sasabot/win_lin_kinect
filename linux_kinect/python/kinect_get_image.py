#!/usr/bin/env python
import paho.mqtt.client as mqtt
import struct
import rospy
import time
from sensor_msgs.msg import *
from linux_kinect.srv import *

port = 1883
topic = '/kinect/stream/image'

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)

def on_message(client, userdata, mqttmsg):
    global image
    global filled
    image = Image()
    image.header.frame_id = frame
    image.header.stamp = rospy.get_rostime()
    image.width = width
    image.height = height
    image.step = 3 * image.width
    image.encoding = 'bgr8'
    image.is_bigendian = True
    image.data = mqttmsg.payload
    filled = True

def on_rosservice(req):
    global filled
    client.publish('/kinect/request/image', '')
    while not filled:
        time.sleep(0.1)
    res = KinectImageResponse(image)
    filled = False
    return(res)

if __name__ == '__main__':
    rospy.init_node('kinect_get_image')
    s = rospy.Service('/kinect/request/image', KinectImage, on_rosservice)

    host = rospy.get_param('~ip')
    frame = rospy.get_param('~frame')
    image = None
    filled = False

    height = 1080
    width = 1920

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)
    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        r.sleep()
