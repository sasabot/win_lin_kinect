#!/usr/bin/env python
import paho.mqtt.client as mqtt
import struct
import rospy
import time
from sensor_msgs.msg import *
from linux_kinect.srv import *
import threading

port = 1883

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)

def on_message(client, userdata, mqttmsg):
    image = Image()
    image.header.frame_id = frame
    image.header.stamp = rospy.get_rostime()
    image.width = width
    image.height = height
    image.step = 3 * image.width
    image.encoding = 'bgr8'
    image.is_bigendian = True
    image.data = mqttmsg.payload
    pub.publish(image)

if __name__ == '__main__':
    rospy.init_node('kinect_get_image')
    pub = rospy.Publisher('rgb/hd', Image, queue_size=1)

    host = rospy.get_param('~ip')
    frame = rospy.get_param('~frame')
    ns = rospy.get_param('~ns')
    topic = '/' + ns + '/stream/image/hd'
    image = None

    height = 1080
    width = 1920

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)
    client.loop_start()

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        r.sleep()
