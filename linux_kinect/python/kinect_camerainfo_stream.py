#!/usr/bin/env python
import paho.mqtt.client as mqtt
import datetime
import struct
import rospy
from sensor_msgs.msg import *

host = ''
port = 1883
topic = '/kinect/stream/camerainfo'

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)

def on_message(client, userdata, mqttmsg):
    print(mqttmsg.topic + ' ' + str(datetime.datetime.now()))
    rosmsg = CameraInfo()
    rosmsg.header.frame_id = 'base_link'
    rosmsg.header.stamp = rospy.get_rostime()
    rosmsg.height = 360
    rosmsg.width = 640
    rosmsg.K = [struct.unpack('f', mqttmsg.payload[0:4])[0], 0, struct.unpack('f', mqttmsg.payload[8:12])[0], 0, struct.unpack('f', mqttmsg.payload[4:8])[0], struct.unpack('f', mqttmsg.payload[12:16])[0], 0, 0, 1]
    pub.publish(rosmsg)

if __name__ == '__main__':
    rospy.init_node('kinect_camerainfo_stream')
    pub = rospy.Publisher('/kinect/camera_info', CameraInfo, queue_size=100)

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)

    client.loop_forever()
