#!/usr/bin/env python
import paho.mqtt.client as mqtt
import datetime
import struct
import rospy
from sensor_msgs.msg import *

port = 1883

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)

def on_message(client, userdata, mqttmsg):
    print(mqttmsg.topic + ' ' + str(datetime.datetime.now()))
    rosmsg = CameraInfo()
    rosmsg.header.frame_id = frame
    rosmsg.header.stamp = rospy.get_rostime()
    rosmsg.height = 360
    rosmsg.width = 640
    rosmsg.D = [struct.unpack('f', mqttmsg.payload[16:20])[0], struct.unpack('f', mqttmsg.payload[20:24])[0], 0, 0, 0]
    rosmsg.R = [1, 0, 0, 0, 1, 0, 0, 0, 1]
    rosmsg.K = [struct.unpack('f', mqttmsg.payload[0:4])[0] * 3.0, 0, struct.unpack('f', mqttmsg.payload[8:12])[0] * 3.0,
                0, struct.unpack('f', mqttmsg.payload[4:8])[0] / 3.0, struct.unpack('f', mqttmsg.payload[12:16])[0] / 3.0,
                0, 0, 1]
    rosmsg.P = [struct.unpack('f', mqttmsg.payload[24:28])[0] / 3.0, 0, struct.unpack('f', mqttmsg.payload[32:36])[0] / 3.0, 0,
                0, struct.unpack('f', mqttmsg.payload[28:32])[0] / 3.0, struct.unpack('f', mqttmsg.payload[36:40])[0] / 3.0, 0,
                0, 0, 1, 0]
    pub.publish(rosmsg)

if __name__ == '__main__':
    rospy.init_node('kinect_camerainfo_stream')
    pub = rospy.Publisher('camera_info', CameraInfo, queue_size=100)

    host = rospy.get_param('~ip')
    frame = rospy.get_param('~frame')
    ns = rospy.get_param('~ns')
    topic = '/' + ns + '/stream/camerainfo'

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)
    client.loop_start()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        r.sleep()
