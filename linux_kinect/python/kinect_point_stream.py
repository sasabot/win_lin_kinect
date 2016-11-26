#!/usr/bin/env python
import paho.mqtt.client as mqtt
import datetime
import rospy
from sensor_msgs.msg import *

host = ''
port = 1883
topic = '/kinect/stream/points'

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)

def on_message(client, userdata, mqttmsg):
    print(mqttmsg.topic + ' ' + str(datetime.datetime.now()))
    rosmsg = PointCloud2()
    rosmsg.header.frame_id = 'base_link'
    rosmsg.header.stamp = rospy.get_rostime()
    rosmsg.fields = [PointField(name='x',offset=0,datatype=7,count=1), PointField(name='y',offset=4,datatype=7,count=1), PointField(name='z',offset=8,datatype=7,count=1), PointField(name='rgb',offset=12,datatype=7,count=1)]
    rosmsg.height = 640
    rosmsg.width = 360
    rosmsg.point_step = 16
    rosmsg.row_step = rosmsg.point_step * rosmsg.width
    rosmsg.is_dense = False
    rosmsg.is_bigendian = True
    rosmsg.data = mqttmsg.payload
    pub.publish(rosmsg)

if __name__ == '__main__':
    rospy.init_node('kinect_point_stream')
    pub = rospy.Publisher('/kinect/stream', PointCloud2, queue_size=100)

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)

    client.loop_forever()
