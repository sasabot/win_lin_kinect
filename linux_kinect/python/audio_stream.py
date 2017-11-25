#!/usr/bin/env python
import paho.mqtt.client as mqtt
import datetime
import rospy
from std_msgs.msg import *

port = 1883
topic = '/kinect/stream/rawaudio'

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)

def on_message(client, userdata, mqttmsg):
    print(mqttmsg.topic + ' ' + str(datetime.datetime.now()))
    rosmsg = UInt8MultiArray()
    rosmsg.data = mqttmsg.payload
    pub.publish(rosmsg)

if __name__ == '__main__':
    rospy.init_node('kinect_rawaudio_stream')
    pub = rospy.Publisher('/kinect/rawaudio', UInt8MultiArray, queue_size=100)

    host = rospy.get_param('~ip')

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)
    client.loop_start()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        r.sleep()
