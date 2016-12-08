#!/usr/bin/env python
import paho.mqtt.client as mqtt
import rospy
import struct
from std_msgs.msg import *

port = 1883
topic = '/kinect/detected/audio'

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)

def on_message(client, userdata, mqttmsg):
    num_audio = struct.unpack('i', mqttmsg.payload[0:1] + '\x00\x00\x00')[0]
    rosmsg = Float32MultiArray()
    rosmsg.data = [0.0] * num_audio
    at = 1
    for i in range(0, num_audio):
        rosmsg.data[i] = struct.unpack('f', mqttmsg.payload[at:at + 4])[0]
        at += 4
    pub.publish(rosmsg)

if __name__ == '__main__':
    rospy.init_node('audio_beam')
    pub = rospy.Publisher('/detected/audio', Float32MultiArray, queue_size=100)

    host = rospy.get_param('~ip')

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)

    client.loop_forever()
