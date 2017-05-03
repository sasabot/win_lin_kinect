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
    rosmsg = Float32MultiArray() # in case multiple audio (does not happen)
    rosmsg.data = [struct.unpack('f', mqttmsg.payload[0:4])[0]]
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
