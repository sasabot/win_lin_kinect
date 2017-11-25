#!/usr/bin/env python
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import *

port = 1883
topic = '/kinect/state/ttsfinished'

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)

def on_rosmessage(msg):
    client.publish('/kinect/request/tts', (msg.data).decode('utf-8'))

def on_message(client, userdata, mqttmsg):
    pub.publish('')

if __name__ == '__main__':
    rospy.init_node('windows_tts')
    pub = rospy.Publisher('/windows/voice/finished', String, queue_size=100)
    sub = rospy.Subscriber('/windows/voice', String, on_rosmessage)

    host = rospy.get_param('~ip')

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)
    client.loop_start()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        r.sleep()
