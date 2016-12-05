#!/usr/bin/env python
import paho.mqtt.client as mqtt
import datetime
import rospy
from std_msgs.msg import *

port = 1883
topic1 = '/kinect/detected/speech/template'
topic2 = '/kinect/detected/speech/dictation'

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic1)
    client.subscribe(topic2)

def on_message(client, userdata, mqttmsg):
    print(mqttmsg.topic + ' ' + str(datetime.datetime.now()))
    if mqttmsg.topic == topic1:
        pub1.publish(mqttmsg.payload)
    elif mqttmsg.topic == topic2:
        pub2.publish(mqttmsg.payload)

def on_rossubscribe(msg):
    if msg.data == '/template/on':
        client.subscribe(topic1)
    elif msg.data == '/template/off':
        client.unsubscribe(topic1)
    elif msg.data == '/dictation/on':
        client.subscribe(topic2)
    elif msg.data == '/dictation/off':
        client.unsubscribe(topic2)

if __name__ == '__main__':
    rospy.init_node('speech_recognition')
    pub1 = rospy.Publisher('/detected/speech/template', String, queue_size=100)
    pub2 = rospy.Publisher('/detected/speech/dictation', String, queue_size=100)
    sub = rospy.Subscriber('/settings/speech', String, on_rossubscribe)

    host = rospy.get_param('~ip')

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)

    client.loop_forever()
