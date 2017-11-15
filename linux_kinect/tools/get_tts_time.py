#!/usr/bin/env python
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import *
import time

port = 1883
topic = '/kinect/state/ttsfinished'

speech = ''
start_time = None
do_next = False

def on_connect(client, userdata, flags, respons_code):
    client.subscribe(topic)

def speak(phrase):
    global speech, start_time, do_next
    do_next = False
    speech = phrase
    start_time = time.time()
    try:
        client.publish('/kinect/request/tts', (phrase).decode('utf-8'))
    except:
        phrase = phrase.encode('utf-8')
        client.publish('/kinect/request/tts', (phrase).decode('utf-8'))

def on_message(client, userdata, mqttmsg):
    global do_next
    print speech, (time.time() - start_time)
    do_next = True

def proceed():
    global do_next
    return do_next

if __name__ == '__main__':
    rospy.init_node('get_tts_time')

    host = rospy.get_param('~ip', 'localhost')
    phrase = rospy.get_param('~say', '')

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)
    client.loop_start()

    if '/' in phrase:
        import yaml
        stream = open(phrase, 'r')
        docs = yaml.load_all(stream)
        for doc in docs:
            for k, v in doc.items():
                if k == 'phrase' or k == 'tail':
                    if v == '':
                        continue
                    speak(v)
                    while not proceed():
                        if rospy.is_shutdown():
                            break
                        pass
    else:
        speak(phrase)
        while not proceed():
            if rospy.is_shutdown():
                break
            pass

    client.disconnect()
