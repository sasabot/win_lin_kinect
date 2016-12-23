#!/usr/bin/env python
import paho.mqtt.client as mqtt
import struct
import rospy
import time
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from linux_kinect.srv import *

port = 1883
topic = '/kinect/response/ocr'

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)

def on_message(client, userdata, mqttmsg):
    global ocr
    results = mqttmsg.payload.split(' ;;; ')
    counts = []
    texts = []
    for res in results:
        vec = res.split(' &&& ')
        counts += [len(vec)]
        texts += vec[:]
    ocr = OcrResponse(counts, texts)

def on_rosservice(req):
    global ocr
    bbuffer = struct.pack('i', len(req.images))[0:2]
    for image in req.images:
        bbuffer += struct.pack('i', image.width)[0:2]
        bbuffer += struct.pack('i', image.height)[0:2]
        bbuffer += image.data[:]
    client.publish('/kinect/request/ocr', bytearray(bbuffer))
    while ocr is None:
        time.sleep(0.1)
    res = ocr
    ocr = None
    return(res)

if __name__ == '__main__':
    rospy.init_node('kinect_get_points')
    s = rospy.Service('/request/ocr', Ocr, on_rosservice)

    host = rospy.get_param('~ip')

    ocr = None

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)

    client.loop_forever()
