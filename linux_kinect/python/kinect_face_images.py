#!/usr/bin/env python
import paho.mqtt.client as mqtt
import struct
import rospy
import rospkg
from sensor_msgs.msg import *
from geometry_msgs.msg import *
from std_msgs.msg import *
import time

port = 1883
header_size = 20

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)
    client.subscribe(dbgtopic)

def on_message(client, userdata, mqttmsg):
    # if debug msg
    if mqttmsg.topic == dbgtopic:
        data = mqttmsg.payload
        rosmsg = Image()
        rosmsg.header.frame_id = 'base_link'
        rosmsg.header.stamp = rospy.get_rostime()
        rosmsg.width = struct.unpack('i', data[0:2] + '\x00\x00')[0]
        rosmsg.height = struct.unpack('i', data[2:4] + '\x00\x00')[0]
        rosmsg.step = rosmsg.width
        rosmsg.encoding = 'mono8'
        rosmsg.is_bigendian = True
        rosmsg.data = data[4:rosmsg.width * rosmsg.height + 4]
        pubdbg.publish(rosmsg)
        return

    msg = UInt8MultiArray()
    # change face id to sensor id
    num_faces = struct.unpack('i', mqttmsg.payload[0:1] + '\x00\x00\x00')[0]
    msg.data += mqttmsg.payload[0:1]
    at = 1
    for i in range(0, num_faces):
        width = struct.unpack('i', mqttmsg.payload[at:at + 2] + '\x00\x00')[0]
        height = struct.unpack('i', mqttmsg.payload[at + 2:at + 4] + '\x00\x00')[0]
        size = width * height * 3
        msg.data += mqttmsg.payload[at:at + 16]
        msg.data += struct.pack('i', sensor_id)[0:4]
        at += header_size
        msg.data += mqttmsg.payload[at:at + size]
        at += size
    pub.publish(msg)

def on_rossubscribe(msg):
    bbuffer = struct.pack('i', len(msg.points))[0:2]
    for p in msg.points:
        bbuffer += struct.pack('f', p.x)[0:4] # for p[i] i >= 1 : partition
        bbuffer += struct.pack('f', p.y)[0:4] # coefficient a
        bbuffer += struct.pack('f', p.z)[0:4] # coefficient b
    client.publish('/kinect/request/facetrack/bounds', bytearray(bbuffer))

if __name__ == '__main__':
    rospy.init_node('kinect_face_images')

    pub = rospy.Publisher('/roboenvcv/cropped/images', UInt8MultiArray, queue_size=100)
    pubdbg = rospy.Publisher('/kinect/face/debug', Image, queue_size=100)

    host = rospy.get_param('~ip')
    sensor_id = rospy.get_param('~sensor_id')
    depth_threshold = rospy.get_param('~depth_threshold')

    topic = '/kinect/detected/face'
    dbgtopic = '/kinect/face/debug'

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)

    # client used in sub, therefore should be initiated after client setup
    sub = rospy.Subscriber('/slam/env/bounds', Polygon, on_rossubscribe)

    time.sleep(2.0) # wait for everything to get prepared

    # set depth
    depthbuffer = struct.pack('i', 1)[0:2]
    depthbuffer += struct.pack('f', 0.0)[0:4] # for p[i] i >= 1 : partition
    depthbuffer += struct.pack('f', 0.0)[0:4] # coefficient a
    depthbuffer += struct.pack('f', depth_threshold)[0:4] # coefficient b
    client.publish('/kinect/request/facetrack/bounds', bytearray(depthbuffer))
    client.loop_start()

    r = rospy.Rate(30)
    while not rospy.is_shutdown():
        r.sleep()
