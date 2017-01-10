#!/usr/bin/env python
import paho.mqtt.client as mqtt
import struct
import rospy
import rospkg
from sensor_msgs.msg import *
from geometry_msgs.msg import *

import chainer
import cv2
import numpy as np
import hyperfacemodel

import threading

port = 1883
topic = '/kinect/detected/face'
dbgtopic = '/kinect/face/debug'

max_faces = 3
header_size = 20

def solve(field, img):
    size = img.shape[0:2]
    img = img.astype(np.float32) / 255.0
    img = cv2.resize(img, (227, 227))
    img = cv2.normalize(img, None, -0.5, 0.5, cv2.NORM_MINMAX)
    imgs = np.asarray([np.transpose(img, (2, 0, 1))])
    x = chainer.Variable(imgs, volatile=True)
    y = models(x)

    face_id = struct.unpack('i', bytearray(field[12:16]))[0]
    print(face_id)
    position = [struct.unpack('f', bytearray(field[0:4]))[0],
                struct.unpack('f', bytearray(field[4:8]))[0],
                struct.unpack('f', bytearray(field[8:12]))[0]]
    print(position)
    pose = (y['pose'].data)[0]
    print(pose)


def backthread(data):
    global onrun

    # parse msg
    num_faces = struct.unpack('i', data[0:1] + '\x00\x00\x00')[0]
    if num_faces == 0:
        runmutex.acquire()
        try:
            onrun = False
        finally:
            runmutex.release()
        return
    imgs = [None for x in range(num_faces)]
    rest = [[] for x in range(num_faces)]
    at = 1
    for i in range(0, num_faces):
        width = struct.unpack('i', data[at:at + 2] + '\x00\x00')[0]
        height = struct.unpack('i', data[at + 2:at + 4] + '\x00\x00')[0]
        rest[i][:] = data[at + 4:at + header_size]
        at += header_size
        raw_array = np.fromstring(data[at:at + width * height * 3], np.uint8)
        imgs[i] = np.reshape(raw_array, (height, width, 3))
        at += width * height * 3

    # conduct face analysis
    threads = [None for x in range(len(imgs))]
    for idx, img in enumerate(imgs):
        threads[idx] = threading.Thread(target=solve, args=(rest[idx], img))
    for t in threads:
        t.start()
    for t in threads:
        t.join()

    print('---')
    runmutex.acquire()
    try:
        onrun = False
    finally:
        runmutex.release()


def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))
    client.subscribe(topic)
    client.subscribe(dbgtopic)


def on_message(client, userdata, mqttmsg):
    global onrun

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
        pub.publish(rosmsg)
        return

    # data = mqttmsg.payload
    # if struct.unpack('i', data[0:1] + '\x00\x00\x00')[0] > 0:
    #     rosmsg = Image()
    #     rosmsg.header.frame_id = 'base_link'
    #     rosmsg.header.stamp = rospy.get_rostime()
    #     rosmsg.width = struct.unpack('i', data[1:3] + '\x00\x00')[0]
    #     rosmsg.height = struct.unpack('i', data[3:5] + '\x00\x00')[0]
    #     rosmsg.step = 3 * rosmsg.width
    #     rosmsg.encoding = 'bgr8'
    #     rosmsg.is_bigendian = True
    #     rosmsg.data = data[header_size + 1:rosmsg.width * rosmsg.height * 3 + header_size + 1]
    #     pub.publish(rosmsg)

    # run on backthread to avoid delays
    runmutex.acquire()
    try:
        if not onrun:
            onrun = True
            thread = threading.Thread(target=backthread, args=(mqttmsg.payload,))
            thread.start()
    finally:
        runmutex.release()


def on_rossubscribe(msg):
    bbuffer = struct.pack('i', len(msg.points))[0:2]
    for p in msg.points:
        bbuffer += struct.pack('f', p.x)[0:4] # for p[i] i >= 1 : partition
        bbuffer += struct.pack('f', p.y)[0:4] # coefficient a
        bbuffer += struct.pack('f', p.z)[0:4] # coefficient b
    client.publish('/kinect/request/facetrack/bounds', bytearray(bbuffer))


if __name__ == '__main__':
    rospy.init_node('face_pose_detector')
    pub = rospy.Publisher('/kinect/face/debug', Image, queue_size=100)

    host = rospy.get_param('~ip')

    rospack = rospkg.RosPack()
    pkgpath = rospack.get_path('linux_kinect')

    models = hyperfacemodel.HyperFaceModel()
    chainer.serializers.load_npz(pkgpath + '/python/model_epoch_190', models)

    onrun = False
    runmutex = threading.Lock()

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(host, port=port, keepalive=60)

    # client used in sub, therefore should be initiated after client setup
    sub = rospy.Subscriber('/slam/env/bounds', Polygon, on_rossubscribe)

    client.loop_forever()
