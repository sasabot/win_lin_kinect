#!/usr/bin/env python
import paho.mqtt.client as mqtt
import datetime
import struct
import rospy
import rospkg
from sensor_msgs.msg import *

import chainer
import cv2
import numpy as np
import hyperfacemodel

import threading

port = 1883
topic = '/kinect/detected/face'

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

    # stop tracking if image was not a face
    if (y['detection'].data)[0][1] < 0.1:
        client.publish('/kinect/request/facetrack/freeid', bytearray(field[12:16]))

    # track sided faces (yaw > 1.0 [rad]) on client, as server cannot track sided faces
    if abs((y['pose'].data)[0][2]) > 1.0:
        nose = (y['landmark'].data)[0][26:28]
        # should only update bounds when face had large movement
        if abs(nose[0]) > 0.25 or abs(nose[1]) > 0.25:
            # * 0.5 to slightly avoid nose being center of image
            # this makes tracking more stable when noise is added to nose position
            dx = struct.pack('i', int(nose[0] * size[0] * 0.5))
            dy = struct.pack('i', int(nose[1] * size[1] * 0.5))
            client.publish('/kinect/request/facetrack/manualadjust', bytearray(field[12:16]) + dx[0:2] + dy[0:2])

    pose = (y['pose'].data)[0]
    print(pose)

def backthread(imgs, rest):
    global onrun
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

def on_message(client, userdata, mqttmsg):
    global onrun
    data = mqttmsg.payload
    num_faces = struct.unpack('i', data[0:1] + '\x00\x00\x00')[0]
    # print(mqttmsg.topic + ' faces: ' + str(num_faces) + ' ' + str(datetime.datetime.now()))
    if num_faces == 0:
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
    runmutex.acquire()
    try:
        if not onrun:
            onrun = True
            thread = threading.Thread(target=backthread, args=(imgs, rest))
            thread.start()
    finally:
        runmutex.release()
    # publish image of first detected face
    rosmsg = Image()
    rosmsg.header.frame_id = 'base_link'
    rosmsg.header.stamp = rospy.get_rostime()
    rosmsg.width = width
    rosmsg.height = height
    rosmsg.step = 3 * rosmsg.width
    rosmsg.encoding = 'bgr8'
    rosmsg.is_bigendian = True
    rosmsg.data = data[header_size + 1:rosmsg.width * rosmsg.height * 3 + header_size + 1]
    pub.publish(rosmsg)

if __name__ == '__main__':
    rospy.init_node('face_pose_detector')
    pub = rospy.Publisher('/kinect/face', Image, queue_size=100)

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

    client.loop_forever()
