#!/usr/bin/env python

# audio play
import pyaudio

# audio plot
import matplotlib.pyplot as plt
from matplotlib import gridspec
import numpy as np
import struct
from itertools import izip

import rospy
from std_msgs.msg import UInt8MultiArray
import sys

# settings for audio play
p = pyaudio.PyAudio()
stream = p.open(format=pyaudio.paInt16,
                channels=1,
                rate=16000,
                output=True)

# settings for audio plot
__buf__  = 1024
ydats = [0 for x in range(__buf__)]
plt.ion()
gs = gridspec.GridSpec(1, 1, height_ratios=[1])
fig = plt.figure(figsize=(6, 2))
ax = fig.add_subplot(gs[0])
ax.set_ylim([-1024,1024])
line, = ax.plot(range(__buf__), ydats, '0.5')

# callback
def callback(data):
    global ydats
    # play audio
    stream.write(data.data)
    # update plot
    pairedd = iter(data.data)
    for a, b in izip(pairedd, pairedd):
        ydats += [struct.unpack('h', a + b)[0]]
    ydats = ydats[-__buf__:]
    line.set_ydata(np.array(ydats))

# rospy
rospy.init_node('playaudio')
rospy.Subscriber("/kinect/rawaudio", UInt8MultiArray, callback)

# ---main---

try:
    while not rospy.is_shutdown():
        fig.canvas.draw() # draw plot

    # finish
    stream.stop_stream()
    stream.close()

    p.terminate()
except ValueError:
    stream.stop_stream()
    stream.close()

    p.terminate()

