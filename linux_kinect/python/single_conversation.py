#!/usr/bin/env python

import rospy
import numpy as np
import time
from collections import namedtuple
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

Result = namedtuple('Result', 'phrase time')

result_ = ''
def resultIn(msg):
    print 'resultIn %s' % msg.data
    global result_
    result_ = Result(msg.data, time.time())

def speak():
    print 'speak %s, %f, %s' % (phrase, speakfor, template)
    if phrase != '':
        speechpub.publish(phrase) # speak phrase
    if template != '':
        templatepub.publish(template) # set answer choices
    time.sleep(speakfor) # wait till speech finish

def listen():
    print 'listen %f' % listenfor
    global result_
    if validtimestamp < 0:
        result_ = Result('', time.time())
    listen_start = time.time()
    while True:
        if (time.time() - listen_start) > listenfor:
            return False
        elif result_.phrase == '':
            pass
        elif validtimestamp > 0 and (time.time() - result_.time) > validtimestamp:
            pass
        else:
            return True

def conversation(req):
    print 'conversation'
    speak()
    flag = listen()
    return TriggerResponse(flag, result_.phrase)

if __name__ == '__main__':
    rospy.init_node('single_conversation')
    phrase = rospy.get_param('~phrase', '')
    speakfor = rospy.get_param('~speakfor', 5.0)
    listenfor = rospy.get_param('~listenfor', 15.0)
    template = rospy.get_param('~template', 'en-US;yes_or_no.xml')
    validtimestamp = rospy.get_param('~validtimestamp', -1)

    rospy.Service('/linux_kinect/single_conversation', Trigger, conversation)
    speechpub = rospy.Publisher('/speech', String, queue_size=10)
    templatepub = rospy.Publisher('/settings/speech', String, queue_size=10)
    rospy.Subscriber('/detected/speech/template', String, resultIn)

    rospy.spin()
