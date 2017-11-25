#!/usr/bin/env python

# brief: rings an alarm on windows linked by /topic
# usage: warn with an alarm when /topic is called

import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import *

port = 1883

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))

def on_rosmessage(msg):
    client.publish('/windows/alarm', alarm)

if __name__ == '__main__':
    rospy.init_node('windows_alarm_node')
    sub = rospy.Subscriber('/topic', String, on_rosmessage)

    host = rospy.get_param('~ip')
    # ~alarm:
    # Default, IM, Mail, Reminder, SMS,
    # Alarm, Alarm.2 ~ Alarm.10, Call, Call.2 ~ Call.10
    alarm = rospy.get_param('~alarm')

    if 'Alarm' in alarm or 'Call' in alarm:
        alarm = 'ms-winsoundevent:Notification.Looping.' + alarm
    else:
        alarm = 'ms-winsoundevent:Notification.' + alarm

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.connect(host, port=port, keepalive=60)
    client.loop_start()

    r = rospy.Rate(60)
    while not rospy.is_shutdown():
        r.sleep()
