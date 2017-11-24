#!/usr/bin/env python
import paho.mqtt.client as mqtt
import rospy
from std_msgs.msg import *

port = 1883

def on_connect(client, userdata, flags, respons_code):
    print('status {0}'.format(respons_code))

if __name__ == '__main__':
    rospy.init_node('network_node')

    host = rospy.get_param('~ip')

    client = mqtt.Client(protocol=mqtt.MQTTv31)
    client.on_connect = on_connect
    client.connect(host, port=port, keepalive=60)
    client.loop_start()

    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        client.publish('/network/alive', '')
        r.sleep()

