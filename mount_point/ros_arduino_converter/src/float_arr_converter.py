#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

extracted_joint_list = [True, True, False, False, True, True, False, True, True, False, False, True, True, False,
                        False, True, True, False, False, False, False, False]


def converter_callback(msg):
    pos = []
    try:
        pos = list(msg.__getattribute__('position'))
    except Exception as e:
        rospy.logfatal('cannot access position attribute %s', e)
        exit(1)

    send_msg = Float32MultiArray()
    send_msg.data = [pos[i] for i in range(22) if extracted_joint_list[i]]

    pub.publish(send_msg)


if __name__ == "__main__":
    rospy.init_node('float_arr_converter')

    pub = rospy.Publisher('/joints_pos', Float32MultiArray, queue_size=1)
    sub = rospy.Subscriber('/cyberglove/raw/joint_states', JointState, converter_callback)

    rospy.spin()
