#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import JointState

extracted_joint_list = [True, False, True, False, True, True, False, True, True, False, False, True, True, False,
                        False, True, True, False, False, False, False, False]

joint_bias = [170, 240, 180, 180, 181, 173, 180, 180, 165, 180,
              180, 188, 180, 180, 180, 192, 248, 180, 180, 180, 180]
joint_coef = [-320, -235, -150, -150, -160, -160, -150, -140, -140, -150,
              -150, -160, -150, -150, -150, -170, -290, -150, -150, -150, -150, -150]


def converter_callback(msg):
    pos = []
    try:
        pos = list(msg.__getattribute__('position'))
    except Exception as e:
        rospy.logfatal('cannot access position attribute %s', e)
        exit(1)

    send_msg = Float32MultiArray()
    send_msg.data = [pos[i]*joint_coef[i]+joint_bias[i] for i in range(22) if extracted_joint_list[i]]

    pub.publish(send_msg)


if __name__ == "__main__":
    rospy.init_node('float_arr_converter')

    pub = rospy.Publisher('/joints_pos', Float32MultiArray, queue_size=1)
    sub = rospy.Subscriber('/cyberglove/raw/joint_states', JointState, converter_callback)

    rospy.spin()
