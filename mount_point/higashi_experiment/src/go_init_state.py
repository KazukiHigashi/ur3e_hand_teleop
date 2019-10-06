#!/usr/bin/env python

import argparse

import rospy

from ur_control.arm import Arm
import ur3_kinematics.e_arm as ur3_arm

import numpy as np

def main():
    rospy.init_node("go_init_state")

    bindings = {
        'scissor' : ([1.5148, -1.7964, 1.8328, -1.7260, -1.6120, -1.6275], "for Scissors-task"),
        'driver'  : ([1.3354, -1.0662, 1.7262, -0.7019, 0.2937,  -1.4613], "for Screwdriver-task")
    }

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument('--robot', action='store_true', help='for the real robot')
    parser.add_argument('--task', type=str)
    args = parser.parse_args(rospy.myargv()[1:])

    global arm
    if args.robot:
        arm = Arm(ft_sensor=True, real_robot=True)
    else:
        arm = Arm(ft_sensor=True, real_robot=False)

    if args.task in bindings:
        rospy.loginfo("Move the initial state {}".format(bindings[args.task][1]))
        arm.set_joint_positions(bindings[args.task][0], t=3)


if __name__ == '__main__':
    main()
