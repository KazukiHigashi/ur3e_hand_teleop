#!/usr/bin/env python

import argparse

import rospy

from ur_control.arm import Arm
import ur3_kinematics.e_arm as ur3_arm

import numpy as np

def main():

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__)
    parser.add_argument('--robot', action='store_true', help='for the real robot')
    args = parser.parse_args(rospy.myargv()[1:])

    rospy.init_node("go_init_state")

    global arm
    if args.robot:
        arm = Arm(ft_sensor=True, real_robot=True)
    else:
        arm = Arm(ft_sensor=True, real_robot=False)

    print("Start Pose:", arm.joint_angles())
    
    goal_position = [1.5148, -1.7964, 1.8328, -1.6100, -1.6120, -1.2040] 
    arm.set_joint_positions(goal_position, t=3)

if __name__ == '__main__':
    main()
