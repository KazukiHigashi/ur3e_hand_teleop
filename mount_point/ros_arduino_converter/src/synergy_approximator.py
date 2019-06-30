#!/usr/bin/python
import sys
import rospkg
import rospy
import utils
import numpy as np
import pca_utils
from std_msgs.msg import Float32MultiArray
from sklearn.decomposition import PCA

_JOINT_NAME = ["T rot.", "T MCP", "T IP", "T abd.", "I MCP", "I PIP", "I DIP", "M MCP", "M PIP", "M DIP",
               "MI abd.", "R MCP", "R PIP", "R DIP", "RM abd.", "P MCP", "P PIP", "P DIP", "PR abd.", "Palm Arch"]


class SynergyManager:
    def __init__(self, mode="grasp"):
        self.cnt = 0
        self.mode = mode

        # {"synergy name":("PCA object", "used joints list for the task")}
        self.pca_dict = {}
        self.pca = None
        self.used_joints_list = None

        self.hand_joint_num = 10  # from Thumb to Pinky (including Palm Arch)

        self.approx_posture = None

    # The number of keydown mean how long the arm move.
    # Therefore When a user press keydown specified times, that mean time of switching task from "grasp" to "switch".

    def add_synergy(self, posture_list, joints_list, pc_num, pca_name):
        pca = PCA(n_components=pc_num)
        pca.fit(posture_list)
        self.pca_dict[pca_name] = pca, joints_list

    def set_synergy(self, name):
        self.pca, self.used_joints_list = self.pca_dict[name]
        print self.pca
        print self.used_joints_list


    def approximate_posture(self, posture):
        if self.approx_posture is None:
            self.approx_posture = [0 for k in range(self.hand_joint_num)]

        rospy.loginfo(self.used_joints_list)
        extracted_posture = [posture[j_id] for j_id in self.used_joints_list]
        approx_extracted_posture = self.pca.inverse_transform(self.pca.transform([extracted_posture]))

        for i, j_id in enumerate(self.used_joints_list):
            self.approx_posture[j_id] = approx_extracted_posture[0][i]

        return self.approx_posture

    # def _change_synergy(self):
    #     if self.cnt >= self.max_keydown_cnt:
    #         rospy.logwarn("## change grasp synergy to switch synergy ##")
    #         self.mode = "switch"
    #         self.pca, self.used_joints_list = self.pca_dict[self.mode]


def synergy_approx_callback(msg, approx_posture=None):
    posture = []
    try:
        posture = list(msg.__getattribute__('data'))
    except Exception as e:
        rospy.logfatal('cannot access an attribute %s', e)
        exit(1)

    # # extract joint
    # selected_joint_list = joint_selector(mode=toss.mode)
    # extracted_posture = [posture[j_id] for j_id in selected_joint_list]
    # # approx by synergy
    # approx_extracted_posture = toss.pca.inverse_transform(toss.pca.transform([extracted_posture]))
    # # restore this to original form
    # # joints not used is fixed in the previous state (especially grasp pose)
    # for j_id, k in zip(selected_joint_list, range(len(selected_joint_list))):
    #     approx_posture[j_id] = approx_extracted_posture[0][k]

    # swap between original one and approx one by sub synergy
    msg.data = tuple(toss.approximate_posture(posture))

    # rospy.loginfo(msg.position)
    pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node('approx_synergy')

    if len(sys.argv) < 3:
        rospy.logerr("usage: rosrun approx_synergy switchable_synergy.py pc_num \'dir_name\' synergy_type")
        rospy.logerr("rospath(ros_arduino_converter)/pca_source/(dir_name)")

    pc_num = int(sys.argv[1])
    dir_name = sys.argv[2]
    synergy_type = str(sys.argv[3])

    if synergy_type == "sub":
        used_joint_list = [0, 1, 2, 3]  # for switch
    else:
        used_joint_list = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]  # for grasp


    # get a posture list for making synergy
    r = rospkg.RosPack()
    data_path = r.get_path('ros_arduino_converter') + "/pca_source/" + dir_name
    rospy.loginfo("******" + data_path + "******")

    toss = SynergyManager()

    raw_posture_list = utils.parse_any_csv(utils.enum_file_name(data_path), joint_num=10)

    # extract joint angle from posture according to used joint list of sub synergy.
    selected_joint_posture_list = [[posture[j_id] for j_id in used_joint_list] for posture in raw_posture_list]
    toss.add_synergy(posture_list=selected_joint_posture_list, joints_list=used_joint_list,
                     pc_num=pc_num, pca_name=synergy_type)
    toss.set_synergy(synergy_type)

    # rospy.loginfo("####################" + str(len(selected_joint_posture_list)))

    pub = rospy.Publisher('/joints_pos', Float32MultiArray, queue_size=1)
    sub = rospy.Subscriber('/joints_pos_old', Float32MultiArray, synergy_approx_callback)

    rospy.spin()
