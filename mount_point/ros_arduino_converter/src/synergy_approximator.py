#!/usr/bin/python
import sys
import rospkg
import rospy
import utils
import numpy as np
import pca_utils
import argparse
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import String
from sklearn.decomposition import PCA

_JOINT_NAME = ["T rot.", "T MCP", "T IP", "T abd.", "I MCP", "I PIP", "I DIP", "M MCP", "M PIP", "M DIP",
               "MI abd.", "R MCP", "R PIP", "R DIP", "RM abd.", "P MCP", "P PIP", "P DIP", "PR abd.", "Palm Arch"]


class SynergyManager:
    def __init__(self, init_syn="grasp", posture_list=None, synergy_description_list=None):
        self.cnt = 0

        self.synergy_description_list = synergy_description_list

        # {"synergy name":("PCA object", "used joints list for the task")}
        self.pca_dict = {}
        self.pca = None
        self.used_joints_list = None

        self.hand_joint_num = 10  # from Thumb to Pinky (including Palm Arch)

        self.approx_posture = None

        self._setup_synergy(posture_list=posture_list)
        self.set_synergy(syn_name=init_syn)


    def _setup_synergy(self, posture_list):
        for name, description in self.synergy_description_list.items():
            used_joint_list = description[0]
            pc_num = description[1]
            selected_joint_posture_list = [[posture[j_id] for j_id in used_joint_list] for posture in posture_list]
            self.add_synergy(posture_list=selected_joint_posture_list, joints_list=used_joint_list, pc_num=pc_num, pca_name=name)

            rospy.loginfo("set %s successfully" % name)
             

    def add_synergy(self, posture_list, joints_list, pc_num, pca_name):
        pca = PCA(n_components=pc_num)
        pca.fit(posture_list)
        self.pca_dict[pca_name] = pca, joints_list


    def set_synergy(self, syn_name):
        self.pca, self.used_joints_list = self.pca_dict[syn_name]
        print self.pca
        print self.used_joints_list


    def approximate_posture(self, posture):
        if self.approx_posture is None:
            self.approx_posture = [140 for k in range(self.hand_joint_num)]

        rospy.loginfo(self.used_joints_list)
        extracted_posture = [posture[j_id] for j_id in self.used_joints_list]
        approx_extracted_posture = self.pca.inverse_transform(self.pca.transform([extracted_posture]))

        for i, j_id in enumerate(self.used_joints_list):
            self.approx_posture[j_id] = approx_extracted_posture[0][i]

        return self.approx_posture



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

def switch_synergy_callback(msg):
    bindings = {
        '0': ("grasp", "Grasp Synergy"),
        '1': ("sub1", "Subsynergy (only 2nd finger)"),
        '2': ("sub2", "Subsynergy (1st, 2nd and 3rd fingers)"),
        '3': ("sub3", "Subsynergy (1st and 2nd)")
    }

    c = msg.data

    if c in bindings:
        toss.set_synergy(syn_name=bindings[c][0])
        rospy.loginfo("switch synergy to %s" % bindings[c][0])
        

def main():
    """Synergy Approximator
    Approximate postures of the humanoid hand by synergies.
    """
    epilog = """
    aiueo
    """
    rospy.init_node('approx_synergy')

    if len(sys.argv) < 3:
        rospy.logerr("usage: rosrun approx_synergy switchable_synergy.py pc_num \'dir_name\' synergy_type")
        rospy.logerr("rospath(ros_arduino_converter)/pca_source/(dir_name)")

    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                     description=main.__doc__,
                                     epilog=epilog)
    parser.add_argument('--pc', action='append', type=int, help='1:grasp 2:sub1(1st2nd3rd fingers) 3:sub2(only 2nd)')
    parser.add_argument('--dir_name', type=str)
    parser.add_argument('--init_syn', type=str)

    args = parser.parse_args(rospy.myargv()[1:])
    dir_name = args.dir_name
    pc_num_list = args.pc
    init_syn = args.init_syn

    # get a posture list for making synergy
    r = rospkg.RosPack()
    data_path = r.get_path('ros_arduino_converter') + "/pca_source/" + dir_name
    rospy.loginfo("******" + data_path + "******")
    raw_posture_list = utils.parse_any_csv(utils.enum_file_name(data_path), joint_num=10)
    
    # set synergy descriptions
    synergy_description_list = {
            'grasp': ([0, 1, 2, 3, 4, 5, 6, 7, 8, 9], pc_num_list[0], "Grasp Synergy"),
            'sub1' : ([0, 1, 2, 3, 4, 5], pc_num_list[1], "Subsynergy(1st, 2nd and 3rd fingers)"),
            'sub2' : ([2, 3], pc_num_list[2], "Subsynergy(only 2nd finger)")
            }

    global toss
    toss = SynergyManager(init_syn=init_syn, posture_list=raw_posture_list, synergy_description_list=synergy_description_list)

    pub = rospy.Publisher('/joints_pos', Float32MultiArray, queue_size=1)
    sub = rospy.Subscriber('/joints_pos_old', Float32MultiArray, synergy_approx_callback)
    switch_synergy_sub = rospy.Subscriber('/key', String, switch_synergy_callback)

    rospy.spin()


if __name__ == '__main__':
    main()

