import utils
import rosbag
import rospy
import sys
from datetime import datetime


def message_to_csv(stream, msg, filter=[], flatten=False):
    """
    stream: StringIO
    msg: message
    """
    try:
        for s in type(msg).__slots__:
            if s in filter:
                val = msg.__getattribute__(s)
                message_to_csv(stream, val, flatten)
    except:
        msg_str = str(msg)
        if msg_str.find(",") is not -1:
            msg_str = msg_str.strip("(")
            msg_str = msg_str.strip(")")
            msg_str = msg_str.strip(" ")
            msg_str = msg_str.strip("(")
        stream.write("," + msg_str)


def message_type_to_csv(stream, msg, parent_content_name=""):
    """
    stream: StringIO
    msg: message
    """
    try:
        for s in type(msg).__slots__:
            val = msg.__getattribute__(s)
            message_type_to_csv(stream, val, ".".join([parent_content_name, s]))
    except:
        stream.write("," + parent_content_name)


def format_csv_filename(form, topic_name):
    global seq
    if form == None:
        return "Convertedbag.csv"
    ret = form.replace('%t', topic_name.replace('/', '-'))
    ret = ret[1:]
    return ret


def bag_to_csv(rosbag_file, csv_folder, topic_names, output_file_format=".csv", filter="position", header=False):
    try:
        bag = rosbag.Bag(rosbag_file)
        streamdict = dict()
    except Exception as e:
        rospy.logfatal('failed to load bag file: %s', e)
        exit(1)

    try:
        for topic, msg, time in bag.read_messages(topics=topic_names):
            # print msg
            if streamdict.has_key(topic):
                stream = streamdict[topic]
            else:
                rosbag_file = rosbag_file.replace(rosbag_file[0:rosbag_file.find('/')], csv_folder)
                rosbag_file = rosbag_file.replace('.bag', output_file_format)
                utils.make_dir_to_path(rosbag_file)
                stream = open(rosbag_file, 'w')
                streamdict[topic] = stream
                # header
                if header:
                    stream.write("time")
                    message_type_to_csv(stream, msg)
                    stream.write('\n')

            stream.write(datetime.fromtimestamp(time.to_time()).strftime('%S.%f'))
            message_to_csv(stream, msg, filter=[filter], flatten=not header)
            stream.write('\n')
        [s.close for s in streamdict.values()]
    except Exception as e:
        rospy.logwarn("fail: %s", e)
    finally:
        bag.close()


if __name__ == '__main__':
    argvs = sys.argv
    argc = len(argvs)

    rosbag_folder_name = argvs[1]
    csv_folder_name = argvs[2]

    if argc <= 2:
        print 'Usage: $ python converter.py (rosbag_folder_path) (new_folder_name)'

    filename_list = utils.enum_file_name(argvs[1])
    for filename in filename_list:
        if ".bag" in filename:
            print filename
            # bag_to_csv(rosbag_file=filename, csv_folder=csv_folder_name,
            #            topic_names=['/cyberglove/calibrated/joint_states'], header=False)
            # bag_to_csv(rosbag_file=filename, csv_folder=csv_folder_name,
            #            topic_names=['/scissor_joint'], filter="data", header=False)
            bag_to_csv(rosbag_file=filename, csv_folder=csv_folder_name,
                       topic_names=['/joints_pos'], filter="data", header=False)
