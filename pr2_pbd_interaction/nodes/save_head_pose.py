#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_pbd_interaction')

from pr2_pbd_interaction.Robot import Robot
from pr2_pbd_interaction.msg import HeadPoseData
import sys
import os
import yaml

import rospy

if __name__ == '__main__':
    rospy.init_node('save_head_pose')
    data_directory = rospy.get_param('/pr2_pbd_interaction/headPosesRoot')
    file_extension = rospy.get_param('/pr2_pbd_interaction/fileExtension', '.yaml')
    if not os.path.exists(data_directory):
        os.makedirs(data_directory)
    pose_msg = HeadPoseData()
    pose_msg.head_pose = Robot.get_robot().get_head_position()
    pose_msg.id = 0
    while os.path.isfile(data_directory + str(pose_msg.id) + file_extension):
        pose_msg.id += 1
    pose_msg.name = sys.argv[0] if len(sys.argv) > 0 else 'Head pose ' + str(pose_msg.id)
    pose_file = open(data_directory + str(pose_msg.id) + file_extension, 'w')
    pose_file.write(yaml.dump(pose_msg))
    pose_file.close()

