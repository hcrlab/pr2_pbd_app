#!/usr/bin/env python
from geometry_msgs.msg import PointStamped, Point
import roslib
from tf import TransformListener
import tf

roslib.load_manifest('pr2_pbd_interaction')

from pr2_pbd_interaction.Robot import Robot
from pr2_pbd_interaction.msg import HeadPoseData
import sys
import os
import yaml

import rospy


def get_head_position():
    try:
        tf_listener = TransformListener()
        ref_frame = "/head_tilt_link"
        timestamp = tf_listener.getLatestCommonTime(ref_frame,
                                                          "/base_link")
        point_stamped = PointStamped()
        point_stamped.header.frame_id = ref_frame
        point_stamped.header.stamp = timestamp
        point_stamped.point = Point(1, 0, 0)
        head_position = tf_listener.transformPoint("/base_link", point_stamped).point
        return head_position
    except (tf.LookupException, tf.ConnectivityException,
            tf.ExtrapolationException) as e:
        rospy.logwarn('Something wrong with transform request for head state.' + str(e))
        return None

if __name__ == '__main__':
    rospy.init_node('save_head_pose')
    data_directory = rospy.get_param('/pr2_pbd_interaction/headPosesRoot', '.')
    file_extension = rospy.get_param('/pr2_pbd_interaction/fileExtension', '.yaml')
    if not os.path.exists(data_directory):
        os.makedirs(data_directory)
    pose_msg = HeadPoseData()
    pose_msg.head_pose = get_head_position()
    pose_msg.id = 0
    while os.path.isfile(data_directory + str(pose_msg.id) + file_extension):
        pose_msg.id += 1
    pose_msg.name = sys.argv[1] if len(sys.argv) > 1 else 'Head pose ' + str(pose_msg.id)
    pose_file = open(data_directory + str(pose_msg.id) + file_extension, 'w')
    pose_file.write(yaml.dump(pose_msg))
    pose_file.close()

