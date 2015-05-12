#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import os
from os import listdir
from os.path import isfile, join
import yaml

from pr2_pbd_interaction.step_types import ManipulationStep
from pr2_pbd_interaction.msg import ActionData, HeadPoseData
from pr2_pbd_interaction.srv import GetSavedActions, GetSavedActionsResponse, \
    GetSavedHeadPoses, GetSavedHeadPosesResponse, ExecuteHeadStep, ExecuteHeadStepResponse


import rospy


def get_saved_head_poses(dummy):
    head_poses = []
    data_directory = rospy.get_param('/pr2_pbd_interaction/headPosesRoot')
    file_extension = rospy.get_param('/pr2_pbd_interaction/fileExtension', '.yaml')
    if not os.path.exists(data_directory):
        return []
    for f in listdir(data_directory):
        file_path = join(data_directory, f)
        if isfile(file_path) and file_path.endswith(file_extension):
            with open(file_path, 'r') as content_file:
                head_poses.append(yaml.load(content_file))
    return GetSavedHeadPosesResponse(head_poses)

def execute_head_step(req):
    step_id = req.step_id
    data_directory = rospy.get_param('/pr2_pbd_interaction/headPosesRoot')
    file_extension = rospy.get_param('/pr2_pbd_interaction/fileExtension', '.yaml')
    file_path = data_directory + str(step_id) + file_extension
    if not os.path.exists(file_path):
        return ExecuteHeadStepResponse(False)
    with open(file_path, 'r') as content_file:
        head_pose = yaml.load(content_file).head_pose
        Robot.get_robot().move_head_to_point(head_pose)
        return ExecuteHeadStepResponse(True)


def get_saved_actions(dummy):
    actions = ManipulationStep.get_saved_actions()
    actionData = []
    for action in actions:
        actionData.append(ActionData(action.name, action.id))
    return GetSavedActionsResponse(actionData)


if __name__ == '__main__':
    rospy.init_node('action_manager')
    s1 = rospy.Service('get_saved_actions', GetSavedActions, get_saved_actions)
    s2 = rospy.Service('get_saved_head_poses', GetSavedHeadPoses, get_saved_head_poses)
    s3 = rospy.Service('execute_head_step', ExecuteHeadStep, execute_head_step)
    rospy.spin()
