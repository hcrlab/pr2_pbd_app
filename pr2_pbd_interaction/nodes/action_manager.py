#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_pbd_interaction')

from pr2_pbd_interaction.step_types import ManipulationStep
from pr2_pbd_interaction.msg import ActionData, HeadPoseData
from pr2_pbd_interaction.srv import GetSavedActions, GetSavedActionsResponse, \
    GetSavedHeadPoses, GetSavedHeadPosesResponse


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
                head_poses.append()
    return GetSavedHeadPosesResponse(head_poses)


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
    rospy.spin()
