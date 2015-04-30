#!/usr/bin/env python
import roslib
roslib.load_manifest('pr2_pbd_interaction')

from pr2_pbd_interaction.step_types import ManipulationStep
from pr2_pbd_interaction.msg import ActionData
from pr2_pbd_interaction.srv import GetSavedActions, GetSavedActionsResponse


import rospy


def get_saved_actions():
    actions = ManipulationStep.get_saved_actions()
    actionData = []
    for action in actions:
        actionData.append(ActionData(action.name, action.id))
    return GetSavedActionsResponse(actionData)


if __name__ == '__main__':
    rospy.init_node('action_manager')
    s1 = rospy.Service('get_saved_actions', GetSavedActions, get_saved_actions)
    rospy.spin()
