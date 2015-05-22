#!/usr/bin/env python
from geometry_msgs.msg import Point
import roslib
roslib.load_manifest('pr2_pbd_interaction')
import os
from os import listdir
from os.path import isfile, join
import yaml

from pr2_pbd_interaction.step_types import ManipulationStep
from pr2_pbd_interaction.msg import ActionData, HeadPoseData, ConditionType
from pr2_pbd_interaction.srv import GetSavedActions, GetSavedActionsResponse, \
    GetSavedHeadPoses, GetSavedHeadPosesResponse, ExecuteHeadStep, ExecuteHeadStepResponse,\
    ExecuteManipulationStep, ExecuteManipulationStepResponse, \
    CheckManipulationConditions, CheckManipulationConditionsResponse
from pr2_pbd_interaction.Robot import Robot
from actionlib import SimpleActionClient
from pr2_controllers_msgs.msg import PointHeadAction, PointHeadGoal

import rospy


def get_saved_head_poses(dummy):
    rospy.loginfo('Getting saved head poses.')
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


def move_head_to_point_and_wait(point):
    rospy.loginfo("Moving head to point and waiting for action to finish")
    headGoal = PointHeadGoal()
    headGoal.target.header.frame_id = 'base_link'
    headGoal.min_duration = rospy.Duration(1.0)
    headGoal.target.point = Point(1,0,1)
    headGoal.target.point = point
    headActionClient.send_goal(headGoal)
    headActionClient.wait_for_result(rospy.Duration(10))
    rospy.loginfo("Head action client finished")


def execute_head_step(req):
    rospy.loginfo('Executing head pose.')
    step_id = req.step_id
    data_directory = rospy.get_param('/pr2_pbd_interaction/headPosesRoot')
    file_extension = rospy.get_param('/pr2_pbd_interaction/fileExtension', '.yaml')
    file_path = data_directory + str(step_id) + file_extension
    if not os.path.exists(file_path):
        return ExecuteHeadStepResponse(0)
    with open(file_path, 'r') as content_file:
        head_pose = yaml.load(content_file).head_pose
        move_head_to_point_and_wait(head_pose)
        return ExecuteHeadStepResponse(1)


def execute_manipulation_step(req):
    rospy.loginfo('Executing manipulation step.')
    step_id = req.step_id
    actions = ManipulationStep.get_saved_actions()
    if step_id >= len(actions):
        return ExecuteHeadStepResponse(0)
    action = actions[step_id]
    action.execute()
    return ExecuteManipulationStepResponse(action.execution_status)


def check_manipulation_conditions(req):
    condition_type = req.condition_type
    rospy.loginfo('Checking condition of type ' + str(condition_type))
    step_id = req.step_id
    actions = ManipulationStep.get_saved_actions()
    if step_id >= len(actions):
        return CheckManipulationConditionsResponse(False)
    action = actions[step_id]
    return CheckManipulationConditionsResponse(action.check_condition(condition_type))


def get_saved_actions(dummy):
    rospy.loginfo('Getting saved manipulation poses')
    actions = ManipulationStep.get_saved_actions()
    actionData = []
    for action in actions:
        actionData.append(ActionData(action.name, action.id))
    return GetSavedActionsResponse(actionData)


if __name__ == '__main__':
    rospy.init_node('action_manager')
    robot = Robot.get_robot()
    rospy.loginfo('Robot initialized.')
    headActionClient = SimpleActionClient('/head_traj_controller/point_head_action', PointHeadAction)
    headActionClient.wait_for_server()
    s1 = rospy.Service('get_saved_actions', GetSavedActions, get_saved_actions)
    rospy.loginfo('Started get saved manipulation actions service.')
    s2 = rospy.Service('get_saved_head_poses', GetSavedHeadPoses, get_saved_head_poses)
    rospy.loginfo('Started get saved head poses service.')
    s3 = rospy.Service('execute_head_step', ExecuteHeadStep, execute_head_step)
    rospy.loginfo('Started execute head step service.')
    s4 = rospy.Service('execute_manipulation_step', ExecuteManipulationStep, execute_manipulation_step)
    rospy.loginfo('Started execute manipulation step service.')
    s5 = rospy.Service('check_manipulation_conditions', CheckManipulationConditions, check_manipulation_conditions)
    rospy.loginfo('Started check manipulation conditions service.')
    rospy.spin()
