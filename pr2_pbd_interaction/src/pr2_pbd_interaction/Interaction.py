'''Main interaction loop'''

import roslib

roslib.load_manifest('pr2_pbd_interaction')
from pr2_pbd_interaction.condition_types.GripperCondition import GripperCondition
from pr2_pbd_interaction.step_types.ArmStep import ArmStep


# Generic libraries
import rospy
import time
from visualization_msgs.msg import MarkerArray

# Local stuff
from pr2_pbd_interaction.World import World
from pr2_pbd_interaction.Robot import Robot
from pr2_pbd_interaction.RobotSpeech import RobotSpeech
from pr2_pbd_interaction.Session import Session
from pr2_pbd_interaction.Response import Response
from pr2_pbd_interaction.Arm import ArmMode
from pr2_pbd_interaction.msg import ArmState, GripperState, ArmStepType
from pr2_pbd_interaction.msg import ArmTarget, Object
from pr2_pbd_interaction.msg import GripperAction, ArmTrajectory
from pr2_pbd_interaction.msg import ExecutionStatus, GuiCommand
from pr2_pbd_speech_recognition.msg import Command
from pr2_social_gaze.msg import GazeGoal


class Interaction:
    '''Finite state machine for the human interaction'''

    _is_programming = True
    _is_recording_motion = False
    _arm_trajectory = None
    _trajectory_start_time = None

    def __init__(self):
        self.world = World.get_world()
        self.robot = Robot.get_robot()
        self.session = Session()
        self._viz_publisher = rospy.Publisher('visualization_marker_array',
                                              MarkerArray)

        rospy.Subscriber('recognized_command', Command, self.speech_command_cb)
        rospy.Subscriber('gui_command', GuiCommand, self.gui_command_cb)

        self._undo_function = None

        self.responses = {
            Command.TEST_MICROPHONE: Response(Interaction.empty_response,
                                              [RobotSpeech.TEST_RESPONSE, GazeGoal.NOD]),
            Command.RELAX_RIGHT_ARM: Response(self.relax_arm, 0),
            Command.RELAX_LEFT_ARM: Response(self.relax_arm, 1),
            Command.OPEN_RIGHT_HAND: Response(self.open_hand, 0),
            Command.OPEN_LEFT_HAND: Response(self.open_hand, 1),
            Command.CLOSE_RIGHT_HAND: Response(self.close_hand, 0),
            Command.CLOSE_LEFT_HAND: Response(self.close_hand, 1),
            Command.STOP_EXECUTION: Response(self.stop_execution, None),
            Command.DELETE_ALL_STEPS: Response(self.delete_all_steps, None),
            Command.DELETE_LAST_STEP: Response(self.delete_last_step, None),
            Command.FREEZE_RIGHT_ARM: Response(self.freeze_arm, 0),
            Command.FREEZE_LEFT_ARM: Response(self.freeze_arm, 1),
            Command.CREATE_NEW_ACTION: Response(self.create_action, None),
            Command.EXECUTE_ACTION: Response(self.execute_action, None),
            Command.NEXT_ACTION: Response(self.next_action, None),
            Command.PREV_ACTION: Response(self.previous_action, None),
            Command.SAVE_ACTION: Response(self.save_action, None),
            Command.SAVE_POSE: Response(self.save_arm_step, None),
            Command.RECORD_OBJECT_POSE: Response(
                self.record_object_pose, None),
            Command.LOOK_DOWN: Response(self.look_down, None),
            Command.LOOK_FORWARD: Response(self.look_forward, None)
        }

        rospy.loginfo('Interaction initialized.')

    def open_hand(self, arm_index):
        '''Opens gripper on the indicated side'''
        initial_condition = GripperCondition(self.robot.get_gripper_position(0),
                                             self.robot.get_gripper_position(1))
        if self.robot.set_gripper_state(arm_index, GripperState.OPEN):
            speech_response = Response.open_responses[arm_index]
            if (Interaction._is_programming and self.session.n_actions() > 0):
                self.save_gripper_step(arm_index, GripperState.OPEN, initial_condition)
                speech_response = (speech_response + ' ' +
                                   RobotSpeech.STEP_RECORDED)
            return [speech_response, Response.glance_actions[arm_index]]
        else:
            return [Response.already_open_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def close_hand(self, arm_index):
        '''Closes gripper on the indicated side'''
        initial_condition = GripperCondition(self.robot.get_gripper_position(0),
                                             self.robot.get_gripper_position(1))
        if Robot.set_gripper_state(arm_index, GripperState.CLOSED):
            speech_response = Response.close_responses[arm_index]
            if (Interaction._is_programming and self.session.n_actions() > 0):
                self.save_gripper_step(arm_index, GripperState.CLOSED, initial_condition)
                speech_response = (speech_response + ' ' +
                                   RobotSpeech.STEP_RECORDED)
            return [speech_response, Response.glance_actions[arm_index]]
        else:
            return [Response.already_closed_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def relax_arm(self, arm_index):
        '''Relaxes arm on the indicated side'''
        if self.robot.set_arm_mode(arm_index, ArmMode.RELEASE):
            return [Response.release_responses[arm_index],
                    Response.glance_actions[arm_index]]
        else:
            return [Response.already_released_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def freeze_arm(self, arm_index):
        '''Stiffens arm on the indicated side'''
        if self.robot.set_arm_mode(arm_index, ArmMode.HOLD):
            return [Response.hold_responses[arm_index],
                    Response.glance_actions[arm_index]]
        else:
            return [Response.already_holding_responses[arm_index],
                    Response.glance_actions[arm_index]]

    def edit_action(self, dummy=None):
        '''Goes back to edit mode'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                return [RobotSpeech.ALREADY_EDITING, GazeGoal.SHAKE]
            else:
                Interaction._is_programming = True
                return [RobotSpeech.SWITCH_TO_EDIT_MODE, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def save_action(self, dummy=None):
        """Saves action to disk"""
        self.session.save_current_action()
        action_name = self.session.get_action_name(self.session.current_action_index)
        if action_name is None:
            action_name = str(self.session.current_action_index)
        return [RobotSpeech.ACTION_SAVED + ' ' +
                action_name, GazeGoal.NOD]
    #
    # def save_action(self, dummy=None):
    #     '''Goes out of edit mode'''
    #     self.session.save_current_action()
    #     Interaction._is_programming = False
    #     action_name = self.session.get_action_name(self.session.current_action_index)
    #     if action_name is None:
    #         action_name = str(self.session.current_action_index)
    #     return [RobotSpeech.ACTION_SAVED + ' ' +
    #             action_name, GazeGoal.NOD]

    def create_action(self, dummy=None):
        '''Creates a new empty action'''
        self.world.clear_all_objects()
        self.session.new_action()
        Interaction._is_programming = True
        action_name = self.session.get_action_name(self.session.current_action_index)
        if action_name is None:
            action_name = str(self.session.current_action_index)
        return [RobotSpeech.SKILL_CREATED + ' ' +
                action_name, GazeGoal.NOD]

    def next_action(self, dummy=None):
        """Switches to next action"""
        if (self.session.n_actions() > 0):
            action_name = self.session.get_action_name(self.session.current_action_index)
            if action_name is None:
                action_name = str(self.session.current_action_index)
            if self.session.next_action():
                self.world.clear_all_objects()
                return [RobotSpeech.SWITCH_SKILL + ' ' +
                        action_name, GazeGoal.NOD]
            else:
                return [RobotSpeech.ERROR_NEXT_SKILL + ' ' +
                        action_name, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def previous_action(self, dummy=None):
        """Switches to previous action"""
        if (self.session.n_actions() > 0):
            action_name = self.session.get_action_name(self.session.current_action_index)
            if action_name is None:
                action_name = str(self.session.current_action_index)
            if self.session.previous_action():
                self.world.clear_all_objects()
                return [RobotSpeech.SWITCH_SKILL + ' ' +
                        action_name, GazeGoal.NOD]
            else:
                return [RobotSpeech.ERROR_PREV_SKILL + ' ' +
                        action_name, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def delete_last_step(self, dummy=None):
        """Deletes last arm step of the current action"""
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                if self.session.n_steps() > 0:
                    self.session.delete_last_step()
                    return [RobotSpeech.LAST_STEP_DELETED, GazeGoal.NOD]
                else:
                    return [RobotSpeech.SKILL_EMPTY, GazeGoal.SHAKE]
            else:
                action_name = self.session.get_action_name(self.session.current_action_index)
                if action_name is None:
                    action_name = str(self.session.current_action_index)
                return ['Action ' + action_name +
                        RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def delete_all_steps(self, dummy=None):
        '''Deletes all steps in the current action'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                if self.session.n_steps() > 0:
                    self.world.clear_all_objects()
                    self.session.clear_current_action()
                    return [RobotSpeech.SKILL_CLEARED, GazeGoal.NOD]
                else:
                    return [RobotSpeech.SKILL_EMPTY, None]
            else:
                action_name = self.session.get_action_name(self.session.current_action_index)
                if action_name is None:
                    action_name = str(self.session.current_action_index)
                return ['Action ' + action_name +
                        RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def stop_execution(self, dummy=None):
        """Stops ongoing execution"""
        if self.robot.is_executing():
            self.robot.stop_execution()
            return [RobotSpeech.STOPPING_EXECUTION, GazeGoal.NOD]
        else:
            return [RobotSpeech.ERROR_NO_EXECUTION, GazeGoal.SHAKE]

    def save_gripper_step(self, arm_index, gripper_state, initial_condition):
        '''Saves an action step that involves a gripper state change'''
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                states = self._get_arm_states()
                step = ArmStep()
                step.type = ArmStepType.ARM_TARGET
                rarm_state = None
                larm_state = None
                if rospy.has_param('use_right_arm'):
                    useright = rospy.get_param('use_right_arm')
                else:
                    useright = True
                if rospy.has_param('use_left_arm'):
                    useleft = rospy.get_param('use_left_arm')
                else:
                    useleft = True
                if useright:
                    rarm_state = states[0]
                if useleft:
                    larm_state = states[1]
                
                step.armTarget = ArmTarget(rarm_state, larm_state, 0.2, 0.2, useright, useleft)
                actions = [self.robot.get_gripper_state(0),
                           self.robot.get_gripper_state(1)]
                actions[arm_index] = gripper_state
                step.gripperAction = GripperAction(actions[0], actions[1])
                prev_step = self.session.get_last_step()
                step.set_gripper_condition(initial_condition if prev_step is None else prev_step.postCond)
                step.postCond = GripperCondition(self.robot.get_gripper_position(0),
                                                 self.robot.get_gripper_position(1))
                self.session.add_step_to_action(step)

    def save_arm_step(self, dummy=None):
        """Saves current arm state as an action step"""
        if (self.session.n_actions() > 0):
            if (Interaction._is_programming):
                states = self._get_arm_states()
                step = ArmStep()
                step.type = ArmStepType.ARM_TARGET
                rarm_state = None
                larm_state = None
                if rospy.has_param('use_right_arm'):
                    useright = rospy.get_param('use_right_arm')
                else:
                    useright = True
                if rospy.has_param('use_left_arm'):
                    useleft = rospy.get_param('use_left_arm')
                else:
                    useleft = True
                if useright:
                    rarm_state = states[0]
                if useleft:
                    larm_state = states[1]
                rospy.loginfo("Use right:")
                rospy.loginfo(useright)
                rospy.loginfo("Use left:")
                rospy.loginfo(useleft)
                step.armTarget = ArmTarget(rarm_state, larm_state, 0.2, 0.2, useright, useleft)
                rospy.loginfo("Left arm state: ")
                rospy.loginfo(larm_state)
                step.gripperAction = GripperAction(
                    self.robot.get_gripper_state(0),
                    self.robot.get_gripper_state(1))
                prev_step = self.session.get_last_step()
                if prev_step is None:
                    step.set_gripper_condition(GripperCondition(self.robot.get_gripper_position(0),
                                                                self.robot.get_gripper_position(1)))
                else:
                    step.set_gripper_condition(prev_step.postCond)
                step.postCond = GripperCondition(self.robot.get_gripper_position(0),
                                                 self.robot.get_gripper_position(1))
                self.session.add_step_to_action(step)
                return [RobotSpeech.STEP_RECORDED, GazeGoal.NOD]
            else:
                action_name = self.session.get_action_name(self.session.current_action_index)
                if action_name is None:
                    action_name = str(self.session.current_action_index)
                return ['Action ' + action_name +
                        RobotSpeech.ERROR_NOT_IN_EDIT, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def look_down(self, dummy=None):
        '''Makes the head look down and saves head pose'''
        Response.perform_gaze_action(GazeGoal.LOOK_DOWN)
        speech_response = RobotSpeech.LOOKING_DOWN
        return [speech_response, None]

    def look_forward(self, dummy=None):
        '''Makes the head look forward and saves head pose'''
        Response.perform_gaze_action(GazeGoal.LOOK_FORWARD)
        speech_response = RobotSpeech.LOOKING_FORWARD
        return [speech_response, None]

    def _get_arm_states(self):
        '''Returns the current arms states in the right format'''
        abs_ee_poses = [Robot.get_ee_state(0),
                        Robot.get_ee_state(1)]
        joint_poses = [Robot.get_joint_state(0),
                       Robot.get_joint_state(1)]

        rel_ee_poses = [None, None]
        states = [None, None]

        nearest_objects = [None, None]

        for arm_index in [0, 1]:
            if arm_index == 0:
                if rospy.has_param('right_action_tf_frame'):
                    desired_tf_frame = rospy.get_param('right_action_tf_frame')
                else:
                    desired_tf_frame = "default"
            else:
                if rospy.has_param('left_action_tf_frame'):
                    desired_tf_frame = rospy.get_param('left_action_tf_frame')
                else:
                    desired_tf_frame = "default"


            if desired_tf_frame == "default":
                if (not World.get_world().has_objects()):
                    # Absolute
                    states[arm_index] = ArmState(ArmState.ROBOT_BASE,
                                                 abs_ee_poses[arm_index], joint_poses[arm_index], Object(), "base_link")
                else:
                    nearest_obj = self.world.get_nearest_object(
                        abs_ee_poses[arm_index])
                    nearest_objects[arm_index] = nearest_obj
                    if (nearest_obj is None):
                        states[arm_index] = ArmState(ArmState.ROBOT_BASE,
                                                     abs_ee_poses[arm_index],
                                                     joint_poses[arm_index], Object(), "base_link")
                    else:
                        # Relative
                        #rospy.loginfo('Pose is relative for arm ' + str(arm_index))
                        rel_ee_poses[arm_index] = World.get_world().transform(
                            abs_ee_poses[arm_index],
                            'base_link', nearest_obj.name)
                        states[arm_index] = ArmState(ArmState.OBJECT,
                                                     rel_ee_poses[arm_index],
                                                     joint_poses[arm_index], nearest_obj, nearest_obj.name)
            elif desired_tf_frame == "base_link":
                states[arm_index] = ArmState(ArmState.ROBOT_BASE,
                                                 abs_ee_poses[arm_index], joint_poses[arm_index], Object(), "base_link")
            else:
                rel_ee_poses[arm_index] = World.get_world().transform(
                    abs_ee_poses[arm_index],
                    'base_link', desired_tf_frame)
                rospy.loginfo("Desired tf: " + desired_tf_frame)
                obj = Object()
                obj.name = ""

                states[arm_index] = ArmState(ArmState.OTHER,
                                             rel_ee_poses[arm_index],
                                             joint_poses[arm_index], obj, desired_tf_frame)

        self.world.process_nearest_objects(nearest_objects)
        return states

    def execute_action(self, dummy=None):
        '''Starts the execution of the current action'''
        if (self.session.n_actions() > 0):
            action_name = self.session.get_action_name(self.session.current_action_index)
            if action_name is None:
                action_name = str(self.session.current_action_index)
            if (self.session.n_steps() > 0):
                self.session.save_current_action()
                action = self.session.get_current_action()
                self.robot.start_execution(action)
                return [RobotSpeech.START_EXECUTION + ' ' +
                        action_name, None]
            else:
                return [RobotSpeech.EXECUTION_ERROR_NOPOSES + ' ' +
                        action_name, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def speech_command_cb(self, command):
        '''Callback for when a speech command is receieved'''
        if command.command in self.responses.keys():
            rospy.loginfo('\033[32m Calling response for command ' +
                          command.command + '\033[0m')
            response = self.responses[command.command]

            if (not self.robot.is_executing()):
                if (self._undo_function != None):
                    response.respond()
                    self._undo_function = None
                else:
                    response.respond()
            else:
                if command.command == Command.STOP_EXECUTION:
                    response.respond()
                else:
                    rospy.logwarn('Ignoring speech command during execution: '
                                  + command.command)
        else:
            switch_command = Command.SWITCH_TO_ACTION + " "
            name_command = Command.CHANGE_ACTION_NAME + " "
            if (switch_command in command.command):
                action_name = command.command[
                              len(switch_command):len(command.command)]
                if (self.session.n_actions() > 0):
                    self.world.clear_all_objects()
                    self.session.switch_to_action_by_name(action_name)
                    response = Response(Interaction.empty_response,
                                        [RobotSpeech.SWITCH_SKILL + action_name,
                                         GazeGoal.NOD])
                    rospy.loginfo('Switched to action ' + action_name)
                else:
                    response = Response(Interaction.empty_response,
                                        [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE])
                response.respond()
            elif (name_command in command.command):
                action_name = command.command[
                              len(name_command):len(command.command)]

                if (len(action_name) > 0):
                    self.session.name_action(action_name)
                    Response(Interaction.empty_response,
                             [RobotSpeech.RENAMED_ACTION + action_name,
                              GazeGoal.NOD]).respond()
                    rospy.loginfo('New name for action: ' + action_name)
                else:
                    rospy.logwarn("New action name has zero length")
                    Response(Interaction.empty_response,
                             [RobotSpeech.ERROR_GENERAL, GazeGoal.SHAKE]).respond()
            else:
                rospy.logwarn('\033[32m This command (' + command.command
                              + ') is unknown. \033[0m')

    def gui_command_cb(self, command):
        '''Callback for when a GUI command is received'''

        if not self.robot.is_executing():
            if (self.session.n_actions() > 0):
                if (command.command == GuiCommand.SWITCH_TO_ACTION):
                    action_no = command.param
                    self.world.clear_all_objects()
                    self.session.switch_to_action(action_no)
                    action_name = self.session.get_action_name(action_no)
                    if action_name is None:
                        action_name = str(action_no)
                    response = Response(Interaction.empty_response,
                                        [RobotSpeech.SWITCH_SKILL + action_name,
                                         GazeGoal.NOD])
                    response.respond()
                elif (command.command == GuiCommand.DELETE_STEP):
                    step_no = command.param
                    self.session.delete_step(step_no)
                    rospy.loginfo('Deleted action step ' + str(step_no))
                elif (command.command == GuiCommand.SELECT_STEP):
                    step_no = command.param
                    self.session.select_step(step_no)
                    rospy.loginfo('Selected arm step ' + str(step_no))
                elif (command.command == GuiCommand.DESELECT_STEP):
                    step_no = command.param
                    self.session.deselect_step(step_no)
                    rospy.loginfo('Deselected arm step ' + str(step_no))
                elif (command.command == GuiCommand.SET_IGNORE_CONDITIONS):
                    step_no = command.param
                    self.session.set_ignore_conditions(step_no, True)
                    rospy.loginfo('Will ignore conditions for step ' + str(step_no))
                elif (command.command == GuiCommand.SET_NO_IGNORE_CONDITIONS):
                    step_no = command.param
                    self.session.set_ignore_conditions(step_no, False)
                    rospy.loginfo('Will not ignore conditions for step ' + str(step_no))
                elif (command.command == GuiCommand.SET_STRATEGY):
                    condition_index = command.param
                    strategy_index = command.param_list[0]
                    self.session.set_current_step_condition_strategy(condition_index, strategy_index)
                    rospy.loginfo('Changed condition failure strategy for the current step')
                elif (command.command == GuiCommand.SET_OBJECT_SIMILARITY_THRESHOLD):
                    step_no = command.param
                    threshold = command.param_float
                    self.session.set_object_similarity_threshold(step_no, threshold)
                    rospy.loginfo('Changed object similarity threshold for step ' + str(step_no))
                else:
                    rospy.logwarn('\033[32m This command (' + command.command
                                  + ') is unknown. \033[0m')
            else:
                response = Response(Interaction.empty_response,
                                    [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE])
                response.respond()
        else:
            rospy.logwarn('Ignoring GUI command during execution: ' +
                          command.command)

    def update(self):
        '''General update for the main loop'''
        self.robot.update()

        if (self.robot.status != ExecutionStatus.NOT_EXECUTING):
            if (self.robot.status != ExecutionStatus.EXECUTING):
                self._end_execution()

        is_world_changed = self.world.update()
        if (self.session.n_actions() > 0):
            action = self.session.get_current_action()
            action.update_viz()
            r_target = self.session.get_requested_arm_targets(0)
            if r_target is not None:
                self.robot.start_move_arm_to_pose(r_target, 0)
                self.session.reset_arm_targets(0)
            l_target = self.session.get_requested_arm_targets(1)
            if l_target is not None:
                self.robot.start_move_arm_to_pose(l_target, 1)
                self.session.reset_arm_targets(1)

            self.session.delete_requested_steps()

            arm_states = self._get_arm_states()
            self.session.change_requested_steps(arm_states[0], arm_states[1])

            if (is_world_changed):
                rospy.loginfo('The world has changed.')
                self.session.get_current_action().update_objects()

        time.sleep(0.1)

    def _end_execution(self):
        """Responses for when the action execution ends"""
        if (self.robot.status == ExecutionStatus.SUCCEEDED):
            Response.say(RobotSpeech.EXECUTION_ENDED)
            Response.perform_gaze_action(GazeGoal.NOD)
        elif (self.robot.status == ExecutionStatus.PREEMPTED):
            Response.say(RobotSpeech.EXECUTION_PREEMPTED)
            Response.perform_gaze_action(GazeGoal.SHAKE)
        elif (self.robot.status == ExecutionStatus.CONDITION_FAILED):
            Response.say(RobotSpeech.EXECUTION_ERROR_CONDITION_FAIL)
            Response.perform_gaze_action(GazeGoal.SHAKE)
        elif (self.robot.status == ExecutionStatus.OBSTRUCTED):
            Response.say(RobotSpeech.EXECUTION_ERROR_OBSTRUCTED)
            Response.perform_gaze_action(GazeGoal.SHAKE)
        elif (self.robot.status == ExecutionStatus.OBJECT_DETECTION_FAILED):
            Response.say(RobotSpeech.OBJECT_NOT_DETECTED)
            Response.perform_gaze_action(GazeGoal.SHAKE)
        elif (self.robot.status == ExecutionStatus.OTHER_ERROR):
            Response.say(RobotSpeech.EXECUTION_ERROR_OTHER)
            Response.perform_gaze_action(GazeGoal.SHAKE)
        else:
            Response.say(RobotSpeech.EXECUTION_ERROR_NOIK)
            Response.perform_gaze_action(GazeGoal.SHAKE)

        self.robot.status = ExecutionStatus.NOT_EXECUTING

    def record_object_pose(self, dummy=None):
        """Makes the robot look for a table and objects"""
        if self.session.n_actions() > 0:
            if (Interaction._is_programming):
                if self.world.update_object_pose():
                    self.session.selected_step = -1
                    self.session.get_current_action().selected_step_id = -1
                    self.session.remember_head_position()
                    return [RobotSpeech.START_STATE_RECORDED, None]
                else:
                    return [RobotSpeech.OBJECT_NOT_DETECTED, GazeGoal.SHAKE]
        else:
            return [RobotSpeech.ERROR_NO_SKILLS, GazeGoal.SHAKE]

    def save_experiment_state(self):
        """Saves session state"""
        self.session.save_current_action()

    @staticmethod
    def empty_response(responses):
        """Default response to speech commands"""
        return responses
