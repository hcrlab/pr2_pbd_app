'''Everything related to an experiment session'''
import os

import rospy

from pr2_pbd_interaction.msg import ExperimentState
from pr2_pbd_interaction.srv import GetExperimentState
from pr2_pbd_interaction.srv import GetExperimentStateResponse
from step_types.ArmStep import ArmStep
from pr2_pbd_interaction.step_types import ManipulationStep


class Session:
    '''This class holds and maintains experimental data: list of Actions'''

    session = None

    def __init__(self):
        Session.session = self

        self.selected_step = -1
        self._selected_arm_step = -1

        action_directory = rospy.get_param('/pr2_pbd_interaction/actionsRoot')
        if not os.path.exists(action_directory):
            os.makedirs(action_directory)
        self.manipulation_actions = ManipulationStep.get_saved_actions()
        self.current_action_index = 0 if len(self.manipulation_actions) > 0 else None
        if self.current_action_index is not None:
            self.selected_step = self.manipulation_actions[self.current_action_index].get_selected_step_id()
            self.manipulation_actions[self.current_action_index].initialize_viz()
        rospy.loginfo("Current action visualization initialized.")

        self._state_publisher = rospy.Publisher('experiment_state',
                                                ExperimentState)
        rospy.Service('get_experiment_state', GetExperimentState,
                      self.get_experiment_state_cb)

        self._update_experiment_state()

        rospy.loginfo("Session initialized.")


    @staticmethod
    def get_session():
        if Session.session == None:
            Session.session = Session()
        return Session.session

    def selected_arm_step_cb(self, selected_step):
        '''Updates the selected step when interactive
        markers are clicked on'''
        self._selected_arm_step = selected_step
        self._update_experiment_state()

    def get_experiment_state_cb(self, dummy):
        ''' Response to the experiment state service call'''
        return GetExperimentStateResponse(self._get_experiment_state())

    def _update_experiment_state(self):
        ''' Publishes a message with the latest state'''
        state = self._get_experiment_state()
        self._state_publisher.publish(state)

    def _get_experiment_state(self):
        ''' Creates a message with the latest state'''
        return ExperimentState(
            (self.manipulation_actions[self.current_action_index].to_string()
             if self.current_action_index is not None
             else ""),
            map(lambda act: act.name, self.manipulation_actions),
            map(lambda act: act.id, self.manipulation_actions),
            -1 if self.current_action_index is None else self.current_action_index,
            -1 if self.selected_step is None else self.selected_step)

    def _get_ref_frames(self, arm_index):
        # TODO
        ''' Returns the reference frames for the steps of the
        current action in array format'''
        ref_frames = []
        for i in range(self.n_steps()):
            action = self.manipulation_actions[self.current_action_index]
            ref_frame = action.get_step_ref_frame(arm_index, i)
            ref_frames.append(ref_frame)
        return ref_frames

    def _get_gripper_states(self, arm_index):
        # TODO
        ''' Returns the gripper states for current action
        in array format'''
        gripper_states = []
        for i in range(self.n_steps()):
            action = self.manipulation_actions[self.current_action_index]
            gripper_state = action.get_step_gripper_state(arm_index, i)
            gripper_states.append(gripper_state)
        return gripper_states

    def select_step(self, step_id):
        self.manipulation_actions[self.current_action_index].select_step(step_id)
        self.selected_step = step_id

    def deselect_step(self, step_id):
        ''' Removes the 6D controls from the interactive marker
        when the indicated action step is deselected'''
        self.manipulation_actions[self.current_action_index].deselect_step(step_id)
        self.selected_step = step_id

    def save_session_state(self, is_save_actions=True):
        if is_save_actions:
            for i in range(self.n_actions()):
                self.manipulation_actions[i].save()

    def new_action(self):
        '''Creates new action'''
        if self.n_actions() > 0:
            self.manipulation_actions[self.current_action_index].reset_viz()
        self.selected_step = -1
        self._selected_arm_step = -1
        newAct = ManipulationStep(name="Unnamed " + str(len(self.manipulation_actions)))
        newAct.save()
        self.manipulation_actions.append(newAct)
        self.current_action_index = len(self.manipulation_actions) - 1
        self._update_experiment_state()

    def n_actions(self):
        """Returns the number of actions programmed so far"""
        return len(self.manipulation_actions)

    def get_current_action(self):
        """Returns the current action"""
        return self.manipulation_actions[self.current_action_index]

    def get_current_step(self):
        """Returns the current action step"""
        return self.manipulation_actions[self.current_action_index].get_selected_step()

    def get_last_step(self):
        """Returns the current action step"""
        return self.manipulation_actions[self.current_action_index].get_last_step()

    def clear_current_action(self):
        '''Removes all steps in the current action'''
        if (self.n_actions() > 0):
            self.manipulation_actions[self.current_action_index].clear()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def save_current_action(self):
        '''Save current action onto hard drive'''
        if (self.n_actions() > 0):
            self.manipulation_actions[self.current_action_index].save()
            self.save_session_state(is_save_actions=False)
        else:
            rospy.logwarn('No skills created yet.')

    def add_step_to_action(self, step):
        '''Add a new step to the current action'''
        if (self.n_actions() > 0):
            self.manipulation_actions[self.current_action_index].add_step(step)
            self.selected_step = self.manipulation_actions[self.current_action_index].get_selected_step_id()
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def set_ignore_conditions(self, step_id, ignore_conditions):
        """ Controls if the specified step should ignore conditions.
        """
        if (self.n_actions() > 0):
            self.manipulation_actions[self.current_action_index].set_ignore_conditions(step_id, ignore_conditions)
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def set_ignore_arm_step_conditions(self, step_id, ignore_conditions):
        """ Controls if the specified arm step should ignore conditions.
        """
        if (self.n_actions() > 0):
            self.manipulation_actions[self.current_action_index].set_ignore_arm_step_conditions(step_id, ignore_conditions)
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def set_arm_step_condition_strategy(self, step_id, condition_index, strategy_index):
        """ Sets the failure strategy for the specified condition of the specified arm step.
        """
        if (self.n_actions() > 0):
            self.manipulation_actions[self.current_action_index].set_arm_step_condition_strategy(step_id, condition_index, strategy_index)
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def set_current_step_condition_strategy(self, condition_index, strategy_index):
        """ Sets the failure strategy for the specified condition of the current step.
        """
        if (self.n_actions() > 0):
            self.manipulation_actions[self.current_action_index].set_condition_strategy(condition_index, strategy_index)
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def set_object_similarity_threshold(self, step_id, threshold):
        if (self.n_actions() > 0):
            self.manipulation_actions[self.current_action_index].set_object_similarity_threshold(step_id, threshold)
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def delete_step(self, step_id):
        """ Removes the specified step of the action.
        """
        if (self.n_actions() > 0):
            self.manipulation_actions[self.current_action_index].delete_step(step_id)
            if self.selected_step == step_id:
                self.selected_step = -1
        else:
            rospy.logwarn('No skills created yet.')
        self._update_experiment_state()

    def delete_last_step(self):
        """Removes the last step of the action"""
        self.delete_step(self.n_steps()-1)

    def get_action_name(self, action_number):
        if self.n_actions() > 0 and 0 <= action_number < self.n_actions():
            action = self.manipulation_actions[action_number]
            if action.name is not None:
                return action.name
        return None

    def switch_to_action(self, action_number):
        """Switches to indicated action"""
        if (self.n_actions() > 0):
            if (action_number < self.n_actions() and action_number >= 0):
                self.save_current_action()
                self.get_current_action().reset_viz()
                self.current_action_index = action_number
                self.get_current_action().initialize_viz()
                success = True
            else:
                rospy.logwarn('Cannot switch to action '
                              + str(action_number))
                success = False
        else:
            rospy.logwarn('No skills created yet.')
            success = False
        self._update_experiment_state()
        return success

    def switch_to_action_by_name(self, action_name):
        return self.switch_to_action(next((i for i, act in enumerate(self.manipulation_actions)
                                           if act.name == action_name), -1))

    def name_action(self, new_name):
        if len(self.manipulation_actions) > 0:
            self.manipulation_actions[self.current_action_index].name = new_name
            self._update_experiment_state()

    def next_action(self):
        """Switches to next action"""
        return self.switch_to_action(self.current_action_index+1)

    def previous_action(self):
        """Switches to previous action"""
        return self.switch_to_action(self.current_action_index-1)

    def n_steps(self):
        """Returns the number of steps in the current action"""
        if (self.n_actions() > 0):
            return self.manipulation_actions[self.current_action_index].n_steps()
        else:
            rospy.logwarn('No skills created yet.')
            return 0

    def get_requested_arm_targets(self, arm_index):
        if self.n_actions() > 0:
            self.manipulation_actions[self.current_action_index].get_requested_targets(arm_index)
        return None

    def reset_arm_targets(self, arm_index):
        if self.n_actions() > 0:
            self.manipulation_actions[self.current_action_index].reset_targets(arm_index)
        return None

    def delete_requested_steps(self):
        if self.n_actions() > 0:
            self.manipulation_actions[self.current_action_index].delete_requested_steps()

    def change_requested_steps(self, r_state, l_state):
        if self.n_actions() > 0:
            self.manipulation_actions[self.current_action_index].change_requested_steps(r_state, l_state)

    def remember_head_position(self):
        if self.n_actions() > 0:
            self.manipulation_actions[self.current_action_index].remember_head_position()


