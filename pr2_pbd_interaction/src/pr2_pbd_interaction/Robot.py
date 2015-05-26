#!/usr/bin/env python
from actionlib_msgs.msg import GoalStatus

import roslib
import tf
from pr2_pbd_interaction.Response import Response

from pr2_pbd_interaction.msg import StepExecutionStatus, ArmStepType
from pr2_pbd_interaction.msg import ExecutionResult


roslib.load_manifest('pr2_pbd_interaction')
import rospy
import tf

import time
import threading
from pr2_pbd_interaction.msg import ArmState, GripperState
from pr2_pbd_interaction.msg import Side
from pr2_pbd_interaction.msg import ExecutionStatus
from geometry_msgs.msg import Pose, Point, PoseStamped, Quaternion, Twist, Vector3, PointStamped
from pr2_pbd_interaction.World import World
from Arm import Arm, ArmMode
from pr2_pbd_interaction.msg import Object


class Robot:
    '''
    Controller for the robot: moves the arms and base.
    '''
    arms = []
    robot = None

    def __init__(self):
        r_arm = Arm(Side.RIGHT)
        l_arm = Arm(Side.LEFT)
        Robot.arms = [r_arm, l_arm]
        self.attended_arm = -1
        self.action = None
        self.preempt = False
        self.is_continue_execution = False
        self.z_offset = 0

        rospy.loginfo('Arms have been initialized.')

        self._result_publisher = rospy.Publisher('execution_result',
                                                ExecutionResult)
        Response.init_clients()

        Robot.arms[0].set_mode(ArmMode.HOLD)
        Robot.arms[1].set_mode(ArmMode.HOLD)
        Robot.arms[0].check_gripper_state()
        Robot.arms[1].check_gripper_state()
        # Robot.arms[Side.RIGHT].close_gripper()
        # Robot.arms[Side.LEFT].close_gripper()
        self.status = ExecutionStatus.NOT_EXECUTING
        rospy.loginfo('Robot initialized.')


    @staticmethod
    def get_robot():
        if Robot.robot is None:
            Robot.robot = Robot()
        return Robot.robot

    @staticmethod
    def set_arm_mode(arm_index, mode):
        '''Set arm to stiff or relaxed'''
        if (mode == Robot.arms[arm_index].arm_mode):
            # Already in that mode
            return False
        else:
            Robot.arms[arm_index].set_mode(mode)
            return True

    @staticmethod
    def set_gripper_state(arm_index, gripper_state):
        '''Set gripper to open or closed'''
        if (gripper_state == Robot.get_gripper_state(arm_index)):
            # Already in that mode
            return False
        else:
            if (gripper_state == GripperState.OPEN):
                Robot.arms[arm_index].open_gripper()
            else:
                Robot.arms[arm_index].close_gripper()
        return True

    def is_executing(self):
        '''Whether or not there is an ongoing execution'''
        return (self.status == ExecutionStatus.EXECUTING)

    def start_execution(self, action, z_offset=0):
        ''' Starts execution of an action'''
        # This will take long, create a thread
        self.preempt = False
        self.z_offset = z_offset
        self.action = action
        thread = threading.Thread(group=None, target=self.execute_action,
                                  name='skill_execution_thread')
        thread.start()

    def execute_action(self):
        self.status = ExecutionStatus.EXECUTING
        rospy.loginfo("Starting execution of action " + self.action.get_name())
        try:
            self.action.execute()
        except Exception as e:
            rospy.logerr("Execution of an action failed: " + str(e))
            if self.status == ExecutionStatus.EXECUTING:
                self.status = ExecutionStatus.OTHER_ERROR
        else:
            action_status = self.action.execution_status
            if action_status == StepExecutionStatus.FAILED:
                self.status = ExecutionStatus.OTHER_ERROR
            else:
                self.status = ExecutionStatus.SUCCEEDED
            rospy.loginfo('Execution finished')
        self._result_publisher.publish(ExecutionResult(ExecutionStatus(self.status), self.action.get_error_msg()))

    def continue_execution(self):
        self.is_continue_execution = True

    def stop_execution(self):
        '''Preempts an ongoing execution'''
        self.preempt = True

    def solve_ik_for_manipulation_step(self, manipulation_step):
        '''Computes joint positions for all end-effector poses
        in an manipulation_step'''

        # Go over steps of the manipulation_step
        for i in range(manipulation_step.n_steps()):
            # For each step check step type
            # If arm target action
            if (manipulation_step.arm_steps[i].type == ArmStepType.ARM_TARGET):
                # Find frames that are relative and convert to absolute

                r_arm, has_solution_r = Robot.solve_ik_for_arm(0,
                                                               manipulation_step.arm_steps[i].armTarget.rArm,
                                                               self.z_offset)
                l_arm, has_solution_l = Robot.solve_ik_for_arm(1,
                                                               manipulation_step.arm_steps[i].armTarget.lArm,
                                                               self.z_offset)

                manipulation_step.arm_steps[i].armTarget.rArm = r_arm
                manipulation_step.arm_steps[i].armTarget.lArm = l_arm
                if (not has_solution_r) or (not has_solution_l):
                    return False

            if (manipulation_step.arm_steps[i].type == ArmStepType.ARM_TRAJECTORY):
                n_frames = len(manipulation_step.arm_steps[i].armTrajectory.timing)
                for j in range(n_frames):
                    r_arm, has_solution_r = Robot.solve_ik_for_arm(0,
                                                                   manipulation_step.arm_steps[
                                                                       i].armTrajectory.r_arm[j],
                                                                   self.z_offset)
                    l_arm, has_solution_l = Robot.solve_ik_for_arm(1,
                                                                   manipulation_step.arm_steps[
                                                                       i].armTrajectory.l_arm[j],
                                                                   self.z_offset)
                    manipulation_step.arm_steps[i].armTrajectory.r_arm[j] = r_arm
                    manipulation_step.arm_steps[i].armTrajectory.l_arm[j] = l_arm
                    if (not has_solution_r) or (not has_solution_l):
                        return False
        return True


    def has_ik_solutions_for_arm_steps(self, arm_steps):
        '''Computes joint positions for all end-effector poses
        in an sequence of arm_steps - without changing them'''

        # Go over steps
        for i in range(len(arm_steps)):
            # For each step check step type
            # If arm target action
            if (arm_steps[i].type == ArmStepType.ARM_TARGET):
                r_arm, has_solution_r = Robot.solve_ik_for_arm(0,
                                                               arm_steps[i].armTarget.rArm,
                                                               self.z_offset)
                l_arm, has_solution_l = Robot.solve_ik_for_arm(1,
                                                               arm_steps[i].armTarget.lArm,
                                                               self.z_offset)
                if (not has_solution_r) or (not has_solution_l):
                    return False

            if (arm_steps[i].type == ArmStepType.ARM_TRAJECTORY):
                n_frames = len(arm_steps[i].armTrajectory.timing)
                for j in range(n_frames):
                    r_arm, has_solution_r = Robot.solve_ik_for_arm(0,
                                                                   arm_steps[
                                                                       i].armTrajectory.r_arm[j],
                                                                   self.z_offset)
                    l_arm, has_solution_l = Robot.solve_ik_for_arm(1,
                                                                   arm_steps[
                                                                       i].armTrajectory.l_arm[j],
                                                                   self.z_offset)
                    if (not has_solution_r) or (not has_solution_l):
                        return False
        return True

    @staticmethod
    def solve_ik_for_arm(arm_index, arm_state, z_offset=0):
        '''Finds an  IK solution for a particular arm pose'''
        # We need to find IK only if the frame is relative to an object
        if (arm_state.refFrame == ArmState.OBJECT):
            #rospy.loginfo('solve_ik_for_arm: Arm ' + str(arm_index) + ' is relative')
            solution = ArmState()
            target_pose = World.get_world().transform(arm_state.ee_pose,
                                          arm_state.refFrameObject.name, 'base_link')

            target_pose.position.z = target_pose.position.z + z_offset

            target_joints = Robot.arms[arm_index].get_ik_for_ee(target_pose,
                                                                arm_state.joint_pose)
            if (target_joints is None):
                rospy.logerr('No IK for relative end-effector pose.')
                return solution, False
            else:
                solution.refFrame = ArmState.ROBOT_BASE
                solution.ee_pose = Pose(target_pose.position,
                                        target_pose.orientation)
                solution.joint_pose = target_joints
                return solution, True
        elif (arm_state.refFrame == ArmState.ROBOT_BASE):
            #rospy.loginfo('solve_ik_for_arm: Arm ' + str(arm_index) + ' is absolute')
            pos = arm_state.ee_pose.position
            target_position = Point(pos.x, pos.y, pos.z + z_offset)
            target_pose = Pose(target_position, arm_state.ee_pose.orientation)
            target_joints = Robot.arms[arm_index].get_ik_for_ee(target_pose,
                                                                arm_state.joint_pose)
            if (target_joints is None):
                rospy.logerr('No IK for absolute end-effector pose.')
                return arm_state, False
            else:
                solution = ArmState()
                solution.refFrame = ArmState.ROBOT_BASE
                solution.ee_pose = Pose(arm_state.ee_pose.position,
                                        arm_state.ee_pose.orientation)
                solution.joint_pose = target_joints
                return solution, True
        else:
            return arm_state, True

    def start_move_arm_to_pose(self, arm_state, arm_index):
        '''Creates a thread for moving the arm to a target pose'''
        self.preempt = False
        thread = threading.Thread(group=None, target=self.move_arm_to_pose,
                                  args=(arm_state, arm_index,),
                                  name='move_to_arm_state_thread')
        thread.start()

    def move_arm_to_pose(self, arm_state, arm_index):
        '''The thread function that makes the arm move to
        a target end-effector pose'''
        rospy.loginfo('Started thread to move arm ' + str(arm_index))
        self.status = ExecutionStatus.EXECUTING
        solution, has_solution = Robot.solve_ik_for_arm(arm_index, arm_state)

        if (has_solution):
            if (arm_index == 0):
                is_successful = self.move_to_joints(solution, None)
            else:
                is_successful = self.move_to_joints(None, solution)

            if (is_successful):
                self.status = ExecutionStatus.SUCCEEDED
            else:
                self.status = ExecutionStatus.OBSTRUCTED
        else:
            self.status = ExecutionStatus.NO_IK

    @staticmethod
    def get_head_position():
        try:
            ref_frame = "/head_tilt_link"
            timestamp = World.get_world().tf_listener.getLatestCommonTime(ref_frame,
                                                              "/base_link")
            point_stamped = PointStamped()
            point_stamped.header.frame_id = ref_frame
            point_stamped.header.stamp = timestamp
            point_stamped.point = Point(1, 0, 0)
            head_position = World.get_world().tf_listener.transformPoint("/base_link", point_stamped).point
            return head_position
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException) as e:
            rospy.logwarn('Something wrong with transform request for head state.' + str(e))
            return None

    def _get_absolute_arm_states(self):
        abs_ee_poses = [Robot.get_ee_state(0),
                        Robot.get_ee_state(1)]
        joint_poses = [Robot.get_joint_state(0),
                       Robot.get_joint_state(1)]
        states = [None, None]
        for arm_index in [0, 1]:
            states[arm_index] = ArmState(ArmState.ROBOT_BASE,
                                         abs_ee_poses[arm_index], joint_poses[arm_index], Object())
        return states

    @staticmethod
    def is_hand_holding_object(arm_index):
        position = Robot.get_gripper_position(arm_index)
        closed = 0.002
        open = 0.078
        if closed < position < open:
            return True
        return False

    def move_head_to_point(self, point):
        rospy.loginfo("Moving head to point")
        Response.look_at_point(point)

    @staticmethod
    def _get_time_to_pose(pose, arm_index):
        ''' Returns the time to get to an arm pose'''
        if (pose is None):
            rospy.logwarn('Arm ' + str(arm_index) + 'will not move.')
            return None
        else:
            time_to_pose = Robot._get_time_bw_poses(
                Robot.arms[arm_index].get_ee_state(),
                pose.ee_pose)
            rospy.loginfo('Duration until next frame for arm ' +
                          str(arm_index) + ': ' + str(time_to_pose))
            return time_to_pose

    def move_to_joints(self, r_arm, l_arm):
        '''Makes the arms move to indicated joint poses'''
        time_to_r_pose = Robot._get_time_to_pose(r_arm, 0)
        time_to_l_pose = Robot._get_time_to_pose(l_arm, 1)

        #  If both arms are moving adjust velocities and find most moving arm
        is_r_moving = (time_to_r_pose != None)
        is_l_moving = (time_to_l_pose != None)
        if (not is_r_moving):
            Response.look_at_point(l_arm.ee_pose.position)
        elif (not is_l_moving):
            Response.look_at_point(r_arm.ee_pose.position)
        else:
            if (time_to_r_pose > time_to_l_pose):
                time_to_l_pose = time_to_r_pose
                Response.look_at_point(r_arm.ee_pose.position)
            else:
                time_to_r_pose = time_to_l_pose
                Response.look_at_point(l_arm.ee_pose.position)

        #  Move arms to target
        if (is_r_moving):
            Robot.arms[0].move_to_joints(r_arm.joint_pose, time_to_r_pose)
        if (is_l_moving):
            Robot.arms[1].move_to_joints(l_arm.joint_pose, time_to_l_pose)

        # Wait until both arms complete the trajectory
        while ((Robot.arms[0].is_executing() or
                    Robot.arms[1].is_executing()) and not self.preempt):
            time.sleep(0.01)
        rospy.loginfo('Arms reached target.')

        # Verify that both arms succeeded
        if ((not Robot.arms[0].is_successful() and is_r_moving) or
                (not Robot.arms[1].is_successful() and is_l_moving)):
            rospy.logwarn('Aborting because arms failed to move to pose.')
            return False
        else:
            return True

    @staticmethod
    def _get_most_moving_arm():
        '''Determines which of the two arms has moved more
        in the recent past'''
        threshold = 0.02
        if (Robot.arms[0].get_movement() < threshold and
                    Robot.arms[1].get_movement() < threshold):
            return -1
        elif (Robot.arms[0].get_movement() < threshold):
            return 1
        else:
            return 0

    @staticmethod
    def get_joint_state(arm_index):
        '''Get joint poritions'''
        return Robot.arms[arm_index].get_joint_state()

    @staticmethod
    def get_gripper_state(arm_index):
        ''' Get gripper status on the indicated side'''
        return Robot.arms[arm_index].get_gripper_state()

    @staticmethod
    def get_gripper_position(arm_index):
        ''' Get gripper status on the indicated side'''
        return Robot.arms[arm_index].get_gripper_position()

    @staticmethod
    def get_ee_state(arm_index):
        ''' Get pose of the end-effector on the indicated side'''
        return Robot.arms[arm_index].get_ee_state()

    @staticmethod
    def _get_time_bw_poses(pose0, pose1, velocity=0.2):
        '''Determines how much time should be allowed for
        moving between two poses'''
        dist = Arm.get_distance_bw_poses(pose0, pose1)
        duration = dist / velocity
        if duration < 0.5:
            duration = 0.5
        return duration

    def update(self):
        '''Periodic update for the two arms'''
        Robot.arms[0].update(self.is_executing())
        Robot.arms[1].update(self.is_executing())

        moving_arm = Robot._get_most_moving_arm()
        if (moving_arm != self.attended_arm and not self.is_executing()):
            # if (moving_arm == -1):
            #     Response.perform_gaze_action(GazeGoal.LOOK_FORWARD)
            # elif (moving_arm == 0):
            #     Response.perform_gaze_action(GazeGoal.FOLLOW_RIGHT_EE)
            # else:
            #     Response.perform_gaze_action(GazeGoal.FOLLOW_LEFT_EE)
            self.attended_arm = moving_arm
