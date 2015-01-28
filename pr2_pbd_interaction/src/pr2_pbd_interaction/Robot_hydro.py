import sys
import threading
import rospy
import moveit_commander
from pr2_mechanism_msgs.srv import SwitchController
from pr2_controllers_msgs.msg import Pr2GripperCommand
from geometry_msgs.msg import Point, Pose, Quaternion

class Robot_hydro:
    '''class for moving arms and getting arm states'''
    RIGHT = 0
    LEFT = 1
    RELAXED = 0
    HOLD = 1
    CLOSED = 0
    OPENED = 1
    SIDE_PREFIX = ["r", "l"]
    ARM_JOINTS = ["r_wrist_roll_link", "l_wrist_roll_link"]
    STARTING_GRIPPER_STATES = [CLOSED, CLOSED]

    arms = []
    robot = None

    def __init__(self):
        #right - 0, left - 1
        rospy.loginfo("Initializing moveit")
        moveit_commander.roscpp_initialize(sys.argv)
        '''initialize moveit'''
        self.arms = moveit_commander.MoveGroupCommander("arms")

        '''is_exec - is the robot currently executing an action'''
        self.is_exec = False

        rospy.loginfo("Moveit initialized, initializing swich controller")
        switch_controller = 'pr2_controller_manager/switch_controller'
        rospy.wait_for_service(switch_controller)
        self.switch_service = rospy.ServiceProxy(switch_controller,
                                                 SwitchController)

        self.gipper_pubs = [rospy.Publisher('/r_gripper_controller/command', Pr2GripperCommand),
            rospy.Publisher('/l_gripper_controller/command', Pr2GripperCommand)]

        '''set initial gripper states'''
        self.gripper_states = Robot.STARTING_GRIPPER_STATES
        # for arm_ind, state in enumerate(Robot.STARTING_GRIPPER_STATES):
        #     self.set_gripper_state(arm_ind, state)


    '''set the gripper states'''
    def set_gripper_state(self, arm_index, g_state):
        # rospy.loginfo("here")
        self.gipper_pubs[arm_index].publish(Pr2GripperCommand(0.1 * g_state,100))
        # self.gripper_states[arm_index] = g_state
        return True

    def get_gripper_states(self):
        return self.gripper_states

    '''freze/relax the arms'''
    def set_arm_mode(self, arm_index, arm_mode):#freese/unfreese
        controller_name = Robot.SIDE_PREFIX[arm_index] + '_arm_controller'
        if (arm_mode == Robot.RELAXED):
            self.switch_service([], [controller_name], 1)
        else:
            self.switch_service([controller_name], [], 1)
        return True

    '''is the robot currently executing an action?'''
    def is_executing(self):
        return self.is_exec

    '''move arms to pose synchronously'''
    def _move_to_pose(self, arms):
        for arm_ind in range(0, 2):
            self.arms.set_pose_target(arms[arm_ind], Robot.ARM_JOINTS[arm_ind])

        self.arms.go()

    '''execute the action (private method)'''
    def _execute_action(self, action, world):
        if (self.is_exec):
            if (action.scan_code == Action.SCAN):
                world.scan_landmarks()
            if (action.scan_code == Action.SCAN_MOVE_ARMS):
                self.hands_up()
                world.scan_landmarks()
                self.hands_down()
            if (action.type == Action.ACTION_QUEUE):
                for act in action.actions:
                    self._execute_action(act, world)
            elif (action.type == Action.POSE):
                best_landmarks = map(world.find_landmark, action.landmark_types)
                def mod_pose((pose, mark)):
                    return Pose(Point(
                        pose.pose.position.x + mark.pose.position.x,
                        pose.pose.position.y + mark.pose.position.y,
                        pose.pose.position.z + mark.pose.position.z),
                        pose.pose.orientation)
                target_pose = map(mod_pose, zip(action.arms, best_landmarks))
                self._move_to_pose(target_pose)
            elif (action.type == Action.GRIPPER):
                for arm_ind, g_state in enumerate(self.gripper_states):
                    self.set_gripper_state(arm_ind, g_state)
            elif (action.type == Action.TRAJECTORY):
                self._execute_trajectory(action.poses)

    # def start_execution(self, action, z_offset=0):
    #     ''' Starts execution of an action'''
    #     # This will take long, create a thread
    #     self.preempt = False
    #     self.z_offset = z_offset
    #     self.action = action
    #     thread = threading.Thread(group=None, target=self._execute_action,
    #                               name='skill_execution_thread')
    #     thread.start()

    '''start executing the action'''
    def start_execution(self, action, world):
        rospy.loginfo("executing...")
        self.is_exec = True
        self._execute_action(action, world)
        self.is_exec = False
        rospy.loginfo("executing complete")

    '''stop executing the action'''
    def stop_execution(self):
        self.is_exec = False
        self.arms.stop()

    '''get current arm states'''
    def get_arm_state(self):
        return [self.arms.get_current_pose(arm_joint) for arm_joint in Robot.ARM_JOINTS]
