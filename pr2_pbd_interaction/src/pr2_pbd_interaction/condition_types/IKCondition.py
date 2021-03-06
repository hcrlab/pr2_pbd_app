#!/usr/bin/env python
import rospy

from pr2_pbd_interaction.condition_types.Condition import Condition
from pr2_pbd_interaction.msg import Strategy


class IKCondition(Condition):
    """
    Checks that the poses in the manipulation are reachable.
    """

    def __init__(self, *args, **kwargs):
        Condition.__init__(self, *args, **kwargs)
        self.steps = None
        if len(args) > 0:
            self.steps = args[0]
        self.available_strategies = [Strategy.FAIL_FAST, Strategy.SKIP_STEP, Strategy.GO_TO_PREVIOUS_STEP]
        self.current_strategy_index = 0

    def set_steps(self, steps):
        self.steps = steps

    def check(self):
        from pr2_pbd_interaction.Robot import Robot

        robot = Robot.get_robot()
        if not robot.has_ik_solutions_for_arm_steps(self.steps):
            self.error_msg = "Problems in finding IK solutions - action contains unreachable poses."
            rospy.logwarn(self.error_msg)
            return False
        return True