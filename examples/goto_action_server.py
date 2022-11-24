#!/usr/bin/env python3

import actionlib
import rospy
import ur10e_examples.msg

from move_group_utils.move_group_utils import MoveGroupUtils
from pilz_robot_program.pilz_robot_program import Circ, Lin, Ptp

# TODO adapt to sequence rather than single goal


class GotoAction:

    # create messages that are used to publish feedback/result
    _feedback = ur10e_examples.msg.GotoFeedback()
    _result = ur10e_examples.msg.GotoResult()

    def __init__(self):

        self.mgi = MoveGroupUtils()
        self._as = actionlib.SimpleActionServer(
            self.mgi.name,
            ur10e_examples.msg.GotoAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self._as.start()

    def execute_callback(self, pose):

        rospy.loginfo(f'{self.mgi.name}: exec_callback')
        # start executing the action
        plan = self.mgi.sequencer.plan(
            Ptp(goal=pose.goal, vel_scale=0.3, acc_scale=0.3)
        )
        self._feedback.planning_succeeded = plan[0]

        # publish the feedback
        self._as.publish_feedback(self._feedback)

        # check that preempt has not been requested by the client
        # in this specific use case the preempt is not required, as
        # there is only one goal to execute. It is in this example for
        # completeness
        if self._as.is_preempt_requested():
            self._as.set_preempted()
            return rospy.loginfo(f'{self.mgi.name}: Action Preempted')

        # if execute returns None motion was executed
        if not self.mgi.sequencer.execute():
            self._result.goal_reached = True
            self._as.set_succeeded(self._result)
            return rospy.loginfo(f'{self.mgi.name}: Action Succeeded')

        else:
            return rospy.loginfo(f'{self.mgi.name}: Action Failed')


if __name__ == '__main__':

    GotoAction()
    rospy.spin()
