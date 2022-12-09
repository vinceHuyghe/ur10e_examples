#!/usr/bin/env python3

from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from ur_msgs.srv import SetIO

from move_group_utils.move_group_utils import MoveGroupUtils
from pilz_robot_program.pilz_robot_program import (Circ, Lin, Ptp, Sequence,
                                                   from_euler)

# TODO test on HW


def robot_program():

    mgi = MoveGroupUtils()

    rospy.loginfo(f'{mgi.name}: waiting for /ur_hardware_interface/set_io')
    rospy.wait_for_service('ur_hardware_interface/set_io', 30)
    set_io = rospy.ServiceProxy('ur_hardware_interface/set_io', SetIO)

    start = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, 0.0)
    pose_l = Pose(position=Point(0.8, -0.6, 0.4),
                  orientation=from_euler(0.0, pi, 0.0))
    pose_r = Pose(position=Point(0.8, 0.6, 0.4),
                  orientation=from_euler(0.0, pi, 0.0))

    success, plan = mgi.sequencer.plan(
        Ptp(goal=start, vel_scale=0.3, acc_scale=0.3))[:2]
    if not success:
        return rospy.logerr('Failed to plan to start position')
    mgi.sequencer.execute(plan)

    success, plan = mgi.sequencer.plan(Ptp(goal=pose_l, vel_scale=0.3, acc_scale=0.3))[:2]
    if not success:
        return rospy.logerr('Failed to plan to pose_l')
    mgi.sequencer.execute(plan)

    # set DO0 to 1 (ON)
    set_io(1, 0, 1)

    success, plan = mgi.sequencer.plan(Ptp(goal=pose_r, vel_scale=0.3, acc_scale=0.3))[:2]
    if not success:
        return rospy.logerr('Failed to plan to pose_r')
    mgi.sequencer.execute(plan)

    # set DO0 to 0 (OFF)
    set_io(1, 0, 0)
    
    return rospy.loginfo('Robot program completed')


if __name__ == '__main__':

    robot_program()
