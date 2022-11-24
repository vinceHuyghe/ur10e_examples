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

    mgi.sequencer.plan(Ptp(goal=start, vel_scale=0.3, acc_scale=0.3))
    mgi.sequencer.execute()

    mgi.sequencer.plan(Ptp(goal=pose_l, vel_scale=0.3, acc_scale=0.3))
    mgi.sequencer.execute()

    set_io(1, 0, 1)

    mgi.sequencer.plan(Ptp(goal=pose_r, vel_scale=0.3, acc_scale=0.3))
    mgi.sequencer.execute()

    set_io(1, 0, 0)


if __name__ == '__main__':

    robot_program()
