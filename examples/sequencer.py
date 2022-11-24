#!/usr/bin/env python3

from math import pi

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion

from move_group_utils.move_group_utils import MoveGroupUtils
from pilz_robot_program.pilz_robot_program import (Circ, Lin, Ptp, Sequence,
                                                   from_euler)


def robot_program():

    mgi = MoveGroupUtils()

    home = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, -pi / 2.0)
    pose_l = Pose(position=Point(0.6, -0.6, 0.4),
                  orientation=from_euler(0.0, pi, 0.0))
    pose_r = Pose(position=Point(0.6, 0.6, 0.4),
                  orientation=from_euler(0.0, pi, 0.0))

    poses = [home, pose_l, pose_r]

    mgi.publish_pose_array([pose_l, pose_r])

    mgi.sequencer.plan(Ptp(goal=home, vel_scale=0.3, acc_scale=0.3))

    for pose in poses:
        mgi.sequencer.plan(Ptp(goal=pose, vel_scale=0.3, acc_scale=0.3))
        mgi.sequencer.execute()


if __name__ == '__main__':

    robot_program()
