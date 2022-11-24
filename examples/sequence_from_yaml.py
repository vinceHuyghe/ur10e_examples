#!/usr/bin/env python3
from math import pi

import rospy
from moveit_commander.conversions import list_to_pose

from move_group_utils.move_group_utils import (MoveGroupUtils,
                                               poses_list_from_yaml)
from pilz_robot_program.pilz_robot_program import Lin, Ptp, Sequence


def robot_program():

    home = (0.0, -pi / 2.0, pi / 2.0, 0.0, pi / 2.0, 0.0)

    mgi = MoveGroupUtils()
    sequence = Sequence()

    sequence.append(Ptp(goal=home))

    # create pose mgs list from yaml
    poses_list = poses_list_from_yaml(
        '/dev_ws/src/ur10e_examples/toolpaths/test.yaml')

    poses = [list_to_pose(pose) for pose in poses_list]

    # publish the poses to rviz for preview
    mgi.publish_pose_array(poses)

    for p in poses:
        sequence.append(Lin(goal=p))

    success, plan = mgi.sequencer.plan(sequence)[:2]
    mgi.display_trajectory(plan)
    mgi.sequencer.execute()


if __name__ == '__main__':

    robot_program()
