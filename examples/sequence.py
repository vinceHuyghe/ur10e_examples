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

    mgi.sequencer.plan(Ptp(goal=home))
    mgi.sequencer.execute()

    sequence = Sequence()

    sequence.append(Ptp(goal=home))
    sequence.append(
        Ptp(
            goal=Pose(
                position=Point(0.4, 0.0, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        )
    )

    sequence.append(
        Lin(
            goal=Pose(
                position=Point(0.4, 0.3, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        ),
        blend_radius=0.01,
    )

    sequence.append(
        Lin(
            goal=Pose(
                position=Point(0.4, -0.3, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        )
    )

    sequence.append(
        Circ(
            goal=Pose(
                position=Point(0.4, 0.0, 0.9),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            center=Point(0.4, 0.0, 0.6),
        ),
        blend_radius=0.01,
    )

    sequence.append(
        Circ(
            goal=Pose(
                position=Point(0.4, 0.3, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            center=Point(0.4, 0.0, 0.6),
        ),
        blend_radius=0.01,
    )

    sequence.append(
        Lin(
            goal=Pose(
                position=Point(0.4, 0.0, 0.6),
                orientation=Quaternion(0.5, 0.5, 0.5, 0.5),
            ),
            vel_scale=0.1,
            acc_scale=0.05,
        )
    )

    success, plan = mgi.sequencer.plan(sequence)[:2]

    mgi.display_trajectory(plan)

    mgi.sequencer.execute()


if __name__ == '__main__':

    robot_program()
