#!/usr/bin/env python3
from math import pi
from typing import List, Tuple

import matplotlib.pyplot as plt
import rospy
import yaml
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit_commander.conversions import list_to_pose
from moveit_msgs.msg import CollisionObject
from std_msgs.msg import Header
from ur_msgs.srv import SetIO

from move_group_utils.move_group_utils import (MoveGroupUtils, make_mesh,
                                               poses_list_from_yaml,
                                               publish_trajectory_markers)
from pilz_robot_program.pilz_robot_program import Lin, Ptp, Sequence

PATH = '/home/v/segmented_shards_16_02_19_54_04/'
SIM = True
ATTACH = True
COLORMAP = plt.cm.get_cmap('tab20')

# motion parameters
HOME = (0.0, -pi/2.0, pi/2.0, -pi/2.0, -pi/2, 0.0)
APPROACH_OFFSET = 0.1
PICK_VEL = 0.1
PICK_ACC = 0.1
MOVE_VEL = 0.1
MOVE_ACC = 0.1

DO: int = 0
WAIT: float = 0.2


def load_shard_data(dir_path: str, frame_id: str
                    ) -> Tuple[List[Pose], List[Pose], List[CollisionObject]]:
    pick = []
    place = []
    co = []
    pose = PoseStamped()
    pose.header = Header()
    pose.header.frame_id = frame_id
    pose.pose = Pose(position=Point(0.0, 0.0, 0.0),
                     orientation=Quaternion(0.0, 0.0, 0.0, 1.0))

    data = yaml.load(open(dir_path + 'log.yaml', 'r'), Loader=yaml.FullLoader)

    for i in range(data['shards']['num_shards']):
        pick.append([float(data['shards']['shard_' + str(i)]['pick']['position']['x']),
                     float(data['shards']['shard_' +
                                          str(i)]['pick']['position']['y']),
                     float(data['shards']['shard_' +
                                          str(i)]['pick']['position']['z']) + 0.01,
                     float(data['shards']['shard_' +
                                          str(i)]['pick']['quaternion']['x']),
                     float(data['shards']['shard_' +
                                          str(i)]['pick']['quaternion']['y']),
                     float(data['shards']['shard_' +
                                          str(i)]['pick']['quaternion']['z']),
                     float(data['shards']['shard_' + str(i)]['pick']['quaternion']['w'])])
        place.append([0.5, 0.5, 0.05 + i*0.02, 0.0, 1.0, 0.0, 0.0])
        co.append(make_mesh('shard_' + str(i), pose,
                  dir_path + data['shards']['shard_' + str(i)]['mesh_path']))

    rospy.loginfo(
        f'loaded dir {dir_path} with {data["shards"]["num_shards"]} shards')

    return pick, place, co


def pick_and_place():

    # initialize node and moveit commander
    mgi = MoveGroupUtils()

    # wait for rviz and moveit to start
    rospy.sleep(3.0)

    # load shard data
    pick, place, co = load_shard_data(
        PATH, mgi.move_group.get_planning_frame())

    # interleave pick and place poses
    poses_as_list = [None]*(len(pick)+len(place))
    poses_as_list[::2] = pick
    poses_as_list[1::2] = place
    # convert to geometry_msgs/Pose
    poses = [list_to_pose(pose) for pose in poses_as_list]

    # add ground_cube collision object
    mgi.setup_scene()
    
    # add shards as collision objects
    for i, collision_object in enumerate(co):
        mgi.add_collision_object(collision_object)
        color = COLORMAP(i / len(co))[:3]
        mgi.set_color(collision_object.id, color[0], color[1], color[2], 1.0)
    mgi.send_colors()
    

    if not SIM:
        # wait for ur_hardware_interface/set_io service
        rospy.wait_for_service('ur_hardware_interface/set_io', 30)
        set_io = rospy.ServiceProxy('ur_hardware_interface/set_io', SetIO)


    # display poses in rviz
    mgi.publish_pose_array(poses)

    if not mgi.plan_and_execute(Ptp(goal=HOME, vel_scale=MOVE_VEL, acc_scale=MOVE_ACC)):
        return

    for i in range(0, len(poses), 2):
        for j in range(2):
            approach = Pose(position=Point(poses[i+j].position.x,
                                           poses[i+j].position.y,
                                           poses[i+j].position.z + APPROACH_OFFSET),
                            orientation=(poses[i+j].orientation))

            if not mgi.plan_and_execute(Ptp(goal=approach, vel_scale=MOVE_VEL, acc_scale=MOVE_ACC)):
                return
            if not mgi.plan_and_execute(Lin(goal=poses[i+j], vel_scale=PICK_VEL, acc_scale=PICK_ACC)):
                return

            if not SIM and j == 0:
                set_io(0, DO, 1)
                rospy.sleep(0.2)
            if ATTACH and j == 0:
                    mgi.robot.manipulator.attach_object(co[int(i/2)].id)

            if not SIM and j == 1:
                set_io(0, DO, 0)
                rospy.sleep(0.2)
            if ATTACH and j == 1:
                    mgi.robot.manipulator.detach_object(co[int(i/2)].id)

            if not mgi.plan_and_execute(Lin(goal=approach, vel_scale=MOVE_VEL, acc_scale=MOVE_ACC)):
                return

    if not mgi.plan_and_execute(Ptp(goal=HOME, vel_scale=MOVE_VEL, acc_scale=MOVE_ACC)):
        return


if __name__ == '__main__':
    pick_and_place()
