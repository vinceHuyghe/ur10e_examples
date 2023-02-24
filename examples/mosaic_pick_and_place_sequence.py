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
from pilz_robot_program.pilz_robot_program import (Lin, Ptp, Sequence,
                                                   SequencePlanningError)

PATH = '/home/v/segmented_shards_24_02_15_22_31/'
SIM = False
ATTACH = True
COLORMAP = plt.cm.get_cmap('tab20')

# motion parameters
HOME = (0.0, -pi/2.0, pi/2.0, -pi, -pi/2, 0)
APPROACH_OFFSET = 0.08
BLEND = 0.070
PICK_VEL = 0.1
PICK_ACC = 0.1
MOVE_VEL = 0.8
MOVE_ACC = 0.8

DO: int = 4
WAIT: float = 0.1


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
                                          str(i)]['pick']['position']['z']),
                     float(data['shards']['shard_' +
                                          str(i)]['pick']['quaternion']['x']),
                     float(data['shards']['shard_' +
                                          str(i)]['pick']['quaternion']['y']),
                     float(data['shards']['shard_' +
                                          str(i)]['pick']['quaternion']['z']),
                     float(data['shards']['shard_' + str(i)]['pick']['quaternion']['w'])])
        # place.append([pick[i][0]+0.3, pick[i][1], pick[i][2], pick[i][3], pick[i][4], pick[i][5], pick[i][6]])
        place.append([float(data['shards']['shard_' + str(i)]['place']['position']['x']),
                     float(data['shards']['shard_' +
                                          str(i)]['place']['position']['y']),
                     float(data['shards']['shard_' +
                                          str(i)]['place']['position']['z']),
                     float(data['shards']['shard_' +
                                          str(i)]['place']['quaternion']['x']),
                     float(data['shards']['shard_' +
                                          str(i)]['place']['quaternion']['y']),
                     float(data['shards']['shard_' +
                                          str(i)]['place']['quaternion']['z']),
                     float(data['shards']['shard_' + str(i)]['place']['quaternion']['w'])])
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
    mgi.move_group.set_end_effector_link('tcp')

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

    if not mgi.plan_and_execute(Ptp(goal=HOME, vel_scale=MOVE_VEL, acc_scale=MOVE_ACC)):
        return

    pp_poses = []
    for pose in poses:
        approach = Pose(position=Point(pose.position.x,
                                       pose.position.y,
                                       pose.position.z + APPROACH_OFFSET),
                        orientation=(pose.orientation))
        pp_poses.append(approach)
        pp_poses.append(pose)
        pp_poses.append(approach)
    mgi.publish_pose_array(pp_poses)

    for i in range(0, len(pp_poses), 6):

        rospy.loginfo(f'pick and place shard {round(i/6)+1}')

        if i == 0:
            start_sequence = Sequence()
            start_sequence.append(
                Ptp(goal=HOME, vel_scale=MOVE_VEL, acc_scale=MOVE_ACC))
            start_sequence.append(
                Ptp(goal=pp_poses[i], vel_scale=MOVE_VEL, acc_scale=MOVE_ACC),
                blend_radius=BLEND)
            start_sequence.append(
                Lin(goal=pp_poses[i+1], vel_scale=PICK_VEL, acc_scale=PICK_ACC))
            if not mgi.plan_and_execute(start_sequence):
                return

        if not SIM:
            set_io(1, DO, 1)
            rospy.sleep(WAIT)
        if ATTACH:
            mgi.robot.manipulator.attach_object(
                co[round(i/6)].id, mgi.move_group.get_end_effector_link())

        pp_sequence = Sequence()
        pp_sequence.append(Lin(
            goal=pp_poses[i+2], vel_scale=PICK_VEL, acc_scale=PICK_ACC),
            blend_radius=BLEND)
        pp_sequence.append(
            Ptp(goal=pp_poses[i+3], vel_scale=MOVE_VEL, acc_scale=MOVE_ACC),
            blend_radius=BLEND)
        pp_sequence.append(Lin(
            goal=pp_poses[i+4], vel_scale=PICK_VEL, acc_scale=PICK_ACC))

        if not mgi.plan_and_execute(pp_sequence):
            return

        if not SIM:
            set_io(1, DO, 0)
            rospy.sleep(WAIT)

        if ATTACH:
            mgi.robot.manipulator.detach_object(co[round(i/6)].id)

        reset_sequence = Sequence()
        reset_sequence.append(Lin(
            goal=pp_poses[i+5], vel_scale=PICK_VEL, acc_scale=PICK_ACC),
            blend_radius=BLEND)

        if i == len(pp_poses) - 6:
            reset_sequence.append(Ptp(
                goal=HOME, vel_scale=MOVE_VEL, acc_scale=MOVE_ACC))
        else:
            reset_sequence.append(Ptp(
                goal=pp_poses[i+6], vel_scale=MOVE_VEL, acc_scale=MOVE_ACC),
                blend_radius=BLEND)
            reset_sequence.append(Lin(
                goal=pp_poses[i+7], vel_scale=PICK_VEL, acc_scale=PICK_ACC))
        if not mgi.plan_and_execute(reset_sequence):
            return


if __name__ == '__main__':
    pick_and_place()
