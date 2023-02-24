#!/usr/bin/env python3
from datetime import datetime
from math import pi
from typing import Tuple

import rospy
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Vector3
from industrial_reconstruction_msgs.srv import (StartReconstruction,
                                                StartReconstructionRequest,
                                                StopReconstruction,
                                                StopReconstructionRequest)
from moveit_commander.conversions import list_to_pose

from move_group_utils.move_group_utils import (MoveGroupUtils,
                                               poses_list_from_yaml,
                                               publish_trajectory_markers)
from pilz_robot_program.pilz_robot_program import Lin, Ptp, Sequence


# motion parameters
# --------------------------------------------

# left side of table
HOME = (0.0, -pi/2.0, pi/2.0, 0, pi/2, pi)
TP_PATH = '/dev_ws/src/ur10e_examples/toolpaths/mosaic_scan_L.yaml'

# # right side of table
# HOME = (-pi, -pi/2.0, pi/2.0, 0, pi/2, pi)
# TP_PATH = '/dev_ws/src/ur10e_examples/toolpaths/mosaic_scan_R.yaml'

MOVE_VEL = 0.5
MOVE_ACC = 0.5
SCAN_VEL = 0.03
SCAN_ACC = 0.005
BLEND = 0.01

# reconstruction parameters
# --------------------------------------------
SCAN = True
OUTPUT_DIR = '/home/v/'


def gen_recon_msg(path: str = OUTPUT_DIR) -> Tuple[StartReconstructionRequest,
                                                   StopReconstructionRequest]:
    # define reconstruction srv msgs
    start_srv_req = StartReconstructionRequest()
    start_srv_req.tracking_frame = 'camera_depth_optical_frame'
    start_srv_req.relative_frame = 'base_link'
    start_srv_req.translation_distance = 0.0
    start_srv_req.rotational_distance = 0.0
    start_srv_req.live = False
    start_srv_req.tsdf_params.voxel_length = 0.001
    start_srv_req.tsdf_params.sdf_trunc = 0.002
    start_srv_req.tsdf_params.min_box_values = Vector3(x=0.0, y=0.0, z=0.0)
    start_srv_req.tsdf_params.max_box_values = Vector3(x=0.0, y=0.0, z=0.0)
    start_srv_req.rgbd_params.depth_scale = 1000
    start_srv_req.rgbd_params.depth_trunc = 0.15
    start_srv_req.rgbd_params.convert_rgb_to_intensity = False

    stop_srv_req = StopReconstructionRequest()
    path = path + datetime.now().strftime('%m_%d_%H_%M') + '.ply'
    stop_srv_req.mesh_filepath = path

    return start_srv_req, stop_srv_req


def robot_program():

    mgi = MoveGroupUtils()
    rospy.sleep(3)

    poses_as_list = poses_list_from_yaml(TP_PATH)
    poses = [list_to_pose(pose) for pose in poses_as_list]

    if SCAN:
        rospy.wait_for_service('/start_reconstruction', timeout=10)
        start_recon = rospy.ServiceProxy(
            '/start_reconstruction', StartReconstruction)
        stop_recon = rospy.ServiceProxy(
            '/stop_reconstruction', StopReconstruction)

    # add table collision table
    mgi.setup_scene()

    # set end effector link to camera
    mgi.move_group.set_end_effector_link('camera_tcp')

    # visualize the toolpath
    mgi.publish_pose_array(poses)

    # move home
    if not mgi.plan_and_execute(Ptp(goal=HOME, vel_scale=MOVE_VEL, acc_scale=MOVE_ACC)):
        return rospy.logerr('robot program: failed to move home')

    # move approach
    if not mgi.plan_and_execute(Ptp(goal=poses[0], vel_scale=MOVE_VEL, acc_scale=MOVE_ACC)):
        return rospy.logerr('robot program: failed to plan approach')

    # scanning sequence
    sequence = Sequence()
    for pose in poses[1:-1]:
        sequence.append(Lin(goal=pose,
                            vel_scale=SCAN_VEL,
                            acc_scale=SCAN_ACC),
                        blend_radius=BLEND)

    sequence.append(Lin(goal=poses[-1],
                    vel_scale=SCAN_VEL,
                    acc_scale=SCAN_ACC))

    success, plan = mgi.plan(sequence)
    if not success:
        return rospy.logerr('robot program: failed to plan sequence')

    if SCAN:
        # start reconstruction
        start_srv_req, stop_srv_req = gen_recon_msg()
        resp = start_recon(start_srv_req)
        if not resp:
            rospy.loginfo('robot program: failed to start reconstruction')
        rospy.loginfo('robot program: started reconstruction')

    mgi.sequencer.execute(plan)

    if SCAN:
        # stop reconstruction
        resp = stop_recon(stop_srv_req)
        if not resp:
            rospy.loginfo('robot program: failed to stop reconstruction')
        rospy.loginfo('robot program: reconstruction stopped successfully')

    # return home
    if not mgi.plan_and_execute(Ptp(goal=HOME, vel_scale=MOVE_VEL, acc_scale=MOVE_ACC)):
        return rospy.logerr('Failed to plan to home position')

    return rospy.loginfo('robot program: program completed')


if __name__ == '__main__':

    robot_program()
