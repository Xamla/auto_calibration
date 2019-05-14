#!/usr/bin/env python3
from __future__ import print_function
from typing import Dict, List, Tuple
import cv2 as cv
import sys
import numpy as np

from tooltip_calibration import TooltipCalibration
import patternlocalisation
from xamla_motion.world_view_client import WorldViewClient
from xamla_motion.motion_client import MoveGroup
from xamla_motion.motion_service import MotionService
from xamla_motion.data_types import Pose

from os import path
import torchfile


def main():
    world_view_client = WorldViewClient()

    input_var = input("Please type in the torch filename (with path) of the sensorhead stereo camera calibration (e.g. \"calibration/torso_cameras/stereo_cams_<serial1>_<serial2>.t7\" ): ")
    cam_calib_fn = str(input_var)
    stereocalib_sensorhead_cams = torchfile.load(cam_calib_fn)
    end_of_right_serial = cam_calib_fn.rfind(".")
    start_of_right_serial = cam_calib_fn.rfind("_")
    right_serial = cam_calib_fn[start_of_right_serial+1:end_of_right_serial]
    end_of_left_serial = start_of_right_serial
    start_of_left_serial = cam_calib_fn.rfind("_", 0, end_of_left_serial)
    left_serial = cam_calib_fn[start_of_left_serial+1:end_of_left_serial]
    print("right_serial:")
    print(right_serial)
    print("left_serial:")
    print(left_serial)
    print("Please type in the exposure time of the cameras [in microseconds],")
    input_var = input("or press \'enter\' to use 120000ms:")
    if input_var == "" :
      exposure_time = 120000
    else :
      exposure_time = int(input_var)
    print("exposure_time:")
    print(exposure_time)
    print(type(exposure_time))

    move_group_names = MotionService.query_available_move_groups()
    move_group_name = move_group_names[2].name # left_arm_torso
    flange_link_name = move_group_names[2].end_effector_link_names[0] # arm_left_link_tool0
    print("For which arm will the tooltip be calibrated? Please type l or r, then press \'Enter\'.")
    print("l: left arm")
    print("r: right arm")
    which = sys.stdin.read(1)
    if which == "r" :
      move_group_name = move_group_names[3].name # right_arm_torso
      flange_link_name = move_group_names[3].end_effector_link_names[0] # arm_right_link_tool0
    print("move_group_name:")
    print(move_group_name)
    move_group = MoveGroup(move_group_name)
    print("flange_link_name")
    print(flange_link_name)
    torso_link_name = "torso_link_b1"
    print("torso_link_name:")
    print(torso_link_name)

    last_slash = cam_calib_fn.rfind("/")
    torso_to_cam_fn = cam_calib_fn[:last_slash] + "/LeftCam_torso_joint_b1.t7"
    print("Please type in the name of the torchfile (with path) containing the transformation between torso joint and torso cameras,")
    input_var = input("or press \'enter\' to use " + torso_to_cam_fn)
    if input_var != "" :
      torso_to_cam_fn = str(input_var)
    print("torso_to_cam_fn:")
    print(torso_to_cam_fn)
    torso_to_cam_t7 = torchfile.load(torso_to_cam_fn)
    torso_to_cam = Pose.from_transformation_matrix(matrix=torso_to_cam_t7, frame_id=torso_link_name, normalize_rotation=False)

    pattern_localizer = patternlocalisation.PatternLocalisation()
    pattern_localizer.circleFinderParams.minArea = 50
    pattern_localizer.circleFinderParams.maxArea = 1000
    pattern_localizer.setPatternData(8, 11, 0.003)
    pattern_localizer.setStereoCalibration(stereocalib_sensorhead_cams)
    #pattern_localizer.setFindCirclesGridFlag(cv.CALIB_CB_ASYMMETRIC_GRID) # Don't use "cv.CALIB_CB_CLUSTERING", because it's much more sensitive to background clutter.
    pattern_localizer.setFindCirclesGridFlag(cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING)

    world_view_folder = '/Calibration'
    storage_folder = '/tmp/calibration/storage_tooltipcalib'

    offline = True

    tooltip_calib = TooltipCalibration(pattern_localizer, stereocalib_sensorhead_cams, left_serial, right_serial, exposure_time,
                                       world_view_client, world_view_folder, move_group, torso_to_cam, storage_folder, torso_link_name,
                                       flange_link_name, offline)

    tooltip_calib.runTooltipCalib()
    


if __name__ == '__main__':
    main()
