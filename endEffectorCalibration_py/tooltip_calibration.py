import numpy as np
import cv2 as cv
import os
import sys
import pickle
from xamla_motion.data_types import Pose
from xamla_vision import XimeaCaptureClient
from xamla_vision import GeniCamCaptureClient
from pyquaternion import Quaternion
from scipy.linalg import sqrtm, inv, norm
from copy import deepcopy
from numpy.linalg import norm


class TooltipCalibration(object):
  def __init__(self, pattern_localizer, stereocalib, left_serial, right_serial, exposure_time, 
               world_view_client, world_view_folder, move_group, hand_eye, storage_folder, link_name,
               flange_link_name, offline):

    self.pattern_localizer = pattern_localizer
    self.stereocalib = stereocalib
    if not offline :
      self.capture_client = XimeaCaptureClient(serials=[left_serial, right_serial])
    self.left_serial = left_serial
    self.right_serial = right_serial
    self.exposure_time = exposure_time
    self.world_view_client = world_view_client
    self.world_view_folder = world_view_folder
    self.move_group = move_group
    self.hand_eye = hand_eye # This is torso_to_cam.
    self.storage_folder = storage_folder
    self.link_name = link_name # This is torso_link_name.
    self.flange_link_name = flange_link_name
    self.offline = offline


  def addOrUpdatePose(self, display_name, folder, pose):
    try:
      get_value = self.world_view_client.get_pose(display_name, folder)
      self.world_view_client.update_pose(display_name, folder, pose)
    except:
      self.world_view_client.add_pose(display_name, folder, pose)


  def runTooltipCalib(self):

    print('Please move robot to capture pose. Then press \'Enter\' and wait.')
    sys.stdin.read(1)
    sys.stdin.read(1)

    if not os.path.isdir(self.storage_folder) :
      os.mkdir(self.storage_folder)

    image_left = None
    image_right = None
    if not self.offline :
      # capture images of calibration target and write them into storage folder
      images = self.capture_client(self.exposure_time)
      image_left = cv.cvtColor(images[self.left_serial], cv.COLOR_GRAY2RGB)
      image_right = cv.cvtColor(images[self.right_serial], cv.COLOR_GRAY2RGB)
      cv.imwrite(os.path.join(self.storage_folder, 'left.png'), image_left)
      cv.imwrite(os.path.join(self.storage_folder, 'right.png'), image_right)
    else :
      # offline version: read images of calibration target
      image_left = cv.imread(os.path.join(self.storage_folder, 'left.png'))
      image_right = cv.imread(os.path.join(self.storage_folder, 'right.png'))

    current_posture_1 = None
    current_pose_1 = None
    if not self.offline :
      # store robot poses into storage folder
      current_posture_1 = self.move_group.get_current_joint_positions()
      current_pose_1 = self.move_group.motion_service.query_pose(self.move_group.name, current_posture_1, self.link_name)
      with open(os.path.join(self.storage_folder, 'posture_1.p'), 'wb') as f:
          pickle.dump(current_posture_1, f)
      with open(os.path.join(self.storage_folder, 'pose_1.p'), 'wb') as f:
          pickle.dump(current_pose_1, f)
    else :
      # offline version: read posture and poses
      with open(os.path.join(self.storage_folder, 'posture_1.p'), 'rb') as f:
        current_posture_1 = pickle.load(f)
      with open(os.path.join(self.storage_folder, 'pose_1.p'), 'rb') as f:
        current_pose_1 = pickle.load(f)

    self.pattern_localizer.setStereoCalibration(self.stereocalib) # self.stereocalib_sensorhead_cams
    camPoseFinalList, circleGridPointsLeft, circleGridPointsRight, pointsInCamCoords = self.pattern_localizer.calcCamPoseViaPlaneFit(image_left, image_right, "left", False, True)
    if (circleGridPointsLeft == None or circleGridPointsRight == None) :
      print("Not all 4 patterns have been found in both camera images.")
      print("-> One of the patterns may lie too near to the image boundary, such that it is cropped after image rectification.")
      print("-> The images might be too dark, i.e. exposure time might be too low, ...")
      print("-> Run pattern search in debug mode to analyze problem:")
      camPoseFinalList, circleGridPointsLeft, circleGridPointsRight, pointsInCamCoords = self.pattern_localizer.calcCamPoseViaPlaneFit(image_left, image_right, "left", True, True)
      return False
    elif (len(circleGridPointsLeft) != 4 or len(circleGridPointsRight) != 4) :
      print("Not all 4 patterns have been found in both camera images.")
      print("-> One of the patterns may lie too near to the image boundary, such that it is cropped after image rectification.")
      print("-> The images might be too dark, i.e. exposure time might be too low, ...")
      print("-> Run pattern search in debug mode to analyze problem:")
      camPoseFinalList, circleGridPointsLeft, circleGridPointsRight, pointsInCamCoords = self.pattern_localizer.calcCamPoseViaPlaneFit(image_left, image_right, "left", True, True)
      return False

    Hc1 = camPoseFinalList["1"]
    Hc2 = camPoseFinalList["3"]
    Hc3 = camPoseFinalList["5"]
    Hc4 = camPoseFinalList["7"]
    print("camPoseFinalList[\"1\"]:")
    print(Hc1)
    print("camPoseFinalList[\"3\"]:")
    print(Hc2)
    print("camPoseFinalList[\"5\"]:")
    print(Hc3)
    print("camPoseFinalList[\"7\"]:")
    print(Hc4)
    print("\n")

    Hg_Pose = deepcopy(current_pose_1)
    H_Pose = deepcopy(self.hand_eye) # hand_eye is torso_eye (= torso_to_cam) here!
    H = deepcopy(H_Pose.transformation_matrix())
    Hg = deepcopy(Hg_Pose.transformation_matrix())
    print("Hg (torso pose):")
    print(Hg)
    print("H (torso_to_cam):")
    print(H)
    print("\n")

    t_mat1 = np.matmul(Hg, np.matmul(H, Hc1))
    t_mat2 = np.matmul(Hg, np.matmul(H, Hc2))
    t_mat3 = np.matmul(Hg, np.matmul(H, Hc3))
    t_mat4 = np.matmul(Hg, np.matmul(H, Hc4))
    print("Pose of pattern with id 1 in base coordinates:")
    print(t_mat1)
    print("Pose of pattern with id 3 in base coordinates:")
    print(t_mat2)
    print("Pose of pattern with id 5 in base coordinates:")
    print(t_mat3)
    print("Pose of pattern with id 7 in base coordinates:")
    print(t_mat4)
    print("\n")

    t1 = np.zeros(shape=2, dtype=np.float64)
    t2 = np.zeros(shape=2, dtype=np.float64)
    t3 = np.zeros(shape=2, dtype=np.float64)
    t4 = np.zeros(shape=2, dtype=np.float64)
    t1[0] = t_mat1[0][3]
    t1[1] = t_mat1[1][3]
    t2[0] = t_mat2[0][3]
    t2[1] = t_mat2[1][3]
    t3[0] = t_mat3[0][3]
    t3[1] = t_mat3[1][3]
    t4[0] = t_mat4[0][3]
    t4[1] = t_mat4[1][3]

    def calc_intersection(p1,p2,q1,q2) :
      x1 = deepcopy(p1[0])
      y1 = deepcopy(p1[1])
      x2 = deepcopy(p2[0])
      y2 = deepcopy(p2[1])
      a1 = deepcopy(q1[0])
      b1 = deepcopy(q1[1])
      a2 = deepcopy(q2[0])
      b2 = deepcopy(q2[1])
      zaehler = -y1 + b1 - (((a1-x1)/(x2-x1)) * (y2-y1))
      nenner  = (((a2-a1)/(x2-x1)) * (y2-y1)) - (b2-b1)
      quotient = zaehler / nenner
      s1 = a1 + quotient * (a2-a1)
      s2 = b1 + quotient * (b2-b1)
      s = np.zeros(shape=2, dtype=np.float64)
      s[0] = s1
      s[1] = s2
      return s

    intersection = calc_intersection(t1,t3,t2,t4)
    
    print("=> Position of cross lines:")
    cross_pos = np.zeros(shape=3, dtype=np.float64)
    cross_pos[0] = intersection[0]
    cross_pos[1] = intersection[1]
    cross_pos[2] = 0.25 * (t_mat1[2][3] + t_mat2[2][3] + t_mat3[2][3] + t_mat4[2][3])
    print(cross_pos)
    print("\n")

    print('Please move tooltip straight down to cross lines. Then press \'Enter\'.')
    sys.stdin.read(1)

    current_posture_2 = None
    current_pose_2 = None
    if not self.offline :
      # store robot pose into storage folder
      current_posture_2 = self.move_group.get_current_joint_positions()
      current_pose_2 = self.move_group.motion_service.query_pose(self.move_group.name, current_posture_2, self.flange_link_name)
      with open(os.path.join(self.storage_folder, 'posture_2.p'), 'wb') as f:
          pickle.dump(current_posture_2, f)
      with open(os.path.join(self.storage_folder, 'pose_2.p'), 'wb') as f:
          pickle.dump(current_pose_2, f)
    else :
      # offline version: read posture and poses
      with open(os.path.join(self.storage_folder, 'posture_2.p'), 'rb') as f:
        current_posture_2 = pickle.load(f)
      with open(os.path.join(self.storage_folder, 'pose_2.p'), 'rb') as f:
        current_pose_2 = pickle.load(f)

    Hg = deepcopy(current_pose_2.transformation_matrix())
    cross_pose = deepcopy(current_pose_2.transformation_matrix())
    cross_pose[0][3] = cross_pos[0]
    cross_pose[1][3] = cross_pos[1]
    cross_pose[2][3] = cross_pos[2]
    print("cross_pose:")
    print(cross_pose)
    cross_pose_worldview = Pose.from_transformation_matrix(matrix=cross_pose, frame_id='world', normalize_rotation=False)
    self.addOrUpdatePose("cross_pose", "/Calibration", cross_pose_worldview)

    hand_tooltip = np.matmul(inv(Hg), cross_pose)

    print("Tooltip (grippertip) pose in flange coordinates (with same rotation as flange):")
    print(hand_tooltip)
    print("\n")
    np.save(os.path.join(self.storage_folder, 'tooltip_pose_in_flange_coordinates.npy'), hand_tooltip)

    print("To relocate the TCP in the 3D View, add a \'tcp_link\' to the file \'robotModel/main.xacro\' of your project folder.")
    print("As \'origin xyz\' of your tcp_link choose the translation vector of your calculated tooltip pose in flange coordinates.")
    print("More precisely, you have to add the following lines inside the <robot> </robot> environment: \n")
    print('<link name=\"tcp_link\" />')
    print('<joint name=\"tcp_joint_F\" type=\"fixed\">')
    print('  <child link=\"tcp_link\" />')
    print('  <parent link=\"e.g. arm_left_link_tool0\" />   <- adapt this to your endeffector link name')
    print('  <origin xyz=\"{:f} {:f} {:f}\" rpy=\"0 0 0\" />'.format(hand_tooltip[0][3], hand_tooltip[1][3], hand_tooltip[2][3]))
    print('</joint>')
    print("\n")
    print("Now, compile the new main.xacro.")
    print("Then, in the \'Configuration View\' once press compile to make the \'tcp_link\' selectable.")
    print("In the \'Configuration View\' under \'MotionPlanning\'->\'MoveIt!\'->\'groups\'->\'<group_name>\' select the new \'tcp_link\' as \'Tip link\'.")
    print("Moreover, under \'MotionPlanning\'->\'MoveIt!\'->\'endEffectors\'->\'<end effector name>\' select the new \'tcp_link\' as \'Parent link\'.")
    print("After compiling the new robot configuration, starting ROS, changing into the 'World View' and choosing the corresponding move group and end effector, the interactive marker appears at the new tcp link.")

    return True
    