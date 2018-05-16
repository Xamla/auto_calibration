# Standard imports
import numpy as np
import sys
#print(sys.path)
#import matplotlib.pyplot as plt
import cv2
import os
import math

from PatternLocalisation import PatternLocalisation


img_path_left = '/home/altrogge/Desktop/Pipettenspitzen_Erkennung/Rack_Images/Rack1/leftrack1_003.png'
img_path_right = '/home/altrogge/Desktop/Pipettenspitzen_Erkennung/Rack_Images/Rack1/rightrack1_003.png'
calibration_path = '/home/altrogge/Desktop/Pipettenspitzen_Erkennung/python_results/stereo_calibration_as_dict_python3.npy'
marker_rows = 3
marker_cols = 5
marker_point_dist = 0.003
marker_ids_as_string = '3,11'
patternDictionary = '/home/altrogge/Desktop/Pipettenspitzen_Erkennung/patDictData.npy'

print('Number of arguments:')
print(len(sys.argv))
print('Argument List:')
print(str(sys.argv))
if len(sys.argv) > 1 :
  img_path_left = sys.argv[1]
if len(sys.argv) > 2 :
  img_path_right = sys.argv[2]
if len(sys.argv) > 3 :
  calibration_path = sys.argv[3]
if len(sys.argv) > 4 :
  marker_rows = int(sys.argv[4])
if len(sys.argv) > 5 :
  marker_cols = int(sys.argv[5])
if len(sys.argv) > 6 :
  marker_point_dist = float(sys.argv[6])
if len(sys.argv) > 7 :
  marker_ids_as_string = sys.argv[7]
if len(sys.argv) > 8 :
  patternDictionary = sys.argv[8]

# load left and right image
img_left = cv2.imread(img_path_left)
img_right = cv2.imread(img_path_right)

# load stereo_calibration
stereo_calibration = np.load(calibration_path).item()
leftCamMat = stereo_calibration["intrinsicLeftCam"]
rightCamMat = stereo_calibration["intrinsicRightCam"]
leftDistCoeffs = stereo_calibration["distLeftCam"]
rightDistCoeffs = stereo_calibration["distRightCam"]

# extract marker ids from string list
marker_ids = []
old_pos = 0
i = 0
strlen = len(marker_ids_as_string)
while i < strlen :
    pos = marker_ids_as_string.find(",", old_pos) # old_pos is start position for search
    if pos != -1 :
      id_string = marker_ids_as_string[old_pos:pos]
      id = int(id_string)
      marker_ids.append(id)
      old_pos = pos+1
      i = i+1
    else :
      id_string = marker_ids_as_string[old_pos:strlen]
      id = int(id_string)
      marker_ids.append(id)
      i = strlen
print("marker ids:")
print(marker_ids)

# search for circle patterns ('markers') in image and ground truth image
patternLocalizer = PatternLocalisation()
patternLocalizer.circleFinderParams.minArea = 500
patternLocalizer.circleFinderParams.maxArea = 4000
patternLocalizer.setPatternIDdictionary(np.load(patternDictionary))
patternLocalizer.setPatternData(marker_rows, marker_cols, marker_point_dist)

# Detection of a camera pose via solvePnP
img_left_undistorted = cv2.undistort(src = img_left.copy(), cameraMatrix = leftCamMat, distCoeffs = leftDistCoeffs)
img_right_undistorted = cv2.undistort(src = img_right.copy(), cameraMatrix = rightCamMat, distCoeffs = rightDistCoeffs)
patternLocalizer.setCamIntrinsics(leftCamMat)
point_list_left, camPoseList_left = patternLocalizer.processImg(img_left_undistorted)  
patternLocalizer.setCamIntrinsics(rightCamMat)
point_list_right, camPoseList_right = patternLocalizer.processImg(img_right_undistorted)

# Detection of a camera pose via plane fit
patternLocalizer.setStereoCalibration(stereo_calibration)
camPoseList_left_viaPlaneFit = patternLocalizer.calcCamPoseViaPlaneFit(img_left, img_right, "left", False)
camPoseList_right_viaPlaneFit = patternLocalizer.calcCamPoseViaPlaneFit(img_left, img_right, "right", False)

# Output for comparison
i = 0
while i < len(marker_ids) :
  print("camPoseList_left['{}']:".format(marker_ids[i]))
  print(camPoseList_left[str(marker_ids[i])])
  print("camPoseList_left_viaPlaneFit['{}']:".format(marker_ids[i]))
  print(camPoseList_left_viaPlaneFit[str(marker_ids[i])])
  print("camPoseList_right['{}']:".format(marker_ids[i]))
  print(camPoseList_right[str(marker_ids[i])])
  print("camPoseList_right_viaPlaneFit['{}']:".format(marker_ids[i]))
  print(camPoseList_right_viaPlaneFit[str(marker_ids[i])])
  i += 1

print('finished')
