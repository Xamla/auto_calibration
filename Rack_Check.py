# -*- coding: utf-8 -*-
"""
Check the occupancy status of a rack of pipette tips:
1.) Generate numbered ROIs for the slots in a completely occupied ground truth rack image
2.) Map the rack image of interest to the ground truth rack image via pattern matching
3.) Determine empty slots in the rack image by opencv blob detection and comparison with 
    the ROIs detected from the ground truth image
"""

# Standard imports
import numpy as np
import sys
#print(sys.path)
#import matplotlib.pyplot as plt
import cv2
import os
import math

from Rect import Rect
from PatternLocalisation import PatternLocalisation


# Generate numbered ROIs for the slots in a ground truth rack image
def generateNumberedROIs(image_path, calibration_path, rack_rows, rack_cols, save_ROI_table, save_ROI_img):

    # load image
    img = cv2.imread(image_path, cv2.IMREAD_GRAYSCALE)
    
    # undistort image
    stereo_calibration = np.load(calibration_path).item()
    leftCamMat = stereo_calibration["intrinsicLeftCam"]
    rightCamMat = stereo_calibration["intrinsicRightCam"]
    leftDistCoeffs = stereo_calibration["distLeftCam"]
    rightDistCoeffs = stereo_calibration["distRightCam"]
    img = cv2.undistort(src = img, cameraMatrix = rightCamMat, distCoeffs = rightDistCoeffs)

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()
    params.thresholdStep = 5
    params.minThreshold = 5
    params.maxThreshold = 100  
    params.minRepeatability = 3
    params.minDistBetweenBlobs = 1
    params.filterByColor = False
    params.blobColor = 0
    params.filterByArea = True
    params.minArea = 4000
    params.maxArea = 8000
    params.filterByCircularity = True
    params.minCircularity = 0.6
    params.maxCircularity = 10
    params.filterByInertia = False
    params.minInertiaRatio = 0.6
    params.maxInertiaRatio = 10
    params.filterByConvexity = True
    params.minConvexity = 0.8
    params.maxConvexity = 10
    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        detector = cv2.SimpleBlobDetector(params)
    else : 
        detector = cv2.SimpleBlobDetector_create(params)
    # Detect blobs.
    keypoints = detector.detect(img)
    print('Number of detected ground truth slots:')
    print(len(keypoints)) 
    # Draw detected blobs as red circles.
    im_with_keypoints = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #cv2.imshow("Keypoints", im_with_keypoints)
    #cv2.waitKey(1000)
    
    # Draw circular ROIs around keypoints:
    img_cr = cv2.imread(image_path)
    radius = int(round((keypoints[0].size / 2.0) * 1.4))
    i = 0
    while i < len(keypoints):
        x = int(round(keypoints[i].pt[0]))
        y = int(round(keypoints[i].pt[1]))
        cv2.circle(img = img_cr, center = (x, y), radius = radius,
                   color = (0,255,255), thickness = 1, lineType = cv2.LINE_AA)
        cv2.putText(img = img_cr, text = str(i), org = (x-5, y+5), 
                    fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.5, color = (0,255,255), 
                    thickness = 1, lineType = 8, bottomLeftOrigin = False)
        i += 1
    #cv2.imshow("Circular ROIs", img_cr)
    #cv2.waitKey(3000)
  
    # Sort keypoints in the order of their y-coordinate
    tmp = []
    i = 0
    while i < len(keypoints):
        tmp.append((i, keypoints[i].pt[1])) # generate a list of tuples
        i += 1
    def getKey(item):
        return item[1]
    indices_ySorted = sorted(tmp, key=getKey)
    keypoints_ySorted = detector.detect(img)
    i = 0
    while i < len(keypoints):
        idx = indices_ySorted[i][0]
        keypoints_ySorted[i] = keypoints[idx]
        i += 1
    # Draw y-sorted keypoints:
    img_cr_ySorted = cv2.imread(image_path)
    i = 0
    while i < len(keypoints_ySorted):
        x = int(round(keypoints_ySorted[i].pt[0]))
        y = int(round(keypoints_ySorted[i].pt[1]))
        cv2.circle(img = img_cr_ySorted, center = (x, y), radius = radius,
                   color = (0,255,255), thickness = 1, lineType = cv2.LINE_AA)
        cv2.putText(img = img_cr_ySorted, text = str(i), org = (x-5, y+5), 
                    fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.5, color = (0,255,255), 
                    thickness = 1, lineType = 8, bottomLeftOrigin = False)
        i += 1
    #cv2.imshow("Keypoints y-sorted", img_cr_ySorted)
    #cv2.waitKey(5000)
    
    # Sort every rack_cols keypoints in the order of their x-coordinate
    tmp2 = []
    tmp3 = []
    i = 0
    while i < len(keypoints_ySorted):
      if (i+1) % rack_cols != 0:
        tmp2.append((i, keypoints_ySorted[i].pt[0]))
      else:
        tmp2.append((i, keypoints_ySorted[i].pt[0]))
        tmp3.append(tmp2)
        tmp2 = []
      i += 1
    i = 0
    indices_xSorted = []
    while i < rack_rows: 
      indices_xSorted_oneRow = sorted(tmp3[i], key=getKey)
      indices_xSorted.append(indices_xSorted_oneRow)
      i += 1
    keypoints_Sorted = detector.detect(img)
    i = 0
    j = 0
    while j < rack_rows:
      i = 0
      while i < rack_cols:
        idx = indices_xSorted[j][i][0]
        keypoints_Sorted[j*rack_cols + i] = keypoints_ySorted[idx]
        i += 1
      j += 1
    # Draw sorted keypoints:
    img_cr_Sorted = cv2.imread(image_path)
    i = 0
    while i < len(keypoints_Sorted):
        x = int(round(keypoints_Sorted[i].pt[0]))
        y = int(round(keypoints_Sorted[i].pt[1]))
        cv2.circle(img = img_cr_Sorted, center = (x, y), radius = radius,
                   color = (0,255,255), thickness = 1, lineType = cv2.LINE_AA)
        cv2.putText(img = img_cr_Sorted, text = str(i), org = (x-5, y+5), 
                    fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.5, color = (0,255,255), 
                    thickness = 1, lineType = 8, bottomLeftOrigin = False)
        i += 1
    #cv2.imshow("Keypoints sorted", img_cr_Sorted)
    #cv2.waitKey(5000)

    # Generate a table with rectangular ROIs
    ROI_table = []
    width = keypoints_Sorted[0].size
    height = keypoints_Sorted[0].size
    #print("width:")
    #print(width)
    #print("height:")
    #print(height)
    tmp = Rect(0,0,0,0)
    i = 0
    while i < len(keypoints_Sorted):
      rect = tmp.fromCenterWidthHeight(keypoints_Sorted[i].pt[0], keypoints_Sorted[i].pt[1], width, height)
      ROI_table.append(rect)
      i += 1
    # Draw ROIs (as rectangles):
    ROI_rect_img = cv2.imread(image_path)
    i = 0
    while i < len(ROI_table):
        tmpX, tmpY = ROI_table[i].center()
        x = int(round(tmpX))
        y = int(round(tmpY))
        minX = int(round(ROI_table[i].minX))
        minY = int(round(ROI_table[i].minY))
        maxX = int(round(ROI_table[i].maxX))
        maxY = int(round(ROI_table[i].maxY))
        cv2.rectangle(img = ROI_rect_img, pt1 = (minX, minY), pt2 = (maxX, maxY), 
                      color = (0,255,255), thickness = 1, lineType = cv2.LINE_AA)
        cv2.putText(img = ROI_rect_img, text = str(i), org = (x-5, y+5), 
                    fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.5, color = (0,255,255), 
                    thickness = 1, lineType = 8, bottomLeftOrigin = False)
        i += 1
    #cv2.imshow("Ground truth image with rectangular ROIs", ROI_rect_img)
    #cv2.waitKey(3000)
    cv2.imwrite(save_ROI_img, ROI_rect_img)
    
    # Save sorted ROIs (as table of rectangles):
    np.save(save_ROI_table, ROI_table)
    
    
# Map a rack image to a ground truth rack image via pattern matching,
# i.e. via markers and cv.findHomography plus cv.warpPerspective
def mapToGroundTruthImage(image_path, image_path_gt, calibration_path, marker_rows, marker_cols, marker_point_dist,
                          marker_ids_as_string, patternDictionary, save_warped_img):

    # load image and ground truth image
    img = cv2.imread(image_path)
    img_gt = cv2.imread(image_path_gt)

    # undistort images
    stereo_calibration = np.load(calibration_path).item()
    leftCamMat = stereo_calibration["intrinsicLeftCam"]
    rightCamMat = stereo_calibration["intrinsicRightCam"]
    leftDistCoeffs = stereo_calibration["distLeftCam"]
    rightDistCoeffs = stereo_calibration["distRightCam"]
    img = cv2.undistort(src = img, cameraMatrix = leftCamMat, distCoeffs = leftDistCoeffs)
    img_gt = cv2.undistort(src = img_gt, cameraMatrix = rightCamMat, distCoeffs = rightDistCoeffs)
    
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
    patternLocalizer.setCamIntrinsics(leftCamMat)
    point_list, camPoseList = patternLocalizer.processImg(img)  
    patternLocalizer.setCamIntrinsics(rightCamMat)
    point_list_gt, camPoseList_gt = patternLocalizer.processImg(img_gt)
    #print("point_list:")
    #print(point_list)
    #print("camPoseList:")
    #print(camPoseList)
    #print("point_list_gt:")
    #print(point_list_gt)
    #print("camPoseList_gt:")
    #print(camPoseList_gt)

    # sort markers and markers_gt correctly
    markers = []
    markers_gt = []
    i = 0
    while i < len(marker_ids) : 
      current_marker_id = str(marker_ids[i])
      markers.append(point_list[current_marker_id])
      markers_gt.append(point_list_gt[current_marker_id])
      i += 1

    # concatenate points of all found markers
    srcPoints = markers[0]
    dstPoints = markers_gt[0]
    i = 1
    while i < len(markers) :
      srcPoints = np.concatenate((srcPoints, markers[i]), axis=0)
      dstPoints = np.concatenate((dstPoints, markers_gt[i]), axis=0)
      i += 1
    #print("srcPoints:")
    #print(srcPoints)
    #print("dstPoints:")
    #print(dstPoints)
    
    # calculate the homography matrix with help of the marker points found in both images
    H, status = cv2.findHomography( srcPoints = srcPoints, dstPoints = dstPoints ) #, method = cv.RANSAC}
    #print("Homography status:")
    #print(status)
    print("Homography matrix H (for mapping to ground truth image):")
    print(H)

    # warp the image via homography matrix, i.e. map it to the ground truth image
    warped = cv2.warpPerspective( src = img, M = H, dsize = ( img.shape[1], img.shape[0] ) )
    #cv2.imshow("warped image",  warped)
    #cv2.waitKey(3000)
    cv2.imwrite(save_warped_img, warped)


# Determine empty rack points (i.e. slots)
def detectEmptyRackPoints(image_path, ROI_table_path, save_empty_slots, save_warped_img_with_ROIs):

    # load warped image
    img = cv2.imread(image_path)

    # load table of ROIs
    ROIs = np.load(ROI_table_path)

    # Detect keypoints via SimpleBlobDetector
    circleFinderParams = cv2.SimpleBlobDetector_Params()
    circleFinderParams.thresholdStep = 5
    circleFinderParams.minThreshold = 5
    circleFinderParams.maxThreshold = 100
    circleFinderParams.minRepeatability = 3
    circleFinderParams.minDistBetweenBlobs = 1
    circleFinderParams.filterByColor = False
    circleFinderParams.blobColor = 0
    circleFinderParams.filterByArea = True
    circleFinderParams.minArea = 3000
    circleFinderParams.maxArea = 8000
    circleFinderParams.filterByCircularity = True
    circleFinderParams.minCircularity = 0.6
    circleFinderParams.maxCircularity = 10
    circleFinderParams.filterByInertia = False
    circleFinderParams.minInertiaRatio = 0.6
    circleFinderParams.maxInertiaRatio = 10
    circleFinderParams.filterByConvexity = True
    circleFinderParams.minConvexity = 0.8
    circleFinderParams.maxConvexity = 10
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        blobDetector = cv2.SimpleBlobDetector(circleFinderParams)
    else : 
        blobDetector = cv2.SimpleBlobDetector_create(circleFinderParams)
    # Find circles in image and ground truth image
    keypoints = blobDetector.detect(image=img)
    print("Number of detected slots in the image to be analyzed:")
    print(len(keypoints))

    # Draw keypoints:
    keypoint_img = cv2.drawKeypoints(img, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #cv2.imshow("Keypoints", keypoint_img)#
    #cv2.waitKey(3000)

    # Detect empty slots:
    slot_occupancy = []
    i = 0
    while i < len(ROIs) :
      slot_occupancy.append(False)
      j = 0
      while j < len(keypoints) :
        x = keypoints[j].pt[0]
        y = keypoints[j].pt[1]
        if ROIs[i].containsPt(x, y) :
          slot_occupancy[i] = True
        j += 1
      i += 1
    
    full = True
    empty_slots = []
    #print("The following slots are empty:")
    i = 0
    while i < len(ROIs) :
      if slot_occupancy[i] == False :
        #print(i)
        empty_slots.append(i)
        full = False
      i += 1
    if full == True :
      print("All slots are occupied.")
    print("Empty slots:")
    print(empty_slots)
    # Save list of empty slots
    np.save(save_empty_slots, empty_slots)

    # draw warped image with ROIs (only at occupied slots)
    occupied = []
    i = 0
    while i < len(ROIs) :
      occupied.append(True)
      i += 1
    i = 0
    while i < len(empty_slots) :
      idx = empty_slots[i]
      occupied[idx] = False
      i += 1

    img_with_ROIs = cv2.imread(image_path)
    i = 0
    while i < len(ROIs) :
      if occupied[i] != False :
        tmpX, tmpY = ROIs[i].center()
        x = int(round(tmpX))
        y = int(round(tmpY))
        minX = int(round(ROIs[i].minX))
        minY = int(round(ROIs[i].minY))
        maxX = int(round(ROIs[i].maxX))
        maxY = int(round(ROIs[i].maxY))
        cv2.rectangle( img = img_with_ROIs, pt1 = (minX, minY), pt2 = (maxX, maxY), 
                       color = (0,255,255), thickness = 1, lineType = cv2.LINE_AA )
        cv2.putText( img = img_with_ROIs, text = str(i), org = (x-5, y+5), 
                     fontFace = cv2.FONT_HERSHEY_SIMPLEX, fontScale = 0.5, color = (0,255,255), 
                     thickness = 1, lineType = 8, bottomLeftOrigin = False )
      i += 1
    #cv2.imshow("warped image with ROIs", img_with_ROIs)
    #cv2.waitKey(3000)
    cv2.imwrite(save_warped_img_with_ROIs, img_with_ROIs)



####################################### Testing the functions ####################################### 

img_path_gt = '/home/altrogge/Desktop/Pipettenspitzen_Erkennung/Rack_Images/Rack1/rightrack1_002.png'
img_path = '/home/altrogge/Desktop/Pipettenspitzen_Erkennung/Rack_Images/Rack1/leftrack1_003.png'
calibration_path = '/home/altrogge/Desktop/Pipettenspitzen_Erkennung/python_results/stereo_calibration_as_dict_python3.npy'
save_gt_ROI_img = '/home/altrogge/Desktop/Pipettenspitzen_Erkennung/python_results/GroundTruthImgWithROIs.png'
save_gt_ROI_table = '/home/altrogge/Desktop/Pipettenspitzen_Erkennung/python_results/ROIs.npy'
marker_rows = 3
marker_cols = 5
marker_point_dist = 0.003
marker_ids = '3,11'
save_warped_img = '/home/altrogge/Desktop/Pipettenspitzen_Erkennung/python_results/leftrack1_003_warped.png'
save_empty_slots = '/home/altrogge/Desktop/Pipettenspitzen_Erkennung/python_results/empty_slots.npy'
save_warped_img_with_ROIs = '/home/altrogge/Desktop/Pipettenspitzen_Erkennung/python_results/leftrack1_003_warped_with_ROIs.png'
patternDictionary = '/home/altrogge/Desktop/Pipettenspitzen_Erkennung/patDictData.npy'

print('Number of arguments:')
print(len(sys.argv))
print('Argument List:')
print(str(sys.argv))
if len(sys.argv) > 1 :
  img_path_gt = sys.argv[1]
if len(sys.argv) > 2 :
  img_path = sys.argv[2]
if len(sys.argv) > 3 :
  calibration_path = sys.argv[3]
if len(sys.argv) > 4 :
  save_gt_ROI_img = sys.argv[4]
if len(sys.argv) > 5 :
  save_gt_ROI_table = sys.argv[5]
if len(sys.argv) > 6 :
  marker_rows = int(sys.argv[6])
if len(sys.argv) > 7 :
  marker_cols = int(sys.argv[7])
if len(sys.argv) > 8 :
  marker_point_dist = float(sys.argv[8])
if len(sys.argv) > 9 :
  marker_ids = sys.argv[9]
if len(sys.argv) > 10 :
  save_warped_img = sys.argv[10]
if len(sys.argv) > 11 :
  save_empty_slots = sys.argv[11]
if len(sys.argv) > 12 :
  save_warped_img_with_ROIs = sys.argv[12]
if len(sys.argv) > 13 :
  patternDictionary = sys.argv[13]

generateNumberedROIs(img_path_gt, calibration_path, 8, 6, save_gt_ROI_table, save_gt_ROI_img)

mapToGroundTruthImage(img_path, img_path_gt, calibration_path, marker_rows, marker_cols, marker_point_dist,
                      marker_ids, patternDictionary, save_warped_img)

detectEmptyRackPoints(save_warped_img, save_gt_ROI_table, save_empty_slots, save_warped_img_with_ROIs)

print('finished')
