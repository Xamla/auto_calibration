# -*- coding: utf-8 -*-
"""
Pattern Detector.

Find a circle pattern via OpenCV's findCirclesGrid and
determine the pattern pose relative to the camera via 
solvePnP or PlaneFit.
"""

# Standard imports
import numpy as np
import sys
import cv2
import os
import math
from numpy.linalg import inv
from numpy.linalg import norm

class PatternLocalisation:
  def __init__(self):
    self.patDictData = {}
    self.pattern = {}
    self.pattern["width"] = 1
    self.pattern["height"] = 1
    self.pattern["pointDist"] = 0
    self.generateDefaultCircleFinderParams()
    self.camIntrinsics = None
    self.stereoCalibration = None
    self.debugParams = {"circleSearch": False, "circlePatternSearch": False, "pose": False}


  def setPatternData(self, width, height, pointDist):
    self.pattern["width"] = width
    self.pattern["height"] = height
    self.pattern["pointDist"] = pointDist


  def setPatternIDdictionary(self, dict):
    self.patDictData = dict


  def setCamIntrinsics(self, camCalib):
    self.camIntrinsics = camCalib


  # used for camera pose calculation via plane fit
  def setStereoCalibration(self, stereoCalib):
    self.stereoCalibration = stereoCalib


  def generateDefaultCircleFinderParams(self):
    self.circleFinderParams = cv2.SimpleBlobDetector_Params()
    self.circleFinderParams.thresholdStep = 5
    self.circleFinderParams.minThreshold = 60
    self.circleFinderParams.maxThreshold = 230
    self.circleFinderParams.minRepeatability = 3
    self.circleFinderParams.minDistBetweenBlobs = 5
    self.circleFinderParams.filterByColor = False
    self.circleFinderParams.blobColor = 0
    self.circleFinderParams.filterByArea = True  # area of the circle in pixels
    self.circleFinderParams.minArea = 200
    self.circleFinderParams.maxArea = 1000
    self.circleFinderParams.filterByCircularity = True
    self.circleFinderParams.minCircularity = 0.6
    self.circleFinderParams.maxCircularity = 10
    self.circleFinderParams.filterByInertia = False
    self.circleFinderParams.minInertiaRatio = 0.6
    self.circleFinderParams.maxInertiaRatio = 10
    self.circleFinderParams.filterByConvexity = True
    self.circleFinderParams.minConvexity = 0.8
    self.circleFinderParams.maxConvexity = 10


  def grayToRGB(self, inputImg):
    if inputImg.shape[2] == 3 :
      return inputImg
    else :
      img = cv2.cvtColor(src = inputImg, code = cv2.COLOR_GRAY2RGB)
      return img


  # Determine a list of all circles in the image
  def findCircles(self, inputImg, doDebug):
    # Setup SimpleBlobDetector parameters.
    # See https://www.learnopencv.com/blob-detection-using-opencv-python-c/ for parameter overview

    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        blobDetector = cv2.SimpleBlobDetector(self.circleFinderParams)
    else : 
        blobDetector = cv2.SimpleBlobDetector_create(self.circleFinderParams)

    resultsTmp = blobDetector.detect(image=inputImg)
    results = []
    i = 0
    while i < len(resultsTmp) :
      results.append({"radius": resultsTmp[i].size/2.0,
                      "angle": resultsTmp[i].angle,
                      "pos": (resultsTmp[i].pt[0], resultsTmp[i].pt[1], 1) })
      i += 1

    circleScale = 16
    shiftBits = 4
    
    if doDebug == True :
      width = int(inputImg.shape[1])
      height = int(inputImg.shape[0])

      imgScale = cv2.resize(inputImg, (width, height))
      imgScale = self.grayToRGB(imgScale)
      cv2.imshow("Scaled Image", imgScale)
      cv2.waitKey(1000)
      print(imgScale.shape)
      i = 0
      while i < len(results) :
        x = int(round(results[i]["pos"][0]*circleScale))
        y = int(round(results[i]["pos"][1]*circleScale))
        radius = int(round(results[i]["radius"]*circleScale))
        cv2.circle(img = imgScale, center = (x, y), radius = radius, color = (0,255,255), 
                   thickness = 2, lineType = cv2.LINE_AA, shift = shiftBits)
        i += 1
      cv2.imshow("circleFinder", imgScale)
      cv2.waitKey(3000)
      cv2.destroyWindow(winname = "circleFinder")

    return results


  # Determine a point list of all circle patterns in the image
  def findCirclePatterns(self, camImgUndist, doDebug) :
    imgMasked = camImgUndist
    # Create a detector with the parameters
    ver = (cv2.__version__).split('.')
    if int(ver[0]) < 3 :
        blobDetector = cv2.SimpleBlobDetector(self.circleFinderParams)
    else : 
        blobDetector = cv2.SimpleBlobDetector_create(self.circleFinderParams)
    found = True
    point_list = []
    hull_list = []
    while found == True :
      found, points = cv2.findCirclesGrid( image = imgMasked,
                                           patternSize = ( self.pattern["width"], self.pattern["height"] ), 
                                           flags = cv2.CALIB_CB_ASYMMETRIC_GRID + cv2.CALIB_CB_CLUSTERING, 
                                           blobDetector = blobDetector )
      if found == True :
        points = np.squeeze(points, axis=1)
        point_list.append(points)
        mins = np.amin(points, axis=0)
        maxs = np.amax(points, axis=0)
        # Mask image to not see the already detected pattern any more 
        hull = np.array([ [mins[0], mins[1]], [mins[0], maxs[1]], [maxs[0], maxs[1]], [maxs[0], mins[1]] ], np.int32)
        cv2.fillConvexPoly(imgMasked, hull, (255,255,255))
        hull_list.append(hull)
        if doDebug == True :
          cv2.imshow("masked image", imgMasked)
          cv2.waitKey(1000)
    return point_list


  # Determine the ID of a circle pattern found in the input image
  def getPatternId(self, imgInput, points) :
    imgGray = cv2.cvtColor(src = imgInput, code = cv2.COLOR_RGB2GRAY)
    nPoints = self.pattern["width"] * self.pattern["height"]
    idRef = []
    idPoints = []

    idRef.append(points[int(math.floor(nPoints/2.0 - 2*self.pattern["width"] -1))])  # dark ref. point, top left
    idRef.append(points[int(math.floor(nPoints/2.0 + 2*self.pattern["width"] +1))])  # dark ref. point, bottom right
    idRef.append(points[int(math.floor(nPoints/2.0 - 2*self.pattern["width"] +1))])  # light ref. point, bottom left
    idRef.append(points[int(math.floor(nPoints/2.0 + 1*self.pattern["width"] +1))])  # light ref. point, center bottom left
    idRef.append(points[int(math.floor(nPoints/2.0 + 2*self.pattern["width"] -1))])  # light ref. point, top left

    # sorted from least significant bit (1) to most significant bit (10)
    idPoints.append(points[int(math.floor(nPoints/2.0 + 2*self.pattern["width"] -0))])  # bit 1
    idPoints.append(points[int(math.floor(nPoints/2.0 - 2*self.pattern["width"] -0))])  # bit 2
    idPoints.append(points[int(math.floor(nPoints/2.0 - 0*self.pattern["width"] -1))])  # bit 3
    idPoints.append(points[int(math.floor(nPoints/2.0 - 0*self.pattern["width"] +1))])  # bit 4
    idPoints.append(points[int(math.floor(nPoints/2.0 + 1*self.pattern["width"] -1))])  # bit 5
    idPoints.append(points[int(math.floor(nPoints/2.0 - 1*self.pattern["width"] -0))])  # bit 6
    idPoints.append(points[int(math.floor(nPoints/2.0 - 1*self.pattern["width"] -1))])  # bit 7
    idPoints.append(points[int(math.floor(nPoints/2.0 - 0*self.pattern["width"] -0))])  # bit 8 (central dot)
    idPoints.append(points[int(math.floor(nPoints/2.0 + 1*self.pattern["width"] -0))])  # bit 9
    idPoints.append(points[int(math.floor(nPoints/2.0 - 1*self.pattern["width"] +1))])  # bit 10

    darkColor  = (self.getPatPointCenterColor(imgGray, idRef[0]) + self.getPatPointCenterColor(imgGray, idRef[1])) / 2.0
    lightColor = (self.getPatPointCenterColor(imgGray, idRef[2]) + self.getPatPointCenterColor(imgGray, idRef[3])
                  + self.getPatPointCenterColor(imgGray, idRef[4])) / 3.0

    darkThresh = darkColor + (lightColor-darkColor)/3.0
    lightThresh = lightColor - (lightColor-darkColor)/3.0
    if lightColor-darkColor < 10 :
      print("ERROR: Overall point ID contrast to low")
      return -1

    patNum = 0
    i = 0
    while i < len(idPoints) :
      pointColor = self.getPatPointCenterColor(imgGray, idPoints[i])
      if pointColor > lightThresh :
        patNum = patNum + (1 << i)
      elif pointColor < darkThresh :
        # do nothing because bit is already set to 0
        not_used = 0
      else :
        print("--- WARNING: low contrast ---")
        if pointColor > (darkColor + (lightColor-darkColor)/2.0) :
          patNum = patNum + (1 << i)
          print("bit identified as 1")
        else :
          print("bit identified as 0")
      i += 1
    
    id, err = self.getID(self.patDictData, patNum)
    return id, err, patNum


  def getPatPointCenterColor(self, imgGray, point) :
    pointColor = 0.0
    floor0 = int(math.floor(point[0]))
    floor1 = int(math.floor(point[1]))
    ceil0  = int(math.ceil(point[0]))
    ceil1  = int(math.ceil(point[1]))
    c1 = int(imgGray[floor1-1][floor0-1])
    c2 = int(imgGray[floor1-1][ceil0-1])
    c3 = int(imgGray[ceil1-1][floor0-1])
    c4 = int(imgGray[ceil1-1][ceil0-1])
    pointColor = c1
    pointColor = pointColor + c2
    pointColor = pointColor + c3
    pointColor = pointColor + c4
    pointColor = pointColor / 4.0
    return pointColor


  def getID(self, dict, pat) :
    id = 0
    err = 0
    if dict[pat] >= 0 and dict[pat] < 1024 :  # no error
      id = dict[pat]
      err = 0
    elif dict[pat] >= 1024 :  # one bit error
      id = dict[pat] - 1024
      err = 1
    else : # two bit errors, can't correct
      id = -1
      err = 2
    # if more then 2 bits are defect, a wrong ID is returned
    # because the coding can't handle more then 2 bit errors
    return id, err


  # Generate ground truth circle center points of the calibration pattern.
  # Z is set to 0 for all points.
  def generatePatternPoints(self, pointsX, pointsY, pointSize) :
    # calculates the groundtruth x, y, z positions of the points of the asymmetric circle pattern
    corners = np.zeros((pointsX * pointsY, 3))
    i = 0
    y = 0
    while y < pointsY :
      x = 0
      while x < pointsX :
        corners[i][0] = (2*x + y%2) * pointSize
        corners[i][1] = y * pointSize
        corners[i][2] = 0
        i += 1
        x += 1
      y += 1
    return corners


  # Calculate the camera pose relative to the pattern via solvePnP.
  def calcCamPose(self, id_list, points_with_ids, doDebug, debugImg) :
    camPoseList = {}
    points3d = self.generatePatternPoints(self.pattern["width"], self.pattern["height"], self.pattern["pointDist"])
    i = 0
    while i < len(id_list) :
      current_id = str(id_list[i])
      poseFound, poseCamRotVector, poseCamTrans = cv2.solvePnP( objectPoints=points3d, 
                                                                imagePoints=points_with_ids[current_id],
                                                                cameraMatrix=self.camIntrinsics, 
                                                                distCoeffs=np.zeros((5,1)) )
      poseCamRotMatrix, jacobian = cv2.Rodrigues(poseCamRotVector)
      camPoseFinal=np.zeros((4,4))
      j = 0
      while j < 3 :
        k = 0
        while k < 3:
          camPoseFinal[j][k] = poseCamRotMatrix[j][k]
          k += 1
        j += 1
      camPoseFinal[0][3] = poseCamTrans[0]
      camPoseFinal[1][3] = poseCamTrans[1]
      camPoseFinal[2][3] = poseCamTrans[2]
      camPoseFinal[3][3] = 1.0
      camPoseList[current_id] = camPoseFinal
      i += 1
      if doDebug and poseFound :
        imgShow = self.grayToRGB(debugImg)
        cv2.drawChessboardCorners(imgShow, patternSize=(self.pattern["width"], self.pattern["height"]), corners=points_with_ids[current_id], patternWasFound=poseFound)
        cv2.imshow("camPoseDebug", imgShow)
        cv2.waitKey(1000)
    return camPoseList


  # Calculate the camera poses relative to all patterns in the images via plane fit.
  # Note1: The stereo calibration has to be set first, ie. patternLocalizer.setStereoCalibration(stereo_calibration).
  # Note2: A left and right image, captured by the stereo camera system, are needed as input.
  #        Both images have to be original, i.e. not undistorted!
  def calcCamPoseViaPlaneFit(self, imgLeft, imgRight, whichCam, doDebug) :
    stereoCalibration = self.stereoCalibration
    leftCamMat = stereoCalibration["intrinsicLeftCam"]
    rightCamMat = stereoCalibration["intrinsicRightCam"]
    leftDistCoeffs = stereoCalibration["distLeftCam"]
    rightDistCoeffs = stereoCalibration["distRightCam"]
    rightLeftCamTrafo = stereoCalibration["trafoLeftCamRightCam"]

    # Stereo Rectify:
    R = rightLeftCamTrafo[:3,:3]
    T = rightLeftCamTrafo[:3,3:4]
    leftR = np.zeros((3, 3), np.float64)
    rightR = np.zeros((3, 3), np.float64)
    leftP = np.zeros((3, 4), np.float64)
    rightP = np.zeros((3, 4), np.float64)
    Q = np.zeros((4, 4), np.float64)

    cv2.stereoRectify ( cameraMatrix1 = leftCamMat.astype(np.float64),
                        distCoeffs1 = leftDistCoeffs.astype(np.float64),
                        cameraMatrix2 = rightCamMat.astype(np.float64),
                        distCoeffs2 = rightDistCoeffs.astype(np.float64),
                        imageSize = (imgLeft.shape[1], imgLeft.shape[0]),
                        R = R.astype(np.float64),
                        T = T.astype(np.float64),
                        R1 = leftR,
                        R2 = rightR,
                        P1 = leftP,
                        P2 = rightP,
                        Q = Q,
                        flags = 0 )

    # Undistortion + rectification:
    mapAImgLeft, mapBImgLeft = cv2.initUndistortRectifyMap ( cameraMatrix = leftCamMat.astype(np.float64),
                                                             distCoeffs = leftDistCoeffs.astype(np.float64),
                                                             R = leftR,
                                                             newCameraMatrix = leftP,
                                                             size = (imgLeft.shape[1], imgLeft.shape[0]),
                                                             m1type = cv2.CV_32FC1 )
    imgLeftRectUndist = cv2.remap (src = imgLeft, map1 = mapAImgLeft, map2 = mapBImgLeft, interpolation = cv2.INTER_NEAREST)


    mapAImgRight, mapBImgRight = cv2.initUndistortRectifyMap ( cameraMatrix = rightCamMat.astype(np.float64),
                                                               distCoeffs = rightDistCoeffs.astype(np.float64),
                                                               R = rightR,
                                                               newCameraMatrix = rightP,
                                                               size = (imgRight.shape[1], imgRight.shape[0]),
                                                               m1type = cv2.CV_32FC1 )
    imgRightRectUndist = cv2.remap (src = imgRight, map1 = mapAImgRight, map2 = mapBImgRight, interpolation = cv2.INTER_NEAREST)

    if doDebug == True :
      cv2.imshow("imgLeftRectUndist", imgLeftRectUndist)
      cv2.imshow("imgRightRectUndist", imgRightRectUndist)
      cv2.waitKey(3000)

    # Determine a point list of all circle patterns in the left/right image
    pointListLeft = self.findCirclePatterns(imgLeftRectUndist.copy(), self.debugParams["circlePatternSearch"])
    pointListRight = self.findCirclePatterns(imgRightRectUndist.copy(), self.debugParams["circlePatternSearch"])

    # Determine the ids of all found circle patterns
    id_list_left = []
    id_list_right = []
    pointsLeft_with_ids = {}
    pointsRight_with_ids = {}
    i = 0
    while i < len(pointListLeft) :
      id_left, err_left, patNum_left = self.getPatternId(imgLeftRectUndist.copy(), pointListLeft[i])
      id_right, err_right, patNum_right = self.getPatternId(imgRightRectUndist.copy(), pointListRight[i])
      id_list_left.append(id_left)
      id_list_right.append(id_right)
      pointsLeft_with_ids[str(id_left)] = pointListLeft[i]
      pointsRight_with_ids[str(id_right)] = pointListRight[i]
      i += 1
    if doDebug == True :
      print("id_list_left:")
      print(id_list_left)
      print("id_list_right:")
      print(id_list_right)
      print("pointsLeft_with_ids:")
      print(pointsLeft_with_ids)
      print("pointsRight_with_ids:")
      print(pointsRight_with_ids)

    # Loop over all found patterns:
    camPoseList = {}
    pattern_count = 0
    while pattern_count < len(id_list_left) :
      current_id = str(id_list_left[pattern_count])
      # Save 2d grid points.
      circlesGridPointsLeft = pointsLeft_with_ids[current_id]
      circlesGridPointsRight = pointsRight_with_ids[current_id]
      nPoints = self.pattern["width"] * self.pattern["height"]
      leftProjPoints = np.zeros((2, nPoints), np.float64) # 2D coordinates (x,y) x #Points
      rightProjPoints = np.zeros((2, nPoints), np.float64)
      i = 0
      while i < nPoints :
        leftProjPoints[0][i] = circlesGridPointsLeft[i][0]
        leftProjPoints[1][i] = circlesGridPointsLeft[i][1]
        rightProjPoints[0][i] = circlesGridPointsRight[i][0]
        rightProjPoints[1][i] = circlesGridPointsRight[i][1]
        i += 1

      resulting4DPoints = np.zeros((4, nPoints), np.float64)

      # TriangulatePoints:
      cv2.triangulatePoints (leftP, rightP, leftProjPoints, rightProjPoints, resulting4DPoints)

      resulting3DPoints = np.zeros((nPoints, 3), np.float64)
      i = 0
      while i < nPoints :
        resulting3DPoints[i][0] = resulting4DPoints[0][i] / resulting4DPoints[3][i]
        resulting3DPoints[i][1] = resulting4DPoints[1][i] / resulting4DPoints[3][i]
        resulting3DPoints[i][2] = resulting4DPoints[2][i] / resulting4DPoints[3][i]
        i += 1

      pointsInCamCoords = np.zeros((nPoints, 3), np.float64)
      if whichCam == "left" :
        i = 0
        while i < nPoints :
          pointsInCamCoords[i] = inv(leftR).dot(resulting3DPoints[i])
          i += 1
      else :
        i = 0
        while i < nPoints :
          pointsInCamCoords[i] = inv(rightR).dot(resulting3DPoints[i])
          i += 1
      if doDebug == True :
        print("pointsInCamCoords:")
        print(pointsInCamCoords)

      # Generate plane through detected 3d points to get the transformation
      # of the pattern into the coordinatesystem of the camera:
      ######################################################################
      # Plane fit with pseudo inverse:
      A = np.zeros((nPoints, 3), np.float64) # 2-dim. tensor of size 15 x 3
      b = np.zeros((nPoints), np.float64)    # 1-dim. tensor of size 15
      i = 0
      while i < nPoints :
        A[i][0] = pointsInCamCoords[i][0]
        A[i][1] = pointsInCamCoords[i][1]
        A[i][2] = 1.0
        b[i] = pointsInCamCoords[i][2]
        i += 1
      # Calculate x = A^-1 * b = (A^T A)^-1 * A^T * b (with pseudo inverse of A)
      At = A.transpose() #A:transpose(1, 2)
      x = np.matmul(inv(np.matmul(At, A)), At).dot(b)

      # Determine z-axis as normal on plane:
      n = np.zeros((3), np.float64)
      n[0] = x[0]
      n[1] = x[1]
      n[2] = -1.0
      z_unit_vec = n / norm(n)

      # Determine x-axis along left boundary points of the pattern
      x_direction = pointsInCamCoords[self.pattern["width"]-1] - pointsInCamCoords[0]
      x_unit_vec = x_direction / norm(x_direction)

      # Determine y-axis along top boundary points of the pattern:
      y_direction = pointsInCamCoords[nPoints-self.pattern["width"]] - pointsInCamCoords[0]
      y_unit_vec = y_direction / norm(y_direction)

      # Check, whether the normal vector z_unit_vec points into the correct direction.
      cross_product = np.cross(x_unit_vec, y_unit_vec)
      cross_product = cross_product / norm(cross_product)
      if z_unit_vec.dot(cross_product) < 0.0 :
        z_unit_vec *= -1.0

      # x_unit_vec.dot(z_unit_vec) has to be zero.
      # Map x_unit_vec onto plane.
      new_x_unit_vec = x_unit_vec - x_unit_vec.dot(z_unit_vec) * z_unit_vec
      if new_x_unit_vec.dot(x_unit_vec) < 0.0 :
        new_x_unit_vec *= -1.0

      # Determine new y_unit_vec as cross product of x_unit_vec and z_unit_vec:
      new_y_unit_vec = np.cross(new_x_unit_vec, z_unit_vec)
      if new_y_unit_vec.dot(y_unit_vec) < 0.0 :
        new_y_unit_vec *= -1.0

      # Transform pattern coordinate system into camera coordinate system:
      # M_B->A = (x_unit_vec, y_unit_vec, z_unit_vec, support vector)
      camPoseFinal = np.zeros((4, 4), np.float64)
      camPoseFinal[0][0] = new_x_unit_vec[0]
      camPoseFinal[1][0] = new_x_unit_vec[1]
      camPoseFinal[2][0] = new_x_unit_vec[2]
      camPoseFinal[0][1] = new_y_unit_vec[0]
      camPoseFinal[1][1] = new_y_unit_vec[1]
      camPoseFinal[2][1] = new_y_unit_vec[2]
      camPoseFinal[0][2] = z_unit_vec[0]
      camPoseFinal[1][2] = z_unit_vec[1]
      camPoseFinal[2][2] = z_unit_vec[2]
      camPoseFinal[0][3] = pointsInCamCoords[0][0]
      camPoseFinal[1][3] = pointsInCamCoords[0][1]
      camPoseFinal[2][3] = pointsInCamCoords[0][2]
      camPoseFinal[3][3] = 1
      if doDebug == True :
        print("camPoseFinal:")
        print(camPoseFinal)
      camPoseList[current_id] = camPoseFinal
      pattern_count += 1

    return camPoseList


  # Determine a point list of all circle patterns in the image, together with the corresponding
  # pattern IDs. Moreover, if an intrinsic camera matrix has been set, calculate the camera poses
  # relative to all found circle patterns
  # Note: The given "inputImg" should be an undistorted image!
  def processImg(self, inputImg):
    camImgUndist = inputImg.copy()

    # Determine a list of all circles in the image
    #circleList = self.findCircles(camImgUndist, self.debugParams["circleSearch"])

    # Determine a point list of all circle patterns in the image
    point_list = self.findCirclePatterns(camImgUndist, self.debugParams["circlePatternSearch"])   

    # Determine the ids of all found circle patterns
    camImgUndist = inputImg.copy()
    id_list = []
    points_with_ids = {}
    i = 0
    while i < len(point_list) :
      id, err, patNum = self.getPatternId(camImgUndist, point_list[i])
      id_list.append(id)
      points_with_ids[str(id)] = point_list[i]
      i += 1    

    # If camera intrinsics are given, determine the camera pose relative to all found circle patterns
    # Otherwise, directly return the list of markers (i.e. pattern points) with ids.
    if self.camIntrinsics is None :
      return points_with_ids, None
    else : 
      camPoseList = self.calcCamPose(id_list, points_with_ids, self.debugParams["pose"], camImgUndist)         
      return points_with_ids, camPoseList

