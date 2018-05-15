# -*- coding: utf-8 -*-
"""
Pattern Detector.

Find a circle pattern via OpenCV's findCirclesGrid and
determine the pattern pose relative to the camera via 
solvePnP.
"""

# Standard imports
import numpy as np
import sys
#print(sys.path)
#import matplotlib.pyplot as plt
import cv2
import os
import math
#from pyflann import *  ## sudo pip install pyflann


class PatternLocalisation:
  def __init__(self):
    self.patDictData = {}
    self.pattern = {}
    self.pattern["width"] = 1
    self.pattern["height"] = 1
    self.pattern["pointDist"] = 0
    self.generateDefaultCircleFinderParams()
    self.camIntrinsics = None
    self.debugParams = {"circleSearch": False, "circlePatternSearch": False, "pose": False}


  def setPatternData(self, width, height, pointDist):
    self.pattern["width"] = width
    self.pattern["height"] = height
    self.pattern["pointDist"] = pointDist


  def setPatternIDdictionary(self, dict):
    self.patDictData = dict


  def setCamIntrinsics(self, camCalib):
    self.camIntrinsics = camCalib


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
      #cv2.imshow("Scaled Image", imgScale)
      #cv2.waitKey(3000)
      #print(imgScale.shape)
      i = 0
      while i < len(results) : #for key,val in ipairs(results) do
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
          cv2.waitKey(2000)
    return point_list


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
        #print("low threshold="..darkThresh..", high threshold="..lightThresh)
        #print("detected value="..pointColor)
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
