--[[
  PatternLocalisation.lua

  Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
--]]

local cv = require "cv"
cv.flann = require "cv.flann"
require "cv.highgui"
require "cv.videoio"
require "cv.imgproc"
require "cv.calib3d"
require "cv.imgcodecs"
require "cv.features2d"


local autocal = require 'auto_calibration.env'
local PatternLocalisation = torch.class("PatternLocalisation", autocal)


function PatternLocalisation:__init()
  self.patDictData = {}

  self.pattern = {}
  self.pattern.width = 1
  self.pattern.height = 1
  self.pattern.pointDist = 0

  self.colorTab = {  -- list of colors for visualisation
    {255,0,0}, {255,170,0}, {95,204,41}, {0,102,153}, {180,61,204}, {204,0,0}, {153,102,0}, {42,255,0}, {0,127,255}, {255,0,255},
    {255,128,128}, {255,204,102}, {0,153,51}, {0,77,153}, {153,0,128}, {255,106,77}, {255,213,0}, {0,255,128}, {0,85,255}, {255,102,229},
    {153,77,61}, {153,128,0}, {61,204,133}, {0,51,153}, {255,0,170}, {255,85,0}, {255,255,0}, {0,255,212}, {102,136,204}, {153,77,128},
    {153,51,0}, {255,255,128}, {0,153,128}, {0,0,255}, {153,0,77}, {255,153,102}, {153,153,77}, {0,255,255}, {0,0,204}, {255,128,191},
    {255,128,0}, {173,204,20}, {0,204,204}, {153,102,255}, {255,0,85}, {153,77,0}, {170,255,0}, {0,213,255}, {170,0,255}, {255,77,136},
    {255,179,102}, {112,153,31}, {0,128,153}, {102,0,153}, {153,0,26}, {153,115,77}, {191,255,128}, {0,170,255}, {122,61,153}, {153,77,89}
  }

  self:generateDefaultCircleFinderParams()

  self.camIntrinsics = nil
  self.stereoCalibration = nil

  self.debugParams = {
    circleSearch = false,
    clustering = false,
    pose = false
  }

  -- parameters for DBScan
  self.DBScanEps = 80 -- defines the size of the point neighbourhood
  self.DBScanMinPts = 6 -- minimum number of points required to form a dense region

  -- constants, used by DBScan
  self.clusterStatus = {
    kUnprocessed = 0,
    kProcessed = 1,
    kNoise = 2,
    kAssigned = 4
  }

  -- rescaling factor for visualisation
  self.imgShowRescale = 0.5
end


function PatternLocalisation:setDBScanParams(eps, minPts)
  self.DBScanEps = eps
  self.DBScanMinPts = minPts
end


function PatternLocalisation:setPatternData(width, height, pointDist)
  self.pattern.width = width
  self.pattern.height = height
  self.pattern.pointDist = pointDist
end


function PatternLocalisation:setPatternIDdictionary(dict)
  self.patDictData = dict
end


function PatternLocalisation:setCamIntrinsics(camCalib)
  self.camIntrinsics = camCalib
end


function PatternLocalisation:setStereoCalibration(stereoCalib)
  self.stereoCalibration = stereoCalib
end


function PatternLocalisation:generateDefaultCircleFinderParams()
  self.circleFinderParams = cv.SimpleBlobDetector_Params {}
  self.circleFinderParams.thresholdStep = 5
  self.circleFinderParams.minThreshold = 10
  self.circleFinderParams.maxThreshold = 230
  self.circleFinderParams.minRepeatability = 3
  self.circleFinderParams.minDistBetweenBlobs = 5
  self.circleFinderParams.filterByColor = false
  self.circleFinderParams.blobColor = 0
  self.circleFinderParams.filterByArea = true -- area of the circle in pixels
  self.circleFinderParams.minArea = 200
  self.circleFinderParams.maxArea = 1000
  self.circleFinderParams.filterByCircularity = true
  self.circleFinderParams.minCircularity = 0.6
  self.circleFinderParams.maxCircularity = 10
  self.circleFinderParams.filterByInertia = false
  self.circleFinderParams.minInertiaRatio = 0.6
  self.circleFinderParams.maxInertiaRatio = 10
  self.circleFinderParams.filterByConvexity = true
  self.circleFinderParams.minConvexity = 0.8
  self.circleFinderParams.maxConvexity = 10
end


function PatternLocalisation:plotCrosshair(img, posX, posY, crossSize)
  local irSize = {x = img:size()[2], y = img:size()[1]} -- to get x, y img size

  local lineLength = crossSize or 6
  local centerX = posX
  local centerY = posY

  cv.line {
    img,
    {(centerX) - 2, (centerY)},
    {(centerX) - 2 - lineLength, (centerY)},
    {0, 0, 0}
  }
  cv.line {
    img,
    {(centerX) + 2, (centerY)},
    {(centerX) + 2 + lineLength, (centerY)},
    {0, 0, 0}
  }

  cv.line {
    img,
    {(centerX), (centerY) - 2},
    {(centerX), (centerY) - 2 - lineLength},
    {0, 0, 0}
  }
  cv.line {
    img,
    {(centerX), (centerY) + 2},
    {(centerX), (centerY) + 2 + lineLength},
    {0, 0, 0}
  }

  for i = -1, 1, 2 do
    cv.line {
      img,
      {(centerX) - 4, (centerY) + i},
      {(centerX) - 2 - lineLength, (centerY) + i},
      {250, 200, 200}
    }
    cv.line {
      img,
      {(centerX) + 4, (centerY) + i},
      {(centerX) + 2 + lineLength, (centerY) + i},
      {250, 200, 200}
    }

    cv.line {
      img,
      {(centerX) + i, (centerY) - 4},
      {(centerX) + i, (centerY) - 2 - lineLength},
      {250, 200, 200}
    }
    cv.line {
      img,
      {(centerX) + i, (centerY) + 4},
      {(centerX) + i, (centerY) + 2 + lineLength},
      {250, 200, 200}
    }
  end
end


-- Generate ground truth circle center points of the calibration pattern.
-- Z is set to 0 for all points.
function PatternLocalisation:generatePatternPoints(pointsX, pointsY, pointSize)
  -- calculates the groundtruth x, y, z positions of the points of the asymmetric circle pattern
  local corners = torch.FloatTensor(pointsX * pointsY, 1, 3):zero()
  local i = 1
  for y = 1, pointsY do
    for x = 1, pointsX do
      corners[i][1][1] = (2 * (x - 1) + (y - 1) % 2) * pointSize
      corners[i][1][2] = (y - 1) * pointSize
      corners[i][1][3] = 0
      i = i + 1
    end
  end
  return corners
end


function PatternLocalisation:rotVectorToMat3x3(vec)
  -- transform a rotation vector as e.g. provided by solvePnP to a 3x3 rotation matrix using the Rodrigues' rotation formula
  local theta = torch.norm(vec)
  local r = vec / theta
  r = torch.squeeze(r)
  local mat = torch.Tensor({{0, -1 * r[3], r[2]}, {r[3], 0, -1 * r[1]}, {-1 * r[2], r[1], 0}})
  r = r:resize(3, 1)

  return torch.eye(3) * math.cos(theta) + (r * r:t()) * (1 - math.cos(theta)) + mat * math.sin(theta)
end


function PatternLocalisation:mirrorPatternPoints(points, patWidth, patHeight, imgDebug)
  -- resort points
  local pointsResorted = torch.FloatTensor(patWidth * patHeight, 1, 2)

  for col = 0, patHeight - 1 do
    for row = 0, patWidth - 1 do
      pointsResorted[col * patWidth + row + 1][1][{}] =
        points[(patWidth) * (patHeight - col) - patWidth + row + 1][1][{}]
    end
  end

  if imgDebug ~= nil then
    for i = 1, patWidth * patHeight do
      self:plotCrosshair(imgDebug, pointsResorted[i][1][1], pointsResorted[i][1][2], 8)
      print("point " .. pointsResorted[i][1][1] .. ", y=" .. pointsResorted[i][1][2])
      cv.imshow {"debug", imgDebug}
      cv.waitKey {1000}
    end
  end

  return pointsResorted
end


local function findMarker(markerList, markerId)
  for i,m in ipairs(markerList) do
    if m.id == markerId then
      return m
    end
  end
  return nil
end


function PatternLocalisation:calcCamPose_original(inputImg, camIntrinsics, patternData, doDebug, imgShowInput)
  doDebug = doDebug or false
  local camPoseFinal

  local blobDetector = cv.SimpleBlobDetector {self.circleFinderParams}

  local found, points =
    cv.findCirclesGrid {
    image = inputImg,
    patternSize = {height = patternData.height, width = patternData.width},
    flags = cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING,
    blobDetector = blobDetector
  }

  if found then
    local points3d = self:generatePatternPoints(patternData.width, patternData.height, patternData.pointDist)
    local poseFound, poseCamRotVector, poseCamTrans =
      cv.solvePnP {
      objectPoints = points3d,
      imagePoints = points,
      cameraMatrix = camIntrinsics,
      distCoeffs = torch.DoubleTensor(1, 5):zero()
    }
    local poseCamRotMatrix = self:rotVectorToMat3x3(poseCamRotVector)
    camPoseFinal = torch.DoubleTensor(4, 4):zero()
    camPoseFinal[{{1, 3}, {1, 3}}] = poseCamRotMatrix
    camPoseFinal[{{1, 3}, {4}}] = poseCamTrans
    camPoseFinal[4][4] = 1
  end

  if doDebug then
    local imgShow
    if imgShowInput ~= nil then
      imgShow = imgShowInput
    else
      imgShow = self:grayToRGB(inputImg)
    end
    if found then
      cv.drawChessboardCorners {image = imgShow, patternSize = pattern, corners = points, patternWasFound = found}
    end
    imgShow = cv.resize {imgShow, {imgShow:size(2) * self.imgShowRescale, imgShow:size(1) * self.imgShowRescale}}
    cv.imshow {"camPoseDebug", imgShow}
    cv.waitKey {1000}
    cv.destroyWindow {winname = "camPoseDebug"}
  end

  return camPoseFinal, points
end


function PatternLocalisation:calcCamPose(inputImg, camIntrinsics, patternData, doDebug, imgShowInput, patternID)
  doDebug = doDebug or false
  imgShowInput = imgShowInput or nil
  patternID = patternID or nil
  local camPoseFinal

  print("Searching calibration target.")
  local found = false
  local points
  if inputImg:dim() == 3 then
    if inputImg:size(3) > 1 then
      -- extract green channel (e.g. of color cams with RGB Bayer Matrix)
      local greenImgLeft = inputImg[{{},{},2}]:clone()
      inputImg = greenImgLeft
    end
  end
  local foundMarkers = self:processImg(inputImg, false) -- if too many circle points are found, point clusters are detected in each of which we search for the pattern
  if next(foundMarkers) ~= nil then
    if patternID ~= nil then
      local m = findMarker(foundMarkers, patternID)
      if m then
        points = m.points
        found = true
      else
        print(string.format("[Warning] Pattern with ID %d not found", patternID))
      end
    else
      points = foundMarkers[1].points
      found = true
    end
  else
    print('[Warning] Trying fallback with standard findCirclesGrid() function...')
    found, points = cv.findCirclesGrid { image = inputImg,
                                         patternSize = { height = self.pattern.height, width = self.pattern.width },
                                         flags = cv.CALIB_CB_ASYMMETRIC_GRID
                                       }
    if found and patternID ~= nil then
      local id = self:getPatternId(inputImg, points, self.pattern)
      if id ~= patternID then
        print(string.format("[Warning] Pattern with ID %d not found", patternID))
        found = false
      end
    else
      if not found then
        print("[Warning] No pattern found")
      end
    end
  end

  if found then
    local points3d = self:generatePatternPoints(patternData.width, patternData.height, patternData.pointDist)
    local poseFound, poseCamRotVector, poseCamTrans =
      cv.solvePnP {
      objectPoints = points3d,
      imagePoints = points,
      cameraMatrix = camIntrinsics,
      distCoeffs = torch.DoubleTensor(1, 5):zero()
    }
    local poseCamRotMatrix = self:rotVectorToMat3x3(poseCamRotVector)
    camPoseFinal = torch.DoubleTensor(4, 4):zero()
    camPoseFinal[{{1, 3}, {1, 3}}] = poseCamRotMatrix
    camPoseFinal[{{1, 3}, {4}}] = poseCamTrans
    camPoseFinal[4][4] = 1
  end

  if doDebug then
    local imgShow
    if imgShowInput ~= nil then
      imgShow = imgShowInput
    else
      imgShow = self:grayToRGB(inputImg)
    end
    if found then
      cv.drawChessboardCorners {image = imgShow, patternSize = { height = patternData.height, width = patternData.width }, corners = points, patternWasFound = found}
    end
    imgShow = cv.resize {imgShow, {imgShow:size(2) * self.imgShowRescale, imgShow:size(1) * self.imgShowRescale}}
    cv.imshow {"camPoseDebug", imgShow}
    cv.waitKey {500}
  end

  return camPoseFinal, points
end


function PatternLocalisation:calcCamPoseViaPlaneFit(imgLeft, imgRight, whichCam, doDebug, imgShowInput, patternID)
  local whichCam = whichCam or "left"
  local doDebug = doDebug or false
  local imgShowInput = imgShowInput or nil
  local patternID = patternID or nil
  local camPoseFinal

  stereoCalib = self.stereoCalibration
  local leftCamMat = stereoCalib.camLeftMatrix
  local rightCamMat = stereoCalib.camRightMatrix
  local leftDistCoeffs = stereoCalib.camLeftDistCoeffs
  local rightDistCoeffs = stereoCalib.camRightDistCoeffs
  local rightLeftCamTrafo = stereoCalib.trafoLeftToRightCam

  -- Stereo Rectify:
  local R = rightLeftCamTrafo[{{1, 3}, {1, 3}}]:double()
  local T = rightLeftCamTrafo[{{1, 3}, {4}}]:double()
  local leftR = torch.DoubleTensor(3, 3)
  local rightR = torch.DoubleTensor(3, 3)
  local leftP = torch.DoubleTensor(3, 4)
  local rightP = torch.DoubleTensor(3, 4)
  local Q = torch.DoubleTensor(4, 4)

  cv.stereoRectify {
    cameraMatrix1 = leftCamMat,
    distCoeffs1 = leftDistCoeffs,
    cameraMatrix2 = rightCamMat,
    distCoeffs2 = rightDistCoeffs,
    imageSize = {imgLeft:size(2), imgLeft:size(1)},
    R = R,
    T = T,
    R1 = leftR,
    R2 = rightR,
    P1 = leftP,
    P2 = rightP,
    Q = Q,
    flags = 0
  }

  -- Undistortion + rectification:
  local mapAImgLeft, mapBImgLeft =
    cv.initUndistortRectifyMap {
    cameraMatrix = leftCamMat,
    distCoeffs = leftDistCoeffs,
    R = leftR,
    newCameraMatrix = leftP,
    size = {imgLeft:size(2), imgLeft:size(1)},
    m1type = cv.CV_32FC1
  }
  local imgLeftRectUndist =
    cv.remap {src = imgLeft, map1 = mapAImgLeft, map2 = mapBImgLeft, interpolation = cv.INTER_NEAREST}

  local mapAImgRight, mapBImgRight =
    cv.initUndistortRectifyMap {
    cameraMatrix = rightCamMat,
    distCoeffs = rightDistCoeffs,
    R = rightR,
    newCameraMatrix = rightP,
    size = {imgRight:size(2), imgRight:size(1)},
    m1type = cv.CV_32FC1
  }
  local imgRightRectUndist =
    cv.remap {src = imgRight, map1 = mapAImgRight, map2 = mapBImgRight, interpolation = cv.INTER_NEAREST}

  -- Detect all circle points of the pattern in the left/right image
  -- (see "AutoCalibration.lua", function "extractPoints")
  --------------------------------------------------------------------
  print("Searching calibration target.")
  local ok1, ok2 = false, false
  local circlesGridPointsLeft
  local circlesGridPointsRight
  -- Searching calibration target in left camera image:
  if imgLeftRectUndist:dim()> 2 and imgLeftRectUndist:size(3) > 1 then
    -- extract green channel (e.g. of color cams with RGB Bayer Matrix)
    local greenImgLeft = imgLeftRectUndist[{{},{},2}]:clone()
    imgLeftRectUndist = greenImgLeft
  end
  local foundMarkersLeft = self:processImg(imgLeftRectUndist, false) -- if too many circle points are found, point clusters are detected in each of which is searched for the pattern
  if next(foundMarkersLeft) ~= nil then
    if patternID ~= nil then
      local m = findMarker(foundMarkersLeft, patternID)
      if m then
        circlesGridPointsLeft = m.points
        ok1 = true
      else
        print(string.format("[Warning] Pattern with ID %d not found in left image", patternID))
      end
    else
      circlesGridPointsLeft = foundMarkersLeft[1].points
      ok1 = true
    end
  else
    print('[Warning] Trying fallback with standard findCirclesGrid() function...')
    local ok, points = cv.findCirclesGrid { image = imgLeftRectUndist,
                                            patternSize = { height = self.pattern.height, width = self.pattern.width },
                                            flags = cv.CALIB_CB_ASYMMETRIC_GRID
                                          }
    if ok and patternID ~= nil then
      local id = self:getPatternId(imgLeftRectUndist, points, self.pattern)
      if id == patternID then
        circlesGridPointsLeft = points
        ok1 = true
      else
        print(string.format("[Warning] Pattern with ID %d not found in left image", patternID))
      end
    else
      if ok then
        circlesGridPointsLeft = points
        ok1 = true
      else
        print("[Warning] No pattern found in left image")
      end
    end
  end
  -- Searching calibration target in right camera image:
  if imgRightRectUndist:dim()> 2 and imgRightRectUndist:size(3) > 1 then
    -- extract green channel (e.g. of color cams with RGB Bayer Matrix)
    local greenImgRight = imgRightRectUndist[{{},{},2}]:clone()
    imgRightRectUndist = greenImgRight
  end
  local foundMarkersRight = self:processImg(imgRightRectUndist, false) -- if too many circle points are found, point clusters are detected in each of which is searched for the pattern
  if next(foundMarkersRight) ~= nil then
    if patternID ~= nil then
      local m = findMarker(foundMarkersRight, patternID)
      if m then
        circlesGridPointsRight = m.points
        ok2 = true
      else
        print(string.format("[Warning] Pattern with ID %d not found in right image", patternID))
      end
    else
      circlesGridPointsRight = foundMarkersRight[1].points
      ok2 = true
    end
  else
    print('[Warning] Trying fallback with standard findCirclesGrid() function...')
    local ok, points = cv.findCirclesGrid { image = imgRightRectUndist,
                                            patternSize = { height = self.pattern.height, width = self.pattern.width },
                                            flags = cv.CALIB_CB_ASYMMETRIC_GRID
                                          }
    if ok and patternID ~= nil then
      local id = self:getPatternId(imgRightRectUndist, points, self.pattern)
      if id == patternID then
        circlesGridPointsRight = points
        ok2 = true
      else
        print(string.format("[Warning] Pattern with ID %d not foundin right image", patternID))
      end
    else
      if ok then
        circlesGridPointsRight = points
        ok2 = true
      else
        print("[Warning] No pattern found in right image")
      end
    end
  end
  print(string.format("ok1: %s", ok1))
  print(string.format("ok2: %s\n", ok2))

  if doDebug then
    local imgShowLeft = self:grayToRGB(imgLeftRectUndist)
    if ok1 then
      cv.drawChessboardCorners {image = imgShowLeft, patternSize = { height = self.pattern.height, width = self.pattern.width }, corners = circlesGridPointsLeft, patternWasFound = ok1}
    end
    imgShowLeft = cv.resize {imgShowLeft, {imgShowLeft:size(2) * self.imgShowRescale, imgShowLeft:size(1) * self.imgShowRescale}}
    cv.imshow {"camPoseLeftDebug", imgShowLeft}
    cv.waitKey {500}
    local imgShowRight = self:grayToRGB(imgRightRectUndist)
    if ok2 then
      cv.drawChessboardCorners {image = imgShowRight, patternSize = { height = self.pattern.height, width = self.pattern.width }, corners = circlesGridPointsRight, patternWasFound = ok2}
    end
    imgShowRight = cv.resize {imgShowRight, {imgShowRight:size(2) * self.imgShowRescale, imgShowRight:size(1) * self.imgShowRescale}}
    cv.imshow {"camPoseRightDebug", imgShowRight}
    cv.waitKey {500}
  end

  local ok = ok1 and ok2
  if ok then
    -- Save 2d grid points.
    local nPoints = self.pattern.width * self.pattern.height
    local leftProjPoints = torch.DoubleTensor(2, nPoints) -- 2D coordinates (x,y) x #Points
    local rightProjPoints = torch.DoubleTensor(2, nPoints)
    for i = 1, nPoints do
      leftProjPoints[1][i] = circlesGridPointsLeft[i][1][1]
      leftProjPoints[2][i] = circlesGridPointsLeft[i][1][2]
      rightProjPoints[1][i] = circlesGridPointsRight[i][1][1]
      rightProjPoints[2][i] = circlesGridPointsRight[i][1][2]
    end

    local resulting4DPoints = torch.DoubleTensor(4, nPoints)

    -- TriangulatePoints:
    ---------------------
    cv.triangulatePoints {leftP, rightP, leftProjPoints, rightProjPoints, resulting4DPoints}

    local resulting3DPoints = torch.DoubleTensor(nPoints, 3)
    for i = 1, nPoints do
      resulting3DPoints[i][1] = resulting4DPoints[1][i] / resulting4DPoints[4][i]
      resulting3DPoints[i][2] = resulting4DPoints[2][i] / resulting4DPoints[4][i]
      resulting3DPoints[i][3] = resulting4DPoints[3][i] / resulting4DPoints[4][i]
    end

    local pointsInCamCoords = torch.DoubleTensor(nPoints, 3)
    if whichCam == "left" then
      for i = 1, nPoints do
        pointsInCamCoords[i] = torch.inverse(leftR) * resulting3DPoints[i]
      end
    else
      for i = 1, nPoints do
        pointsInCamCoords[i] = torch.inverse(rightR) * resulting3DPoints[i]
      end
    end

    -- Generate plane through detected 3d points to get the transformation
    ----------------------------------------------------------------------
    -- of the pattern into the coordinatesystem of the camera:
    ----------------------------------------------------------
    -- Plane fit with pseudo inverse:
    local A = torch.DoubleTensor(nPoints, 3) -- 2-dim. tensor of size 168 x 3
    local b = torch.DoubleTensor(nPoints) -- 1-dim. tensor of size 168
    for i = 1, nPoints do
      A[i][1] = pointsInCamCoords[i][1]
      A[i][2] = pointsInCamCoords[i][2]
      A[i][3] = 1.0
      b[i] = pointsInCamCoords[i][3]
    end
    -- Calculate x = A^-1 * b = (A^T A)^-1 * A^T * b (with pseudo inverse of A)
    local At = A:transpose(1, 2)
    local x = torch.mv(torch.mm(torch.inverse(torch.mm(At, A)), At), b)

    -- Determine z-axis as normal on plane:
    local n = torch.DoubleTensor(3)
    n[1] = x[1]
    n[2] = x[2]
    n[3] = -1.0
    local z_unit_vec = torch.div(n, torch.norm(n))

    -- Determine x-axis along left boundary points of the pattern
    local x_direction = pointsInCamCoords[self.pattern.width] - pointsInCamCoords[1]
    local x_unit_vec = torch.div(x_direction, torch.norm(x_direction))

    -- Determine y-axis along top boundary points of the pattern:
    local y_direction = pointsInCamCoords[nPoints - self.pattern.width + 1] - pointsInCamCoords[1]
    local y_unit_vec = torch.div(y_direction, torch.norm(y_direction))

    -- Check, whether the normal vector z_unit_vec points into the correct direction.
    local cross_product = torch.DoubleTensor(3)
    cross_product = torch.cross(x_unit_vec, y_unit_vec)
    cross_product = torch.div(cross_product, torch.norm(cross_product))
    if z_unit_vec * cross_product < 0.0 then
      z_unit_vec = -1.0 * z_unit_vec
    end

    -- x_unit_vec * z_unit_vec has to be zero.
    -- Map x_unit_vec onto plane.
    local new_x_unit_vec = x_unit_vec - x_unit_vec * z_unit_vec * z_unit_vec
    if new_x_unit_vec * x_unit_vec < 0.0 then
      new_x_unit_vec = -1.0 * new_x_unit_vec
    end

    -- Determine new y_unit_vec as cross product of x_unit_vec and z_unit_vec:
    local new_y_unit_vec = torch.cross(new_x_unit_vec, z_unit_vec)
    if new_y_unit_vec * y_unit_vec < 0.0 then
      new_y_unit_vec = -1.0 * new_y_unit_vec
    end

    -- Transform pattern coordinate system into camera coordinate system:
    -- M_B->A = (x_unit_vec, y_unit_vec, z_unit_vec, support vector)
    camPoseFinal = torch.DoubleTensor(4, 4):zero()
    camPoseFinal[{{1, 3}, {1}}] = new_x_unit_vec
    camPoseFinal[{{1, 3}, {2}}] = new_y_unit_vec
    camPoseFinal[{{1, 3}, {3}}] = z_unit_vec
    camPoseFinal[{{1, 3}, {4}}] = pointsInCamCoords[1] -- = support vector
    camPoseFinal[4][4] = 1
  else
    print("PATTERN NOT FOUND!!!")
    camPoseFinal = torch.eye(4)
  end

  return ok, camPoseFinal, circlesGridPointsLeft, circlesGridPointsRight
end


function PatternLocalisation:calcCamPoseMirrored(pointsResorted)
  local points3d = self:generatePatternPoints(self.pattern.width, self.pattern.height, self.pattern.pointDist)

  local poseFound, poseCamRotVector, poseCamTrans =
    cv.solvePnP {
    objectPoints = points3d,
    imagePoints = pointsResorted,
    cameraMatrix = self.camIntrinsics,
    distCoeffs = torch.DoubleTensor(1, 5):zero()
  }
  print(poseFound)
  print(poseCamRotVector)
  print("done")
  local poseCamRotMatrix = self:rotVectorToMat3x3(poseCamRotVector)
  local camPoseFinal = torch.DoubleTensor(4, 4):zero()
  camPoseFinal[{{1, 3}, {1, 3}}] = poseCamRotMatrix
  camPoseFinal[{{1, 3}, {4}}] = poseCamTrans
  camPoseFinal[4][4] = 1

  return camPoseFinal
end


-- Calculates a rectangle of a point set (torch.Tensor of dim n,2) , usefull for ROI or cropping
function PatternLocalisation:calcBoundingRect(pointTensor, border, imgWidth, imgHeight)
  local points = torch.squeeze(pointTensor)
  local rectPoints = cv.boundingRect {points}

  local rect = torch.IntTensor(2, 2):zero() -- top left and bottom right corner of the ROI rect

  if rectPoints.x - border >= 1 then
    rect[1][1] = rectPoints.x - border
  else
    rect[1][1] = 1
  end

  if rectPoints.y - border >= 1 then
    rect[1][2] = rectPoints.y - border
  else
    rect[1][2] = 1
  end

  if rectPoints.x + rectPoints.width + border < imgWidth then
    rect[2][1] = rectPoints.x + rectPoints.width + border
  else
    rect[2][1] = imgWidth
  end

  if rectPoints.y + rectPoints.height + border < imgHeight then
    rect[2][2] = rectPoints.y + rectPoints.height + border
  else
    rect[2][2] = imgHeight
  end

  return rect
end


-- Calculates the cam pose for a pattern. The pattern has to be the only pattern within the region of interesst defined by rectROI
function PatternLocalisation:calcCamPoseROI(inputImg, rectROI, doDebug)
  doDebug = doDebug or false
  if inputImg:dim() ~= 2 then
    print("Only 2dim grayscale images supported (for speed reason). Provided image has size ")
    print(inputImg:size())
    return nil
  end

  local imgROI = torch.ByteTensor(inputImg:size(1), inputImg:size(2)):fill(255)
  imgROI[{{rectROI[1][2], rectROI[2][2]}, {rectROI[1][1], rectROI[2][1]}}] =
    inputImg[{{rectROI[1][2], rectROI[2][2]}, {rectROI[1][1], rectROI[2][1]}}]

  if doDebug == true then
    local imgShow
    if self.imgShowRescale ~= nil then
      imgShow = cv.resize {imgROI, {imgROI:size(2) * self.imgShowRescale, imgROI:size(1) * self.imgShowRescale}}
    else
      imgShow = imgROI
    end

    cv.imshow {"imgPatROI", imgShow}
    cv.waitKey {1000}
    cv.destroyWindow {winname = "imgPatROI"}
  end

  return self:processImg(imgROI)
end


function PatternLocalisation:findCircles(inputImg, doDebug)
  -- Setup SimpleBlobDetector parameters.
  local blobDetector = cv.SimpleBlobDetector {self.circleFinderParams}
  local resultsTmp = blobDetector:detect {image = inputImg}
  local results = {}
  for i = 1, resultsTmp.size do
    table.insert(
      results,
      {
        radius = resultsTmp.data[i].size / 2.0,
        angle = resultsTmp.data[i].angle,
        pos = torch.DoubleTensor({resultsTmp.data[i].pt.x, resultsTmp.data[i].pt.y, 1})
      }
    )
  end

  local circleScale = 16
  local shiftBits = 4
  if doDebug == true then
    local rescaleFactor = 1
    if self.imgShowRescale ~= nil then
      rescaleFactor = self.imgShowRescale
    end

    local imgScale = cv.resize {inputImg, {inputImg:size(2) * rescaleFactor, inputImg:size(1) * rescaleFactor}}
    imgScale = self:grayToRGB(imgScale)
    for key, val in ipairs(results) do
      cv.circle {
        img = imgScale,
        center = {x = val.pos[1] * rescaleFactor * circleScale, y = val.pos[2] * rescaleFactor * circleScale},
        radius = val.radius * rescaleFactor * circleScale,
        color = {0, 255, 255},
        thickness = 2,
        lineType = cv.LINE_AA,
        shift = shiftBits
      }
    end

    cv.imshow {"circleFinder", imgScale}
    cv.waitKey {500}
  end

  return results
end


-- cluster: list, containing index numbers of list circleCenters

function PatternLocalisation:expandCluster(
  kdtree,
  circleCenters,
  eps,
  minNPoints,
  currPointIndex,
  neighbours,
  pointStatus,
  cluster)
  table.insert(cluster, currPointIndex)
  pointStatus[currPointIndex] = bit.bor(pointStatus[currPointIndex], self.clusterStatus.kAssigned)

  -- convert torchTensor to list so we can append points
  local neighbourList = {}
  for i = 1, neighbours:size(2) do
    table.insert(neighbourList, neighbours[1][i])
  end

  local loopEnd = #neighbourList
  local i = 1
  while i <= loopEnd do
    local index = neighbourList[i]

    if bit.band(pointStatus[index], self.clusterStatus.kProcessed) == 0 then
      pointStatus[index] = bit.bor(pointStatus[index], self.clusterStatus.kProcessed)

      local queryPoint = torch.FloatTensor(1, 2)
      queryPoint[1][1] = circleCenters[index].pos[1]
      queryPoint[1][2] = circleCenters[index].pos[2]

      local indices = torch.IntTensor(1, minNPoints):fill(0)
      local dists = torch.FloatTensor(1, minNPoints):fill(0)
      local tmp  -- not used
      tmp, indices, dists =
        kdtree:radiusSearch {
        query = queryPoint,
        radius = eps * eps,
        maxResults = minNPoints,
        indices = indices,
        dists = dists
      }

      if indices[1][minNPoints] > 0 then
        for i = 1, indices:size(2) do
          table.insert(neighbourList, indices[1][i])
          loopEnd = #neighbourList
        end
      end
    end

    if bit.band(pointStatus[index], self.clusterStatus.kAssigned) == 0 then
      table.insert(cluster, index)
      pointStatus[index] = bit.bor(pointStatus[index], self.clusterStatus.kAssigned)
    end

    i = i + 1
  end
end


function PatternLocalisation:DBScan(circleCenters, eps, minNPoints)
  local clusterList = {}

  if #circleCenters < 1 then
    return clusterList -- return an empty cluster list because clustering less then one point is not possible
  end

  local circleCentersTensor = torch.FloatTensor(#circleCenters, 2)
  for i, val in ipairs(circleCenters) do
    circleCentersTensor[i][1] = val.pos[1]
    circleCentersTensor[i][2] = val.pos[2]
  end

  local params = cv.flann.KDTreeIndexParams {trees = 10}
  local kdtree = cv.flann.Index {circleCentersTensor, params, cv.FLANN_DIST_EUCLIDEAN}

  local pointStatus = torch.ByteTensor(#circleCenters):fill(self.clusterStatus.kUnprocessed)

  for i = 1, #circleCenters do
    if bit.band(pointStatus[i], self.clusterStatus.kProcessed) == 0 then
      pointStatus[i] = bit.bor(pointStatus[i], self.clusterStatus.kProcessed)

      local queryPoint = torch.FloatTensor(1, 2)
      queryPoint[1][1] = circleCenters[i].pos[1]
      queryPoint[1][2] = circleCenters[i].pos[2]

      -- indices and dists are 1xn int and float tensors, respectivly
      local indices = torch.IntTensor(1, minNPoints):fill(0)
      local dists = torch.FloatTensor(1, minNPoints):fill(0)
      local tmp  -- not used
      tmp, indices, dists =
        kdtree:radiusSearch {
        query = queryPoint,
        radius = eps * eps,
        maxResults = minNPoints,
        indices = indices,
        dists = dists
      }

      if indices[1][minNPoints] <= 0 or indices[1][minNPoints] ~= indices[1][minNPoints] then -- 0 or nan
        pointStatus[i] = bit.bor(pointStatus[i], self.clusterStatus.kNoise)
      else
        local nextCluster = {}
        table.insert(clusterList, nextCluster)
        self:expandCluster(kdtree, circleCenters, eps, minNPoints, i, indices, pointStatus, nextCluster)
      end
    end
  end

  return clusterList
end


-- Debugging function
function PatternLocalisation:espChecking(circleCentersInput, queryX, queryY, debugImg)
  local circleCenters = torch.FloatTensor(#circleCentersInput, 2)
  for i, val in ipairs(circleCentersInput) do
    circleCenters[i][1] = val.pos[1]
    circleCenters[i][2] = val.pos[2]
  end

  local params = cv.flann.KDTreeIndexParams {trees = 10}
  local kdtree = cv.flann.Index {circleCenters, params, cv.FLANN_DIST_EUCLIDEAN}

  local queryPoint = torch.FloatTensor(1, 2)
  queryPoint[1][1] = queryX
  queryPoint[1][2] = queryY
  local nNeighbours = 6

  local indices, dists = kdtree:knnSearch {queryPoint, nNeighbours}

  local circleScale = 16
  local shiftBits = 4
  local rescaleFactor = 1.0
  cv.circle {
    img = debugImg,
    center = {x = queryPoint[1][1] * rescaleFactor * circleScale, y = queryPoint[1][2] * rescaleFactor * circleScale},
    radius = 5 * circleScale,
    color = {50, 50, 255},
    thickness = 3,
    lineType = cv.LINE_AA,
    shift = shiftBits
  }

  for i = 1, nNeighbours do
    local center = {}
    center.x = circleCentersInput[indices[1][i]].pos[1] * rescaleFactor * circleScale
    center.y = circleCentersInput[indices[1][i]].pos[2] * rescaleFactor * circleScale
    local radius = circleCentersInput[indices[1][i]].radius * rescaleFactor * circleScale

    cv.circle {
      img = debugImg,
      center = center,
      radius = radius,
      color = {255, 0, 255},
      thickness = 1,
      lineType = cv.LINE_AA,
      shift = shiftBits
    }
  end

  cv.imshow {"testESP", debugImg}
  cv.waitKey {1000}
end


-- Set all pixels to color color, except inside the polygon defined by points
-- points is torch.IntTensor(nPolygonPoints, 2)

function PatternLocalisation:maskImage(inputImg, points, color)
  color = color or 255

  local mask = torch.ByteTensor(inputImg:size(1), inputImg:size(2), 1):fill(1)
  cv.fillConvexPoly {mask, points, {0}}
  local applyMask = torch.eq(mask, 1)

  -- If image is RGB image, expand the mask to contain all three channels
  if inputImg:dim() == 3 and inputImg:size(3) == 3 then
    applyMask = applyMask:expand((#applyMask)[1], (#applyMask)[2], 3)
  end

  -- apply the mask to a copy of the provided image
  local imgMasked = inputImg:clone()
  imgMasked[applyMask] = 255 -- set all masked pixels to white

  -- return the masked image
  return imgMasked
end

function PatternLocalisation:RGBToGray(inputImg)
  local img
  if inputImg:dim() == 3 and inputImg:size(3) > 1 then
    img = cv.cvtColor {src = inputImg, code = cv.COLOR_RGB2GRAY}
  else
    img = inputImg:clone()
  end

  return img
end


function PatternLocalisation:grayToRGB(inputImg)
  local img
  if inputImg:dim() == 3 and inputImg:size(3) > 1 then
    img = inputImg:clone()
  else
    img = cv.cvtColor {src = inputImg, code = cv.COLOR_GRAY2RGB}
  end

  return img
end


function PatternLocalisation:getPatPointCenterColor(imgGray, point)
  local pointColor = 0.0

  pointColor = imgGray[math.floor(point[2])][math.floor(point[1])]
  pointColor = pointColor + imgGray[math.floor(point[2])][math.ceil(point[1])]
  pointColor = pointColor + imgGray[math.ceil(point[2])][math.floor(point[1])]
  pointColor = pointColor + imgGray[math.ceil(point[2])][math.ceil(point[1])]
  pointColor = pointColor / 4.0

  return pointColor
end


function PatternLocalisation:getID(dict, pat)
  local id
  local err

  if dict[pat + 1] >= 0 and dict[pat + 1] < 1024 then -- no error
    id = dict[pat + 1]
    err = 0
  elseif dict[pat + 1] >= 1024 then -- one bit error
    id = dict[pat + 1] - 1024
    err = 1
  else -- two bit errors, can't correct
    id = -1
    err = 2
  end
  -- if more then 2 bits are defect, a wrong ID is returned
  -- because the coding can't handle more then 2 bit errors

  return id, err
end


function PatternLocalisation:getPatternId(imgInput, points, pattern)
  local imgGray = torch.squeeze(self:RGBToGray(imgInput))

  local refPoints = {}
  table.insert(refPoints, points[1][1])
  table.insert(refPoints, points[pattern.width * 2][1])
  table.insert(refPoints, points[pattern.width * (pattern.height - 1) + 1][1])
  table.insert(refPoints, points[pattern.width * pattern.height][1])

  local nPoints = pattern.width * pattern.height
  local idRef = {}
  local idPoints = {}

  table.insert(idRef, points[math.ceil(nPoints / 2.0 - 2 * pattern.width - 1)][1]) -- dark ref. point, top left
  table.insert(idRef, points[math.ceil(nPoints / 2.0 + 2 * pattern.width + 1)][1]) -- dark ref. point, bottom right
  table.insert(idRef, points[math.ceil(nPoints / 2.0 - 2 * pattern.width + 1)][1]) -- light ref. point, bottom left
  table.insert(idRef, points[math.ceil(nPoints / 2.0 + 1 * pattern.width + 1)][1]) -- light ref. point, center bottom left
  table.insert(idRef, points[math.ceil(nPoints / 2.0 + 2 * pattern.width - 1)][1]) -- light ref. point, top left

  -- sorted from least significant bit (1) to most significant bit (10)
  table.insert(idPoints, points[math.ceil(nPoints / 2.0 + 2 * pattern.width - 0)][1]) -- bit 1
  table.insert(idPoints, points[math.ceil(nPoints / 2.0 - 2 * pattern.width - 0)][1]) -- bit 2
  table.insert(idPoints, points[math.ceil(nPoints / 2.0 - 0 * pattern.width - 1)][1]) -- bit 3
  table.insert(idPoints, points[math.ceil(nPoints / 2.0 - 0 * pattern.width + 1)][1]) -- bit 4
  table.insert(idPoints, points[math.ceil(nPoints / 2.0 + 1 * pattern.width - 1)][1]) -- bit 5
  table.insert(idPoints, points[math.ceil(nPoints / 2.0 - 1 * pattern.width - 0)][1]) -- bit 6
  table.insert(idPoints, points[math.ceil(nPoints / 2.0 - 1 * pattern.width - 1)][1]) -- bit 7
  table.insert(idPoints, points[math.ceil(nPoints / 2.0 - 0 * pattern.width - 0)][1]) -- bit 8 (central dot)
  table.insert(idPoints, points[math.ceil(nPoints / 2.0 + 1 * pattern.width - 0)][1]) -- bit 9
  table.insert(idPoints, points[math.ceil(nPoints / 2.0 - 1 * pattern.width + 1)][1]) -- bit 10

  local darkColor =
    (self:getPatPointCenterColor(imgGray, idRef[1]) + self:getPatPointCenterColor(imgGray, idRef[2])) / 2.0
  local lightColor =
    (self:getPatPointCenterColor(imgGray, idRef[3]) + self:getPatPointCenterColor(imgGray, idRef[4]) +
    self:getPatPointCenterColor(imgGray, idRef[5])) /
    3.0

  --print("Contrast: "..lightColor-darkColor)
  local darkThresh = darkColor + (lightColor - darkColor) / 3.0
  local lightThresh = lightColor - (lightColor - darkColor) / 3.0
  if lightColor - darkColor < 15 then
    print("ERROR: Overall point ID contrast to low")
    return -1
  end

  local patNum = 0
  for i, point in ipairs(idPoints) do
    local pointColor = self:getPatPointCenterColor(imgGray, point)
    if pointColor > lightThresh then
      patNum = patNum + bit.lshift(1, i - 1)
    elseif pointColor < darkThresh then
      -- do nothing because bit is already set to 0
    else
      print("--- WARNING: low contrast ---")
      print("low threshold=" .. darkThresh .. ", high threshold=" .. lightThresh)
      print("detected value=" .. pointColor)
      if pointColor > darkColor + (lightColor - darkColor) / 2.0 then
        patNum = patNum + bit.lshift(1, i - 1)
        print("bit identified as 1")
      else
        print("bit identified as 0")
      end
    end
  end

  local id, err = self:getID(self.patDictData, patNum)
  return id, err, patNum
end


function PatternLocalisation:processImg(inputImg, withCamPoseCalculation)
  local camImgUndist = inputImg
  local calcCamPose = true
  if withCamPoseCalculation ~= nil then
    calcCamPose = withCamPoseCalculation
  end

  local circleList = self:findCircles(camImgUndist, self.debugParams.circleSearch)

  -- Locate all clusters of circles in the image
  --local timer = torch.Timer()
  local clusterList = self:DBScan(circleList, self.DBScanEps, self.DBScanMinPts)
  --timer:stop()
  --print(string.format("DBScan run time: %.1f ms", timer:time().real*1000))
  --print("Number of clusters: "..#clusterList)
  --timer:reset()  -- to restart: timer:resume()

  local circleScale = 16
  local shiftBits = 4

  -- Create the list of hull polygons, containing one cluster each
  local hullList = {}
  local imgHulls, colorNum
  if self.debugParams.clustering == true or self.debugParams.pose == true then
    imgHulls = self:grayToRGB(camImgUndist)
    colorNum = 1
  end
  for i, cluster in ipairs(clusterList) do
    if self.debugParams.clustering == true then
      print("Processing cluster " .. i .. " with nPoints=" .. #cluster)
    end
    local nPatternPoints = self.pattern.width * self.pattern.height
    if #cluster >= nPatternPoints and #cluster < nPatternPoints * 2 then -- process only clusters which have at least as many points as the searched pattern and not more then two times the number of pattern points
      local clusterPointsTensor = torch.IntTensor(#cluster, 2)
      for j, pointIdx in ipairs(cluster) do
        clusterPointsTensor[j][1] = circleList[pointIdx].pos[1]
        clusterPointsTensor[j][2] = circleList[pointIdx].pos[2]
      end

      local hull = torch.squeeze(cv.convexHull {points = clusterPointsTensor, clockwise = true})
      hull = hull:float()
      local mean = torch.mean(hull, 1)
      local area = cv.contourArea {hull}

      local scale = 1.2 + 6000.0 / area
      hull[{{}, 1}]:mul(scale)
      hull[{{}, 1}]:add((1 - scale) * mean[1][1])

      hull[{{}, 2}]:mul(scale)
      hull[{{}, 2}]:add((1 - scale) * mean[1][2])

      table.insert(hullList, hull)

      -- Visualisation
      if self.debugParams.clustering == true then
        --   Draw the circles
        for j, pointIdx in ipairs(cluster) do
          local center = {}
          center.x = circleList[pointIdx].pos[1] * circleScale
          center.y = circleList[pointIdx].pos[2] * circleScale
          local radius = circleList[pointIdx].radius * circleScale

          cv.circle {
            img = imgHulls,
            center = center,
            radius = radius,
            color = self.colorTab[colorNum],
            thickness = 2 * 1.0 / self.imgShowRescale,
            lineType = cv.LINE_AA,
            shift = shiftBits
          }
        end

        --   Draw the polygon hull
        for j = 1, hull:size(1) - 1 do
          cv.line {
            img = imgHulls,
            pt1 = {hull[j][1], hull[j][2]},
            pt2 = {hull[j + 1][1], hull[j + 1][2]},
            color = self.colorTab[colorNum],
            thickness = 2 * 1.0 / self.imgShowRescale,
            lineType = cv.LINE_AA
          }
        end

        cv.line {
          img = imgHulls,
          pt1 = {hull[hull:size(1)][1], hull[hull:size(1)][2]},
          pt2 = {hull[1][1], hull[1][2]},
          color = self.colorTab[colorNum],
          thickness = 2 * 1.0 / self.imgShowRescale,
          lineType = cv.LINE_AA
        }

        colorNum = colorNum + 1
      end
    end
  end

  -- check each hull (cluster) for point pattern and calc the cam pose if the cluster containts a point pattern
  local camPoseList = {}
  colorNum = 1
  local noIdPat = -1 -- Pattern with no IDs get assigned negative numbers
  for i, hull in ipairs(hullList) do
    if self.debugParams.clustering == true then
      cv.putText {
        imgHulls,
        text = "" .. i,
        org = {x = hull[hull:size(1)][1], y = hull[hull:size(1)][2]},
        fontFace = cv.FONT_HERSHEY_SIMPLEX,
        fontScale = 0.7 * 1.0 / self.imgShowRescale,
        color = {70, 70, 70},
        thickness = 3
      }
    end

    local imgMasked = self:maskImage(camImgUndist, hull:int())
    local camPose
    local pointsSorted
    local patternFound

    if self.camIntrinsics ~= nil and calcCamPose == true then
      camPose, pointsSorted = self:calcCamPose_original(imgMasked, self.camIntrinsics, self.pattern)
    else -- we don't have intrinsic cam parameters, so just calculate the point list but not the cam pose
      local blobDetector = cv.SimpleBlobDetector {self.circleFinderParams}
      patternFound, pointsSorted =
        cv.findCirclesGrid {
        image = imgMasked,
        patternSize = {height = self.pattern.height, width = self.pattern.width},
        flags = cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING,
        blobDetector = blobDetector
      }
    end

    if useMirroredPattern then
    end

    if (camPose ~= nil) or (patternFound == true) then
      local id = self:getPatternId(camImgUndist, pointsSorted, self.pattern)
      if id > 0 then
        table.insert(camPoseList, {id = id, pose = camPose, points = pointsSorted})

        if self.debugParams.pose == true then
          -- visualisation, mainly for debugging
          if pointsSorted ~= nil then
            self:plotCrosshair(imgHulls, pointsSorted[1][1][1], pointsSorted[1][1][2], 8)
            cv.putText {
              imgHulls,
              text = "" .. id,
              org = {x = pointsSorted[1][1][1] + 8, y = pointsSorted[1][1][2] + 8},
              fontFace = cv.FONT_HERSHEY_SIMPLEX,
              fontScale = 0.8 * 1.0 / self.imgShowRescale,
              color = {80, 80, 255},
              thickness = 3
            }
            print("ID " .. id)
            if camPose ~= nil then
              print("cam pose: ")
              print(camPose)
            end
          end

          colorNum = colorNum + 1
        end
      else
        print("Pose found but ID is invalid. (point cluster " .. i .. ")")
        table.insert(camPoseList, {id = noIdPat, pose = camPose, points = pointsSorted})
        noIdPat = noIdPat - 1

        if self.debugParams.pose == true then
          -- visualisation, mainly for debugging
          if pointsSorted ~= nil then
            self:plotCrosshair(imgHulls, pointsSorted[1][1][1], pointsSorted[1][1][2], 4)
            cv.putText {
              imgHulls,
              text = "" .. noIdPat,
              org = {x = pointsSorted[1][1][1] + 8, y = pointsSorted[1][1][2] + 8},
              fontFace = cv.FONT_HERSHEY_SIMPLEX,
              fontScale = 0.6 * 1.0 / self.imgShowRescale,
              color = {0, 0, 255},
              thickness = 1
            }
            print(camPose)
          end

          colorNum = colorNum + 1
        end
      end
    else
      if self.debugParams.clustering == true or self.debugParams.pose == true then
        print("No pattern found in point cluster " .. i)
      end
    end
  end

  if self.debugParams.clustering == true or self.debugParams.pose == true then
    local imgShow
    if self.imgShowRescale ~= nil then
      imgShow = cv.resize {imgHulls, {imgHulls:size(2) * self.imgShowRescale, imgHulls:size(1) * self.imgShowRescale}}
    else
      imgShow = imgHulls
    end

    cv.imshow {"hulls", imgShow}
    cv.waitKey {500}
  end

  return camPoseList
end
