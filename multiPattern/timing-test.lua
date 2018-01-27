local torch = require 'torch'

local cv = require 'cv'
require 'cv.highgui'
require 'cv.videoio'
require 'cv.imgproc'
require 'cv.calib3d'
require 'cv.imgcodecs'
require 'cv.features2d'

require 'multiPattern'
local db = require 'dbtool'
db.scan()



local function grabImgUndist(imgPath, camSerial, camMatrix, camDistCoeffs)
   local camImgDist = cv.imread{imgPath}
   local imgUndist = cv.undistort{src = camImgDist, cameraMatrix = camMatrix, distCoeffs = camDistCoeffs}

   return imgUndist
end



local function main(imgPath)

  -- load camera calibration data
  local camCalib = torch.load("/data/camImgs_2016-11-23_A/2016-11-23_105406/boardCamCalibration.t7")
  local camMatrix = camCalib.camMatrix
  local camDistCoeffs = camCalib.distCoeffs

  -- configure multi pattern finder, settings for pattern on PCB board
  local patFinder = PatternLocalisation()

      patFinder.circleFinderParams.minThreshold=10
      patFinder.circleFinderParams.maxThreshold=150
      patFinder.circleFinderParams.thresholdStep=20
      patFinder.circleFinderParams.minRepeatability = 2

      patFinder.circleFinderParams.minArea = 150

--[[  patFinder.circleFinderParams.minArea=250
  patFinder.circleFinderParams.maxArea=5000
  patFinder.circleFinderParams.minCircularity = 0.8 --]]
  patFinder.imgShowRescale = 0.4

  patFinder:setPatternIDdictionary(db.configuration.patternIDs)
  patFinder:setDBScanParams(60, 6)

  pattern = {width=3, height=5, pointDist=0.0025}
  patFinder:setPatternData(pattern.width, pattern.height, pattern.pointDist)
  patFinder:setCamIntrinsics(camMatrix)
  patFinder.debugParams.circleSearch=false
  patFinder.debugParams.clustering=true
  patFinder.debugParams.pose=true

  local img = grabImgUndist(imgPath, camCalib.serial, camMatrix, camDistCoeffs)
  img = patFinder:RGBToGray(img)

  local timer = torch.Timer()
  local markerList = patFinder:processImg(img)
  print('*** Time elapsed for initial pose calculation: ' .. timer:time().real .. ' seconds')

  local timer = torch.Timer()
  patFinder.circleFinderParams.minThreshold=120
  patFinder.circleFinderParams.thresholdStep=20
  patFinder.circleFinderParams.maxThreshold=200
  local rect = patFinder:calcBoundingRect(markerList[1].points, 200, img:size(2),img:size(1))
  local poseFast = patFinder:calcCamPoseROI(img, rect, false)
  print('*** Time elapsed for pose: ' .. timer:time().real .. ' seconds')
  print("--- results ---")
  print("full search")
  print(markerList[1].pose)
  print("fast search")
  print(poseFast)
  
end


main("/data/camImgs_2016-11-23_A/D-gripper-100ms-newFocus/cam_40600628_000007.png")
