local torch = require 'torch'

local cv = require 'cv'
require 'cv.highgui'
require 'cv.videoio'
require 'cv.imgproc'
require 'cv.calib3d'
require 'cv.imgcodecs'
require 'cv.features2d'

require 'multiPattern'
local db = require 'db'

-- calibration data
local calibData = db.calibration.cam



local camMatrix = calibData.camMatrix
local camDistCoeffs = calibData.distCoeffs
local camSerial = calibData.serial
local camUndistortMaps = {}    -- undistiortion maps
local handEye = db.calibration.handEye -- 4x4 matrix -- TODO check if this is really a Matrix which transforms from RobotWorld 2 Cam KO!!!!


local function grabImgUndist(imgPath, camSerial, camMatrix, camDistCoeffs)
   local camImgDist = cv.imread{imgPath}
   local imgUndist = cv.undistort{src = camImgDist, cameraMatrix = camMatrix, distCoeffs = camDistCoeffs}

   return imgUndist
end



local function main(imgPath)

  local imgOnBoard = grabImgUndist(imgPath.."/cam_35371951_000001.png", "35371951", calibData.onboardCam.camMatrix, calibData.onboardCam.distCoeffs)
  local imgScan = grabImgUndist(imgPath.."/cam_28670151_000001.png", "28670151", calibData.scanCam.camMatrix, calibData.scanCam.distCoeffs)

  local patFinder = PatternLocalisation()
  patFinder.circleFinderParams.minThreshold=10
  patFinder.circleFinderParams.maxThreshold=250
  patFinder.circleFinderParams.thresholdStep=5
  patFinder.circleFinderParams.minRepeatability = 2
  
  patFinder.circleFinderParams.minArea = 300
  patFinder.circleFinderParams.maxArea=5000
  patFinder.imgShowRescale = 0.4

  patFinder:setPatternIDdictionary(db.configuration.patternIDs)
  patFinder:setDBScanParams(90, 6)

  pattern = {width=3, height=5, pointDist=0.0025}
  patFinder:setPatternData(pattern.width, pattern.height, pattern.pointDist)
  patFinder.debugParams.circleSearch=true
  patFinder.debugParams.clustering=true
  patFinder.debugParams.pose=true


  patFinder:setCamIntrinsics(calibData.scanCam.camMatrix)
  img = patFinder:RGBToGray(imgScan)

  local camPoseList = patFinder:processImg(img)
  print(camPoseList)
  local idIndex=-1
  for i,val in ipairs(camPoseList) do
    if val.id == -1 then
      idIndex=i
      break
    end
  end

  local camPoseMirrored
  if idIndex>0 then 
    print("pattern found")
    local pointsResorted = patFinder:mirrorPatternPoints(camPoseList[idIndex].points, pattern.width, pattern.height)
    local id=patFinder:getPatternId(imgScan, pointsResorted, pattern)
    print("ID: "..id)
    camPoseMirrored = patFinder:calcCamPoseMirrored(pointsResorted)
  end

  print("camPoseMirrored")
  print(camPoseMirrored)

end


main("multiPattern")
