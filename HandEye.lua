--[[
  HandEye.lua

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


-- Hand-Pattern or Hand-Eye calibration, depending on if we have an
-- extern or an onboard camera setup.
-- Tested with UR5 and SDA10D.

--package.path = package.path .. ";../../lua/auto_calibration/?.lua"
--package.path = package.path .. ";/home/xamla/Rosvita.Control/lua/auto_calibration/?.lua"
--local calib = require 'handEyeCalibration'
local motionLibrary = require 'xamlamoveit.motionLibrary'
local xutils = require 'xamlamoveit.xutils'
local datatypes = require 'xamlamoveit.datatypes'
local rosvita = require 'xamlamoveit.rosvita'

local cv = require "cv"
require "cv.highgui"
require "cv.videoio"
require "cv.imgproc"
require "cv.calib3d"
require "cv.imgcodecs"
require "cv.features2d"

require 'image'

require 'ximea.ros.XimeaClient'
--require 'GenICamCameraClient'
--require "multiPattern.PatternLocalisation"

local ros = require 'ros'
local tf = ros.tf

local M_PI = 3.14159265359
--local handEye = {}

local autocal = require 'auto_calibration.env'
local CalibrationMode = autocal.CalibrationMode
local HandEye = torch.class('autoCalibration.HandEye', autocal)


local function printf(...)
  print(string.format(...))
end


function HandEye:__init(configuration, calibration_folder_name, move_group, motion_service, camera_client, gripper, xamla_mg)
  self.configuration = configuration
  self.move_group = move_group
  self.camera_client = camera_client
  self.xamla_mg = xamla_mg
  self.motion_service = motion_service
  self.world_view_client = rosvita.WorldViewClient.new(motion_service.node_handle)

  self.move_groups = self.motion_service:getMoveGroup()  -- If no name is specified first move group is used (and this is linked to all endeffectors)
  self.gripper = gripper
  self.calibration_folder_name = calibration_folder_name

  -- current folder; contains links to the used calibration files
  self.current_path = path.join(configuration.output_directory, 'current')

  local left_camera = self.configuration.cameras[self.configuration.left_camera_id]
  local right_camera = self.configuration.cameras[self.configuration.right_camera_id]

  local mode = self.configuration.calibration_mode
  if mode == CalibrationMode.SingleCamera then
    -- assemble the calibration file name based on the serial of the camera
    if left_camera ~= nil then
      self.left_camera_serial = left_camera.serial
      self.calibration_fn_left = string.format('cam_%s.t7', self.left_camera_serial)
      self.calibration_path_left = path.join(self.current_path, self.calibration_fn_left)
    end
    if right_camera ~= nil then
      self.right_camera_serial = right_camera.serial
      self.calibration_fn_right = string.format('cam_%s.t7', self.right_camera_serial)
      self.calibration_path_right = path.join(self.current_path, self.calibration_fn_right)
    end
  elseif mode == CalibrationMode.StereoRig then
    -- assemble the stereo calibration file name based on the serials of the cameras
    if left_camera ~= nil and right_camera ~= nil then
      self.left_camera_serial = left_camera.serial
      self.right_camera_serial = right_camera.serial
    end
    self.calibration_fn = string.format('stereo_cams_%s_%s.t7', self.left_camera_serial, self.right_camera_serial)
    self.stereo_calibration_path = path.join(self.current_path, self.calibration_fn)
    self:loadStereoCalibration(self.stereo_calibration_path)
  end
  self.tcp_frame_of_reference, self.tcp_end_effector_name = self:getEndEffectorName()
end


function HandEye:loadStereoCalibration(stereo_calib_fn)
  -- check first if there is an existing stereo calibration file
  if path.exists(stereo_calib_fn) then
    local stereoCalib = torch.load(stereo_calib_fn)
    self.stereoCalibration = stereoCalib
    self.leftCameraMatrix = stereoCalib.camLeftMatrix
    self.rightCameraMatrix = stereoCalib.camRightMatrix
    self.leftDistCoeffs = stereoCalib.camLeftDistCoeffs
    self.rightDistCoeffs =  stereoCalib.camRightDistCoeffs
    self.rightLeftCamTrafo = stereoCalib.trafoLeftToRightCam
    print('read stereo calibration file '..stereo_calib_fn)
    return true
  else
    print('Calibration file '..stereo_calib_fn..' does not exist.')
    print('Please calibrate cameras first.')
    return false
  end
end


function HandEye:loadCalibration(calib_fn)
  -- check first if there is an existing calibration file
  if path.exists(calib_fn) then
    local calib = torch.load(calib_fn)
    self.calibration = calib
    self.cameraMatrix = calib.camMatrix
    self.distCoeffs = calib.distCoeffs
    print('read calibration file '..calib_fn)
    return true
  else
    print('Calibration file '..calib_fn..' does not exist.')
    print('Please calibrate camera first.')
    return false
  end
end


function HandEye:getEndEffectorName()
  local move_group_names, move_group_details = self.move_group.motion_service:queryAvailableMoveGroups()
  -- find out the index of the selected move_group
  local index = 1
  for i = 1, #move_group_names do
    if move_group_names[i] == self.configuration.move_group_name then
      index = i
      printf("Move group: %s (with index: %d)", self.configuration.move_group_name, index)
    end
  end
  local tcp_frame_of_reference = move_group_details[move_group_names[index]].end_effector_link_names[1]
  local tcp_end_effector_name = move_group_details[move_group_names[index]].end_effector_names[1]
  return tcp_frame_of_reference, tcp_end_effector_name
end


local function createPatternLocalizer(self)
  local pattern_geometry = self.configuration.circle_pattern_geometry
  local pattern_localizer = autocal.PatternLocalisation()
  pattern_localizer.circleFinderParams.minArea = 300
  pattern_localizer.circleFinderParams.maxArea = 4000
  pattern_localizer:setPatternIDdictionary(torch.load("/home/xamla/Rosvita.Control/lua/auto_calibration/patternIDdictionary.t7"))
  pattern_localizer:setDBScanParams(100, 10)
  pattern_localizer.debugParams = { circleSearch = false, clustering = false, pose = false }
  pattern_localizer:setPatternData(pattern_geometry[2], pattern_geometry[1], pattern_geometry[3])
  pattern_localizer:setStereoCalibration(self.stereoCalibration)
  self.pattern_localizer = pattern_localizer
end


function HandEye:captureImageNoWait(camera_configuration)
  local camera_serial = camera_configuration.serial
  local exposure = camera_configuration.exposure
  print("HandEye:captureImageNoWait: camera serial:")
  print(camera_serial)
  -- capture image
  if next(self.camera_client) ~= nil then
    self.camera_client:setExposure(exposure, {camera_serial})
  else
    print('No camera connected! -> No image capturing possible!')
    return nil
  end
  local image = self.camera_client:getImages({camera_serial})
  if image:nDimension() > 2 then
    image = cv.cvtColor{image, nil, cv.COLOR_RGB2BGR}
  end
  if image:nDimension() == 2 then
    image = cv.cvtColor{image, nil, cv.COLOR_GRAY2BGR}
  end
  return image
end


-- Note: This hand-eye calibration can only be used with a stereo camera setup!
-- params: imgData = {imgDataLeft = {imagePaths= {}}, imgDataRight = {imagePaths= {}}}
-- output: Returns the transformation camera_to_tcp or pattern_to_tcp, depending on if we have
--         an 'onboard' camera setup or an 'extern' camera setup
function HandEye:calibrate(imgData)

  --first load the latest calibration file
  local mode = self.configuration.calibration_mode
  local success = false
  if mode == CalibrationMode.SingleCamera then
    if imgData.imgDataLeft ~= nil then
      print('HandEye:calibrate loading calibration file: '..self.calibration_path_left)
      success = self:loadCalibration(self.calibration_path_left)
    elseif imgData.imgDataRight ~= nil then
      print('HandEye:calibrate loading calibration file: '..self.calibration_path_right)
      success = self:loadCalibration(self.calibration_path_right)
    end
  elseif mode == CalibrationMode.StereoRig then
    print('HandEye:calibrate loading calibration file: '..self.stereo_calibration_path)
    success = self:loadStereoCalibration(self.stereo_calibration_path)
  end
  if success == false then
    return
  end

  local output_path = path.join(self.configuration.output_directory, self.calibration_folder_name)
  -- the <calibration_name> folder has to exist or be created to be able to store the hand eye matrices
  os.execute('mkdir -p ' .. output_path)

  -- load calibration images and TCP data
  local imgDataLeft = imgData.imgDataLeft
  local imgDataRight = imgData.imgDataRight
  local imgDataSingle = imgDataLeft
  if imgDataLeft == nil then
    imgDataSingle = imgDataRight
  end

  local Hg = {}
  local Hc = {}

  -- extract pattern points:
  createPatternLocalizer(self)
  local imagesTakenForHandPatternCalib = {}
  if mode == CalibrationMode.StereoRig then
    for i, fn in ipairs(imgDataLeft.imagePaths) do
      local fnLeft = imgDataLeft.imagePaths[i]
      local fnRight = imgDataRight.imagePaths[i]
      local imgLeft = cv.imread {fnLeft}
      local imgRight = cv.imread {fnRight}
      local robotPose = imgData.jsposes.recorded_poses[i]
      local ok, patternPoseRelToCamera = self.pattern_localizer:calcCamPoseViaPlaneFit(imgLeft, imgRight, 'left', false, nil, self.configuration.circle_pattern_id)
      if ok then
        local cameraPoseRelToPattern = torch.inverse(patternPoseRelToCamera)
        local cameraPatternTrafo = cameraPoseRelToPattern -- This is used for an extern camera setup.
        if self.configuration.camera_location_mode == 'onboard' then
          -- We have an onboard camera setup, thus we have to use patternPoseRelToCamera.
          cameraPatternTrafo = patternPoseRelToCamera
        end
        table.insert(Hc, cameraPatternTrafo)
        table.insert(Hg, robotPose)
        table.insert(imagesTakenForHandPatternCalib, i)
      end
    end
  elseif mode == CalibrationMode.SingleCamera then
    for i, fn in ipairs(imgDataSingle.imagePaths) do
      local fn = imgDataSingle.imagePaths[i]
      local img = cv.imread {fn}
      local robotPose = imgData.jsposes.recorded_poses[i]
      local patternPoseRelToCamera, points3d = self.pattern_localizer:calcCamPose(img, self.cameraMatrix, self.pattern_localizer.pattern, false, nil, self.configuration.circle_pattern_id)
      if patternPoseRelToCamera ~= nil then
        local cameraPoseRelToPattern = torch.inverse(patternPoseRelToCamera)
        local cameraPatternTrafo = cameraPoseRelToPattern -- This is used for an extern camera setup.
        if self.configuration.camera_location_mode == 'onboard' then
          -- We have an onboard camera setup, thus we have to use patternPoseRelToCamera.
          cameraPatternTrafo = patternPoseRelToCamera
        end
        table.insert(Hc, cameraPatternTrafo)
        table.insert(Hg, robotPose)
        table.insert(imagesTakenForHandPatternCalib, i)
      end
    end
  end

  -- H = pose of the pattern/camera in TCP coordinate frame
  -- 'extern' camera setup: pattern pose in tcp coordinates
  -- 'onboard' camera setup: camera pose in tcp coordinates
  local H, res, res_angle = calib.calibrate(Hg, Hc)

  if self.configuration.camera_location_mode == 'onboard' then
    print("Temporary Hand-Eye matrix:") -- TCP <-> Camera
    print(H)
  else
    print("Temporary Hand-Pattern matrix:") -- TCP <-> Pattern
    print(H)
  end
  print("#Hc:")
  print(#Hc)
  print("#imagesTakenForHandPatternCalib:")
  print(#imagesTakenForHandPatternCalib)
  print("imagesTakenForHandPatternCalib:")
  print(imagesTakenForHandPatternCalib)

  local file_output_path = path.join(output_path, 'imagesTakenForHandPatternCalib.t7')
  torch.save(file_output_path, imagesTakenForHandPatternCalib)
  local link_target = path.join('..', self.calibration_folder_name, 'imagesTakenForHandPatternCalib.t7')
  local current_output_path = path.join(self.current_path, 'imagesTakenForHandPatternCalib.t7')
  os.execute('rm -f ' .. current_output_path)
  os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)

  -- perform cross validation
  local bestHESolution, alignmentErrorTest, alignmentError = calib.calibrateViaCrossValidation(Hg, Hc, #Hg-2, 5)
  if self.configuration.camera_location_mode == 'onboard' then
    print("Best Hand-Eye solution:") -- TCP <-> Camera
    print(bestHESolution)
  else
    print("Best Hand-Pattern solution:") -- TCP <-> Pattern
    print(bestHESolution)
  end
  bestHESolution = bestHESolution or H

  -- save result and create links at the 'current' folder
  if self.configuration.camera_location_mode == 'onboard' then
    file_output_path = path.join(output_path, 'HandEye.t7')
    torch.save(file_output_path, bestHESolution)
    link_target = path.join('..', self.calibration_folder_name, 'HandEye.t7')
    current_output_path = path.join(self.current_path, 'HandEye.t7')
    os.execute('rm -f ' .. current_output_path)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
    self.H_camera_to_tcp = bestHESolution
  else
    file_output_path = path.join(output_path, 'HandPattern.t7')
    torch.save(file_output_path, bestHESolution)
    link_target = path.join('..', self.calibration_folder_name, 'HandPattern.t7')
    current_output_path = path.join(self.current_path, 'HandPattern.t7')
    os.execute('rm -f ' .. current_output_path)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
    self.H_pattern_to_tcp = bestHESolution
  end

  if self.configuration.camera_location_mode == 'onboard' then
    file_output_path = path.join(output_path, 'Hc_patternToCam.t7')
    torch.save(file_output_path, Hc)
    link_target = path.join('..', self.calibration_folder_name, 'Hc_patternToCam.t7')
    current_output_path = path.join(self.current_path, 'Hc_patternToCam.t7')
    os.execute('rm -f ' .. current_output_path)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
  else
    file_output_path = path.join(output_path, 'Hc_camToPattern.t7')
    torch.save(file_output_path, Hc)
    link_target = path.join('..', self.calibration_folder_name, 'Hc_camToPattern.t7')
    current_output_path = path.join(self.current_path, 'Hc_camToPattern.t7')
    os.execute('rm -f ' .. current_output_path)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
  end

  file_output_path = path.join(output_path, 'Hg_tcpToBase.t7')
  torch.save(file_output_path, Hg)
  link_target = path.join('..', self.calibration_folder_name, 'Hg_tcpToBase.t7')
  current_output_path = path.join(self.current_path, 'Hg_tcpToBase.t7')
  os.execute('rm -f ' .. current_output_path)
  os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)

  -- calculate camera/pattern pose in base coordinates
  if self.configuration.camera_location_mode == 'onboard' then
    local patternBaseTrafo = Hg[1] * bestHESolution * Hc[1]
    print("base -> pattern trafo (i.e. pattern pose in base coordinates):")
    print(patternBaseTrafo)
    file_output_path = path.join(output_path, 'PatternBase.t7')
    torch.save(file_output_path, patternBaseTrafo)
    link_target = path.join('..', self.calibration_folder_name, 'PatternBase.t7')
    current_output_path = path.join(self.current_path, 'PatternBase.t7')
    os.execute('rm -f ' .. current_output_path)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
    return bestHESolution, patternBaseTrafo
  else
    local cameraBaseTrafo = Hg[1] * bestHESolution * Hc[1]
    if imgData.jsposes.recorded_pose_of_reference ~= nil then
      local cameraRefFrameTrafo = torch.inverse(imgData.jsposes.recorded_pose_of_reference) * cameraBaseTrafo
      print(string.format("%s -> camera trafo:", self.configuration.camera_reference_frame))
      if self.configuration.calibration_mode == "StereoRig" then
        print("Note: For a stereo setup, this is the pose of the left camera with serial: "..self.configuration.cameras[self.configuration.left_camera_id].serial)
        print(string.format("      in %s coordinates.", self.configuration.camera_reference_frame))
      end
      print(cameraRefFrameTrafo)
      self.H_cam_to_refFrame = cameraRefFrameTrafo
      file_output_path = path.join(output_path, string.format('LeftCam_%s.t7', self.configuration.camera_reference_frame))
      torch.save(file_output_path, cameraRefFrameTrafo)
      link_target = path.join('..', self.calibration_folder_name, string.format('LeftCam_%s.t7', self.configuration.camera_reference_frame))
      current_output_path = path.join(self.current_path, string.format('LeftCam_%s.t7', self.configuration.camera_reference_frame))
      os.execute('rm -f ' .. current_output_path)
      os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
      return bestHESolution, cameraRefFrameTrafo
    else
      print("base -> camera trafo (i.e. camera pose in base coordinates):")
      if self.configuration.calibration_mode == "StereoRig" then
        print("Note: For a stereo setup, this is the pose of the left camera with serial: "..self.configuration.cameras[self.configuration.left_camera_id].serial)
      end
      print(cameraBaseTrafo)
      self.H_cam_to_base = cameraBaseTrafo
      file_output_path = path.join(output_path, 'LeftCamBase.t7')
      torch.save(file_output_path, cameraBaseTrafo)
      link_target = path.join('..', self.calibration_folder_name, 'LeftCamBase.t7')
      current_output_path = path.join(self.current_path, 'LeftCamBase.t7')
      os.execute('rm -f ' .. current_output_path)
      os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
      return bestHESolution, cameraBaseTrafo
    end
  end
end


-- captures a camera image, or in case of 'StereoRig' mode a pair of stereo images
-- detects the camera<->pattern transformation
function HandEye:detectPattern()
  --1. capture pair of images
  local left_camera_config = self.configuration.cameras[self.configuration.left_camera_id]
  local right_camera_config = self.configuration.cameras[self.configuration.right_camera_id]
  local left_img
  local right_img
  local img
  local which
  if self.configuration.calibration_mode == CalibrationMode.SingleCamera then
    if left_camera_config ~= nil and right_camera_config ~= nil then
      print("Which camera has been used for hand-eye/pattern calibration?")
      print("Please again choose 1 or 2. Then press 'Enter'")
      print("1: left camera: "..left_camera_config.serial)
      print("2: right camera: "..right_camera_config.serial)
      which = io.read("*n")
      if which == 1 then
        img = self:captureImageNoWait(left_camera_config)
      elseif which == 2 then
        img = self:captureImageNoWait(right_camera_config)
      end
    elseif left_camera_config ~= nil then
      img = self:captureImageNoWait(left_camera_config)
    elseif right_camera_config ~= nil then
      img = self:captureImageNoWait(right_camera_config)
    end
  elseif self.configuration.calibration_mode == CalibrationMode.StereoRig then
    left_img = self:captureImageNoWait(left_camera_config)
    right_img = self:captureImageNoWait(right_camera_config)
  end
  --2. detect camera<->pattern trafo
  createPatternLocalizer(self)
  local ok = false
  local cameraPatternTrafo = nil
  if self.configuration.calibration_mode == CalibrationMode.StereoRig then
    if left_img == nil or right_img == nil then
      print("Left and/or right image are \'nil\'.")
      return nil
    end
    ok, cameraPatternTrafo = self.pattern_localizer:calcCamPoseViaPlaneFit(left_img, right_img, 'left', false, nil, self.configuration.circle_pattern_id)
    if not ok then
      return nil
    end
  elseif self.configuration.calibration_mode == CalibrationMode.SingleCamera then
    if img == nil then
      print("Image is \'nil\'.")
      return false, nil
    end
    cameraPatternTrafo, points3d = self.pattern_localizer:calcCamPose(img, self.cameraMatrix, self.pattern_localizer.pattern, false, nil, self.configuration.circle_pattern_id)
    if cameraPatternTrafo == nil then
      return nil
    end
  end
  return cameraPatternTrafo
end


function HandEye:generateRelativeRotation(O)
  local O = O or torch.eye(4,4)
  local q = tf.Quaternion.new()
  local roll = 0.1 -- 0.0
  local pitch = -0.1 -- -0.3
  local yaw = 0.0
  q:setEuler(yaw, pitch, roll)
  local R = torch.eye(4,4)
  R[{{1,3}, {1,3}}] = q:toMatrixTensor()
  print(R)
  return O*R
end

function HandEye:generateRelativeTranslation(O)
  local O = O or torch.eye(4,4)
  local x_offset = 0.0
  local y_offset = 0.0
  local z_offset = 0.0
  H = torch.eye(4,4)
  H[1][4] = x_offset
  H[2][4] = y_offset
  H[3][4] = z_offset
  return O*H
end


-- captures a camera image, or in case of 'StereoRig' mode a pair of stereo images
-- detects the camera<->pattern transformation
-- predicts the camera<->pattern transformation for the case of a robot motion
-- performs the robot motion
function HandEye:movePattern()
  -- set the folder to 'current'
  local files_path = self.current_path
  print('Calling HandEye:movePattern() method')
  --if the calibration data is missing, read it from the file
  if self.configuration.camera_location_mode == 'onboard' then
    if self.H_camera_to_tcp == nil then
      self.H_camera_to_tcp = torch.load(files_path .. "/HandEye.t7")
    end
  else
    if self.H_pattern_to_tcp == nil then
      self.H_pattern_to_tcp = torch.load(files_path .. "/HandPattern.t7")
    end
  end

  --1. move to the last posture of the taught capture postures
  local last_pose = self.configuration.capture_poses[#self.configuration.capture_poses]
  print("Moving to the last pose of the taught capture poses ...")
  self.move_group:moveJoints(last_pose)

  --2. detect the corresponding camera<->pattern trafo
  local left_camera_config = self.configuration.cameras[self.configuration.left_camera_id]
  local right_camera_config = self.configuration.cameras[self.configuration.right_camera_id]
  local left_img
  local right_img
  local img
  local which

  if self.configuration.calibration_mode == CalibrationMode.SingleCamera then
    if left_camera_config ~= nil and right_camera_config ~= nil then
      print("Which camera has been used for hand-eye/pattern calibration?")
      print("Please choose 1 or 2. Then press 'Enter'")
      print("1: left camera: "..left_camera_config.serial)
      print("2: right camera: "..right_camera_config.serial)
      which = io.read("*n")
      if which == 1 then
        img = self:captureImageNoWait(left_camera_config)
      elseif which == 2 then
        img = self:captureImageNoWait(right_camera_config)
      end
    elseif left_camera_config ~= nil then
      img = self:captureImageNoWait(left_camera_config)
    elseif right_camera_config ~= nil then
      img = self:captureImageNoWait(right_camera_config)
    end
  elseif self.configuration.calibration_mode == CalibrationMode.StereoRig then
    left_img = self:captureImageNoWait(left_camera_config)
    right_img = self:captureImageNoWait(right_camera_config)
  end

  createPatternLocalizer(self)

  local ok = false
  local cameraPatternTrafo = nil
  if self.configuration.calibration_mode == CalibrationMode.StereoRig then
    if left_img == nil or right_img == nil then
      print("Left and/or right image are \'nil\'.")
      return false, nil
    end
    ok, cameraPatternTrafo = self.pattern_localizer:calcCamPoseViaPlaneFit(left_img, right_img, 'left', false, nil, self.configuration.circle_pattern_id)
    if not ok then
      print('pattern not found!')
      return ok, cameraPatternTrafo
    end
  elseif self.configuration.calibration_mode == CalibrationMode.SingleCamera then
    if img == nil then
      print("Image is \'nil\'.")
      return false, nil
    end
    cameraPatternTrafo, points3d = self.pattern_localizer:calcCamPose(img, self.cameraMatrix, self.pattern_localizer.pattern, false, nil, self.configuration.circle_pattern_id)
    if cameraPatternTRafo == nil then
      print('pattern not found!')
      return false, cameraPatternTrafo
    end
  end
  print('detected cameraPatternTrafo before motion:')
  print(cameraPatternTrafo)

  --3. compute a relative transformation (corresponding to a motion of the robot)
  --   and compute (i.e. predict) the camera<->pattern trafo after this motion
  if self.configuration.camera_location_mode == 'onboard' then -- 'onboard' camera setup
    if self.H_camera_to_tcp ~= nil then
      print('about to do a movement..')
      local relative_transformation = self:generateRelativeRotation()
      relative_transformation = self:generateRelativeTranslation(relative_transformation)
      self.predicted_cameraPatternTrafo = cameraPatternTrafo * relative_transformation
      print('prediction for cameraPatternTrafo after motion:')
      print(self.predicted_cameraPatternTrafo)
      print('compare with the next pattern detection:')
      local pose_tcp = self.H_camera_to_tcp * (relative_transformation * torch.inverse(self.H_camera_to_tcp))
      local current_pose_tcp = self.move_group:getCurrentPose()
      local pose_tcp_in_base_coord = current_pose_tcp:toTensor() * pose_tcp
      local transf = tf.Transform.new()
      transf:fromTensor(pose_tcp_in_base_coord)
      print("transf:")
      print(transf)
      local datatypes_pose_tcp = datatypes.Pose()
      datatypes_pose_tcp:setTranslation(pose_tcp_in_base_coord[{{1,3},4}])
      datatypes_pose_tcp:setRotation(transf:getRotation())
      local collision_check = false
      local end_effector = self.move_group:getEndEffector(self.tcp_end_effector_name)
      print("Moving the robot slightly ...")
      --end_effector:movePoseLinear(datatypes_pose_tcp, self.configuration.velocity_scaling, collision_check)
      end_effector:movePoseLinear(datatypes_pose_tcp, 0.8, collision_check, 0.5)
      return ok, self.predicted_cameraPatternTrafo
    else
      print('please calibrate the robot first')
    end
  else -- 'extern' camera setup
    if self.H_pattern_to_tcp ~= nil then
      print('about to do a movement..')
      local relative_transformation = self:generateRelativeRotation()
      relative_transformation = self:generateRelativeTranslation(relative_transformation)
      self.predicted_cameraPatternTrafo = cameraPatternTrafo * relative_transformation
      print('prediction for cameraPatternTrafo after motion:')
      print(self.predicted_cameraPatternTrafo)
      print('compare with the next pattern detection:')
      local pose_tcp = self.H_pattern_to_tcp * (relative_transformation * torch.inverse(self.H_pattern_to_tcp))
      local current_pose_tcp = self.move_group:getCurrentPose()
      local pose_tcp_in_base_coord = current_pose_tcp:toTensor() * pose_tcp
      local transf = tf.Transform.new()
      transf:fromTensor(pose_tcp_in_base_coord)
      local datatypes_pose_tcp = datatypes.Pose()
      datatypes_pose_tcp:setTranslation(pose_tcp_in_base_coord[{{1,3},4}])
      datatypes_pose_tcp:setRotation(transf:getRotation())
      local collision_check = false
      local end_effector = self.move_group:getEndEffector(self.tcp_end_effector_name)
      print("Moving the robot slightly ...")
      --end_effector:movePoseLinear(datatypes_pose_tcp, self.configuration.velocity_scaling, collision_check)
      end_effector:movePoseLinear(datatypes_pose_tcp, 0.8, collision_check, 0.5)
      return ok, self.predicted_cameraPatternTrafo
    else
      print('please calibrate the robot first')
    end
  end
end


-- Compute some metric for the evaluation of the hand-eye calibration
local function metricCalculation(prediction, detection)
  local H1 = tf.Transform.new()
  local H2 = tf.Transform.new()
  H1:fromTensor(prediction)
  H2:fromTensor(detection)
  local R1 = prediction[{{1,3}, {1,3}}]
  local R2 = detection[{{1,3}, {1,3}}]

  -- translation error (norm of difference)
  local err_t = torch.norm(prediction[{{1,4}, {4}}] - detection[{{1,4}, {4}}])
  -- translation error for each axis
  local err_axes = prediction[{{1,4}, {4}}] - detection[{{1,4}, {4}}]
  -- rotation error (norm of difference of Euler angles)
  local err_r = torch.norm(H1:getRotation():toTensor() - H2:getRotation():toTensor())

  -- error given  by  the  angle  from  the  axis–angle  representation  of rotation
  -- the angle of rotation of a matrix R in the axis–angle representation is given by arccos( {Tr(R) -1} /2)
  -- if R1 ~= R2 => R1 * R2.inv() ~= Identity => angle of rotation ~= 0
  -- => arccos(angle of rotation) ~= pi/2 = 1.57..
  local err_r2 = torch.acos( torch.trace(R1* torch.inverse(R2) -1) / 2 )
  local err_r3 = torch.abs(err_r2 - M_PI/2)

  print('translation error (norm of difference) [in m]:', err_t, ' ( = ', err_t * 1000, ' mm)')
  print('translation error for each axis [in mm]:')
  print(err_axes * 1000)
  print('euler angles prediction:')
  print(H1:getRotation():toTensor())
  print('euler angles detection:')
  print(H2:getRotation():toTensor())
  print('rotation error (norm of difference of Euler angles) [in radians]:', err_r, ' ( = ', err_r * 180/M_PI, ' degree)')
  print('rotation error metric #2 [in radians]:', err_r3, ' ( = ', err_r3 * 180/M_PI, ' degree)')

  return err_t, err_axes, err_r, err_r3
end


-- Evaluation of the hand eye calibration (with only one robot movement)
function HandEye:evaluateCalibration()
  print('HandEye:evaluateCalibration()')
  local ok, prediction = self:movePattern() -- stores the predicted pose of the pattern at self.predicted_cameraPatternTrafo and returns it
  if not ok then
    print('aborting evaluation')
    return
  end
  local detection = self:detectPattern()
  if detection == nil then
    print('pattern not detected, aborting evaluation')
    return
  end
  print('detected cameraPatternTrafo after motion:')
  print(detection)

  -- compute some metric
  metricCalculation(prediction, detection)
end


function HandEye:publishHandEye()
  local files_path = self.current_path
  if self.configuration.camera_location_mode == 'onboard' then
    if self.H_camera_to_tcp == nil then
      self.H_camera_to_tcp = torch.load(files_path .. "/HandEye.t7")
      if self.H_camera_to_tcp == nil then
        print("could not find HandEye.t7")
        return
      end
    end
    print(self.H_camera_to_tcp)
    local hand_eye_pose = datatypes.Pose()
    hand_eye_pose.stampedTransform:fromTensor(self.H_camera_to_tcp)
    hand_eye_pose:setFrame(self.tcp_frame_of_reference)
    local ok, error = self.world_view_client:addPose("HandEye",'', hand_eye_pose)
    if not ok then
      ok, error = self.world_view_client:updatePose("HandEye", '', hand_eye_pose)
      if not ok then
        print(error)
      end
    end
  elseif self.configuration.camera_location_mode == 'extern' then
    if self.configuration.camera_reference_frame == 'BASE' or self.configuration.camera_reference_frame == nil then
      if self.H_cam_to_base == nil then
        self.H_cam_to_base = torch.load(files_path .. "/LeftCamBase.t7")
        if self.H_cam_to_base == nil then
          print("could not find LeftCamBase.t7")
          return
        end
      end
      print(self.H_cam_to_base)
      local cam_base_pose = datatypes.Pose()
      cam_base_pose.stampedTransform:fromTensor(self.H_cam_to_base)
      cam_base_pose:setFrame('world')
      local ok, error = self.world_view_client:addPose("CamBase", '', cam_base_pose)
      if not ok then
        ok, error = self.world_view_client:updatePose("CamBase", '', cam_base_pose)
        if not ok then
          print(error)
        end
      end
    else
      if self.H_cam_to_refFrame == nil then
        self.H_cam_to_refFrame = torch.load(files_path .. string.format("/LeftCam_%s.t7", self.configuration.camera_reference_frame))
        if self.H_cam_to_refFrame == nil then
          print(string.format("could not find LeftCam_%s.t7", self.configuration.camera_reference_frame))
          return
        end
      end
      print(self.H_cam_to_refFrame)
      local cam_refFrame_pose = datatypes.Pose()
      cam_refFrame_pose.stampedTransform:fromTensor(self.H_cam_to_refFrame)
      local link_name = string.gsub(self.configuration.camera_reference_frame, "joint", "link")
      cam_refFrame_pose:setFrame(link_name)
      local ok, error = self.world_view_client:addPose(string.format("Cam_%s", link_name), '', cam_refFrame_pose)
      if not ok then
        ok, error = self.world_view_client:updatePose(string.format("Cam_%s", link_name), '', cam_refFrame_pose)
        if not ok then
          print(error)
        end
      end
    end
  end
end

-- Evaluation of the hand eye calibration (with several robot movements)
-- Details:
-- For several, previously taught evaluation poses (i.e. joint configurations)
-- the corresponding tcp poses are calculated via forward kinematic and
-- the new camera<->pattern trafo is predicted for each of these poses.
-- After that the robot moves are actually performed and the new camera<->pattern trafos
-- are measured via plane fit (for stereo setup) or solvePnP (for single camera setup)
-- and compared with the prediction.
function HandEye:evaluateCalibrationComplex()

  -- set the folder to 'current'
  local files_path = self.current_path
  -- if the calibration data is missing, read it from the file
  if self.configuration.camera_location_mode == 'onboard' then
    if self.H_camera_to_tcp == nil then
      self.H_camera_to_tcp = torch.load(files_path .. "/HandEye.t7")
    end
  else
    if self.H_pattern_to_tcp == nil then
      self.H_pattern_to_tcp = torch.load(files_path .. "/HandPattern.t7")
    end
  end

  --1. move to the last posture of the taught capture postures
  local last_pose = self.configuration.capture_poses[#self.configuration.capture_poses]
  print("Moving to the last pose of the taught capture poses ...")
  self.move_group:moveJoints(last_pose)
  local tcp_old = self.move_group:getCurrentPose():toTensor()

  --2. determine the corresponding camera<->pattern transformation
  print("Calculation of the camera<->pattern transformation..")
  local left_camera_config = self.configuration.cameras[self.configuration.left_camera_id]
  local right_camera_config = self.configuration.cameras[self.configuration.right_camera_id]
  local left_img
  local right_img
  local img
  local which

  if self.configuration.calibration_mode == CalibrationMode.SingleCamera then
    if left_camera_config ~= nil and right_camera_config ~= nil then
      print("Which camera has been used for hand-eye/pattern calibration?")
      print("Please choose 1 or 2. Then press 'Enter'")
      print("1: left camera: "..left_camera_config.serial)
      print("2: right camera: "..right_camera_config.serial)
      which = io.read("*n")
      if which == 1 then
        img = self:captureImageNoWait(left_camera_config)
      elseif which == 2 then
        img = self:captureImageNoWait(right_camera_config)
      end
    elseif left_camera_config ~= nil then
      img = self:captureImageNoWait(left_camera_config)
    elseif right_camera_config ~= nil then
      img = self:captureImageNoWait(right_camera_config)
    end
  elseif self.configuration.calibration_mode == CalibrationMode.StereoRig then
    left_img = self:captureImageNoWait(left_camera_config)
    right_img = self:captureImageNoWait(right_camera_config)
  end

  createPatternLocalizer(self)

  local ok = false
  local cameraPatternTrafo_old = nil
  if self.configuration.calibration_mode == CalibrationMode.StereoRig then
    if left_img == nil or right_img == nil then
      print("Left and/or right image are \'nil\'.")
      return
    end
    ok, cameraPatternTrafo_old = self.pattern_localizer:calcCamPoseViaPlaneFit(left_img, right_img, 'left', false, nil, self.configuration.circle_pattern_id)
    if not ok then
      print('pattern not found!')
      return ok, cameraPatternTrafo_old
    end
  elseif self.configuration.calibration_mode == CalibrationMode.SingleCamera then
    if img == nil then
      print("Image is \'nil\'.")
      return false, nil
    end
    cameraPatternTrafo_old, points3d = self.pattern_localizer:calcCamPose(img, self.cameraMatrix, self.pattern_localizer.pattern, false, nil, self.configuration.circle_pattern_id)
    if cameraPatternTrafo_old == nil then
      print('pattern not found!')
      return false, cameraPatternTrafo_old
    end
  end
  if self.configuration.camera_location_mode == 'extern' then
    cameraPatternTrafo_old = torch.inverse(cameraPatternTrafo_old)
  end
  print("old camera<->pattern trafo:")
  print(cameraPatternTrafo_old)

  -- 3. for each evaluation pose predict the corresponding camera<->pattern trafo
  --    and compare it with the measured camera<->pattern trafo after movement
  local end_effector = self.move_group:getEndEffector(self.tcp_end_effector_name)
  local eval_poses = self.configuration.eval_poses
  if next(eval_poses) == nil then
    print('No evaluation poses taught! Teach evaluation poses first.')
    return eval_poses
  end
  local translation_error = {}
  local translation_error_axes = {}
  local rotation_error1 = {}
  local rotation_error2 = {}
  local err_t_avg, err_t_axes_avg, err_r1_avg, err_r2_avg = 0, 0, 0, 0
  local count = 0
  for i = 1, #eval_poses do
    print(string.format("Predict the camera<->pattern trafo for evaluation pose %d:", i))
    assert(torch.isTypeOf(eval_poses[i], datatypes.JointValues), string.format("Evaluation pose %d is not of type \'datatypes.JointValues\'!", i))
    local tcp_new = end_effector:computePose(eval_poses[i]):toTensor()
    local prediction
    if self.configuration.camera_location_mode == 'onboard' then
      prediction = torch.inverse(self.H_camera_to_tcp) * torch.inverse(tcp_new) * tcp_old * self.H_camera_to_tcp * cameraPatternTrafo_old
    else
      prediction = torch.inverse(self.H_pattern_to_tcp) * torch.inverse(tcp_new) * tcp_old * self.H_pattern_to_tcp * cameraPatternTrafo_old
    end
    print("Prediction of new camera<->pattern trafo:")
    print(prediction)

    print('Compare with pattern detection:')

    print(string.format("Moving to evaluation pose %d ...", i))
    self.move_group:moveJoints(eval_poses[i])
    tcp_new = self.move_group:getCurrentPose():toTensor()

    local ok = false
    local detection = nil
    if self.configuration.calibration_mode == CalibrationMode.SingleCamera then
      if left_camera_config ~= nil and right_camera_config ~= nil then
        if which == 1 then
          img = self:captureImageNoWait(left_camera_config)
        elseif which == 2 then
          img = self:captureImageNoWait(right_camera_config)
        end
      elseif left_camera_config ~= nil then
        img = self:captureImageNoWait(left_camera_config)
      elseif right_camera_config ~= nil then
        img = self:captureImageNoWait(right_camera_config)
      end
      if img == nil then
        print('Please connect cameras first!')
        return false
      end
      detection, points3d = self.pattern_localizer:calcCamPose(img, self.cameraMatrix, self.pattern_localizer.pattern, false, nil, self.configuration.circle_pattern_id)
      if detection == nil then
        print('Pattern not found! Taking the next image..')
      else
        if self.configuration.camera_location_mode == 'extern' then
          detection = torch.inverse(detection)
        end
        print("Detected new camera<->pattern trafo:")
        print(detection)
        -- compute some metric
        translation_error[i], translation_error_axes[i], rotation_error1[i], rotation_error2[i] = metricCalculation(prediction, detection)
        err_t_avg = err_t_avg + translation_error[i]
        err_t_axes_avg = err_t_axes_avg + translation_error_axes[i]
        err_r1_avg = err_r1_avg + rotation_error1[i]
        err_r2_avg = err_r2_avg + rotation_error2[i]
        count = count + 1
        tcp_old = tcp_new
        cameraPatternTrafo_old = detection
      end
    elseif self.configuration.calibration_mode == CalibrationMode.StereoRig then
      left_img = self:captureImageNoWait(left_camera_config)
      right_img = self:captureImageNoWait(right_camera_config)
      if left_img == nil or right_img == nil then
        print('Please connect cameras first!')
        return false
      end
      ok, detection = self.pattern_localizer:calcCamPoseViaPlaneFit(left_img, right_img, 'left', false, nil, self.configuration.circle_pattern_id)
      if not ok then
        print('Pattern not found! Taking the next image..')
      else
        if self.configuration.camera_location_mode == 'extern' then
          detection = torch.inverse(detection)
        end
        print("Detected new camera<->pattern trafo:")
        print(detection)
        -- compute some metric
        translation_error[i], translation_error_axes[i], rotation_error1[i], rotation_error2[i] = metricCalculation(prediction, detection)
        err_t_avg = err_t_avg + translation_error[i]
        err_t_axes_avg = err_t_axes_avg + translation_error_axes[i]
        err_r1_avg = err_r1_avg + rotation_error1[i]
        err_r2_avg = err_r2_avg + rotation_error2[i]
        count = count + 1
        tcp_old = tcp_new
        cameraPatternTrafo_old = detection
      end
    end
  end

  err_t_avg = err_t_avg / count
  err_t_axes_avg = err_t_axes_avg / count
  err_r1_avg = err_r1_avg / count
  err_r2_avg = err_r2_avg / count
  print('========================================================================================')
  print('average translation error (norm of difference) [in m]:', err_t_avg, ' ( = ', err_t_avg * 1000, ' mm)')
  print('average translation error for each axis [in mm]:')
  print(err_t_axes_avg * 1000)
  print('average rotation error (norm of difference of Euler angles) [in radians]:', err_r1_avg, ' ( = ', err_r1_avg * 180/M_PI, ' degree)')
  print('average rotation error metric #2 [in radians]:', err_r2_avg, ' ( = ', err_r2_avg * 180/M_PI, ' degree)')
  print('========================================================================================')
end


return HandEye
