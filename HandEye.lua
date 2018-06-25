--[[
  Hand-Pattern or Hand-Eye calibration, depending on if we have an extern
  or an onboard camera setup.
  Current limitations:
  * Only working with stereo camera setup
  * Only working with our standard calibration pattern (circle pattern of id 21)
  * Only tested with UR5 and SDA10D (especially if we have more than one move group, the current implementation will probably only work with an SDA)
  * Only working with ximea cameras
  * The "evaluateCalibration" function only evaluates the calculation of the "Camera<->Pattern transformation via plane fit. The TCP<->Pattern transformation (or TCP<->Camera transformation in case of 'onboard') is not evaluated.
]]
local xamla3d = require 'xamla3d'
local calib = require 'handEyeCalibration'
local motionLibrary = require 'xamlamoveit.motionLibrary'
local xutils = require 'xamlamoveit.xutils'
local datatypes = require 'xamlamoveit.datatypes'

local cv = require "cv"
require "cv.highgui"
require "cv.videoio"
require "cv.imgproc"
require "cv.calib3d"
require "cv.imgcodecs"
require "cv.features2d"

require 'image'

local pcl = require 'pcl'

require 'ximea.ros.XimeaClient'
require 'GenICamClient'
require "multiPattern.PatternLocalisation"

local ros = require 'ros'
local tf = ros.tf
local listener = tf.TransformListener
local debug = require 'DebugTools'

local function tryRequire(module_name)
  local ok, val = pcall(function() return require(module_name) end)
  if ok then
    return val
  else
    return nil
  end
end

local slstudio = tryRequire('slstudio')

local offline = true  -- in case we are reading images from files and not really connecting to the driver set offline to true

-- Important: Everything has to be in meters.

local DEFAULT_CIRCLE_PATTERN_GEOMETRY = torch.Tensor({21, 8, 0.005})
local DEFAULT_CIRCLE_PATTERN_ID = 21

local patternLocalizer = PatternLocalisation()
patternLocalizer.circleFinderParams.minArea = 500
patternLocalizer.circleFinderParams.maxArea = 4000
patternLocalizer:setPatternIDdictionary(
  torch.load("patternIDdictionary.t7")
)
patternLocalizer:setDBScanParams(130, 10)
patternLocalizer:setPatternData(
  DEFAULT_CIRCLE_PATTERN_GEOMETRY[2],
  DEFAULT_CIRCLE_PATTERN_GEOMETRY[1],
  DEFAULT_CIRCLE_PATTERN_GEOMETRY[3]
)
local M_PI = 3.14159265359

local handEye = {}


local MARKER_ID = 21 -- currently only working with our standard pattern for calibration
--local BEAMER_CALIBRATION_FILENAME = "/home/xamla/Rosvita.Control/projects/UR5/sls_2018-03-02_155849.xml"

local HandEye = torch.class('autoCalibration.HandEye', handEye)

local marker_frame_id = 'int_marker'
local world_frame_id = 'world'
local pcloud_frame_id = 'camera_left'
local left_camera_frame_id = 'camera_left'
local desired_tcp_frame_id = 'desired_tcp'
local picking_pose_frame_id = 'picking_pose'
local pre_picking_pose_frame_id = 'pre_picking_pose'
local pattern_pcloud_frame_id = 'pattern_cloud'
local pattern_frame_id = 'pattern'
local torso_frame_id = 'torso_link_b1'


local function fprint(...)
    print(string.format(...))
end


function HandEye:__init(configuration, calibration_folder_name, move_group, motion_service, camera_client, gripper, xamla_mg)
  self.cloud_topics = {'point_cloud', 'filtered_cloud'}
  self.img_topics = {'pattern_detection_left', 'pattern_detection_right'}
  self.debug = debug.new(self.cloud_topics, self.img_topics)
  print('debug class initialised')
  self.configuration = configuration
  self.move_group = move_group
  self.camera_client = camera_client
  self.fingertip_frame_id = 'tcp_fingertip_link'
  self.xamla_mg = xamla_mg
  self.motion_service = motion_service

  self.move_groups = self.motion_service:getMoveGroup()  -- If no name is specified first move group is used (and this is linked to all endeffectors)
  self.rc = self:getEndEffectors(self.move_groups) -- gets all endeffectors of the first move group

  self.right_move_group, self.left_move_group, self.both_move_group, self.xamla_mg_both = nil, nil, nil, nil
  if self.rc ~= nil then
    self.right_move_group = self.motion_service:getMoveGroup(self.rc.right_move_group_name)
    self.left_move_group = self.motion_service:getMoveGroup(self.rc.left_move_group_name)
    self.both_move_group = self.motion_service:getMoveGroup(self.rc.both_move_group_name)
    self.xamla_mg_both = motionLibrary.MoveGroup(self.motion_service, self.rc.both_move_group_name) -- motion client
  end

  self.gripper = gripper
  self.calibration_folder_name = calibration_folder_name

  -- current folder; contains links to the used calibration files
  self.current_path = path.join(configuration.output_directory, 'current')

  -- assemble the stereo calibration file name based on the serials of the cameras
  local left_camera = self.configuration.cameras[configuration.left_camera_id]
  local right_camera = self.configuration.cameras[configuration.right_camera_id]
  self.left_camera_serial = left_camera.serial
  self.right_camera_serial = right_camera.serial
  self.calibration_fn = string.format('stereo_cams_%s_%s.t7', self.left_camera_serial, self.right_camera_serial)
  self.stereo_calibration_path = path.join(self.current_path, self.calibration_fn)
  self:loadStereoCalibration(self.stereo_calibration_path)
  self.tcp_frame_of_reference, self.tcp_end_effector_name = self:getEndEffectorName()

  -- request the tf between the fingertips and the current end effector
  print('self:requestTf...', self.tcp_frame_of_reference, ' ', self.tcp_frame_of_reference)
  local tf_available, H_fingers_to_tcp = self.debug:requestTf(self.tcp_frame_of_reference, self.tcp_frame_of_reference)
  if not tf_available then
    print('tf not available! '..self.fingertip_frame_id..' to '..self.tcp_end_effector_name)
    return
  end
  self.H_fingers_to_tcp = H_fingers_to_tcp
  print('self.H_fingers_to_tcp')
  print(self.H_fingers_to_tcp)
end


function HandEye:loadStereoCalibration(stereo_calib_fn)
  -- check first if there is an existing stereo calibration file
  if path.exists(stereo_calib_fn) then
    local stereoCalib = torch.load(stereo_calib_fn)
    patternLocalizer:setStereoCalibration(stereoCalib)
    self.leftCameraMatrix = stereoCalib.camLeftMatrix --stereoCalib.intrinsicLeftCam
    self.rightCameraMatrix = stereoCalib.camRightMatrix --stereoCalib.intrinsicRightCam
    self.leftDistCoeffs = stereoCalib.camLeftDistCoeffs --stereoCalib.distLeftCam
    self.rightDistCoeffs =  stereoCalib.camRightDistCoeffs --stereoCalib.distRightCam
    self.rightLeftCamTrafo = stereoCalib.trafoLeftToRightCam  --stereoCalib.trafoLeftCamRightCam
    print('read stereo calibration file '..stereo_calib_fn)
    --print("stereo calibration:")
    --print(stereoCalib)
    --print("patternLocalizer.stereoCalibration:")
    --print(patternLocalizer.stereoCalibration)
  end
end


function HandEye:getEndEffectorName()
  local move_group_names, move_group_details = self.move_group.motion_service:queryAvailableMoveGroups()
  print('HandEye:getEndEffectorName() move_group_names')
  print(move_group_names)
  --print(move_group_details)

  -- find out the index of the selected move_group
  local index = 1
  for i = 1, #move_group_names do
    if move_group_names[i] == self.configuration.move_group_name then
      index = i
      print(self.configuration.move_group_name, ' index = ', index)
    end
  end
  --local index = 1
  local tcp_frame_of_reference = move_group_details[move_group_names[index]].end_effector_link_names[1]
  local tcp_end_effector_name = move_group_details[move_group_names[index]].end_effector_names[1]
  return tcp_frame_of_reference,tcp_end_effector_name
end


function HandEye:getEndEffectors(move_groups)
  local move_group_names, move_group_details = move_groups.motion_service:queryAvailableMoveGroups()
  local rc = {}
  print('HandEye:getEndEffectors(): move_group_names')
  print(move_group_names)

  if #move_group_names == 1 then -- only one move group (e.g. UR)
    rc.tcp_frame_of_reference = move_group_details[move_group_names[1]].end_effector_link_names[1]
    rc.tcp_end_effector_name = move_group_details[move_group_names[1]].end_effector_names[1]
    rc.move_group_name = move_group_names[1]
  end

  if #move_group_names >= 6 then -- 6 or more move groups (e.g. SDA)
    -- the following is special for SDA
    local index = 5
    rc.left_tcp_frame_of_reference = move_group_details[move_group_names[index]].end_effector_link_names[1]
    rc.left_tcp_end_effector_name = move_group_details[move_group_names[index]].end_effector_names[1]
    rc.left_move_group_name = move_group_names[index]

    index = 6
    rc.right_tcp_frame_of_reference = move_group_details[move_group_names[index]].end_effector_link_names[1]
    rc.right_tcp_end_effector_name = move_group_details[move_group_names[index]].end_effector_names[1]
    rc.right_move_group_name = move_group_names[index]

    index = 1
    rc.both_tcp_frame_of_reference = move_group_details[move_group_names[index]].end_effector_link_names[1]
    rc.both_tcp_end_effector_name = move_group_details[move_group_names[index]].end_effector_names[1]
    rc.both_move_group_name = move_group_names[index]
  end

  return rc
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

  --first load the latest stereo calibration file
  print('HandEye:calibrate loading calibration file: '..self.stereo_calibration_path)
  self:loadStereoCalibration(self.stereo_calibration_path)

  local output_path = path.join(self.configuration.output_directory, self.calibration_folder_name)
  -- the <calibration_name> folder has to exist or be created to be able to store the hand eye matrices
  os.execute('mkdir -p ' .. output_path)

  -- load calibration images and TCP data
  local imgDataLeft = imgData.imgDataLeft
  local imgDataRight = imgData.imgDataRight

  local Hg = {}
  local Hc = {}

  -- extract pattern points:
  local imagesTakenForHandPatternCalib = {}
  for i, fn in ipairs(imgDataLeft.imagePaths) do

    local fnLeft = imgDataLeft.imagePaths[i]
    local fnRight = imgDataRight.imagePaths[i]
    local imgLeft = cv.imread {fnLeft}
    local imgRight = cv.imread {fnRight}

    -- Note: If MoveGroup 'endeffector' has been used, then TCP is the gripper tip.
    -- Use wrist_3_link i.e. flanch pose instead of gripper tip pose.

    local robotPose = imgData.jsposes.recorded_poses[i]
    --local robotPose = imgDataLeft.jointPoses[i].ee_link
    --local robotPose = imgDataLeft.jointPoses[i].wrist_3_link

    print(imgLeft:size(1))
    print(imgLeft:size(2))
    print(imgLeft:size(3))

    local ok, patternPoseRelToCamera = patternLocalizer:calcCamPoseViaPlaneFit(imgLeft, imgRight, 'left')
    -- alternatively, use the monocular approach
    --local patternPoseRelToCamera, points3d = patternLocalizer:calcCamPose(imgLeft, leftCameraMatrix, patternLocalizer.pattern)

    if ok then
      local cameraPoseRelToPattern = torch.inverse(patternPoseRelToCamera)
      local cameraPatternTrafo = cameraPoseRelToPattern
      if self.configuration.camera_location_mode == 'onboard' then
        print('We have an onboard camera setup, thus we have to use \"patternPoseRelToCamera\".')
        cameraPatternTrafo = patternPoseRelToCamera
      else
        print('We have an extern camera setup, thus we have to use \"cameraPoseRelToPattern\"')
      end
      table.insert(Hc, cameraPatternTrafo)
      table.insert(Hg, robotPose)
      table.insert(imagesTakenForHandPatternCalib, i)
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
  self.debug:publishTf(bestHESolution, self.tcp_frame_of_reference, "pattern_HE")

  -- save result
  if self.configuration.camera_location_mode == 'onboard' then
    file_output_path = path.join(output_path, 'HandEye.t7')
    torch.save(file_output_path, bestHESolution)
    link_target = path.join('..', self.calibration_folder_name, 'HandEye.t7')
    current_output_path = path.join(self.current_path, 'HandEye.t7')
    os.execute('rm ' .. current_output_path)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
    self.H_camera_to_tcp = bestHESolution
  else
    file_output_path = path.join(output_path, 'HandPattern.t7')
    torch.save(file_output_path, bestHESolution)
    link_target = path.join('..', self.calibration_folder_name, 'HandPattern.t7')
    current_output_path = path.join(self.current_path, 'HandPattern.t7')
    os.execute('rm ' .. current_output_path)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
    self.H_pattern_to_tcp = bestHESolution
  end

  if self.configuration.camera_location_mode == 'onboard' then
    file_output_path = path.join(output_path, 'Hc_patternToCam.t7')
    torch.save(file_output_path, Hc)
    link_target = path.join('..', self.calibration_folder_name, 'Hc_patternToCam.t7')
    current_output_path = path.join(self.current_path, 'Hc_patternToCam.t7')
    os.execute('rm ' .. current_output_path)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
  else
    file_output_path = path.join(output_path, 'Hc_camToPattern.t7')
    torch.save(file_output_path, Hc)
    link_target = path.join('..', self.calibration_folder_name, 'Hc_camToPattern.t7')
    current_output_path = path.join(self.current_path, 'Hc_camToPattern.t7')
    os.execute('rm ' .. current_output_path)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
  end

  file_output_path = path.join(output_path, 'Hg_tcpToBase.t7')
  torch.save(file_output_path, Hg)
  link_target = path.join('..', self.calibration_folder_name, 'Hg_tcpToBase.t7')
  current_output_path = path.join(self.current_path, 'Hg_tcpToBase.t7')
  os.execute('rm ' .. current_output_path)
  os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)

  -- create links at the 'current' folder

  -- calculate camera/pattern pose in base coordinates
  if self.configuration.camera_location_mode == 'onboard' then
    local patternBaseTrafo = Hg[1] * bestHESolution * Hc[1]
    print("base -> pattern trafo (i.e. pattern pose in base coordinates):")
    print(patternBaseTrafo)
    self.H_pattern_to_base = patternBaseTrafo
    file_output_path = path.join(output_path, 'PatternBase.t7')
    torch.save(file_output_path, patternBaseTrafo)
    link_target = path.join('..', self.calibration_folder_name, 'PatternBase.t7')
    current_output_path = path.join(self.current_path, 'PatternBase.t7')
    os.execute('rm ' .. current_output_path)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
    return bestHESolution, patternBaseTrafo
  else
    local cameraBaseTrafo = Hg[1] * bestHESolution * Hc[1]
    print("base -> camera trafo (i.e. camera pose in base coordinates):")
    print(cameraBaseTrafo)
    self.H_cam_to_base = cameraBaseTrafo

    print('publish tf world_frame_id camera_left')
    -- publish
    for  i = 1, 10 do
      self.debug:publishTf(cameraBaseTrafo, world_frame_id, left_camera_frame_id)
    end
    print('publish tf camera_left pattern')
    self.debug:publishTf(torch.inverse(Hc[#Hc]), left_camera_frame_id, pattern_frame_id)

    file_output_path = path.join(output_path, 'LeftCamBase.t7')
    torch.save(file_output_path, cameraBaseTrafo)
    link_target = path.join('..', self.calibration_folder_name, 'LeftCamBase.t7')
    current_output_path = path.join(self.current_path, 'LeftCamBase.t7')
    os.execute('rm ' .. current_output_path)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)

    -- request the tf to the torso link (if it exists)
    print('requesting tf from camera left to torso...')
    local tf_available, H_cam_to_torso_tf = self.debug:requestTf(left_camera_frame_id, torso_frame_id)
    current_output_path = path.join(self.current_path, 'LeftCamTorso.t7')
    os.execute('rm ' .. current_output_path)
    if not tf_available then
      print('tf not available! '..left_camera_frame_id..' to '..torso_frame_id)
    else
      local H_cam_to_torso = H_cam_to_torso_tf:toTensor()
      file_output_path = path.join(output_path, 'LeftCamTorso.t7')
      torch.save(file_output_path, H_cam_to_torso)
      link_target = path.join('..', self.calibration_folder_name, 'LeftCamTorso.t7')
      os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
      self.H_cam_to_torso = H_cam_to_torso
    end
    return bestHESolution, cameraBaseTrafo
  end
end


-- captures a pair of stereo images
-- detects the cameraPatternTrafo using the stereo method, and publishes a tf with its pose
function HandEye:detectPattern()
  --1. capture pair of images
  local left_camera_config = self.configuration.cameras[self.configuration.left_camera_id]
  local right_camera_config = self.configuration.cameras[self.configuration.right_camera_id]
  
  local left_img
  local right_img
  --if not offline then -- for pattern detection after movement, offline makes no sense!
    left_img = self:captureImageNoWait(left_camera_config)
    right_img = self:captureImageNoWait(right_camera_config)
  --else
  --  -- offline (only for testing cases) -> load last captured images
  --  local current_directory_path = path.join(self.configuration.output_directory, 'capture/')
  --  nr = #self.configuration.capture_poses
  --  left_img_path = current_directory_path .. 'cam_' .. self.configuration.cameras.left.serial .. string.format('_%03d.png', nr)
  --  right_img_path = current_directory_path .. 'cam_' .. self.configuration.cameras.right.serial .. string.format('_%03d.png', nr)
  --  left_img = cv.imread{left_img_path}
  --  right_img = cv.imread{right_img_path}
  --end

  local ok, cameraPatternTrafo = patternLocalizer:calcCamPoseViaPlaneFit(left_img, right_img, 'left')
  if not ok then
    self.debug:publishImg(left_img)
    return nil
  end

  for i = 1, 20 do
    self.debug:publishTf(cameraPatternTrafo,'camera_left', pattern_frame_id)
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


-- captures a pair of stereo images
-- detects the cameraPatternTrafo using the stereo method
-- predicts the cameraPatternTrafo for the case of a robot motion
-- performs the robot motion
function HandEye:movePattern()
  -- set the folder to 'current'
  local files_path = self.current_path
  print('Calling HandEye:movePattern() method')
  --if the calibration data is missing, read it from the file
  if self.configuration.camera_location_mode == 'onboard' then
    if self.H_camera_to_tcp == nil or self.H_pattern_to_base == nil then
      self.H_pattern_to_base = torch.load(files_path .. "/PatternBase.t7")
      self.H_camera_to_tcp = torch.load(files_path .. "/HandEye.t7")
    end
  else
    if self.H_pattern_to_tcp == nil or self.H_cam_to_base == nil then
      self.H_cam_to_base = torch.load(files_path .. "/LeftCamBase.t7")
      self.H_pattern_to_tcp = torch.load(files_path .. "/HandPattern.t7")
    end
  end

  --1. move to the last posture of the taught capture postures
  local last_pose = self.configuration.capture_poses[#self.configuration.capture_poses]
  print("Moving to the last pose of the taught capture poses ...")
  self.move_group:moveJoints(last_pose)

  --2. estimate the pose of the pattern (i.e. the last capture pose)
  local left_camera_config = self.configuration.cameras[self.configuration.left_camera_id]
  local right_camera_config = self.configuration.cameras[self.configuration.right_camera_id]

  local left_img
  local right_img
  if not offline then
    left_img = self:captureImageNoWait(left_camera_config)
    right_img = self:captureImageNoWait(right_camera_config)
  else
    -- offline (only for testing cases) -> load last captured images
    local current_directory_path = path.join(self.configuration.output_directory, 'capture/')
    nr = #self.configuration.capture_poses
    left_img_path = current_directory_path .. 'cam_' .. self.configuration.cameras.left.serial .. string.format('_%03d.png', nr)
    right_img_path = current_directory_path .. 'cam_' .. self.configuration.cameras.right.serial .. string.format('_%03d.png', nr)
    left_img = cv.imread{left_img_path}
    right_img = cv.imread{right_img_path}
  end
  self.debug:publishImg(left_img)

  local ok, cameraPatternTrafo = patternLocalizer:calcCamPoseViaPlaneFit(left_img, right_img, 'left')
  if not ok then
    print('pattern not found!')
    return ok, cameraPatternTrafo
  end

  print('detected cameraPatternTrafo before motion:')
  print(cameraPatternTrafo)
  self.debug:publishTf(cameraPatternTrafo, 'camera_left', 'pattern')

  -- Now we compute a relative transformation (corresponding to a motion of the robot)
  -- and compute (i.e. predict) the cameraPatternTrafo after this motion.
  if self.configuration.camera_location_mode == 'onboard' then -- 'onboard' camera setup
    if self.H_camera_to_tcp ~= nil and self.H_pattern_to_base ~= nil then
      print('about to do a movement..')
      -- b_H_c : pattern pose in base_link coordinates
      -- g_H_p : camera pose in tcp coordinates
      local relative_transformation = self:generateRelativeRotation()
      relative_transformation = self:generateRelativeTranslation(relative_transformation)
      self.debug:publishTf(relative_transformation, 'left_cam', 'left_cam_shift')
      self.debug:publishTf(relative_transformation, 'left_cam', 'left_cam_shift')
      self.debug:publishTf(relative_transformation, 'left_cam', 'left_cam_shift')
      self.predicted_cameraPatternTrafo = cameraPatternTrafo*relative_transformation
      print('prediction for cameraPatternTrafo after motion:')
      print(self.predicted_cameraPatternTrafo)
      print('compare with the next pattern detection:')
      local pose_tcp = self.H_camera_to_tcp * (relative_transformation * torch.inverse(self.H_camera_to_tcp))
      local current_pose_tcp = self.move_group:getCurrentPose()
      local pose_tcp_in_base_coord = current_pose_tcp:toTensor() * pose_tcp -- pose_tcp * current_pose_tcp ???
      --local stampedTransfDesiredTcp = self.debug:publishTf(pose_tcp, self.tcp_frame_of_reference, self.tcp_frame_of_reference..'_shift')
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
    if self.H_pattern_to_tcp ~= nil and self.H_cam_to_base ~= nil then
      print('about to do a movement..')
      -- b_H_c : camera pose in base_link coordinates
      -- g_H_p : pattern pose in tcp coordinates
      local relative_transformation = self:generateRelativeRotation()
      relative_transformation = self:generateRelativeTranslation(relative_transformation)
      self.debug:publishTf(relative_transformation, 'pattern', 'pattern_shift')
      self.debug:publishTf(relative_transformation, 'pattern', 'pattern_shift')
      self.debug:publishTf(relative_transformation, 'pattern', 'pattern_shift')
      self.predicted_cameraPatternTrafo = cameraPatternTrafo*relative_transformation
      print('prediction for cameraPatternTrafo after motion:')
      print(self.predicted_cameraPatternTrafo)
      print('compare with the next pattern detection:')
      local pose_tcp = self.H_pattern_to_tcp * (relative_transformation * torch.inverse(self.H_pattern_to_tcp))
      local current_pose_tcp = self.move_group:getCurrentPose()     
      local pose_tcp_in_base_coord = current_pose_tcp:toTensor() * pose_tcp -- pose_tcp * current_pose_tcp ???
      --local stampedTransfDesiredTcp = self.debug:publishTf(pose_tcp, self.tcp_frame_of_reference, self.tcp_frame_of_reference..'_shift')
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

  -- from http://cmp.felk.cvut.cz/~hodanto2/data/hodan2016evaluation.pdf
  -- error given  by  the  angle  from  the  axis–angle  representation  of rotation
  -- the angle of rotation of a matrix R in the axis–angle representation is given by arccos( {Tr(R) -1} /2)
  -- if R1 ~= R2 => R1 * R2.inv() ~= Identity => angle of rotation ~= 0
  -- => arccos(angle of rotation) ~= pi/2 = 1.57..
  local err_r2 = torch.acos( torch.trace(R1* torch.inverse(R2) -1) / 2 )
  local err_r3 = torch.abs(err_r2 - M_PI/2)

  print('translation error (norm of difference) [in m]:', err_t, ' ( = ', err_t * 1000, ' mm)')
  print('translation error for each axis [in m]:')
  print(err_axes)
  print('euler angles prediction:')
  print(H1:getRotation():toTensor())
  print('euler angles detection:')
  print(H2:getRotation():toTensor())
  print('rotation error (norm of difference of Euler angles):', err_r)
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


-- Evaluation of the hand eye calibration (with several robot movements)
-- Details:
-- For several, previously taught evaluation poses (i.e. joint configurations)
-- the corresponding tcp poses are calculated via forward kinematic and 
-- the new camera<->pattern trafo is predicted for each of these poses.
-- After that the robot moves are actually performed and the new camera<->pattern trafos
-- are measured via plane fit and compared with the prediction.
function HandEye:evaluateCalibrationComplex()

  -- set the folder to 'current'
  local files_path = self.current_path
  --if the calibration data is missing, read it from the file
  if self.configuration.camera_location_mode == 'onboard' then
    if self.H_camera_to_tcp == nil or self.H_pattern_to_base == nil then
      self.H_pattern_to_base = torch.load(files_path .. "/PatternBase.t7")
      self.H_camera_to_tcp = torch.load(files_path .. "/HandEye.t7")
    end
  else
    if self.H_pattern_to_tcp == nil or self.H_cam_to_base == nil then
      self.H_cam_to_base = torch.load(files_path .. "/LeftCamBase.t7")
      self.H_pattern_to_tcp = torch.load(files_path .. "/HandPattern.t7")
    end
  end

  --1. move to the last posture of the taught capture postures
  local last_pose = self.configuration.capture_poses[#self.configuration.capture_poses]
  print("Moving to the last pose of the taught capture poses ...")
  self.move_group:moveJoints(last_pose)
  local tcp_old = self.move_group:getCurrentPose():toTensor()

  --2. determine the camera<->pattern transformation
  print("Calculation of the camera<->pattern transformation..")
  local left_camera_config = self.configuration.cameras[self.configuration.left_camera_id]
  local right_camera_config = self.configuration.cameras[self.configuration.right_camera_id]
  local left_img
  local right_img
  if not offline then
    left_img = self:captureImageNoWait(left_camera_config)
    right_img = self:captureImageNoWait(right_camera_config)
  else
    -- offline (only for testing cases) -> load last captured images
    local current_directory_path = path.join(self.configuration.output_directory, 'capture/')
    local nr = #self.configuration.capture_poses
    local left_img_path = current_directory_path .. 'cam_' .. self.configuration.cameras.left.serial .. string.format('_%03d.png', nr)
    local right_img_path = current_directory_path .. 'cam_' .. self.configuration.cameras.right.serial .. string.format('_%03d.png', nr)
    left_img = cv.imread{left_img_path}
    right_img = cv.imread{right_img_path}
  end
  local ok, cameraPatternTrafo_old = patternLocalizer:calcCamPoseViaPlaneFit(left_img, right_img, 'left')
  if not ok then
    print('pattern not found!')
    return ok, cameraPatternTrafo_old
  end
  print("old camera<->pattern trafo:")
  print(cameraPatternTrafo_old)

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

    left_img = self:captureImageNoWait(left_camera_config)
    right_img = self:captureImageNoWait(right_camera_config)
    if left_img == nil or right_img == nil then
      print('Please connect cameras first!')
      return false
    end
    local ok, detection = patternLocalizer:calcCamPoseViaPlaneFit(left_img, right_img, 'left')
    if not ok then
      print('Pattern not found! Taking the next image..')
    else
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
  err_t_avg = err_t_avg / count
  err_t_axes_avg = err_t_axes_avg / count
  err_r1_avg = err_r1_avg / count
  err_r2_avg = err_r2_avg / count
  print('average translation error (norm of difference) [in m]:', err_t_avg, ' ( = ', err_t_avg * 1000, ' mm)')
  print('average translation error for each axis [in m]:')
  print(err_t_axes_avg)
  print('average rotation error (norm of difference of Euler angles):', err_r1_avg)
  print('average rotation error metric #2 [in radians]:', err_r2_avg, ' ( = ', err_r2_avg * 180/M_PI, ' degree)')
end


function filterOutliers(cloud)
  local indices = pcl.Indices()
  local removeIndices = pcl.Indices()
  local output = pcl.PointCloud(pcl.PointXYZRGBA)
  local radius = 2
  minNeighbours = 20
  negative = false
  local keep_organized = true
  print('is input cloud organized? '..#cloud .. ' = ' .. cloud:getWidth() .. ' x ' ..cloud:getHeight())
  pcl.filter.radiusOutlierRemoval(cloud, radius, minNeighbours, nil, output, false,removeIndices, keep_organized)
  print('performed radiusOutlierRemoval - point cloud dimensions are ' ..#output .. ' = ' .. output:getWidth() .. ' x ' .. output:getHeight())
  print('input cloud size ' .. #cloud)
  print('removeIndices size ' ..#removeIndices)
  print('output cloud size ' ..#output)
  return output, removeIndices
end


function filterStatisticalOutliers(cloud)
  --filter.statisticalOutlierRemoval(input, meanK, stddevMulThresh, indices, output, negative, removed_indices)
    local indices = pcl.Indices()
    local removed_indices = pcl.Indices()
    local meanK = 5
    local stddevMulThresh = 0.3
    local output = pcl.PointCloud(pcl.PointXYZRGBA)
    local keepOrganized = true
    print('is input cloud organized? '..#cloud .. ' = ' .. cloud:getWidth() .. ' x ' ..cloud:getHeight())
    pcl.filter.statisticalOutlierRemoval(cloud, meanK, stddevMulThresh, nil, output, false, removed_indices, keepOrganized)
    print('performed filterStatisticalOutliers - point cloud dimensions are ' ..#output .. ' = ' .. output:getWidth() .. ' x ' .. output:getHeight())
    print('indices size= ' .. #removed_indices)
    return output, removed_indices
end


function filterRadiusOutliers(cloud)
  local indices = pcl.Indices()
  local removeIndices = pcl.Indices()
  local output = pcl.PointCloud(pcl.PointXYZRGBA)
  local radius = 0.002
  local minNeighbours = 20
  local negative = false
  local keep_organized = true
  print('is input cloud organized? '..#cloud .. ' = ' .. cloud:getWidth() .. ' x ' ..cloud:getHeight())
  --  function filter.radiusOutlierRemoval(input, radius, minNeighbors, indices, output, negative)
  pcl.filter.radiusOutlierRemoval(cloud,radius, minNeighbours,nil, output, false)
  print('performed radiusOutlierRemoval - point cloud dimensions are ' ..#output .. ' = ' .. output:getWidth() .. ' x ' .. output:getHeight())
  print('input cloud size ' .. #cloud)
  print('removeIndices size ' ..#removeIndices)
  print('output cloud size ' ..#output)
  return output, removeIndices
end


function HandEye:getStereoPointCloud()
  if slstudio ~= nil then
    local code = slstudio:initScanStereo("./calibration/current/StereoCalibration.xml", slstudio.CAMERA_ROS, self.left_camera_serial, slstudio.CAMERA_ROS, self.right_camera_serial, 300, 2500, true);
    local code, cloud, imageOn, imageOff, imageOnboard, imageShading = slstudio:scanStereo(30)
    local points = cloud:pointsXYZ()
    local filtered = pcl.PointCloud(pcl.PointXYZ)
    local start_time = os.clock()
    filtered, indices = filterStatisticalOutliers(cloud)
    local end_time = os.clock()
    print('filterStatisticalOutliers time (s)=', end_time - start_time)
    slstudio:quitScanStereo()
    self.debug:publishCloud(cloud, pcloud_frame_id)
    self.debug:publishCloud(filtered, pcloud_frame_id, 'filtered_cloud')
    return cloud
  else
    print('Cannot get stereo point cloud. Package \"slstudio\" is missing.')
  end
end


function HandEye:getPointCloud()
  if slstudio ~= nil then
    local code = slstudio:initScan("./calibration/current/sls.xml", slstudio.CAMERA_ROS, self.left_camera_serial, 100, 3000, false)  -- 300, 2500, true);
    local code, cloud, imageOn, imageOff, imageOnboard, imageShading = slstudio:scan(20)
    local points = cloud:pointsXYZ()
    local filtered = pcl.PointCloud(pcl.PointXYZ)
    --points:mul(.001)
    local start_time = os.clock()
    filtered, indices = filterStatisticalOutliers(cloud)
    local end_time = os.clock()
    print('filterStatisticalOutliers time (s)=', end_time - start_time)
    --print("libslstudio returned: ", code, cloud)
    slstudio:quitScan()
    self.debug:publishCloud(filtered, pcloud_frame_id)
    return filtered
  else
    print('Cannot get point cloud. Package \"slstudio\" is missing.')
  end
end


function HandEye:locateCirclePatternInStereoPointCloud()

  -- read cloud and imageOn from disk
  local path_stereo = '/home/xamla/code/workspace/prototyping_garcia/bagfiles/stereos/fifth_scan/'
  local file_stereo_left = path_stereo .. 'cloud_left.pcd';
  local file_stereo_right= path_stereo .. 'cloud_right.pcd';
  local file_img_left = path_stereo .. 'slstudio/mono-on0-flipped.png'
  local file_img_right = path_stereo .. 'slstudio/mono-on1-flipped.png'
  local cloud_left = pcl.PointCloud(pcl.PointXYZ)
  local cloud_right = pcl.PointCloud(pcl.PointXYZ)
  print('openning file ', file_stereo_left)
  cloud_left:loadPCDFile(file_stereo_left)
  cloud_right:loadPCDFile(file_stereo_right)

  local imageOn = cv.imread {file_img_left, cv.IMREAD_GRAYSCALE}
  local imageOnBGR = cv.cvtColor{imageOn, nil, cv.COLOR_GRAY2BGR}

  -- imageOn: use this guy to find the pattern in the point cloud
  local cloud_right_frame_id = 'camera_right' -- make sure we publish the point cloud in the camera_left frame and not camera_left_cloud
  local cloud_left_frame_id = 'camera_left'
  local camera_left_id = 'camera_left'
  local camera_right_id = 'camera_right'
  self.debug:publishCloud(cloud_left, cloud_left_frame_id)
  --self.debug_right:publishCloud(cloud_right, cloud_right_frame_id)


  local left_img = cv.imread {file_img_left, cv.IMREAD_GRAYSCALE}
  local left_img_bgr = cv.cvtColor{left_img, nil, cv.COLOR_GRAY2BGR}
  local right_img = cv.imread {file_img_right, cv.IMREAD_GRAYSCALE}
  local right_img_bgr = cv.cvtColor{right_img, nil, cv.COLOR_GRAY2BGR}


  --self:debugPatternDetection(right_img_bgr)
  --self:debugPatternDetection(left_img_bgr)

  --local ok, pattern_pose_stereo = self:calcPatternPoseRelativeToCam(
  --  left_img_bgr,
  --  right_img_bgr,
  --  'left')
  local ok, pattern_pose_stereo = patternLocalizer:calcCamPoseViaPlaneFit(left_img_bgr, right_img_bgr, 'left')

  print(' self.leftDistCoeffs =',  self.leftDistCoeffs)
  print(' self.leftCameraMatrix =',  self.leftCameraMatrix)
  local pattern_pose_mono, points3d = patternLocalizer:calcPatternPose(left_img_bgr, self.leftCameraMatrix, self.leftDistCoeffs, patternLocalizer.pattern)
  print(' self.rightDistCoeffs =',  self.rightDistCoeffs)
  print(' self.rightCameraMatrix =',  self.rightCameraMatrix)
  local pattern_pose_mono_right, points3d = patternLocalizer:calcPatternPose(right_img_bgr, self.rightCameraMatrix, self.rightDistCoeffs, patternLocalizer.pattern)




  if not ok then
    return ok
  end
  for i = 1, 20 do
    self.debug:publishTf(pattern_pose_stereo, camera_left_id, 'pattern_stereo')
    self.debug:publishTf(pattern_pose_mono, camera_left_id, 'pattern_mono_left')
    self.debug:publishTf(pattern_pose_mono_right, camera_right_id, 'pattern_mono_right')
  end

  --print('pcloud misalignment=', H_pattern_cloud_to_cam)


end


function HandEye:debugPatternDetection(imageOnBGR)
  local circleFinderParams = cv.SimpleBlobDetector_Params {}
  circleFinderParams.thresholdStep = 5
  circleFinderParams.minThreshold = 60 -- 60
  circleFinderParams.maxThreshold = 230
  circleFinderParams.minRepeatability = 3
  circleFinderParams.minDistBetweenBlobs = 5
  circleFinderParams.filterByColor = false
  circleFinderParams.blobColor = 50 --0
  circleFinderParams.filterByArea = true -- area of the circle in pixels
  circleFinderParams.minArea = 500 -- 500
  circleFinderParams.maxArea = 4000
  circleFinderParams.filterByCircularity = true
  circleFinderParams.minCircularity = 0.6 --0.6
  circleFinderParams.maxCircularity = 10  --10
  circleFinderParams.filterByInertia = false --false
  circleFinderParams.minInertiaRatio = 0.6
  circleFinderParams.maxInertiaRatio = 10
  circleFinderParams.filterByConvexity = true
  circleFinderParams.minConvexity = 0.8
  circleFinderParams.maxConvexity = 10
  local blobDetector = cv.SimpleBlobDetector {circleFinderParams}

  local ok1,
    circlesGridPointsLeft =
    cv.findCirclesGrid {
        image = imageOnBGR,
        patternSize = {height = 21, width = 8},
        flags = cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING,
        blobDetector = blobDetector
  }

  if ok1 then
    print('Found the pattern in the point cloud image')
  else
    print('Couldnt find the pattern in the point cloud image')
    self.debug:publishImg(imageOnBGR)
    return ok1
  end

  --for debugging purposes, draw the detected circles and publish the image
  cv.drawChessboardCorners{
      image = imageOnBGR,
      patternSize = {height = 21, width = 8}, --cols 21, rows 8 patternLocalizer.pattern.width, patternLocalizer.pattern.height
      corners =circlesGridPointsLeft,
      patternfound = ok1
  }
  if ok1 then
    self.debug:publishImg(imageOnBGR)
  end
end


function HandEye:locateCirclePatternInPointCloud()

  local code = slstudio:initScan("./calibration/current/sls.xml", slstudio.CAMERA_ROS, self.left_camera_serial, 100, 3000, true);  --300, 2500, true);
  --local code = slstudio:initScan("./calibration/current/sls.xml", slstudio.CAMERA_ROS, "CAMAU1639042", 100, 3000, true);  --300, 2500, true);
  local code, cloud, imageOn, imageOff, imageOnboard, imageShading = slstudio:scan(20); --20
  local imageOnBGR = cv.cvtColor{imageOn, nil, cv.COLOR_GRAY2BGR}
  slstudio:quitScan()

  -- imageOn: use this guy to find the pattern in the point cloud
  local pcloud_frame_id_local = 'camera_left' -- make sure we publish the point cloud in the camera_left frame and not camera_left_cloud
  self.debug:publishCloud(cloud, pcloud_frame_id_local)

  local circleFinderParams = cv.SimpleBlobDetector_Params {}
  circleFinderParams.thresholdStep = 5
  circleFinderParams.minThreshold = 60 -- 60
  circleFinderParams.maxThreshold = 230
  circleFinderParams.minRepeatability = 3
  circleFinderParams.minDistBetweenBlobs = 5
  circleFinderParams.filterByColor = false
  circleFinderParams.blobColor = 50 --0
  circleFinderParams.filterByArea = true -- area of the circle in pixels
  circleFinderParams.minArea = 500 -- 500
  circleFinderParams.maxArea = 4000
  circleFinderParams.filterByCircularity = true
  circleFinderParams.minCircularity = 0.6 --0.6
  circleFinderParams.maxCircularity = 10  --10
  circleFinderParams.filterByInertia = false --false
  circleFinderParams.minInertiaRatio = 0.6
  circleFinderParams.maxInertiaRatio = 10
  circleFinderParams.filterByConvexity = true
  circleFinderParams.minConvexity = 0.8
  circleFinderParams.maxConvexity = 10
  local blobDetector = cv.SimpleBlobDetector {circleFinderParams}

  local ok1,
    circlesGridPointsLeft =
    cv.findCirclesGrid {
        image = imageOnBGR,
        patternSize = {height = 21, width = 8},
        flags = cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING,
        blobDetector = blobDetector
  }

  if ok1 then
    print('Found the pattern in the point cloud image')
  else
    print('Couldnt find the pattern in the point cloud image')
    self.debug:publishImg(imageOnBGR)
    return ok1
  end

  --for debugging purposes, draw the detected circles and publish the image
  cv.drawChessboardCorners{
      image = imageOnBGR,
      patternSize = {height = 21, width = 8}, --cols 21, rows 8 patternLocalizer.pattern.width, patternLocalizer.pattern.height
      corners =circlesGridPointsLeft,
      patternfound = ok1
  }


  local origin = circlesGridPointsLeft[161][1]
  local end_x = circlesGridPointsLeft[168][1]
  local end_y = circlesGridPointsLeft[1][1]

  cv.circle{
        img = imageOnBGR,
        center = {origin[1], origin[2]},
        color = {255,255,255}, -- white
        radius = 15,
        thickness = 3,
        line_type = 8,
  }
  print('origin in image coordinates:', origin[1], ' ', origin[2], ' dimensions of img=', imageOnBGR:size()[2])
  print('end_x in image coordinates:', end_x)
  print('end_y in image coordinates:', end_y)

  --get corresponding 3D coordinates from the point cloud
  local points = cloud:pointsXYZ()
  local origin_3d = points[origin[2]][origin[1]]
  local end_x_3d = points[end_x[2]][end_x[1]]
  local end_y_3d = points[end_y[2]][end_y[1]]
  print('origin in 3D coordinates:',origin_3d)
  print('end_x_3d in 3D coordinates:',end_x_3d)
  print('end_y_3d in 3D coordinates:',end_y_3d)

  local vec_x = end_x_3d - origin_3d
  local vec_y = end_y_3d - origin_3d

  self.debug:publishImg(imageOnBGR)

  local x_unit_vec = torch.div(vec_x, torch.norm(vec_x))
  local y_unit_vec = torch.div(vec_y, torch.norm(vec_y))

  -- Check, whether the normal vector z_unit_vec points into the correct direction.
  --local z_unit = torch.DoubleTensor(3)
  local z_unit_vec = torch.cross(x_unit_vec,y_unit_vec)

  local pattern_pose_pcloud = torch.DoubleTensor(4, 4):zero()
  pattern_pose_pcloud[{{1, 3}, {1}}] = x_unit_vec
  pattern_pose_pcloud[{{1, 3}, {2}}] = y_unit_vec
  pattern_pose_pcloud[{{1, 3}, {3}}] = z_unit_vec
  pattern_pose_pcloud[{{1, 3}, {4}}] = origin_3d
  pattern_pose_pcloud[4][4] = 1


  local pattern_pose_cam = self:detectPattern()
  local H_pattern_cloud_to_cam = pattern_pose_pcloud*torch.inverse(pattern_pose_cam)
  print('relative_transf=',H_pattern_cloud_to_cam)

  for i = 1, 20 do
    self.debug:publishTf(pattern_pose_pcloud, pcloud_frame_id, pattern_pcloud_frame_id)
  end

  print('pcloud misalignment=', H_pattern_cloud_to_cam)
  -- save this pose to publish it later on as the 'corrected' cam_pose for point cloud data
  --local path = './calibration'
  --torch.save(path .. "/PCloudCam.t7", torch.inverse(H_pattern_cloud_to_cam))

end


function HandEye:getPoseFromPatternPoints(points_3d)


  local A = torch.DoubleTensor(nPoints, 3) -- 2-dim. tensor of size 168 x 3
  local b = torch.DoubleTensor(nPoints)    -- 1-dim. tenosr of size 168
  for i = 1, nPoints do
      A[i][1] = points_3d[i][1]
      A[i][2] = points_3d[i][2]
      A[i][3] = 1.0
      b[i] = points_3d[i][3]
  end

  -- Calculate x = A^-1 * b = (A^T A)^-1 * A^T * b (mit Pseudoinverser von A)
  At = A:transpose(1, 2)
  x = torch.mv(torch.mm(torch.inverse(torch.mm(At, A)), At), b)

  n = torch.DoubleTensor(3)
  n[1] = x[1]
  n[2] = x[2]
  n[3] = -1.0
  z_unit_vec = torch.div(n, torch.norm(n))
end


function HandEye:getLowestPointAtIntMarkerCloud(src_cloud)
  local points = src_cloud:pointsXYZ()
  local margin_in_m = 0.005
  local lowest_z = 999999
  local lowest_p
  for row = 1,points:size()[1] do
    for col = 1,points:size()[2] do
      if math.abs(points[row][col][1]) < margin_in_m and math.abs(points[row][col][2]) < margin_in_m then
        if points[row][col][3] < lowest_z then
          lowest_z = points[row][col][3]
          lowest_p = points[row][col]
        end
      end
    end
  end
  lowest_p[3] = lowest_p[3] + 0.005 -- put an offset to be safe..
  print('lowest point =', lowest_p)
  return lowest_p
end


function HandEye:createPickingPose(point_marker_frame)
  -- the tfs will be in the interactive marker's frame of reference, keeping the same orientation
  local H_new_pose_to_tcp = torch.eye(4,4)
  H_new_pose_to_tcp[1][4] = point_marker_frame[1]
  H_new_pose_to_tcp[2][4] = point_marker_frame[2]
  H_new_pose_to_tcp[3][4] = point_marker_frame[3]
  local H_pre_pick_pose = torch.eye(4,4)
  H_pre_pick_pose[1][4] = H_new_pose_to_tcp[1][4]
  H_pre_pick_pose[2][4] = H_new_pose_to_tcp[2][4]
  H_pre_pick_pose[3][4] = H_new_pose_to_tcp[3][4] - 0.1
  -- loop to fill in the buffer
  for i=1,20 do
    --print('publishing tf from '..marker_frame_id..' to '..picking_pose_frame_id)
    self.debug:publishTf(H_new_pose_to_tcp, marker_frame_id, picking_pose_frame_id)
    self.debug:publishTf(H_pre_pick_pose, marker_frame_id, pre_picking_pose_frame_id)
  end

  return H_new_pose_to_tcp
end


function HandEye:moveToMarker()

  -- get the current pose of the marker
  -- requestTf(source_frame_id = marker, target_frame_id = world)
  --                               target = world   source = end effector (marker)
  local tf_available, H_marker_to_world = self.debug:requestTf(marker_frame_id, world_frame_id)
  if not tf_available then
    print('tf not available! '..world_frame_id..' to '..marker_frame_id)
    return
  end
  print('H_marker_to_world')
  print(H_marker_to_world)
  print('self.H_fingers_to_tcp')
  print(self.H_fingers_to_tcp)

  local H_desired = H_marker_to_world:toTensor() * self.H_fingers_to_tcp:toTensor()
  H_marker_to_world:fromTensor(H_desired)

  --H_marker_to_world:mul(self.H_fingers_to_tcp:toTransform(), H_marker_to_world:toTransform())
  print('after multiplication H_marker_to_world=')
  print(H_marker_to_world)

  --move the end effector there..
  local check_for_collisions = true
  H_marker_to_world:set_frame_id(world_frame_id)
  H_marker_to_world:set_child_frame_id(self.tcp_frame_of_reference)
  print(H_marker_to_world)

  self.debug:publishTf(H_marker_to_world:toTensor(), world_frame_id, desired_tcp_frame_id)
  self.debug:publishTf(H_marker_to_world:toTensor(), world_frame_id, desired_tcp_frame_id)
  self.debug:publishTf(H_marker_to_world:toTensor(), world_frame_id, desired_tcp_frame_id)

  local collision_check = true
  self.move_group:moveL(self.tcp_end_effector_name, H_marker_to_world, self.configuration.velocity_scaling, collision_check) -- aendert sich noch!!!
  --self.end_effector:moveL(self.tcp_end_effector_name, H_marker_to_world, self.configuration.velocity_scaling, collision_check)

end


function HandEye:moveToMarkerTests()

  -- get the current pose of the marker
  -- requestTf(source_frame_id = marker, target_frame_id = world)
  --                               target = world   source = end effector (marker)
  local tf_available, H_tcp_to_world = self.debug:requestTf(self.tcp_frame_of_reference, world_frame_id)
  if not tf_available then
    print('tf not available! '..world_frame_id..' to '..self.tcp_frame_of_reference)
    return
  end
  --move the end effector there..
  local collision_check = true
  print(H_tcp_to_world)
  self.move_group:moveL(self.tcp_end_effector_name, H_tcp_to_world, self.configuration.velocity_scaling, collision_check) -- aendert sich noch!!!
  --self.end_effector:moveL(self.tcp_end_effector_name, H_tcp_to_world, self.configuration.velocity_scaling, collision_check)

end


function HandEye:moveToStart()
  local base_poses = self.configuration.base_poses
  assert(base_poses ~= nil)
  self.move_group:moveJoints(base_poses['start'])
end


function HandEye:moveLToTf(target_tf)

   self.debug:publishTf(target_tf:toTensor(), target_tf:get_frame_id(), 'desired_tcp')
  print('published tf. now calling moveL...')
  -- move there
  local check_for_collisions = true

  print('self.configuration.velocity_scaling')
  print(self.configuration.velocity_scaling)
  local xamla_ee = xamla_mg:getEndEffector(self.tcp_end_effector_name)
  local tmp_pos = datatypes.Pose()
  tmp_pose.StampedTransform:setData(target_tf)
  xamla_ee:movePoseLinear(tmp_pose, self.configuration.velocity_scaling, check_for_collisions) -- aendert sich noch!!!

end


function HandEye:readKeySpinning()
  local function spin()
      if not ros.ok() then
          return false, 'ros shutdown requested'
      else
          ros.spinOnce()
          return true
      end
  end
  return xutils.waitKey(spin)
end


function HandEye:moveLToTfSupervised(target_tf)

  self.debug:publishTf(target_tf:toTensor(), target_tf:get_frame_id(), 'desired_tcp')
  print('published tf. now calling moveLSupervised...')
  -- move there
  local check_for_collisions = true
  local velocity_scaling = 1
  local accelerationScaling = 1
  local do_interaction = true

  local function done_cb(state, result)
    print('done.')
    do_interaction = false
    result_state = state
    result_payload = result
  end

  local xamla_ee = xamla_mg:getEndEffector(self.tcp_end_effector_name)
  local tmp_pos = datatypes.Pose()
  tmp_pose.StampedTransform:setData(target_tf)

  local handle = xamla_ee:movePoseLinearSupervised(tmp_pose, velocity_scaling, check_for_collisions, accelerationScaling, done_cb) -- aendert sich evtl. noch!!!
  assert(torch.isTypeOf(handle, motionLibrary.SteppedMotionClient))

  local input
  while ros.ok() and do_interaction do
      print('Step through planned trajectory:')
      print('=====')
      print("'+'             next position")
      print("'-'             previous position")
      print("'ESC' or 'q'    quit")
      print()
      input = self:readKeySpinning()
      if input == '+' then
        handle:next()
      elseif input == '-' then
          handle:previous()
      elseif string.byte(input) == 27 or input == 'q' then
          do_interaction = false
          handle:abort()
      elseif input == 'f' then
          print('feedback: ', handle:getFeedback())
      end
  end

  local ok, msg = handle:getResult()
  print(msg)
  handle:shutdown()


end


function HandEye:moveToInitPoseSupervised()

  local homeJointStateSDA10d =
    datatypes.JointValues(
    datatypes.JointSet(
        {
          "torso_joint_b1",
          "arm_left_joint_1_s",
          "arm_left_joint_2_l",
          "arm_left_joint_3_e",
          "arm_left_joint_4_u",
          "arm_left_joint_5_r",
          "arm_left_joint_6_b",
          "arm_left_joint_7_t",
          "arm_right_joint_1_s",
          "arm_right_joint_2_l",
          "arm_right_joint_3_e",
          "arm_right_joint_4_u",
          "arm_right_joint_5_r",
          "arm_right_joint_6_b",
          "arm_right_joint_7_t"
        }
    ),
    torch.Tensor {
      -0.0320090651512146,
      1.8986761569976807,
      -0.42934417724609375,
      -1.8726563453674316,
      -1.4231847524642944,
      -1.1743154525756836,
      -1.6150994300842285,
      1.17659330368042,
      -1.8620822429656982,
      0.8524501323699951,
      2.7826714515686035,
      -1.713228702545166,
      -0.6352958679199219,
      -0.909939169883728,
      -0.8238078355789185
    }
    )
    local collision_check = true
    local handle = self.xamla_mg_both:moveJSupervised(homeJointStateSDA10d, self.configuration.velocity_scaling, collision_check)
    assert(torch.isTypeOf(handle, motionLibrary.SteppedMotionClient))

    xutils.enableRawTerminal()
    local input
    while ros.ok() and do_interaction do
        print('Step through planned trajectory:')
        print('=====')
        print("'+'             next position")
        print("'-'             previous position")
        print("'ESC' or 'q'    quit")
        print()
        input = readKeySpinning()
        if input == '+' then
            handle:next()
        elseif input == '-' then
            handle:previous()
        elseif string.byte(input) == 27 or input == 'q' then
            do_interaction = false
            handle:abort()
        elseif input == 'f' then
            print('feedback: ', handle:getFeedback())
        end
    end
    xutils.restoreTerminalAttributes()

    local ok, msg = handle:getResult()
    print(msg)
    handle:shutdown()


end

local function computIK(self, Tf, tcp_end_effector_name)
  local end_effector = self.move_group:getEndEffector(tcp_end_effector_name)
  local motion_service = self.move_group.motion_service
  local plan_parameters = self.move_group:buildPlanParameters()
  local tmp_pose = datatypes.Pose()
  tmp_pose.StampedTransform:setData(Tf)
  local err, solution = motion_service:queryIK(tmp_pose, plan_parameters, self.move_group:getCurrentJointValues(), end_effector.link_name)
  assert(err[1].val == 1)
  return datatypes.JointValues(datatypes.JointSet(solution.joint_names), solution.solutions[1])
end

function HandEye:movePToTf(Tf)

  -- transform the desired fingertips pose to end effector coordinate frame
  local H_desired = Tf:toTensor() * self.H_fingers_to_tcp:toTensor()


  local H_desired_stamped = tf.StampedTransform.new()
  H_desired_stamped:fromTensor(H_desired)
  H_desired_stamped:set_frame_id(world_frame_id)
  H_desired_stamped:set_child_frame_id(self.tcp_frame_of_reference)
  self.debug:publishTf(H_desired_stamped:toTensor(), world_frame_id, 'desired_tcp')
  local res = computIK(self, H_desired_stamped, tcp_end_effector_name)
  -- move there
  local collision_check = true
  self.move_group:moveJoints(res, self.configuration.velocity_scaling, collision_check)

end


--
--  Creates a pose for the end effector based on the tf published by interactive marker
--  requests a point cloud. Traces the pose down along the world z axis to the highest point
--  and creates a new tf that defines a picking pose
--  Requires that:
--    1- th publisher.lua is running and publishing the tfs for all cameras
--    2- python int_marker.py is running to publish the tf of the desired pose for the tcp
function HandEye:pickingPose()

  -- open gripper
  self.gripper:move{width=0.06, speed=0.2} -- move open

  --request the current pose of int_marker
  local tf_available, H_marker_to_world = self.debug:requestTf(world_frame_id, marker_frame_id)
  if not tf_available then
    print('tf not available! '..world_frame_id..' to '..marker_frame_id)
    return
  end
  print('tf '..world_frame_id..' to '..marker_frame_id)
  print(H_marker_to_world)
  src_cloud = self:getStereoPointCloud()

  -- the point cloud is in camera coordinates, transform it to marker_frame_id coordinates
  tf_available, H_cam_to_marker = self.debug:requestTf(pcloud_frame_id, marker_frame_id)
  if not tf_available then
    print('tf not available! '..pcloud_frame_id..' to '..marker_frame_id)
    return
  end
  local dst_cloud = pcl.transformCloud(src_cloud, dst_cloud, H_cam_to_marker:toTensor():float())
  -- see if all went fine
  self.debug:publishCloud(dst_cloud, marker_frame_id)

  -- now look for the point with the smallest z: (x= ~0, y= ~0, z)
  local lowest_point_marker_frame = self:getLowestPointAtIntMarkerCloud(dst_cloud)
  self:createPickingPose(lowest_point_marker_frame)
  local tf_available, Tf_pre_picking_pose = self.debug:requestTf(pre_picking_pose_frame_id, world_frame_id)
  if not tf_available then
    print('tf not available! '..world_frame_id..' to '..pre_picking_pose_frame_id)
    return
  end

  local tf_available, Tf_picking_pose = self.debug:requestTf(picking_pose_frame_id, world_frame_id)
  if not tf_available then
    print('tf not available! '..world_frame_id..' to '..picking_pose_frame_id)
    return
  end

  self:movePToTf(Tf_pre_picking_pose)
  self:moveLToTf(Tf_picking_pose)

  -- close the gripper and move back to start pose
  self.gripper:move{width=0.0, speed=0.2, force=20, stop_on_block=false} -- move closed
  self:moveLToTf(Tf_pre_picking_pose)
  --self:moveToStart()

end


function HandEye:moveToIntMarkerPoseWithPrePick()

  -- open gripper
  self.gripper:move{width=0.06, speed=0.2} -- move open
  --request the current pose of int_marker
  local tf_available, H_marker_to_world_tf = self.debug:requestTf(marker_frame_id, world_frame_id)
  if not tf_available then
    print('tf not available! '..world_frame_id..' to '..marker_frame_id)
    return
  end
  print('tf '..world_frame_id..' to '..marker_frame_id)
  local pick_pose_tf = H_marker_to_world_tf
  local pre_pick_pose_tf = H_marker_to_world_tf:clone()
  local pre_pick_tensor = pre_pick_pose_tf:toTensor()
  pre_pick_tensor[{{3}, {4}}] = pre_pick_tensor[{{3}, {4}}] + 0.1
  pre_pick_pose_tf:fromTensor(pre_pick_tensor)
  pre_pick_pose_tf.moveit_msgs_StampedTransform = pick_pose_tf.moveit_msgs_StampedTransform
  pre_pick_pose_tf.moveit_msgs_StampedPose = pick_pose_tf.moveit_msgs_StampedPose
  print(H_marker_to_world_tf)
  print('translation component:', H_marker_to_world_tf:toTensor()[{{1,3}, {4}}])

  print('pre_pick_pose_tf')
  print(pre_pick_pose_tf)
  print('pick_pose_tf')
  print(pick_pose_tf)

  self:moveLToTf(pre_pick_pose_tf)
  self:moveLToTf(pick_pose_tf)
  -- close gripper
  self.gripper:move{width=0.0, speed=0.2, force=20, stop_on_block=false} -- move closed
  self:moveLToTf(pre_pick_pose_tf)


end


return HandEye
