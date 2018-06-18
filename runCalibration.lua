#!/usr/bin/env th
--[[

  Configuration script for the
  Xamla Auto Camera Calibration

  Copyright 2018 Andreas Koepf, Xamla/PROVISIO GmbH
  All rights reserved.

  This script is part of the Rosvita robot programming system.
  You may only use it in production when you own a valid
  Rosvita license.

]]
local ros = require 'ros'
local datatypes = require 'xamlamoveit.datatypes'
local motionLibrary = require 'xamlamoveit.motionLibrary'
local xutils = require 'xamlamoveit.xutils'
local grippers = require 'xamlamoveit.grippers.env'
local autoCalibration = require 'autoCalibration_env'
local CalibrationMode = autoCalibration.CalibrationMode
local BASE_POSE_NAMES = autoCalibration.BASE_POSE_NAMES
require 'ximea.ros.XimeaClient'
require 'GenICamClient'

require 'AutoCalibration'
local HandEye = require 'HandEye'

local offline = false --true  -- in case we are reading images from files and not really connecting to the driver set offline to true


local prompt
local configuration
local motion_service
local ximea_client
local auto_calibration
local hand_eye
local end_effector


local function moveToStartPose(wait)
  print('Moving to start pose...')
  if wait ~= false then
    prompt:anyKey('press key when ready')
  end
  auto_calibration:moveToStart()
end


local function altCalibrationPaths()
  prompt:printTitle('Alternative calibration paths')
  auto_calibration:altCalibrationPaths()
  prompt:anyKey()
end


local function showCurrentConfiguration()
  prompt:printTitle('Current configuration')
  print(configuration)
  prompt:anyKey()
end


local function pickCalibrationTarget(wait)
  print('Picking calibration target...')
  if wait ~= false then
    prompt:anyKey('press key when ready')
  end
  auto_calibration:pickCalibrationTarget()
end


local function returnCalibrationTarget(wait)
  print('Returning calibartion target...')
  if wait ~= false then
    prompt:anyKey('press key when ready')
  end
  auto_calibration:returnCalibrationTarget()
end


local function runCaptureSequence(wait)
  local pos_list = configuration.capture_poses
  if pos_list == nil or #pos_list == 0 then
    prompt:anyKey('No capture poses defined.')
    return
  end

  printf('Running capture sequence with %d poses...', #pos_list)
  local go = true
  if wait ~= false then
    print('enter to continue, ESC to cancel')
    go = prompt:waitEnterOrEsc()
  end

  if go then

    -- check whether we are simulating or not
    if offline then
      auto_calibration:simulateCapture()
    else
      auto_calibration:runCaptureSequence()
    end
  end
end

--http://notebook.kulchenko.com/algorithms/alphanumeric-natural-sorting-for-humans-in-lua
function alphanumsort(o)
   local function conv(s)
      local res, dot = "", ""
      for n, m, c in tostring(s):gmatch"(0*(%d*))(.?)" do
         if n == "" then
            dot, c = "", dot..c
         else
            res = res..(dot == "" and ("%03d%s"):format(#m, m)
                                  or "."..n)
            dot, c = c:match"(%.?)(.*)"
         end
         res = res..c:gsub(".", "\0%0")
      end
      return res
   end
   table.sort(o,
      function (a, b)
         local ca, cb = conv(a), conv(b)
         return ca < cb or ca == cb and a < b
      end)
   return o
end


local function findDirectories(pattern, directory)
  pattern = pattern or '.'
  local l = {}
  if path.exists(directory) then
    for path_name in path.dir(directory) do
      local full_path = path.join(directory, path_name)
      print(full_path)
      if path.isdir(full_path) and path_name:match(pattern) then
        l[#l+1] = full_path
      end
    end
  end
  return alphanumsort(l)
end


function getFileName(url)
  return url:match("^.+/(.+)$")
end


local function selectAndCalibrateMonocularCamera(folder)

  --need to generate again the available folders/cameras

  local generateMenuOptions = function()

    -- generate menu options dynamically
    local available_cameras = findDirectories("%d%d%d%d%d%d%d%d", folder)
    local menu_options = {}
    for key, dir in pairs(available_cameras) do
        menu_options[#menu_options + 1] = {tostring(#menu_options + 1), string.format("Select  ('%s') ", dir), function() return auto_calibration:monoCalibration(nil,path.join(dir,'capture')) end}
    end

    menu_options[#menu_options + 1] = { 'ESC', 'Quit', false }
    return menu_options
  end
  prompt:showMenu('Calibration Folder Selection', generateMenuOptions)
end


local function selectCalibrationFolder()

    local generateMenuOptions = function()

    -- generate menu options dynamically
    local directories = findDirectories("._%d%d%d%d%d%d", './calibration')
    local menu_options = {}
    for key, folder in pairs(directories) do
        menu_options[#menu_options + 1] = {tostring(#menu_options + 1), string.format("Select  ('%s') ", folder), function() selectAndCalibrateMonocularCamera(folder) return false end}
    end

    menu_options[#menu_options + 1] = { 'ESC', 'Quit', false }
    return menu_options
  end
  prompt:showMenu('Calibration Folder Selection', generateMenuOptions)
end


-- used for the monocular camera calibration
-- iterates over the cameras available in the configuration file
-- the user needs to select one
local function selectCamera()

  ids = {}
  for cam_index,serial  in pairs(configuration.cameras) do
    print('camera=', configuration.cameras[cam_index].serial)
    ids[#ids + 1] = configuration.cameras[cam_index].serial
  end

  local choice = prompt:chooseFromList(ids, 'Available Ximea cameras:')
  if choice ~= nil then
    print('selected camera ', choice)
  end
  return choice
end


local function calibrateCamera(wait)
  local mode = configuration.calibration_mode

  if mode == CalibrationMode.SingleCamera then

    -- still need to ask which camera, there need not only be one
    --local ok = selectCalibrationFolder()
    serial = selectCamera()
    local ok = auto_calibration:monoCalibration(nil, nil,serial)
    if ok then
      print('Calibration result:')
      print(auto_calibration.calibration)
    else
      print('Calibration failed.')
    end

  elseif mode == CalibrationMode.StereoRig then
    local ok = auto_calibration:stereoCalibration()
    if ok then
      print('Stereo calibration result:')
      print(auto_calibration.stereo_calibration)
    else
      print('Stereo calibration failed.')
    end

  elseif mode == CalibrationMode.StructuredLightSingleCamera then
    print("-- START CALIB")
    auto_calibration:monoStructuredLightCalibration()

  end
  if wait ~= false then
    prompt:anyKey()
  end
end


local function listCurrentImages(directory, serial)
  pattern = "cam_"..serial.."_%d+%.png$"
  local l = {}
  if path.exists(directory) then
    for file_name in path.dir(directory) do
      local full_path = path.join(directory, file_name)
      if path.isfile(full_path) and file_name:match(pattern) then
        l[#l+1] = full_path
      end
    end
  end
  return alphanumsort(l)
end


local function generateCurrentCapturedImageLog(serial)
  current_path = './calibration/capture/'
  list_imgs = listCurrentImages(current_path, serial)
  print(list_imgs)

  imgData = {imagePaths = {}}
  for key,path in pairs(list_imgs) do
    imgData.imagePaths[#imgData.imagePaths + 1]  = path
  end
  return imgData
end


local function handEye()
  prompt:printTitle('Hand-eye Calibration')

  --load the jsposes.t7 file

  if offline then
    local offline_jsposes_fn = path.join(configuration.output_directory, 'offline', 'jsposes.t7')
    print('Reading offline_jsposes_fn='..offline_jsposes_fn)
    if path.exists(offline_jsposes_fn) then
      jsposes = torch.load(offline_jsposes_fn)
    else
      print('Offline poses file does not exist: '..offline_jsposes_fn)
      return false
    end

  else
    local jsposes_fn = './calibration/current/jsposes.t7'
    if path.exists(jsposes_fn) then
      jsposes = torch.load(jsposes_fn)
    else
      print('Offline poses file does not exist: '..jsposes_fn)
      return false
    end

  end

  left_cam_data = generateCurrentCapturedImageLog(configuration.cameras[configuration.left_camera_id].serial)
  right_cam_data = generateCurrentCapturedImageLog(configuration.cameras[configuration.right_camera_id].serial)
  img_data = {}
  img_data.imgDataLeft = left_cam_data
  img_data.imgDataRight = right_cam_data
  img_data.jsposes = jsposes

  hand_eye:calibrate(img_data)
  prompt:anyKey()
end


local function saveCalibration()
  auto_calibration:saveCalibration()
  prompt:anyKey()
end


local function runFullCycle(wait)
  prompt:printTitle('Full Cycle')
  if wait ~= false then
    print('enter to continue, ESC to cancel')
    local go = prompt:waitEnterOrEsc()
    if not go then
      return
    end
  end


  if offline then
    runCaptureSequence(false)
    calibrateCamera(false)
    saveCalibration()
    handEye()
  else
    pickCalibrationTarget(false)
    runCaptureSequence(false)
    returnCalibrationTarget(false)
    calibrateCamera(false)
    saveCalibration()
    handEye()
  end

end


--moves the pattern some distance along its x or y-axis
local function movePattern()
  prompt:printTitle('Move Pattern')
  hand_eye:movePattern()
end

local function detectPattern()
  prompt:printTitle('Detect Pattern')
  hand_eye:detectPattern()
end

local function detectPatternPointCloud()
  prompt:printTitle('Detect Pattern in Point Cloud')
  hand_eye:locateCirclePatternInPointCloud()
end

local function detectPatternStereoPointCloud()
  prompt:printTitle('Detect Pattern in Stereo Point Cloud')
  hand_eye:locateCirclePatternInStereoPointCloud()
end

local function getPointCloud()
  prompt:printTitle('Get point cloud')
  hand_eye:getPointCloud()
end

local function getStereoPointCloud()
  prompt:printTitle('Get stereo point cloud')
  hand_eye:getStereoPointCloud()
end

local function evaluateCalibration()
  prompt:printTitle('Evaluate calibration')
  hand_eye:evaluateCalibration()
end

local function pickingPose()
  prompt:printTitle('Create picking pose')
  hand_eye:pickingPose()
end

local function moveToMarker()
  prompt:printTitle('Create picking pose')
  hand_eye:moveToMarker()
end

local function moveToMarkerWithPrePickPose()
  prompt:printTitle('Moving to int marker with pre-pick pose')
  hand_eye:moveToIntMarkerPoseWithPrePick()
end

local function moveToInitPoseSupervised()
  prompt:printTitle('Move to stored pose in supervised mode')
  hand_eye:moveToInitPoseSupervised()
end

local function closeGripper()
  prompt:printTitle('Close gripper')
  auto_calibration:closeGripper()
end

local function openGripper()
  prompt:printTitle('Open gripper')
  auto_calibration:openGripper()
end


local function menuEvaluateCalibration()

  local menu_options =
  {

    { 'e', 'Evaluate calibration', evaluateCalibration },
    { 'm', 'Move the pattern 5cm towards the camera', movePattern },
    { 'd', 'Detect pattern', detectPattern },
    { 'g', 'Detect pattern in stereo point cloud', detectPatternStereoPointCloud },
    { 'z', 'Create picking pose', pickingPose },
    { 'n', 'Move end effector to int marker', moveToMarkerWithPrePickPose },
    { 'i', 'Move to initial pose supervised', moveToInitPoseSupervised },
    { 'q', 'Get point cloud', getPointCloud },
    { 's', 'Get stereo point cloud', getStereoPointCloud },

    { 'ESC', 'Quit', false },
  }
  prompt:showMenu('Evaluate Calibration Menu', menu_options)

end


local function showMainMenu()
  local menu_options =
  {
    { 'h', 'Move to start pose', moveToStartPose },
    { 'p', 'Pick calibration target', pickCalibrationTarget },
    { 'r', 'Return calibration target', returnCalibrationTarget },
    { 'c', 'Capture calibration images', runCaptureSequence },
    { 'a', 'Calibrate camera', calibrateCamera },
    { 'b', 'Hand-eye calibration', handEye },
    { 'f', 'Full calibraton cycle', runFullCycle },
    { 'e', 'Evaluate calibration menu', menuEvaluateCalibration },
    { 'x', 'Close gripper', closeGripper },
    { 'y', 'Open gripper', openGripper },


    { 's', 'Save calibration', saveCalibration },
    { '1', 'Show current configuration', showCurrentConfiguration },
    { 't', 'Test alternative calibration paths', altCalibrationPaths },
    { 'ESC', 'Quit', false },
  }
  prompt:showMenu('Main Menu', menu_options)
end


local function checkKeys(t, key_list)
  if t == nil or type(t) ~= 'table' then
    return false
  end

  for i,k in ipairs(key_list) do
    if t[k] == nil then
      return false
    end
  end
  return true
end


local function validateConfiguration()

  if configuration.move_group_name == nil or #configuration.move_group_name == 0 then
    print('No move-group was specified in configuration.')
    return false
  end

  if not checkKeys(configuration.base_poses, BASE_POSE_NAMES) then
    print("Base poses missing.")
    return false
  end

  if configuration.capture_poses == nil or #configuration.capture_poses == 0 then
    print("No capture poses were defined.")
    return false
  end

  local cameras = configuration.cameras
  if cameras == nil or #table.keys(cameras) == 0 then
    print("No camera configuration was defined.")
    return false
  end

  local mode = configuration.calibration_mode
  if mode == CalibrationMode.SingleCamera or mode == CalibrationMode.StructuredLightSingleCamera then

    local camera_configuration = cameras[configuration.left_camera_id]
    if camera_configuration == nil then
      print("No camera was selected.")
      return false
    end

    if camera_configuration.serial == nil or #camera_configuration.serial == 0 then
      printf("Invalid camera serial number '%s'.", camera_configuration.serial)
      return false
    end

  elseif mode == CalibrationMode.StereoRig then

    local left_camera_configuration = cameras[configuration.left_camera_id]
    local right_camera_configuration = cameras[configuration.left_camera_id]
    if left_camera_configuration == nil then
      print("No left camera was selected.")
      return false
    end
    if right_camera_configuration == nil then
      print("No right camera was selected.")
      return false
    end

    if left_camera_configuration.serial == nil or #left_camera_configuration.serial == 0 then
      printf("Invalid left camera serial number '%s'.", left_camera_configuration.serial)
      return false
    end
    if right_camera_configuration.serial == nil or #right_camera_configuration.serial == 0 then
      printf("Invalid right camera serial number '%s'.", right_camera_configuration.serial)
      return false
    end

  else
    printf("Unsupported configuration mode '%s'.", mode)
    return false
  end

  return true
end


local function main(nh)
  -- parse command line
  local cmd = torch.CmdLine()
  cmd:text()
  cmd:text()
  cmd:text('Xamla AutoCalibration script.')
  cmd:text()
  cmd:option('-cfg', 'configuration.t7', 'configuration input filename')
  cmd:option('-scan', false, 'specify to enable structured light scanning')

  local opt = cmd:parse(arg)
  local filename = opt.cfg

  configuration = torch.load(filename)

  camera_client = {}
  -- in case we are reading images from files and not really connecting to the driver set offline to true
  if not offline then
    if configuration.camera_type == 'ximea' then
      camera_client = XimeaClient(nh, 'ximea_mono', false, false)
    elseif configuration.camera_type == 'genicam' then
      camera_client = GenICamClient(nh, 'genicam_mono', false, false)
    end
  end

  if not validateConfiguration() then
    print("Configuration not valid. Use the 'configureCalibration' script to create a valid configuration.")
    return
  end

  local move_group = motion_service:getMoveGroup(configuration.move_group_name)
  move_group:setVelocityScaling(configuration.velocity_scaling)
  printf('Set velocity scaling: %f', configuration.velocity_scaling)

  local motion_service = motionLibrary.MotionService(nh)
  local xamla_mg = motionLibrary.MoveGroup(motion_service, configuration.move_group_name) -- motion client

  auto_calibration = autoCalibration.AutoCalibration(configuration, move_group, camera_client)
  hand_eye = HandEye.new(configuration, auto_calibration.calibration_folder_name, move_group, motion_service, ximea_client, auto_calibration.gripper, xamla_mg)
  showMainMenu()

  -- shutdown system
  if not offline then
    camera_client:shutdown()
  end
end


local function init()
  -- initialize ROS
  ros.init('runCalibration', ros.init_options.AnonymousName)
  local nh = ros.NodeHandle()
  local sp = ros.AsyncSpinner()
  sp:start()

  motion_service = motionLibrary.MotionService(nh)

  prompt = xutils.Prompt()
  prompt:enableRawTerminal()
  ok, err = pcall(function() main(nh) end)
  prompt:restoreTerminalAttributes()

  if not ok then
    print(err)
  end

  motion_service:shutdown()
  sp:stop()
  ros.shutdown()
end


init()
