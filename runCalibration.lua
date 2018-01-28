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
local grippers = require 'xamlamoveit.grippers'
local autoCalibration = require 'autoCalibration_env'
local CalibrationMode = autoCalibration.CalibrationMode
local BASE_POSE_NAMES = autoCalibration.BASE_POSE_NAMES
require 'ximea.ros.XimeaClient'
require 'AutoCalibration'


local prompt
local configuration
local motion_service
local ximea_client
local auto_calibration


local function moveToStartPose(wait)
  print('Moving to start pose...')
  if wait ~= false then
    prompt:anyKey('press key when ready')
  end
  auto_calibration:moveToStart()
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
    auto_calibration:runCaptureSequence()
  end
end


local function calibrateCamera(wait)
  local ok = auto_calibration:monoCalibration()
  if ok then
    print('Calibration result:')
    print(auto_calibration.calibration)
  else
    print('Calibration failed.')
  end
  if wait ~= false then
    prompt:anyKey()
  end
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
  
  pickCalibrationTarget(false)
  runCaptureSequence(false)
  returnCalibrationTarget(false)
  calibrateCamera(false)
end


local function saveCalibration()
  if auto_calibration.calibration == nil then
    print('No calibration available to save.')
  else

    local mode = configuration.calibration_mode
    if mode == CalibrationMode.SingleCamera then

      local left_camera = self.configuration.cameras[configuration.left_camera_id]
      local camera_serial = left_camera.serial

      -- generate output directory path
      local calibration_name = os.date(configuration.calibration_name_template)
      local output_directory = path.join(configuration.output_directory, calibration_name)
      os.execute('mkdir -p ' .. output_directory)
      local calbration_fn = string.format('cam_%s.t7', camera_serial)
      local calibration_file_path = path.join(output_directory, calbration_fn)
      torch.save(calibration_file_path, auto_calibration.calibration)
      print('Calibration saved to: ' .. calibration_file_path)

      -- also linking calibration in current directory
      local current_output_directory = path.join(configuration.output_directory, 'current')
      local current_output_path = path.join(current_output_directory, calbration_fn)
      os.execute('mkdir -p ' .. current_output_directory)
      os.execute('rm ' .. current_output_path)
      local link_target = path.join('..', calibration_name, calbration_fn)
      os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
      printf("Created link in '%s' -> '%s'", current_output_path, link_target)
    elseif mode == CalibrationMode.StructuredLightSingleCamera then
      print('TODO')
    elseif mode == CalibrationMode.StereoRig then
      print('TODO')
    end
  end
  prompt:anyKey()
end


local function showMainMenu()
  local menu_options =
  {
    { 'h', 'Move to start pose', moveToStartPose },
    { 'p', 'Pick calibration target', pickCalibrationTarget },
    { 'r', 'Return calibration target', returnCalibrationTarget },
    { 'c', 'Capture calibration images', runCaptureSequence },
    { 'a', 'Calibrate camera', calibrateCamera },
    { 'f', 'Full calibraton cycle', runFullCycle },
    { 's', 'Save calibration', saveCalibration },
    { '1', 'Show current configuration', showCurrentConfiguration },
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

    print('Stereo calibration not yet supported')
    return false

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

  if not validateConfiguration() then
    print("Configuration not valid. Use the 'configureCalibration' script to create a valid configuration.")
    return
  end

  local move_group = motion_service:getMoveGroup(configuration.move_group_name)  
  move_group:setVelocityScaling(configuration.velocity_scaling)
  printf('Set velocity scaling: %f', configuration.velocity_scaling)

  auto_calibration = autoCalibration.AutoCalibration(configuration, move_group, ximea_client)

  showMainMenu()
end


local function init()
  -- initialize ROS
  ros.init('runCalibration', ros.init_options.AnonymousName)
  local nh = ros.NodeHandle()
  local sp = ros.AsyncSpinner()
  sp:start()

  motion_service = motionLibrary.MotionService(nh)
  ximea_client = XimeaClient(nh, 'ximea_mono', false, false)

  prompt = xutils.Prompt()
  prompt:enableRawTerminal()
  ok, err = pcall(function() main(nh) end)
  prompt:restoreTerminalAttributes()

  if not ok then
    print(err)
  end

  motion_service:shutdown()
  ximea_client:shutdown()
  sp:stop()
  ros.shutdown()
end


init()
