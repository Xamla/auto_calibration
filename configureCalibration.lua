#!/usr/bin/env th

--[[

  Xamla Auto Camera Calibration

  Copyright 2018 Andreas Koepf, Xamla/PROVISIO GmbH
  All rights reserved.

]]

--[[
Shell command to increase USB buffers (e.g. required for Ximea cams to work):
> sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null <<<0
]]

local ros = require 'ros'
local datatypes = require 'xamlamoveit.datatypes'
local motionLibrary = require 'xamlamoveit.motionLibrary'
local xutils = require 'xamlamoveit.xutils'
local grippers = require 'xamlamoveit.grippers'
local autoCalibration = require 'autoCalibration_env'
local CalibrationMode = autoCalibration.CalibrationMode
require 'ximea.ros.XimeaClient'
require 'AutoCalibration'


local GET_CONNECTED_DEVICES_SERVICE_NAME = '/ximea_mono/get_connected_devices'
local DEFAULT_CAMERA_ID = 'left'
local DEFAULT_EXPOSURE = 120000
local DEFAULT_SLEEP_BEFORE_CAPTURE = 1.0


local motion_service -- motion service
local move_group -- move group
local auto_calibration
local ximea_client -- ximea client
local prompt
local move_group_names, move_group_details
local ximea_serials
local filename = 'configuration.t7'


-- create Camera configuration table
local function createCameraConfiguration(id, serial, exposure, sleep_before_capture)
  return {
    id = id,
    serial = serial,
    exposure = exposure or DEFAULT_EXPOSURE,    -- wait time in microsecongs
    sleep_before_capture = sleep_before_capture or DEFAULT_SLEEP_BEFORE_CAPTURE   -- wait time in s
  }
end


local configuration = {
  move_group_name = nil,
  calibration_mode = CalibrationMode.Mono,
  cameras = {},
  left_camera_id = nil,
  right_camera_id = nil,
  output_directory = './calibration/',
  calibration_name_template = '%Y-%m-%d_%H%M%S',
  circle_pattern_geometry = torch.Tensor({21, 8, 5.0}), -- rows, cols, pointDist
  circle_pattern_id = 21,
  checkerboard_pattern_geometry = torch.Tensor({7, 11, 10}),
  velocity_scaling = 0.2,
  base_poses = {},
  capture_poses = {},
}


local function getCameraIds()
  return table.keys(configuration.cameras)
end



local function queryXimeaSerials(nh)
  local getConnectedDevices = nh:serviceClient(GET_CONNECTED_DEVICES_SERVICE_NAME, ros.SrvSpec('ximea_msgs/GetConnectedDevices'), false)
  local response = getConnectedDevices:call()
  getConnectedDevices:shutdown()
  if response == nil then
    return {}
  end
  return response.serials
end


local function teachNamedPositions(position_names)
  local qs, poses = {}, {}

  for i,name in ipairs(position_names) do
    printf("Please move robot to pose: '%s'. Press 'Enter' when ready.", name)
    local go = prompt:waitEnterOrEsc()
    if not go then
      return nil
    end

    local q = move_group:getCurrentJointValues()
    print("Joint positions:")
    print(q.values)

    local p = move_group:getCurrentPose()
    printf("Pose: '%s':", name)
    print(p)

    qs[name] = q
    poses[name] = q
  end

  return qs, poses
end


local function definePos(pos_list, last_q, i)
  printf("Please move robot to pose #%d. Press 'Enter' when ready or 'ESC' to cancel.", i)
  local go = prompt:waitEnterOrEsc()
  if not go then
    return nil
  end

  local q = move_group:getCurrentJointValues()

  pos_list[i] = q
  printf("Joint configuration #%d:", i)
  print(q:getNames())
  print(q.values)

  return q
end


local function editPoseList(pos_list)
  local last_q = move_group:getCurrentJointValues()
  for i=1,#pos_list do
    printf('Moveing to pos #%d...', i)
    local q = pos_list[i]
    moveJ(q)
    last_q = definePos(pos_list, last_q, i)
    if last_q == nil then
      return nil -- abort
    end
  end
  return pos_list
end


local function recordPoseList(count)
  local last_q = move_group:getCurrentJointValues()
  local pos_list = {}

  for i=1,count do
    last_q = definePos(pos_list, last_q, i)
    if last_q == nil then
      return nil
    end
  end
  return pos_list
end



local function moveJ(pos)
  move_group:moveJ(pos)
end


local function moveToStartPose()
  local base_poses = configuration.base_poses
  if base_poses == nil or #table.keys(base_poses) == 0 then
    prompt:anyKey('Error: Base poses missing. Please teach base poses first.')
    return
  end

  print('Moving to start pose...')
  prompt:anyKey('press key when ready')
  auto_calibration:moveToStart()
end


local function moveToCaptureBase()
  local base_poses = configuration.base_poses
  if base_poses == nil or #table.keys(base_poses) == 0 then
    prompt:anyKey('Error: Base poses missing. Please teach base poses first.')
    return
  end

  print('Moving to capture base pose...')
  prompt:anyKey('press key when ready')
  auto_calibration:moveToCaptureBase()
end


local function pickCalibrationTarget(wait)
  local base_poses = configuration.base_poses
  if base_poses == nil or #table.keys(base_poses) == 0 then
    prompt:anyKey('Error: Base poses missing. Please teach base poses first.')
    return
  end

  print('Picking calibration target...')
  if wait ~= false then
    prompt:anyKey('press key when ready')
  end

  auto_calibration:pickCalibrationTarget()
end


local function returnCalibrationTarget(wait)
  local base_poses = configuration.base_poses
  if base_poses == nil or #table.keys(base_poses) == 0 then
    prompt:anyKey('Error: Base poses missing. Please teach base poses first.')
    return
  end

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
    auto_calibration:runCaptureSequenceWithoutCapture()
  end

  return true
end


local function runFullCycle(wait)
  prompt:printTitle('Full Cycle')
  if wait ~= false then
    print('enter to continue, ESC to cancel')
    local go = prompt:waitEnterOrEsc()
    if not go then
      return false
    end
  end

  pickCalibrationTarget(false)
  runCaptureSequence(false)
  returnCalibrationTarget(false)
  return true
end


local function runFullCycleLoop()
  prompt:printTitle('Full Cycle Loop')
  print('enter to continue, ESC to cancel')
  while true do
    local ok, err = pcall(function() runFullCycle(false) end)
    if not ok then
      print('Failed: ' .. err)
      break
    end
    if prompt:kbhit() then
      break
    end
  end
end


local function openGripper()
  auto_calibration:openGripper()
end


local function closeGripper()
  auto_calibration:closeGripper()
end


local function homeGripper()
  auto_calibration:homeGripper()
end


local function ackGripper()
  auto_calibration:ackGripper()
end


local function showActuatorMenu()
  local menu_options = {
    { 'h', 'Move to start pose', moveToStartPose },
    { 's', 'Move to capture base', moveToCaptureBase },
    { 'p', 'Pick calibration target', pickCalibrationTarget },
    { 'r', 'Return calibration target', returnCalibrationTarget },
    { 'c', 'Execute capture sequence moves (without capture)', runCaptureSequence },
    { 'f', 'Execute full cycle (pick -> capture -> return)', runFullCycle },
    { 'l', 'Full cycle LOOP ..', runFullCycleLoop },
    { 'o', 'Open gripper (release)', openGripper },
    { 'g', 'Close gripper (grip)', closeGripper },
    { 'x', 'Home gripper', homeGripper },
    { 'a', 'Ack gripper', ackGripper },
    { 'ESC', 'Return to main menu...', false },
  }
  prompt:showMenu('Actuator Menu', menu_options);
end


local function dumpConfiguration()
  prompt:printTitle('Current configuration')
  print(configuration)
  prompt:anyKey()
end


local function createMoveGroup()
  move_group = motion_service:getMoveGroup(configuration.move_group_name)
  move_group:setVelocityScaling(0.2)
  auto_calibration = autoCalibration.AutoCalibration(configuration, move_group, ximea_client)
end


local function selectMoveGroup()
  prompt:printTitle('Select Move Group')
  printf("Currently selected move group: '%s'", configuration.move_group_name)
  local choice = prompt:chooseFromList(move_group_names, 'Available move groups:')
  if choice == nil then
    return
  elseif configuration.move_group_name == choice then
    print('Move group not changed.')
  else

    configuration.move_group_name = choice
    printf("Changed move group to: '%s'", configuration.move_group_name)
    if #configuration.base_poses > 0 then
      print('Clearing base poses.')
      configuration.base_poses = {}
    end
    if #configuration.capture_poses > 0 then
      print('Clearing capture poses.')
      configuration.capture_poses = {}
    end

    createMoveGroup()
  end
  prompt:anyKey()
end


local function setPatternId()
  prompt:printTitle('Set Calibration Pattern ID')
  printf('Current calibration pattern ID: %d', configuration.circle_pattern_id)
  print('Enter patern ID:')
  local value = prompt:readNumber()
  if value ~= nil and value > 0 and value == value then
    configuration.circle_pattern_id = math.floor(value)
    printf('New calibration pattern ID: %d', configuration.circle_pattern_id)
  else
    print('Invalid pattern ID value. Nothing changed.')
  end
  prompt:anyKey()
end


local function teachBasePoses()
  prompt:printTitle('Teach Base Poses')
  local base_poses = teachNamedPositions({
    'start',
    'pre_pick_marker',
    'pick_marker',
    'post_pick_marker',
    'camera1_base'
  })
  if base_poses ~= nil then
    configuration.base_poses = base_poses
    print('New base poses defined.')
  else
    print('Teach operation cancelled. Using old values.')
  end
  prompt:anyKey()
end


local function teachNewCapturePoses()
  print('How many? Enter the number of poses to teach:')
  local count = prompt:readNumber()
  if count > 0 and count == count then
    local poses = recordPoseList(count)
    if poses ~= nil then
      configuration.capture_poses = poses
    end
  else
    print('Invalid input. Nothing changed')
  end
  prompt:anyKey()
end


local function editExistingCapturePoses()
  editPoseList(configuration.capture_poses)
end


local function teachCapturePoses()
  local menu_options = {
    { 't', 'Teach new capture poses', teachNewCapturePoses }
  }
  if configuration.capture_poses ~= nil and #configuration.capture_poses > 0 then
    menu_options[#menu_options + 1] = { 'e', 'Edit existing capture poses', editExistingCapturePoses }
  end
  menu_options[#menu_options + 1] = { 'ESC', 'Return to main menu...', false }
  prompt:showMenu('Teach Calibration Poses', menu_options)
end


local function setVelocityScaling()
  prompt:printTitle('Set Velocity Scaling')
  printf('Current velocity scaling: %f', configuration.velocity_scaling)
  print('Enter velocity scaling (0-1):')
  local value = prompt:readNumber()
  if value == nil or value < 0 or value > 1 then
    print('Input out of range or not a number.')
  else
    move_group:setVelocityScaling(value)
    configuration.velocity_scaling = value
    printf('New velocity scaling value: %f', configuration.velocity_scaling)
  end
  prompt:anyKey()
end


local function setCalibrationMode(mode)
  configuration.calibration_mode = mode
  printf('New calibration mode: %s', configuration.calibration_mode)
  prompt:anyKey()
end


local function selectCalibrationMode()
  local menu_options =
  {
    { '1', 'Mono camera calibration', function() setCalibrationMode(CalibrationMode.Mono) end },
    { '2', 'Stereo camera calibraiton', function() setCalibrationMode(CalibrationMode.Stereo) end },
    { '3', 'Mono structured light calibration', function() setCalibrationMode(CalibrationMode.CalibrationMode.StructuredLightMono) end },
    { 'ESC', 'Return to main menu', false },
  }
  prompt:showMenu('Main Menu', menu_options)
end


local function saveConfiguration()
  prompt:printTitle('Save Configuration')
  printf("Enter filename: (leave empty to use default filename '%s')", filename)
  local fn = prompt:readLine()
  if fn ~= nil and #fn > 0 then
    filename = fn
  end
  torch.save(filename, configuration)
  printf("Configuration saved to file '%s'.", filename)
  prompt:anyKey()
end


local function addCameraConfiguration()
  local default_id = DEFAULT_CAMERA_ID
  prompt:printTitle('Add Camera')
  printf("Enter ID of camera configuration (default '%s'):", default_id)
  local id = prompt:readLine()
  if id == nil or #id == 0 then
    id = default_id
  end
  local existing_ids = getCameraIds()
end


local function deleteCameraConfiguration(id)
  configuration.cameras
end


local function selectCameraSerial()
  prompt:printTitle('Select Camera Serial')
  printf("Selected camera S/N: %s", configuration.camera_serial or 'N/A')
  if ximea_serials ~= nil and #ximea_serials > 0 then
    local choice = prompt:chooseFromList(ximea_serials, 'Available Ximea cameras:')
    if choice == nil then
      return
    elseif configuration.camera_serial == choice then
      print('Camera not changed.')
    else
      configuration.camera_serial = choice
      printf("Changed camera serial to: '%s'", configuration.camera_serial)
    end
  else
    print('No camera available.')
  end
  prompt:anyKey()
end


local function setCameraExposure(serial)
  prompt:printTitle('Set Camera Exposure')
  printf('Current exposure value: %f ms', configuration.exposure / 1000.0)
  print('Enter exposure duration (in ms):')
  local value = prompt:readNumber()
  if value ~= nil and value > 0 and value == value then -- last is check for NaN
    configuration.exposure = math.floor(value * 1000.0)
    printf('New exposure value: %f ms', configuration.exposure * 1000)
  else
    print('Invalid exposure value. Nothing changed.')
  end
  prompt:anyKey()
end


local function editCamera(serial)
  local menu_options =
  {
    { '1', 'Edit camera exposure duration', function() return setCameraExposure(serial) end },
    { '2', 'Delete camera configuration', function() return deleteCameraConfiguration(serial) end },
    { 'ESC', 'Return to camera menu', false }
  }
  prompt:showMenu(string.format("Camera Configuration '%s'", serial), menu_options)
end


local function selectCamera(camera_type)
  prompt:printTitle(string.format("Select Camera '%s'", camera_type))
  printf("Selected camera ID: %s", configuration[camera_type] or 'N/A')

  local ids = getCameraIds()
  if ids ~= nil and #ids > 0 then
    local choice = prompt:chooseFromList(ids, 'Available Ximea cameras:')
    if choice == nil then
      return
    else
      configuration[camera_type] = choice
      printf("Set %s to camera ID '%s'.", camera_type, choice)
    end
  else
    print('No camera configuration available. Please add a camera configuration first.')
  end
  prompt:anyKey()
end


local function editCameraSetup()

  local generateMenuOptions = function()
    local menu_options = 
    {
      { '1', 'Add camera configuration', addCameraConfiguration },
      { '2', string.format("Select left camera ('%s') (used for single cam/SL calibration)", configuration.right_camera_id), function() selectCamera('left_camera_id') end },
      { '3', string.format("Select right camera ('%s')", configuration.right_camera_id), function() selectCamera('right_camera_id') end   }
    }
    menu_options[#menu_options + 1] = { 'ESC', 'Quit', false }
    return menu_options
  end

  prompt:showMenu('Camera Setup', generateMenuOptions)
end


local function showMainMenu()
  -- use generator function to display current configuration values
  local generateMenuOptions = function()
    return {
      { '1', string.format('Set calibration mode (%s)', configuration.calibration_mode), selectCalibrationMode },
      { '2', string.format("Select move-group ('%s')", configuration.move_group_name), selectMoveGroup },
      { '3', string.format('Edit camera setup (%d configured)', #configuration.cameras), editCameraSetup },
      { '4', string.format('Set circle pattern ID (%d)', configuration.circle_pattern_id), setPatternId },
      { '5', 'Teach base poses',  teachBasePoses },
      { '6', string.format('Teach capture poses (%d defined)', #configuration.capture_poses), teachCapturePoses },
      { '7', string.format('Set velocity scaling (%f)', configuration.velocity_scaling), setVelocityScaling },
      { '8', 'Actuator menu', showActuatorMenu },
      { '9', 'Dump configuration', dumpConfiguration },
      { 's', 'Save configuration',  saveConfiguration },
      { 'ESC', 'Quit', false },
    }
  end
  prompt:showMenu('Main Menu', generateMenuOptions)
end


local function main(nh)
  prompt:printXamlaBanner()
  print(' AutoCalibration v.0 Configuration\n\n')

  -- initialize motion service & query move groups
  motion_service = motionLibrary.MotionService(nh)
  move_group_names, move_group_details = motion_service:queryAvailableMoveGroups()
  if #move_group_names > 0 and (configuration.move_group_name == nil or #configuration.move_group_name == 0) then
    configuration.move_group_name = move_group_names[1]  -- by default use first available move group
  end

  -- query camera serial numbers
  ximea_serials = {}
  ximea_serials = queryXimeaSerials(nh)
  if #ximea_serials > 0 and #configuration.cameras == 0 then
    -- create default configuration left
    configuration.cameras[DEFAULT_CAMERA_ID] = createCameraConfiguration(DEFAULT_CAMERA_ID, ximea_serials[1])
    configuration.left_camera_id = DEFAULT_CAMERA_ID
  end

  if #ximea_serials > 0 then
    ximea_client = XimeaClient(nh, 'ximea_mono', false, false)
  end

  createMoveGroup()

  prompt:showList(move_group_names, 'Available MoveGroups:')
  prompt:showList(ximea_serials, 'Available Ximea Cameras:')

  showMainMenu()

  if auto_calibration ~= nil then
    auto_calibration:shutdown()
  end

  -- shutdown system
  if ximea_client ~= nil then
    ximea_client:shutdown()
  end

  motion_service:shutdown()
end


-- wrapper around main to ensure that terminal flag are restored in case of an error
local function init()
  -- parse command line
  local cmd = torch.CmdLine()
  cmd:text()
  cmd:text()
  cmd:text('Xamla AutoCalibration configuration script.')
  cmd:text()
  cmd:option('-edit', '', 'configuration input filename')

  local opt = cmd:parse(arg)

  if #opt.edit > 0 then
    filename = opt.edit
    configuration = torch.load(filename)
  end

  -- initialize ROS
  ros.init('configureCalibration', ros.init_options.AnonymousName)
  local nh = ros.NodeHandle()
  local sp = ros.AsyncSpinner()
  sp:start()

  prompt = xutils.Prompt()
  prompt:enableRawTerminal()
  ok, err = pcall(function() main(nh) end)
  prompt:restoreTerminalAttributes()

  if not ok then
    print(err)
  end

  sp:stop()
  ros.shutdown()
end


init()
