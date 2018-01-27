local ros = require 'ros'
local datatypes = require 'xamlamoveit.datatypes'
local motionLibrary = require 'xamlamoveit.motionLibrary'
local xutils = require 'xamlamoveit.xutils'
local grippers = require 'xamlamoveit.grippers'
require 'ximea.ros.XimeaClient'
require 'AutoCalibration'


--[[
Shell command to increase USB buffers (e.g. required for Ximea cams to work):
> sudo tee /sys/module/usbcore/parameters/usbfs_memory_mb >/dev/null <<<0
]]


local GET_CONNECTED_DEVICES_SERVICE_NAME = '/ximea_mono/get_connected_devices'


local motion_service -- motion service
local move_group -- move group
local auto_calibration
local ximea_client -- ximea client
local prompt


local move_group_names, move_group_details
local ximea_serials
local filename = 'configuration.t7'


local configuration = {
  move_group_name = nil,
  camera_serial = nil,
  exposure = 120000,
  sleep_before_capture = 1.0,
  output_directory = './calibration/',
  calibration_name_template = '%Y-%m-%d_%H%M%S',
  circle_pattern_geometry = torch.Tensor({21, 8, 5.0}), -- rows, cols, pointDist
  circle_pattern_id = 21,
  velocity_scaling = 0.2,
  scan = false,
  base_poses = {},
  capture_poses = {},
}


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


local function showCurrentConfiguration()
  prompt:printTitle('Current configuration')
  print(configuration)
  prompt:anyKey()
end


local function createMoveGroup()
  move_group = motion_service:getMoveGroup(configuration.move_group_name)
  move_group:setVelocityScaling(0.2)
  auto_calibration = AutoCalibration(configuration, move_group, ximea_client)
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


local function setCameraExposure()
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


local function showMainMenu()
  local menu_options =
  {
    { '1', 'Show current configuration', showCurrentConfiguration },
    { '2', 'Select move-group', selectMoveGroup },
    { '3', 'Select camera serial', selectCameraSerial },
    { '4', 'Set camera exposure', setCameraExposure },
    { '5', 'Set pattern ID', setPatternId },
    { '6', 'Teach base poses',  teachBasePoses },
    { '7', 'Teach capture poses', teachCapturePoses },
    { '8', 'Set velocity scaling', setVelocityScaling },
    { '9', 'Actuator menu', showActuatorMenu },
    { 's', 'Save configuration',  saveConfiguration },
    { 'ESC', 'Quit', false },
  }
  prompt:showMenu('Main Menu', menu_options)
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
  if #ximea_serials > 0 and (configuration.camera_serial == nil or #configuration.camera_serial == 0) then
    configuration.camera_serial = ximea_serials[1]
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
