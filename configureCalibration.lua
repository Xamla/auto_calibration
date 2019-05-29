--[[
  configureCalibration.lua

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

local ros = require 'ros'
local tf = ros.tf
--local pcl = require 'pcl'
local cv = require 'cv'
local datatypes = require 'xamlamoveit.datatypes'
local rosvita = require 'xamlamoveit.rosvita'
local motionLibrary = require 'xamlamoveit.motionLibrary'
local xutils = require 'xamlamoveit.xutils'
local grippers = require 'xamlamoveit.grippers'

local ac = require 'auto_calibration'
local CalibrationMode = ac.CalibrationMode
local CalibrationFlags = ac.CalibrationFlags
local BASE_POSE_NAMES = ac.BASE_POSE_NAMES

require 'ximea.ros.XimeaClient'
local GenICamClient = ac.GenICamCameraClient

local GET_CONNECTED_XIMEA_DEVICES_SERVICE_NAME = '/ximea_mono/get_connected_devices'
local GET_CONNECTED_GENICAM_DEVICES_SERVICE_NAME = '/camera_aravis_node/get_connected_devices'
local DEFAULT_CAMERA_ID = 'left'
local DEFAULT_EXPOSURE = 120000
local DEFAULT_SLEEP_BEFORE_CAPTURE = 1.0
local RIGHT_CAMERA_ID = 'right'


local motion_service
local move_group
local auto_calibration
local camera_client
local prompt
local move_group_names, move_group_details
local camera_serials
local filename = 'configuration.t7'
local nh


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
  calibration_mode = CalibrationMode.SingleCamera,
  move_group_name = nil,
  cameras = {},
  left_camera_id = nil,
  right_camera_id = nil,
  output_directory = '/tmp/calibration/',
  calibration_directory_template = '%Y-%m-%d_%H%M%S/',
  calibration_name_template = '%Y-%m-%d_%H%M%S',
  calibration_flags_name = 'Default',
  circle_pattern_geometry = torch.Tensor({21, 8, 0.005}), -- cols, rows, pointDist
  circle_pattern_id = 21,
  checkerboard_pattern_geometry = torch.Tensor({7, 12, 10}),
  velocity_scaling = 0.2,
  base_poses = {},
  capture_poses = {},
  gripper_key = '',
  camera_location_mode = 'extern',
  camera_type = 'ximea',
  eval_poses = {},
  camera_reference_frame = 'BASE',
  debug_output = false
}


local function table_contains(table, element)
  for _, value in pairs(table) do
    if value == element then
      return true
    end
  end
  return false
end


local function setGripper(key)
  configuration.gripper_key = key
  printf('New selected gripper key: %s', configuration.gripper_key)
  prompt:anyKey()
end


local function selectGripper()
  local p = io.popen("rosservice list | grep 'robotiq_driver'")
  local robotiq = p:read("*all")
  p:close()
  p = io.popen("rosservice list | grep 'wsg_driver'")
  local wsg = p:read("*all")
  p:close()

  local function contains(table, element)
  for _, value in pairs(table) do
    if value == element then
      return true
    end
  end
  return false
  end

  local function detect_gripper_names(gripper_names, gripper_services_str)
    local _, count = gripper_services_str:gsub('\n', '\n')
    str_start = 0
    for i = 1, count do
      str_end = string.find(gripper_services_str, '\n', str_start+1)
      local current = string.sub(gripper_services_str, str_start+1, str_end)
      a,b = string.find(current, 'driver/')
      c,d = string.find(current, '/', b+1)
      local name = string.sub(current, 1, c-1)
      if not contains(gripper_names, name) then
        table.insert(gripper_names, name)
      end
      str_start = str_end
    end
  end

  local gripper_names = {}
  if robotiq ~= nil then
    detect_gripper_names(gripper_names, robotiq)
  end
  if wsg ~= nil then
    detect_gripper_names(gripper_names, wsg)
  end
 
  local menu_options = {}
  for i = 1, #gripper_names do
    menu_options[i] = { string.format('%d', i), gripper_names[i], function() setGripper(gripper_names[i]) return false end}
  end
  if #gripper_names == 0 then
    menu_options[#menu_options + 1] = { 'No gripper available!', '', false }
  end
  menu_options[#menu_options + 1] = { 'ESC', 'Quit', false }
  prompt:showMenu('Gripper Selection', menu_options)
  -- Initialize gripper
  move_group_names = motion_service:queryAvailableMoveGroups()
  if not table_contains(move_group_names, configuration.move_group_name) then
    print(string.format("%s is no available move group!", configuration.move_group_name))
    print("Please choose one of the available move groups.")
    local choice = prompt:chooseFromList(move_group_names, 'Available move groups:')
    configuration.move_group_name = choice
  end
  move_group = motion_service:getMoveGroup(configuration.move_group_name)
  move_group:setVelocityScaling(0.2)
  auto_calibration = ac.AutoCalibration(configuration, move_group, camera_client)
end


local function queryCameraSerials()
  if configuration.camera_type == 'ximea' then
    local getConnectedDevices = nh:serviceClient(GET_CONNECTED_XIMEA_DEVICES_SERVICE_NAME, ros.SrvSpec('ximea_msgs/GetConnectedDevices'), false)
    local response = getConnectedDevices:call()
    getConnectedDevices:shutdown()
    if response == nil then
      return {}
    end
    return response.serials
  elseif configuration.camera_type == 'genicam' then
    local getConnectedDevices = nh:serviceClient(GET_CONNECTED_GENICAM_DEVICES_SERVICE_NAME, ros.SrvSpec('camera_aravis/GetConnectedDevices'), false)
    local response = getConnectedDevices:call()
    getConnectedDevices:shutdown()
    if response == nil then
      print('could not reach GENICAM')
      return {}
    end
    return response.serials
  end
end


local function setCameraType(key)
  configuration.camera_type = key
  camera_serials = {}
  camera_serials = queryCameraSerials()
  if next(camera_serials) == nil then
    camera_serials = queryCameraSerials()
  end
  if #camera_serials > 0 and #table.keys(configuration.cameras) == 0 then
    -- create default configuration left
    configuration.cameras[DEFAULT_CAMERA_ID] = createCameraConfiguration(DEFAULT_CAMERA_ID, camera_serials[1])
    configuration.left_camera_id = DEFAULT_CAMERA_ID
    --if there is a second camera, create the configuration for it
    if #camera_serials == 2 then
      configuration.cameras[RIGHT_CAMERA_ID] = createCameraConfiguration(RIGHT_CAMERA_ID, camera_serials[2])
      configuration.right_camera_id = RIGHT_CAMERA_ID
    end
  end
  if #camera_serials > 0 then
    if configuration.camera_type == 'ximea' then
      camera_client = XimeaClient(nh, 'ximea_mono', false, false)
    elseif configuration.camera_type == 'genicam' then
      camera_client = GenICamClient(nh, 'genicam_mono', false, false)
    end
  end
  printf('New selected camera type: %s', configuration.camera_type) 
  prompt:showList(camera_serials, string.format('Available Cameras of type %s:', configuration.camera_type))
  prompt:anyKey()
end


local function selectCameraType()
  local menu_options = {}
  menu_options[1] = { '1', 'ximea', function() setCameraType('ximea') return false end}
  menu_options[2] = { '2', 'genicam', function() setCameraType('genicam') return false end}
  menu_options[3] = { 'ESC', 'Quit', false }
  prompt:showMenu('Camera Type Selction', menu_options)
end


local function getCameraIds()
  return table.keys(configuration.cameras)
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
    move_group:moveJoints(q)
    last_q = definePos(pos_list, last_q, i)
    if last_q == nil then
      return nil -- abort
    end
    torch.save(filename, configuration)
    printf("Configuration automatically saved to file '%s'.", filename)
  end

  print('How many additional poses? Enter the number of additional poses to teach:')
  local count = prompt:readNumber()
  if count > 0 and count == count then
    for i=#pos_list+1,#pos_list+count do
      last_q = definePos(pos_list, last_q, i)
      if last_q == nil then
        return nil
      end
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
  move_group_names = motion_service:queryAvailableMoveGroups()
  if not table_contains(move_group_names, configuration.move_group_name) then
    print(string.format("%s is no available move group!", configuration.move_group_name))
    print("Please choose one of the available move groups.")
    local choice = prompt:chooseFromList(move_group_names, 'Available move groups:')
    configuration.move_group_name = choice
  end
  move_group = motion_service:getMoveGroup(configuration.move_group_name)
  move_group:setVelocityScaling(0.2)
  auto_calibration = ac.AutoCalibration(configuration, move_group, camera_client)
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


local function setPatternGeometry()
  prompt:printTitle('Set Calibration Pattern Geometry')
  print('Current calibration pattern geometry (cols, rows, distance):')
  print(configuration.circle_pattern_geometry)
  print('Enter number of columns:')
  local cols = prompt:readNumber()
  print('Enter number of rows:')
  local rows = prompt:readNumber()
  print('Enter point distance:')
  local dist = prompt:readNumber()
  if rows ~= nil and rows > 0 and cols ~= nil and cols > 0 and dist ~= nil and dist > 0 then
    configuration.circle_pattern_geometry = torch.Tensor({cols, rows, dist})
    print('New calibration pattern geometry:')
    print(configuration.circle_pattern_geometry)
  else
    print('Invalid pattern geometry. Nothing changed.')
  end
  prompt:anyKey()
end


local function setPatternData()
  setPatternId()
  setPatternGeometry()
end

local function teachBasePoses()
  prompt:printTitle('Teach Base Poses')
  local base_poses = teachNamedPositions(BASE_POSE_NAMES)
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


local function movePoseCollisionFree(end_effector, target_pose, seed)
    local velocity_scaling = 0.5
    local plan_parameters = end_effector.move_group:buildPlanParameters(velocity_scaling)
    local seed = seed or end_effector.move_group:getCurrentJointValues()

    local ik_ok, solution =
        end_effector.motion_service:queryIK(
        target_pose,
        plan_parameters,
        seed:select(end_effector.move_group:getJointNames()),
        end_effector.link_name
    )

    if ik_ok[1].val ~= 1 then
        print('Failed inverse kinematic call')
        return false
    end
    local goal = datatypes.JointValues(seed.joint_set:clone(), solution[1].values)
    -- plan trajectory
    --local ok, joint_trajectory, ex_plan_parameters = end_effector.move_group:planMoveJointsCollisionFree(goal, velocity_scaling)
    local ok, err, joint_trajectory, ex_plan_parameters = pcall(function () return end_effector.move_group:planMoveJointsCollisionFree(goal, velocity_scaling) end)
    if ok then
        -- Collision check is not necessary since moveIt will handle the planning?
        -- -> see comment in Xamla/xamlamoveit/motionLibrary/MoveGroup.lua "moveJointsCollisionFree"
        --return end_effector.motion_service:executeJointTrajectory(joint_trajectory, true) -- true for collision check!!!
        local execute_ok = pcall(function () return end_effector.motion_service:executeJointTrajectory(joint_trajectory, true) end) -- true for collision check!!!
        if execute_ok then
          return true
        else
          print('Could not execute the planned joint trajectory.')
          return false
        end
    else
        print('Could not find a path to the given joint values.')
        return false
    end
end


local function CalcPointPositions (arg)
-- Calculate the "true" 3D position (x,y,z) of the circle centers of the circle pattern.
-- z position is set to 0 for all points
--
-- Input params:
  --  arg.pointsX  -- number of points in horizontal direction
  --  arg.pointsY  -- number of points in vertical direction
  --  arg.pointDistance -- distance between two points of the pattern in meter
-- Return value:
--    Position of the circle centers

  local corners = torch.FloatTensor(arg.pointsX*arg.pointsY, 1, 3):zero()
  local i=1
  for y=1, arg.pointsY do
    for x=1, arg.pointsX do
      corners[i][1][1] = (2*(x-1) + (y-1)%2) * arg.pointDistance
      corners[i][1][2] = (y-1)*arg.pointDistance
      corners[i][1][3] = 0
      i = i+1
    end
  end
  return corners
end


local function normalize(v)
   return v / torch.norm(v)
end


--- Calculates the TCP pose required to point the TCP z axis to point at with TCP at position eye
-- eye becomes origin, 'at' lies on x-axis
-- @param eye torch.Tensor(3,1); TCP position in x, y, z
-- @param at torch.Tensor(3,1); Point to look at
-- @param up torch.Tensor({0, 0, 1}); up direction of the camera
-- @return torch.Tensor(4,4), robot pose
local function PointAtPose(eye, at, up, handEye)
  local zaxis = normalize(at - eye)
  local xaxis = normalize(torch.cross(zaxis, up))
  local yaxis = torch.cross(zaxis, xaxis)

  local basis = torch.Tensor(3,3)
  basis[{{},{1}}] = xaxis
  basis[{{},{2}}] = yaxis
  basis[{{},{3}}] = zaxis

  local t = tf.Transform()
  t:setBasis(basis)
  t:setOrigin(eye)
  local tcpLookat=t:toTensor():clone()

  if handEye ~= nil then
    tcpLookat = tcpLookat * torch.inverse(handEye)
  end

  return tcpLookat
end


function RotVectorToRotMatrix(vec)
  -- transform a rotation vector as e.g. provided by solvePnP to a 3x3 rotation matrix using the Rodrigues' rotation formula
  -- see e.g. http://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#void%20Rodrigues%28InputArray%20src,%20OutputArray%20dst,%20OutputArray%20jacobian%29
  --
  -- Input parameters:
  --   vec = vector to transform
  -- Return value:
  --   3x3 rotation matrix

  local theta = torch.norm(vec)
  local r = vec/theta
  r = torch.squeeze(r)
  local mat = torch.Tensor({{0, -1*r[3], r[2]}, {r[3], 0, -1*r[1]}, {-1*r[2], r[1], 0}})
  r = r:resize(3,1)

  local result = torch.eye(3)*math.cos(theta) + (r*r:t())*(1-math.cos(theta)) + mat*math.sin(theta)

  return result
end


local function transformMatrixToQuaternion(rot)
  local sqrt = math.sqrt
  local trace = rot[1][1] + rot[2][2] + rot[3][3]
  local _next = { 2, 3, 1 }
  local q = torch.zeros(4)
  if trace > 0 then
    local r = sqrt(trace + 1)
    local s = 0.5 / r
    q[1] = 0.5 * r
    q[2] = (rot[3][2] - rot[2][3]) * s
    q[3] = (rot[1][3] - rot[3][1]) * s
    q[4] = (rot[2][1] - rot[1][2]) * s
  else
    local i = 1
    if rot[2][2] > rot[1][1] then
      i = 2
    end
    if rot[3][3] > rot[i][i] then
      i = 3
    end
    local j = _next[i]
    local k = _next[j]
    local t = rot[i][i] - rot[j][j] - rot[k][k] + 1
    local r = sqrt(t)
    local s = 0.5 / sqrt(t)
    local w = (rot[k][j] - rot[j][k]) * s
    q[1] = w
    q[i+1] = 0.5 * r
    q[j+1] = (rot[j][i] + rot[i][j]) * s
    q[k+1] = (rot[k][i] + rot[i][k]) * s
  end
  return q/q:norm()
end


local function transformQuaternionToMatrix(q)
  local w = q[1]
  local x = q[2]
  local y = q[3]
  local z = q[4]
  local result = torch.DoubleTensor(3,3)
  result[1][1] = 1 - 2*y*y - 2*z*z
  result[1][2] = 2*x*y - 2*w*z
  result[1][3] = 2*x*z + 2*w*y
  result[2][1] = 2*x*y + 2*w*z
  result[2][2] = 1 - 2*x*x - 2*z*z
  result[2][3] = 2*y*z - 2*w*x
  result[3][1] = 2*x*z - 2*w*y
  result[3][2] = 2*y*z + 2*w*x
  result[3][3] = 1 - 2*x*x - 2*y*y
  return result
end


local function calc_avg_heye(H1, H2)
  local Q = torch.DoubleTensor(2, 4)
  local avg_pos = torch.zeros(3)

  avg_pos = (H1[{{1,3},{4}}] + H2[{{1,3},{4}}]) / 2.0
  Q[1] = transformMatrixToQuaternion(H1[{{1,3},{1,3}}])
  Q[2] = transformMatrixToQuaternion(H2[{{1,3},{1,3}}])

  local QtQ = Q:t() * Q
  local e, V = torch.symeig(QtQ, 'V')
  local maxEigenvalue, maxEig_index = torch.max(e,1)
  local avg_q = V:t()[maxEig_index[1]]
  local avg_rot = transformQuaternionToMatrix(avg_q)
  local avg_H = torch.DoubleTensor(4,4)
  avg_H[{{1,3},{1,3}}] = avg_rot
  avg_H[{{1,3},{4}}] = avg_pos
  avg_H[4][4] = 1.0
  return avg_H
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


local function teachNewEvalPoses()
  print('How many? Enter the number of poses to teach:')
  local count = prompt:readNumber()
  if count > 0 and count == count then
    local poses = recordPoseList(count)
    if poses ~= nil then
      configuration.eval_poses = poses
    end
  else
    print('Invalid input. Nothing changed')
  end
  prompt:anyKey()
end


local function editExistingEvalPoses()
  editPoseList(configuration.eval_poses)
end


local function teachEvalPoses()
  local menu_options = {
    { 't', 'Teach new poses for evaluation', teachNewEvalPoses }
  }
  if configuration.eval_poses ~= nil and #configuration.eval_poses > 0 then
    menu_options[#menu_options + 1] = { 'e', 'Edit existing poses for evaluation', editExistingEvalPoses }
  end
  menu_options[#menu_options + 1] = { 'ESC', 'Return to main menu...', false }
  prompt:showMenu('Teach Evaluation Poses', menu_options)

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
    { '1', 'Single camera calibration', function() setCalibrationMode(CalibrationMode.SingleCamera) return false end },
    { '2', 'Stereo rig calibraiton', function() setCalibrationMode(CalibrationMode.StereoRig) return false end },
    { 'ESC', 'Return to main menu', false },
  }
  prompt:showMenu('Main Menu', menu_options)
end


local function setCameraLocation(location)
  configuration.camera_location_mode = location
  printf('New camera location mode: %s', configuration.camera_location_mode)
  prompt:anyKey()
end


local function selectCameraReferenceFrame()
  prompt:printTitle('Select Reference Frame of Extern Camera(s)')
  print("Note: Camera setup is extern from selected move group, but may be fixed onto another arm or torso of the robot.")
  print("      If this is the case, choose the corresponding reference frame (joint), otherwise choose \'BASE\'.")
  printf("Currently selected reference frame: '%s'", configuration.camera_reference_frame)
  local mg_all = motion_service:getMoveGroup(move_group_names[1])
  move_group_names = motion_service:queryAvailableMoveGroups()
  if not table_contains(move_group_names, configuration.move_group_name) then
    print(string.format("%s is no available move group!", configuration.move_group_name))
    print("Please choose one of the available move groups.")
    local choice = prompt:chooseFromList(move_group_names, 'Available move groups:')
    configuration.move_group_name = choice
  end
  local mg_chosen = motion_service:getMoveGroup(configuration.move_group_name)
  local reference_frames = {}
  table.insert(reference_frames, 'BASE')
  for i = 1,#mg_all.details.joint_names do
    local joint = mg_all.details.joint_names[i]
    local flag = 0
    for j = 1,#mg_chosen.details.joint_names do
      if joint == mg_chosen.details.joint_names[j] then
        flag = 1
      end
    end
    if flag == 0 then
      table.insert(reference_frames, mg_all.details.joint_names[i])
    end
  end
  local choice = prompt:chooseFromList(reference_frames, 'Available reference frames:')
  if choice == nil then
    return
  elseif configuration.camera_reference_frame == choice then
    print('Reference frame not changed.')
  else
    configuration.camera_reference_frame = choice
    printf("Changed reference frame to: '%s'", configuration.camera_reference_frame)
  end
end


local function selectCameraLocation()
  local menu_options =
  {
    { '1', 'Extern camera setup', function() setCameraLocation('extern') return false end },
    { '2', 'Onboard camera setup', function() setCameraLocation('onboard') return false end },
    { 'ESC', 'Return to main menu', false },
  }
  prompt:showMenu('Edit Camera Location', menu_options)

  if configuration.camera_location_mode == 'extern' and #move_group_names > 1 then
    selectCameraReferenceFrame()
  end
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


local function changeDebugOutput(debug)
  configuration.debug_output = debug
  print(string.format('New debug output mode: %s', configuration.debug_output))
  prompt:anyKey()
end


local function setDebugOutput()
  local menu_options =
  {
    { '1', 'true', function() changeDebugOutput(true) return false end },
    { '2', 'false', function() changeDebugOutput(false) return false end },
    { 'ESC', 'Return to main menu', false },
  }
  prompt:showMenu('Set debug output to true or false', menu_options)
end


local function addCameraConfiguration()
  prompt:printTitle('Add Camera')
  local default_id = 'right'
  if camera_serials ~= nil and #camera_serials > 0 then
    printf("Enter ID of new camera configuration (default '%s'):", default_id)
    local id = prompt:readLine()
    if id == nil or #id == 0 then
      id = default_id
    end

    local serial = prompt:chooseFromList(camera_serials, 'Choose camera S/N:')
    if serial ~= nil then
      local camera_configuration = createCameraConfiguration(id, serial)
      configuration.cameras[id] = camera_configuration

      print('Added new camera configuration:')
      print(camera_configuration)
    end

  else
    print('No camera available.')
  end

  prompt:anyKey()
end


local function deleteCameraConfiguration(id)
  configuration.cameras[id] = nil
  if configuration.left_camera_id == id then
    configuration.left_camera_id = nil
  end
  if configuration.right_camera_id == id then
    configuration.right_camera_id = nil
  end
  return false
end


local function selectCameraSerial()
  prompt:printTitle('Select Camera Serial')
  printf("Selected camera S/N: %s", configuration.camera_serial or 'N/A')
  if camera_serials ~= nil and #camera_serials > 0 then
    local choice = prompt:chooseFromList(camera_serials, 'Available cameras:')
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


local function setCameraExposure(id)
  local camera_configuration = configuration.cameras[id]
  prompt:printTitle('Set Camera Exposure')
  printf('Current exposure value: %f ms', camera_configuration.exposure / 1000.0)
  print('Enter exposure duration (in ms):')
  local value = prompt:readNumber()
  if value ~= nil and value > 0 and value == value then -- last is check for NaN
    camera_configuration.exposure = math.floor(value * 1000.0)
    printf('New exposure value: %f ms', camera_configuration.exposure / 1000.0)
  else
    print('Invalid exposure value. Nothing changed.')
  end
  prompt:anyKey()
end


local function editCamera(id)
  local camera_configuration = configuration.cameras[id]
  print(camera_configuration)
  local generateMenuOptions = function()
    return {
      { '1', string.format('Edit camera exposure duration (%f ms)', camera_configuration.exposure / 1000.0), function() return setCameraExposure(id) end },
      { '2', 'Delete camera configuration', function() return deleteCameraConfiguration(id) end },
      { 'ESC', 'Return to camera menu', false }
    }
  end
  prompt:showMenu(string.format("Camera Configuration '%s'", id), generateMenuOptions)
end


local function selectCamera(camera_type)
  prompt:printTitle(string.format("Select Camera '%s'", camera_type))
  printf("Selected camera ID: %s", configuration[camera_type] or 'N/A')

  local ids = getCameraIds()
  if ids ~= nil and #ids > 0 then
    local choice = prompt:chooseFromList(ids, 'Available cameras:')
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
      { 'a', 'Add camera configuration', addCameraConfiguration },
      { 'l', string.format("Select left camera ('%s')", configuration.left_camera_id), function() selectCamera('left_camera_id') end },
      { 'r', string.format("Select right camera ('%s')", configuration.right_camera_id), function() selectCamera('right_camera_id') end }
    }
    local i = 1
    for k,v in pairs(configuration.cameras) do
      menu_options[#menu_options + 1] = { tostring(i), string.format("Edit camera configuration '%s'", k), function() editCamera(k) end }
      i = i + 1
    end
    if configuration.camera_location_mode == 'extern' then
      menu_options[#menu_options + 1] = { 'm', string.format('Select camera mounting (\'%s\', reference frame: \'%s\')', configuration.camera_location_mode, configuration.camera_reference_frame), selectCameraLocation }
    else
      menu_options[#menu_options + 1] = { 'm', string.format('Select camera mounting (\'%s\')', configuration.camera_location_mode), selectCameraLocation }
    end
    menu_options[#menu_options + 1] = { 't', string.format('Select camera type (\'%s\')', configuration.camera_type), selectCameraType }
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
      { '3', string.format('Edit camera setup (%d configured, \'%s\' setup, \'%s\' type)', #table.keys(configuration.cameras), configuration.camera_location_mode, configuration.camera_type), editCameraSetup },
      { '4', string.format('Set circle pattern data (ID: %d, cols: %d, rows: %d, point distance: %f)', configuration.circle_pattern_id, configuration.circle_pattern_geometry[1], configuration.circle_pattern_geometry[2], configuration.circle_pattern_geometry[3]), setPatternData },
      { '5', 'Teach base poses',  teachBasePoses },
      { '6', string.format('Teach capture poses (%d defined)', #configuration.capture_poses), teachCapturePoses },
      { '7', string.format('Set velocity scaling (%f)', configuration.velocity_scaling), setVelocityScaling },
      { '8', 'Actuator menu', showActuatorMenu },
      { '9', 'Dump configuration', dumpConfiguration },
      { 'g', string.format('Gripper selection (%s)', configuration.gripper_key) , selectGripper },
      { 'e', string.format('Teach poses for evaluation (%d defined)', #configuration.eval_poses), teachEvalPoses },
      { 'd', string.format('Debug output setting (%s)', configuration.debug_output),  setDebugOutput },
      { 's', 'Save configuration',  saveConfiguration },
      { 'ESC', 'Quit', false },
    }
  end
  prompt:showMenu('Main Menu', generateMenuOptions)
end


local function main()
  prompt:enableRawTerminal()

  prompt:printXamlaBanner()
  print(' AutoCalibration v.0 Configuration\n\n')

  -- initialize motion service & query move groups
  motion_service = motionLibrary.MotionService(nh)
  move_group_names, move_group_details = motion_service:queryAvailableMoveGroups()
  if #move_group_names > 0 and (configuration.move_group_name == nil or #configuration.move_group_name == 0) then
    configuration.move_group_name = move_group_names[1]  -- by default use first available move group
  end

  -- query camera serial numbers
  camera_serials = {}
  camera_serials = queryCameraSerials()
  if next(camera_serials) == nil then
    camera_serials = queryCameraSerials()
  end
  if #camera_serials > 0 and #table.keys(configuration.cameras) == 0 then
    -- create default configuration left
    configuration.cameras[DEFAULT_CAMERA_ID] = createCameraConfiguration(DEFAULT_CAMERA_ID, camera_serials[1])
    configuration.left_camera_id = DEFAULT_CAMERA_ID
    --if there is a second camera, create the configuration for it
    if #camera_serials == 2 then
      configuration.cameras[RIGHT_CAMERA_ID] = createCameraConfiguration(RIGHT_CAMERA_ID, camera_serials[2])
      configuration.right_camera_id = RIGHT_CAMERA_ID
    end
  end
  if #camera_serials > 0 then
    if configuration.camera_type == 'ximea' then
      camera_client = XimeaClient(nh, 'ximea_mono', false, false)
    elseif configuration.camera_type == 'genicam' then
      camera_client = GenICamClient(nh, 'genicam_mono', false, false)
    end
  end

  createMoveGroup()

  prompt:showList(move_group_names, 'Available MoveGroups:')
  prompt:showList(camera_serials, string.format('Available Cameras of type %s:', configuration.camera_type))

  local p = io.popen("pwd")
  local start_path = p:read("*l")
  p:close()
  if not string.match(start_path, "projects") then
    print('\n')
    print('===================================================================================')
    print('= WARNING:                                                                        =')
    print('= Configuration data (capture poses ...) will get lost after stopping Rosvita!    =')
    print('= To permanently save configuration data, start script from your project folder:  =')
    print('= cd /home/xamla/Rosvita.Control/projects/<your_project_folder>                   =')
    print('= th ../../lua/auto_calibration/configureCalibration.lua                          =')
    print('===================================================================================')
  end

  showMainMenu()

  if auto_calibration ~= nil then
    auto_calibration:shutdown()
  end

  -- shutdown system
  if camera_client ~= nil then
    camera_client:shutdown()
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
  cmd:option('-cfg', '', 'configuration input filename')

  local opt = cmd:parse(arg)

  if #opt.cfg > 0 then
    filename = opt.cfg
    configuration = torch.load(filename)
  end

  -- initialize ROS
  ros.init('configureCalibration', ros.init_options.AnonymousName)
  nh = ros.NodeHandle()
  local sp = ros.AsyncSpinner()
  sp:start()

  prompt = xutils.Prompt()
  local terminal_attributes = xutils.saveTerminalAttributes()
  ok, err = xpcall(main, debug.traceback)
  xutils.restoreTerminalAttributes(terminal_attributes)

  --prompt = xutils.Prompt()
  --prompt:enableRawTerminal()
  --ok, err = pcall(function() main() end)
  --prompt:restoreTerminalAttributes()

  if not ok then
    print(err)
  end

  sp:stop()
  ros.shutdown()
end


init()
