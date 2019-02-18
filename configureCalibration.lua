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
  output_directory = './calibration/',
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
    local velocity_scaling = 0.1
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
    local ok, joint_trajectory, ex_plan_parameters =
        end_effector.move_group:planMoveJointsCollisionFree(goal, velocity_scaling)
    if ok then
        -- Collision check is not necessary since moveIt will handle the planning?
        -- -> see comment in Xamla/xamlamoveit/motionLibrary/MoveGroup.lua "moveJointsCollisionFree"
        return end_effector.motion_service:executeJointTrajectory(joint_trajectory, true) -- true for collision check!!!
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


-- Input: hand-eye matrix, (left) camera matrix and (left) camera distortion
local function captureSphereSampling_endOfArmCams()
  print('How many? Enter the number of capture poses:')
  local count = prompt:readNumber()
  local min_radius = 0.4
  local max_radius = 0.5
  local target_jitter = 0.015

  print("Enter filename of initial guess for hand-eye matrix (without quotation marks!)")
  local heye_fn = prompt:readLine()
  local heye = nil
  if heye_fn ~= "" then
    heye = torch.load(heye_fn)
  else
    print("Hand-Eye-Matrix not found!")
    return nil
  end
  print("Hand-eye matrix:")
  print(heye)
  print("Note: If hand-eye matrix is for tcp at last joint of robot arm (instead of tcp at gripper tip), then the tcp in the project configuration has to be set accordingly before starting the sphere sampling!")
  print("Enter filename of initial guess for stereo calibration")
  local stereoCalib_fn = prompt:readLine()
  local intrinsics = nil
  local distortion = nil
  local interCamTrafo = nil
  if stereoCalib_fn ~= "" then
    stereoCalib = torch.load(stereoCalib_fn)
    intrinsics = stereoCalib.camLeftMatrix:double()
    distortion = stereoCalib.camLeftDistCoeffs:double()
    interCamTrafo = stereoCalib.trafoLeftToRightCam:double()
  else
    print("Stereocalibration not found!")
    return nil
  end
  print("Left camera matrix:")
  print(intrinsics)
  print("Left camera distortion:")
  print(distortion)
  print("Transformation between cameras:")
  print(interCamTrafo)

  local heye_2 = heye * torch.inverse(interCamTrafo)
  print("Hand-eye matrix of right camera:")
  print(heye_2)
  local avg_heye = calc_avg_heye(heye, heye_2)
  print("average hand-eye:")
  print(avg_heye)

  local ok, centers = false, nil

  local pos_list = {}
  local file_names = {}
  local recorded_joint_values = {}   -- joint values after getImage calls
  local recorded_joint_tensors = {}  -- joint values as tensor after getImage calls
  local recorded_poses = {}          -- end effector poses after getImage calls
  local output_directory = path.join(configuration.output_directory, 'capture_shpere_sampling')
  print('Deleting output directory')
  os.execute('rm -rf '.. output_directory)
  print('Creating output directory')
  os.execute('mkdir -p ' .. output_directory)
  local left_camera = configuration.cameras[configuration.left_camera_id]
  local right_camera = configuration.cameras[configuration.right_camera_id]
  local ee_names = move_group:getEndEffectorNames()
  local ee = move_group:getEndEffector(ee_names[1])
  local calibPattern = { width = 8, height = 21, pointDistance = 0.005 }

  local overviewPose = move_group:getCurrentPose()
  local check = false

  while ros.ok() and not ok do

    print("Please move robot to overview pose and press 'Enter' when ready.")
    prompt:waitEnterOrEsc()
    overviewPoseTensor = move_group:getCurrentPose():toTensor()
    print("overviewPoseTensor:")
    print(overviewPoseTensor)

    -- TCP coordinate system is rotated 180° around y-axis of world coordinate system
    -- to look down to the table (such that the z-axis of the tcp coordinate system points to the table)
    --overviewPoseTensor[1] = torch.Tensor({-1.0,  0.0,  0.0,  move_group:getCurrentPose():toTensor()[1][4]})
    --overviewPoseTensor[2] = torch.Tensor({ 0.0,  1.0,  0.0,  move_group:getCurrentPose():toTensor()[2][4]})
    --overviewPoseTensor[3] = torch.Tensor({ 0.0,  0.0, -1.0,  move_group:getCurrentPose():toTensor()[3][4]})

    -- capture images
    local image_left
    local image_right


    if left_camera ~= nil then
      if left_camera.sleep_before_capture > 0 then
        printf('wait before capture %f s... ', left_camera.sleep_before_capture)
        sys.sleep(left_camera.sleep_before_capture)
      end
      --camera_client:setExposure(left_camera.exposure, {left_camera.serial})
      camera_client:setExposure(120000, {left_camera.serial})
      image_left = camera_client:getImages({left_camera.serial})
      if image_left:nDimension() > 2 then
        image_left = cv.cvtColor{image_left, nil, cv.COLOR_RGB2BGR}
      end
    end
    if right_camera ~= nil then
      if right_camera.sleep_before_capture > 0 then
        printf('wait before capture %f s... ', right_camera.sleep_before_capture)
        sys.sleep(right_camera.sleep_before_capture)
      end
      --camera_client:setExposure(right_camera.exposure, {right_camera.serial})
      camera_client:setExposure(120000, {right_camera.serial})
      image_right = camera_client:getImages({right_camera.serial})
      if image_right:nDimension() > 2 then
        image_right = cv.cvtColor{image_right, nil, cv.COLOR_RGB2BGR}
      end
    end


    -- load images for simulation
    --image_left = cv.imread { "calibration/cam_4103130811_001.png" }
    --image_right = cv.imread { "calibration/cam_4103189394_001.png" }

    ok_left, centers_left = cv.findCirclesGrid{ image = image_left,
                                                patternSize = { height = configuration.circle_pattern_geometry[1], width = configuration.circle_pattern_geometry[2] },
                                                flags=cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING }
    local ok_right = true -- if we do not have a right camera, ok_right should be true per default
    if right_camera ~= nil then
      ok_right, centers_right = cv.findCirclesGrid{ image = image_right,
                                                    patternSize = { height = configuration.circle_pattern_geometry[1], width = configuration.circle_pattern_geometry[2] },
                                                    flags=cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING }
    end
    ok = ok_left and ok_right
    print("ok:")
    print(ok)
    if not ok then
      print('WARNING! Calibration pattern not found. Please move the target into camera view!')
    end
  end


  local circlePositions = CalcPointPositions{ pointsX = configuration.circle_pattern_geometry[2],
                                              pointsY = configuration.circle_pattern_geometry[1],
                                              pointDistance = configuration.circle_pattern_geometry[3] }
  local poseFound, poseCamRotVector, poseCamTrans = cv.solvePnP{ objectPoints = circlePositions,
                                                                 imagePoints = centers_left,
                                                                 cameraMatrix = intrinsics,
                                                                 distCoeffs = distortion }
  if not poseFound then
    error('could not calculate pose from calibration pattern')
  end
  print("poseCamRotVector:")
  print(poseCamRotVector)
  print("poseCamTrans:")
  print(poseCamTrans)

  local poseCamRotMatrix = RotVectorToRotMatrix(poseCamRotVector)
  print("poseCamRotMatrix:")
  print(poseCamRotMatrix)

  -- assemble the 4x4 transformation matrix
  local transfer = torch.eye(4)
  transfer[{{1,3}, {1,3}}] = poseCamRotMatrix
  transfer[{{1,3}, {4}}] = poseCamTrans
  print("transfer:")
  print(transfer)
  local offset = torch.mv(transfer, torch.Tensor({0.04,0.05,0,0}))
  transfer[{{},4}]:add(offset)
  local t = overviewPoseTensor * avg_heye * transfer
  targetPoint = t[{{1,3},4}]
  print('identified target point:')
  print(targetPoint)
  prompt:anyKey()

  -- Once write all joint values to disk (e.g. to get the torso joint value of an SDA, etc.)
  local mg_names = move_group.motion_service:queryAvailableMoveGroups()
  local mg_all = move_group.motion_service:getMoveGroup(mg_names[1])
  local joint_values_all = mg_all:getCurrentJointValues()
  local all_vals_fn1 = path.join(output_directory, "all_vals.t7")
  local all_vals_fn2 = path.join(output_directory, "all_vals_tensors.t7")
  torch.save(all_vals_fn1, joint_values_all)
  torch.save(all_vals_fn2, joint_values_all.values)

  -- Preparation of saving joint values and poses to disk after each move
  local jsposes = {}
  jsposes.recorded_joint_values = {}
  jsposes.recorded_poses = {}
  local jsposes_tensors = {}
  jsposes_tensors.recorded_joint_values = {}
  jsposes_tensors.recorded_poses = {}
  local poses_fn1 = path.join(output_directory, "jsposes.t7")
  local poses_fn2 = path.join(output_directory, "jsposes_tensors.t7")

  -- Loop over all #count many poses, the user wants to sample
  local up = torch.DoubleTensor({0,0, 1})
  local cnt = 1
  local enough = false
  while not enough do

    -- generate random point in positive half shere
    local origin
    while true do
      origin = torch.randn(3) -- 1D Tensor of size 3 filled with random numbers
                              -- from a normal distribution with mean zero and variance one.
      origin[3] = math.max(0.01, math.abs(origin[3])) -- z-component has to be positive
      origin:div(origin:norm()) -- normalization
      if origin[3] > 0.8 then -- z-component has to be > 0.8
        break
      end
    end
    print("origin:")
    print(origin)

    -- Lets express the position we want to look at relative to our pattern.
    -- The targets z-axis goes into the table so we have a negative z-value w.r.t. the pattern.
    origin:mul(torch.lerp(min_radius, max_radius, math.random())) -- = minRadius + random in [0,1) * (maxRadius-minRadius)
    -- ==> Each point has to lie on a sphere with radius between minRadius and maxRadius!
    print("origin:")
    print(origin)

    origin:add(targetPoint)
    print("origin:")
    print(origin)

    local target = targetPoint + math.random() * target_jitter - 0.5 * target_jitter

    local up_ = up

    up_ = t[{1,{1,3}}] -- use pattern x axis in world

    if math.random(2) == 1 then
      up_ = -up_
    end
    print("up_:")
    print(up_)

    local movePoseTensor = PointAtPose(origin, target, up_, avg_heye)
    print("movePoseTensor:")
    print(movePoseTensor)

    local movePose = datatypes.Pose()
    movePose.stampedTransform:fromTensor(movePoseTensor)
    movePose:setFrame("world")
    print("movePose:")
    print(movePose)

    -- move to movePose and search calibration pattern
    print("current pose:")
    print(move_group:getCurrentPose())
    printf('Moveing to movePose #%d ...', cnt)
    -- with this seed being set to random values, we have an arm rebuilding
    local seed = ee.move_group:getCurrentJointValues()
    seed.values = torch.randn(7) -- 1D Tensor of size 7 filled with random numbers in ]-1,+1[
    check = movePoseCollisionFree(ee, movePose, seed)
    print("check:")
    print(check)

    if check then
      sys.sleep(0.5)

      prompt:anyKey()

            -- Capture images and save joint values and poses:
      if left_camera ~= nil then
        --camera_client:setExposure(left_camera.exposure, {left_camera.serial})
        camera_client:setExposure(120000, {left_camera.serial})
        local image = camera_client:getImages({left_camera.serial})
        if image:nDimension() > 2 then
          image = cv.cvtColor{image, nil, cv.COLOR_RGB2BGR}
        end
        -- write image to disk
        local fn = path.join(output_directory, string.format('cam_%s_%03d.png', left_camera.serial, cnt))
        printf("Writing image: %s", fn)
        local ok_write = cv.imwrite{fn, image}
        assert(ok_write, 'Could not write image.')
        file_names[#file_names+1] = fn
      end
      if right_camera ~= nil then
        --camera_client:setExposure(right_camera.exposure, {right_camera.serial})
        camera_client:setExposure(120000, {right_camera.serial})
        local image = camera_client:getImages({right_camera.serial})
        if image:nDimension() > 2 then
          image = cv.cvtColor{image, nil, cv.COLOR_RGB2BGR}
        end
        -- write image to disk
        local fn = path.join(output_directory, string.format('cam_%s_%03d.png', right_camera.serial, cnt))
        printf("Writing image: %s", fn)
        local ok_write = cv.imwrite{fn, image}
        assert(ok_write, 'Could not write image.')
        file_names[#file_names+1] = fn
      end
      collectgarbage()

      -- Save joint values for image capturing:
      local q = move_group:getCurrentJointValues()
      pos_list[cnt] = q
      printf("Joint configuration #%d:", cnt)
      print(q)

      local p = move_group:getCurrentPose()

      recorded_joint_values[#recorded_joint_values+1] = q
      recorded_joint_tensors[#recorded_joint_tensors+1] = q.values
      recorded_poses[#recorded_poses+1] = p:toTensor()

      -- Write table of joint values and poses to disk after each move,
      -- so that they are not lost if the robot has to be stopped by emergency.
      jsposes.recorded_joint_values[cnt] = recorded_joint_values[cnt]
      jsposes.recorded_poses[cnt] = recorded_poses[cnt]
      jsposes_tensors.recorded_joint_values[cnt] = recorded_joint_tensors[cnt]
      jsposes_tensors.recorded_poses[cnt] = recorded_poses[cnt]
      print("jsposes:")
      print(jsposes)
      print("jsposes_tensors:")
      print(jsposes_tensors)
      print(string.format('Saving poses files to: %s and %s.', poses_fn1, poses_fn2))
      torch.save(poses_fn1, jsposes)
      torch.save(poses_fn2, jsposes_tensors)

      cnt = cnt + 1
      if cnt > count then
        enough = true
      end
    end
  end

  configuration.capture_poses = pos_list
end


-- Input: hand-pattern matrix, (left) camera matrix, (left) camera distortion and trafoLeftToRightCam
local function captureSphereSampling_torsoCams()
  print('How many? Enter the number of capture poses:')
  local count = prompt:readNumber()

  local world_view_client = rosvita.WorldViewClient.new(motion_service.node_handle)

  print("Enter filename of initial guess for hand-pattern matrix (without quotation marks!)")
  local hand_pattern_fn = prompt:readLine()
  local hand_pattern = nil
  if hand_pattern_fn ~= "" then
    hand_pattern = torch.load(hand_pattern_fn)
  else
    hand_pattern = torch.DoubleTensor({
      {  0.0059,   0.9997,  -0.0256, -0.0271   },
      { -0.0008,   0.0256,   0.9997, -0.0894   },
      {  1.0000,  -0.0059,   0.0010,  0.0214   },
      {  0.0000,   0.0000,   0.0000,  1.0000   }
    })
  end
  print("Hand-pattern matrix:")
  print(hand_pattern)
  print("Note: If hand-pattern matrix is for tcp at last joint of robot arm (instead of tcp at gripper tip), then the tcp in the project configuration has to be set accordingly before starting the sphere sampling!")
  print("Enter filename of initial guess for stereo calibration")
  local stereoCalib_fn = prompt:readLine()
  local trafoLeftToRightCam = nil
  if stereoCalib_fn ~= "" then
    stereoCalib = torch.load(stereoCalib_fn)
    trafoLeftToRightCam = stereoCalib.trafoLeftToRightCam:double()
  else
    trafoLeftToRightCam = torch.DoubleTensor({
      {  0.9660,  -0.0412,  -0.2551,  0.1886   },
      {  0.0428,   0.9991,   0.0005,  0.0037   },
      {  0.2549,  -0.0114,   0.9669,  0.0268   },
      {  0.0000,   0.0000,   0.0000,  1.0000   }
    })
  end
  print("Left to right camera transformation:")
  print(trafoLeftToRightCam)
  print("Enter filename of initial guess for transformation from torso to left camera")
  local leftCam_torso_fn = prompt:readLine()
  local leftCam_torso = nil
  if leftCam_torso_fn ~= "" then
    leftCam_torso = torch.load(leftCam_torso_fn)
  else
    leftCam_torso = torch.DoubleTensor({
      {  0.1028,   0.7777,   0.6202,  0.2321   },
      {  0.9910,  -0.0265,  -0.1311,  0.0951   },
      { -0.0855,   0.6281,  -0.7734,  0.4490   },
      {  0.0000,   0.0000,   0.0000,  1.0000   }
    })
  end
  print("Torso to left camera transformation:")
  print(leftCam_torso)
  -- Publish "hand_pattern" in WorldView
  local hand_pattern_pose = datatypes.Pose()
  hand_pattern_pose.stampedTransform:fromTensor(hand_pattern)
  hand_pattern_pose:setFrame('arm_right_link_tool0')
  world_view_client:addPose("hand_pattern_guess", 'Calibration', hand_pattern_pose)

  local ok, centers = false, nil

  local pos_list = {}
  local file_names = {}
  local recorded_joint_values = {}   -- joint values after getImage calls
  local recorded_joint_tensors = {}  -- joint values as tensor after getImage calls
  local recorded_poses = {}          -- end effector poses after getImage calls
  local output_directory = path.join(configuration.output_directory, 'capture_shpere_sampling')
  print('Deleting output directory')
  os.execute('rm -rf '.. output_directory)
  print('Creating output directory')
  os.execute('mkdir -p ' .. output_directory)
  local left_camera = configuration.cameras[configuration.left_camera_id]
  local right_camera = configuration.cameras[configuration.right_camera_id]
  local ee_names = move_group:getEndEffectorNames()
  local ee = move_group:getEndEffector(ee_names[1])
  local calibPattern = { width = 8, height = 21, pointDistance = 0.005 }

  -- Get current pose of left and right torso camera in world coordinates, as well as the pose between them:
  local err_code, torso_joint_b1_base, err_msg = motion_service:queryPose(move_group:getName(), move_group:getCurrentJointValues(), "torso_link_b1")
  local torsoPoseMsg = torso_joint_b1_base.pose
  local torsoTransform = tf.Transform() -- create tf.Transform from torsoPoseMsg
  torsoTransform:setOrigin({ torsoPoseMsg.position.x, torsoPoseMsg.position.y, torsoPoseMsg.position.z })
  torsoTransform:setRotation(tf.Quaternion(torsoPoseMsg.orientation.x, torsoPoseMsg.orientation.y, torsoPoseMsg.orientation.z, torsoPoseMsg.orientation.w))
  local torso_base = torsoTransform:toTensor()
  local leftCam_base = torso_base * leftCam_torso
  print("leftCam_base:")
  print(leftCam_base)
  local rightCam_base = leftCam_base * torch.inverse(trafoLeftToRightCam)
  print("rightCam_base:")
  print(rightCam_base)
  -- Publish camera poses in WorldView
  local leftCam_base_pose = datatypes.Pose()
  local rightCam_base_pose = datatypes.Pose()
  leftCam_base_pose.stampedTransform:fromTensor(leftCam_base)
  rightCam_base_pose.stampedTransform:fromTensor(rightCam_base)
  leftCam_base_pose:setFrame('world')
  rightCam_base_pose:setFrame('world')
  world_view_client:addPose("leftCam_base", 'Calibration', leftCam_base_pose)
  world_view_client:addPose("rightCam_base", 'Calibration', rightCam_base_pose)
  --Interpolate translation
  local one = tf.Transform()
  local one_pos = leftCam_base_pose:toStampedPoseMsg().pose.position
  local one_rot = leftCam_base_pose:toStampedPoseMsg().pose.orientation
  one:setOrigin({ one_pos.x, one_pos.y, one_pos.z })
  one:setRotation(tf.Quaternion(one_rot.x, one_rot.y, one_rot.z, one_rot.w))
  local two = tf.Transform()
  local two_pos = rightCam_base_pose:toStampedPoseMsg().pose.position
  local two_rot = rightCam_base_pose:toStampedPoseMsg().pose.orientation
  two:setOrigin({ two_pos.x, two_pos.y, two_pos.z })
  two:setRotation(tf.Quaternion(two_rot.x, two_rot.y, two_rot.z, two_rot.w))
  local between = tf.Transform()
  between:setRotation(one:getRotation():slerp(two:getRotation(), 0.5))
  between:setOrigin({ one_pos.x+0.5*(two_pos.x-one_pos.x), one_pos.y+0.5*(two_pos.y-one_pos.y), one_pos.z+0.5*(two_pos.z-one_pos.z) })
  print("Pose between leftCam and rightCam:")
  print(between:toTensor())
  -- Publish pose between cameras in WorldView
  local between_pose = datatypes.Pose()
  between_pose.stampedTransform:fromTensor(between:toTensor())
  between_pose:setFrame('world')
  world_view_client:addPose("betweenCams_base", 'Calibration', between_pose)

  local overviewPose = move_group:getCurrentPose()
  local check = false

  while ros.ok() and not ok do

    print("Please move robot to overview pose and press 'Enter' when ready.")
    prompt:waitEnterOrEsc()
    overviewPoseTensor = move_group:getCurrentPose():toTensor()
    print("overviewPoseTensor:")
    print(overviewPoseTensor)

    -- capture images
    local image_left
    local image_right


    if left_camera ~= nil then
      if left_camera.sleep_before_capture > 0 then
        printf('wait before capture %f s... ', left_camera.sleep_before_capture)
        sys.sleep(left_camera.sleep_before_capture)
      end
      --camera_client:setExposure(left_camera.exposure, {left_camera.serial})
      camera_client:setExposure(80000, {left_camera.serial})
      image_left = camera_client:getImages({left_camera.serial})
      --print("ok1")
      if image_left:nDimension() > 2 then
        image_left = cv.cvtColor{image_left, nil, cv.COLOR_RGB2BGR}
      end
      --print("ok2")
      -- write image to disk
      local fn_startImg = path.join(output_directory, string.format('cam_%s_start.png', left_camera.serial))
      printf("Writing image: %s", fn_startImg)
      local ok_write_start = cv.imwrite{fn_startImg, image_left}
      assert(ok_write_start, 'Could not write startimage.')
      --print("ok3")
    end
    if right_camera ~= nil then
      if right_camera.sleep_before_capture > 0 then
        printf('wait before capture %f s... ', right_camera.sleep_before_capture)
        sys.sleep(right_camera.sleep_before_capture)
      end
      --camera_client:setExposure(right_camera.exposure, {right_camera.serial})
      camera_client:setExposure(80000, {right_camera.serial})
      image_right = camera_client:getImages({right_camera.serial})
      if image_right:nDimension() > 2 then
        image_right = cv.cvtColor{image_right, nil, cv.COLOR_RGB2BGR}
      end
      -- write image to disk
      fn_startImg = path.join(output_directory, string.format('cam_%s_start.png', right_camera.serial))
      printf("Writing image: %s", fn_startImg)
      ok_write_start = cv.imwrite{fn_startImg, image_right}
      assert(ok_write_start, 'Could not write startimage.')
    end


    -- load images for simulation
    --image_left = cv.imread { "calibration/cam_CAMAU1639042_start.png" }
    --image_right = cv.imread { "calibration/cam_CAMAU1710001_start.png" }

    ok_left, centers_left = cv.findCirclesGrid{ image = image_left,
                                                patternSize = { height = configuration.circle_pattern_geometry[1], width = configuration.circle_pattern_geometry[2] },
                                                flags=cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING }
    local ok_right = true -- if we do not have a right camera, ok_right should be true per default
    if right_camera ~= nil then
      ok_right, centers_right = cv.findCirclesGrid{ image = image_right,
                                                    patternSize = { height = configuration.circle_pattern_geometry[1], width = configuration.circle_pattern_geometry[2] },
                                                    flags=cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING }
    end
    ok = ok_left and ok_right
    print("ok:")
    print(ok)
    if not ok then
      print('WARNING! Calibration pattern not found. Please move the target into camera view!')
    end
  end

  local t = between:toTensor():clone()
  print("t = pose between left and right cam:")
  print(t)
  targetPoint = t[{{1,3},4}]
  print('identified target point:')
  print(targetPoint)
  prompt:anyKey()

  -- Once write all joint values to disk (e.g. to get the torso joint value of an SDA, etc.)
  local mg_names = move_group.motion_service:queryAvailableMoveGroups()
  local mg_all = move_group.motion_service:getMoveGroup(mg_names[1])
  local joint_values_all = mg_all:getCurrentJointValues()
  local all_vals_fn1 = path.join(output_directory, "all_vals.t7")
  local all_vals_fn2 = path.join(output_directory, "all_vals_tensors.t7")
  torch.save(all_vals_fn1, joint_values_all)
  torch.save(all_vals_fn2, joint_values_all.values)

  -- Preparation of saving joint values and poses to disk after each move
  local jsposes = {}
  jsposes.recorded_joint_values = {}
  jsposes.recorded_poses = {}
  local jsposes_tensors = {}
  jsposes_tensors.recorded_joint_values = {}
  jsposes_tensors.recorded_poses = {}
  local poses_fn1 = path.join(output_directory, "jsposes.t7")
  local poses_fn2 = path.join(output_directory, "jsposes_tensors.t7")

  -- Loop over all #count many poses, the user wants to sample
  local cnt = 1
  local enough = false
  local origin = torch.DoubleTensor(3)
  local start_origin
  local radius
  local radius_jitter = 0.05
  while not enough do

    -- generate random point in a part of the half sphere in front of the cameras

    if cnt == 1 then -- take start position of pattern as origin
      local world_pattern = move_group:getCurrentPose():toTensor() * hand_pattern
      print("world_pattern:")
      print(world_pattern)
      origin[1] = world_pattern[1][4] -- 0.12655776739120483
      origin[2] = world_pattern[2][4] -- 0.51348876953125
      origin[3] = world_pattern[3][4] -- 1.0787900686264038
      start_origin = origin
      print("start pattern-origin:")
      print(start_origin)
      print("Distance of current pattern pose (origin) from camera-interpolation-pose (target):")
      radius = torch.norm(targetPoint - origin)
      print(radius)
    else
      -- generate a random position as origin with correct distance from target
      -- and only slight angle difference from start
      while true do
        origin = start_origin:clone()
        origin[1] = origin[1] + math.random()*0.01
        origin[2] = origin[2] + math.random()*0.01
        origin[3] = origin[3] + math.random()*0.01
        if torch.norm(origin - start_origin) < 0.02 then
          print("new pattern-origin:")
          print(origin)
          break
        end
      end
    end

    local movePoseTensor = torch.eye(4)
    movePoseTensor[1][4] = origin[1]
    movePoseTensor[2][4] = origin[2]
    movePoseTensor[3][4] = origin[3]
    movePoseTensor[{{1,3},{1,3}}] = overviewPoseTensor[{{1,3},{1,3}}]
    -- movePoseTensor is still in pattern coordinates and does not have the correct rotation, yet.

    -- Lets express the position we want to look at relative to our camera.
    -- The z-axis of the pattern has to point away from the camera.
    current_tcp_pose = move_group:getCurrentPose():toTensor()
    pattern_pose_in_world = current_tcp_pose * hand_pattern
    pattern_xaxis = pattern_pose_in_world[{{1,3},1}]
    zaxis = normalize(targetPoint - origin)
    zaxis = -1.0 * zaxis
    yaxis = normalize(torch.cross(zaxis, pattern_xaxis))
    xaxis = normalize(torch.cross(yaxis, zaxis))
    local basis = torch.Tensor(3,3)
    basis[{{},{1}}] = xaxis
    basis[{{},{2}}] = yaxis
    basis[{{},{3}}] = zaxis
    local t = tf.Transform()
    t:setBasis(basis)
    t:setOrigin(origin)
    movePoseTensor = t:toTensor():clone()
    movePoseTensor = movePoseTensor * torch.inverse(hand_pattern)

    print("movePoseTensor (with correct rotation and in TCP-coordinates):")
    print(movePoseTensor)

    local movePose = datatypes.Pose()
    movePose.stampedTransform:fromTensor(movePoseTensor)
    movePose:setFrame("world")

    -- move to movePose and search calibration pattern
    printf('Moveing to movePose #%d ...', cnt)

    local seed = ee.move_group:getCurrentJointValues()
    if cnt % 2 == 0 then
      print("cnt for random seed:", cnt)
      seed.values = torch.randn(7) -- 1D Tensor of size 7 filled with random numbers in ]-1,+1[
    end
    check = movePoseCollisionFree(ee, movePose, seed)
    print("check:")
    print(check)

    if check then
      sys.sleep(0.5)

      prompt:anyKey()

      -- Save joint values for image capturing:
      local q = move_group:getCurrentJointValues()
      pos_list[cnt] = q
      printf("Joint configuration #%d:", cnt)
      print(q)

      local p = move_group:getCurrentPose()

      recorded_joint_values[#recorded_joint_values+1] = q
      recorded_joint_tensors[#recorded_joint_tensors+1] = q.values
      recorded_poses[#recorded_poses+1] = p:toTensor()

      -- Write table of joint values and poses to disk after each move,
      -- so that they are not lost if the robot has to be stopped by emergency.
      jsposes.recorded_joint_values[cnt] = recorded_joint_values[cnt]
      jsposes.recorded_poses[cnt] = recorded_poses[cnt]
      jsposes_tensors.recorded_joint_values[cnt] = recorded_joint_tensors[cnt]
      jsposes_tensors.recorded_poses[cnt] = recorded_poses[cnt]
      print("jsposes:")
      print(jsposes)
      print("jsposes_tensors:")
      print(jsposes_tensors)
      print(string.format('Saving poses files to: %s and %s.', poses_fn1, poses_fn2))
      torch.save(poses_fn1, jsposes)
      torch.save(poses_fn2, jsposes_tensors)


      -- Capture images and save joint values and poses:
      if left_camera ~= nil then
        --camera_client:setExposure(left_camera.exposure, {left_camera.serial})
        camera_client:setExposure(80000, {left_camera.serial})
        local image = camera_client:getImages({left_camera.serial})
        if image:nDimension() > 2 then
          image = cv.cvtColor{image, nil, cv.COLOR_RGB2BGR}
        end
        -- write image to disk
        local fn = path.join(output_directory, string.format('cam_%s_%03d.png', left_camera.serial, cnt))
        printf("Writing image: %s", fn)
        local ok_write = cv.imwrite{fn, image}
        assert(ok_write, 'Could not write image.')
        file_names[#file_names+1] = fn
      end
      if right_camera ~= nil then
        --camera_client:setExposure(right_camera.exposure, {right_camera.serial})
        camera_client:setExposure(80000, {right_camera.serial})
        local image = camera_client:getImages({right_camera.serial})
        if image:nDimension() > 2 then
          image = cv.cvtColor{image, nil, cv.COLOR_RGB2BGR}
        end
        -- write image to disk
        local fn = path.join(output_directory, string.format('cam_%s_%03d.png', right_camera.serial, cnt))
        printf("Writing image: %s", fn)
        local ok_write = cv.imwrite{fn, image}
        assert(ok_write, 'Could not write image.')
        file_names[#file_names+1] = fn
      end
      collectgarbage()


      cnt = cnt + 1
      if cnt > count then
        enough = true
      end
    end
  end

  configuration.capture_poses = pos_list
end


local function captureSphereSampling()
  local menu_options = {
    { 'e', 'end-of-arm camera setup', captureSphereSampling_endOfArmCams },
    { 't', 'torso camera setup', captureSphereSampling_torsoCams }
  }
  menu_options[#menu_options + 1] = { 'ESC', 'Return to main menu...', false }
  prompt:showMenu('Capture Sphere Sampling', menu_options)
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
      { 'a', string.format('Generate capture poses via sphere sampling (%d defined)', #configuration.capture_poses), captureSphereSampling },
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
