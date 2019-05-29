--[[
  AutoCalibration.lua

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
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local xutils = require 'xamlamoveit.xutils'
local xamlamoveit = require 'xamlamoveit'
local datatypes = xamlamoveit.datatypes

local cv = require 'cv'
require 'cv.imgproc'
require 'cv.imgcodecs'
require 'cv.highgui'
require 'cv.calib3d'

require 'ximea.ros.XimeaClient'

local grippers = require 'xamlamoveit.grippers.env'
local gripper_force = 20
local prompt = xutils.Prompt()

-- Initialize the pseudo random number generator
math.randomseed( os.time() )
math.random(); math.random(); math.random()


local function tryRequire(module_name)
  local ok, val = pcall(function() return require(module_name) end)
  if ok then
    return val
  else
    return nil
  end
end

local xml = tryRequire('xml')  -- for exporting the .t7 calibration file to .xml


local function readKeySpinning()
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


local autocal = require 'auto_calibration.env'
local CalibrationMode = autocal.CalibrationMode
local CalibrationFlags = autocal.CalibrationFlags
local AutoCalibration = torch.class('autoCalibration.AutoCalibration', autocal)


--creates a gripper client for the specified key
local function constructGripper(grippers, key, nh)
  local gripper_action_name = nil
    
  if string.find(key, 'robotiq') ~= nil then
    gripper_force = 50
    gripper_action_name = string.format('%s/gripper_command', key)
    return grippers['GenericRosGripperClient'].new(nh, gripper_action_name)
  elseif string.find(key, 'wsg') ~= nil then
    local gripper_namespace = key
    gripper_action_name = 'gripper_control'
    --gripper_action_name = string.format('%s/gripper_control', key)
    return grippers['WeissTwoFingerModel'].new(nh, gripper_namespace, gripper_action_name)
  end
end


local function initializeGripperServices(self)
  local node_handle = ros.NodeHandle()
  self.node_handle = node_handle
  local key = self.configuration.gripper_key -- get the key (gripper action name) from the configuration
  self.gripper = constructGripper(grippers, key, self.node_handle)
  if self.gripper ~= nil then
    print('Gripper:')
    print(self.gripper)
    print('AutoCalibration calling home gripper')
    self.gripper:home()
  end
end


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


function AutoCalibration:__init(configuration, move_group, camera_client, sl_studio)
  self.configuration = configuration
  self.config_class = ConfigurationCalibration.new(configuration)
  self.move_group = move_group
  self.camera_client = camera_client
  self.sl_studio = sl_studio
  if string.find(self.configuration.gripper_key, 'robotiq') then
    gripper_force = 50
  end

  local ok, err = pcall(function() initializeGripperServices(self) end)
  if not ok then
    error('Gripper initialization failed: ' .. err)
  end

  self.tcp_frame_of_reference, self.tcp_end_effector_name = self:getEndEffectorName()
  --create the calibrate/current folder if it does not exist
  os.execute('mkdir -p ' .. configuration.output_directory)
  --self.current_path = path.join(configuration.output_directory, 'current')
  --os.execute('mkdir -p ' .. self.current_path)

  -- create an output directory path so that jsposes.t7 and the calibration file are stored in the same directory
  self.calibration_folder_name = os.date(configuration.calibration_directory_template)
  self.output_directory = path.join(configuration.output_directory, self.calibration_folder_name)
  print('Output directory for calibration data: '.. self.output_directory)

  -- if we are simulating a capture, we will load an old jsposes file
  self.offline_jsposes_fn = path.join(configuration.output_directory, 'offline', 'jsposes.t7')
end


function AutoCalibration:testMoveGroups()
  local selected_move_group_name = self.configuration.move_group_name
  local move_group_names, move_group_details = self.move_group.motion_service:queryAvailableMoveGroups()
  print('AutoCalibration:testMoveGroups() move_group_names, move_group_details')
  print(move_group_names)
  print(move_group_details)
  local index = 1
  local tcp_frame_of_reference = move_group_details[selected_move_group_name].end_effector_link_names[index]
  local tcp_end_effector_name = move_group_details[selected_move_group_name].end_effector_names[index]
  print('config selected tcp_end_effector='..tcp_end_effector_name)
end


function AutoCalibration:getEndEffectorName()
  local move_group_names, move_group_details = self.move_group.motion_service:queryAvailableMoveGroups()
  local index = 1
  local tcp_frame_of_reference = move_group_details[move_group_names[1]].end_effector_link_names[index]
  local tcp_end_effector_name = move_group_details[move_group_names[1]].end_effector_names[index]
  return tcp_frame_of_reference,tcp_end_effector_name
end


function AutoCalibration:shutdown()
  if self.gripper ~= nil then
    self.gripper:shutdown()
  end
  self.node_handle:shutdown()
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
  self.pattern_localizer = pattern_localizer
end


local function moveJ(self, pos)
  assert(pos ~= nil, 'Target position is nil.')
  self.move_group:moveJoints(pos, self.configuration.velocity_scaling)
  --check that we arrived where we expected
  local curPose = self.move_group:getCurrentPose():toTensor()
  local curJoints = self.move_group:getCurrentJointValues()
end


local function moveJointsCollisionFree(end_effector, target_posture, velocity_scaling)
  -- plan trajectory
  local ok, err, joint_trajectory, ex_plan_parameters = pcall(function () return end_effector.move_group:planMoveJointsCollisionFree(target_posture, velocity_scaling) end)
  if ok then
    -- execute trajectory
    local execute_ok = pcall(function () return end_effector.motion_service:executeJointTrajectory(joint_trajectory, true) end)
    if execute_ok then
      return true
    else
      print('Could not execute the joint trajectory.')
      return false
    end
  else
    print('Could not find a path to the given joint values.')
    return false
  end
end


local function movePoseCollisionFree(end_effector, target_pose, seed, velocity_scaling)
  local plan_parameters = end_effector.move_group:buildPlanParameters(velocity_scaling)
  local seed = seed or end_effector.move_group:getCurrentJointValues()
  -- calculate joint values by inverse kinematic call
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
  -- plan and execute trajectory
  local success = moveJointsCollisionFree(end_effector, goal, velocity_scaling)
  return success
end


function AutoCalibration:moveToStart()
  local base_poses = self.configuration.base_poses
  assert(base_poses ~= nil, 'Base poses are not defined.')
  assert(base_poses['start'] ~= nil, 'Target position is nil.')
  local ee_names = self.move_group:getEndEffectorNames()
  local ee = self.move_group:getEndEffector(ee_names[1])
  moveJointsCollisionFree(ee, base_poses['start'], 0.05)
end


function AutoCalibration:moveToCaptureBase()
  local base_poses = self.configuration.base_poses
  assert(base_poses ~= nil, 'Base poses are not defined.')
  local ee_names = self.move_group:getEndEffectorNames()
  local ee = self.move_group:getEndEffector(ee_names[1])
  moveJointsCollisionFree(ee, base_poses['camera1_base'], self.configuration.velocity_scaling)
end


function AutoCalibration:pickCalibrationTarget()
  self.gripper:home()
  print('close the gripper')
  self.gripper:move{width=0.0, speed=0.2, force=gripper_force, stop_on_block=false} -- move closed
  local base_poses = self.configuration.base_poses
  assert(base_poses ~= nil, 'Base poses are not defined.')
  print('move to start')
  moveJ(self, base_poses['start'])
  print('open the gripper')
  local t = self.gripper:move{width=0.05, speed=0.2, force=gripper_force} -- move open
  print("Is the gripper open? Type 1 (Yes) or 2 (No) and press \'Enter\'.")
  print("1 Yes")
  print("2 No")
  local open_check = io.read("*n")
  if open_check ~= 1 then
    return false
  end  
  print('move to pre pick pose')
  moveJ(self, base_poses['pre_pick_marker'])
  print('move to pick pose')
  moveJ(self, base_poses['pick_marker'])
  print('grasp calibration target')
  -- If width of calibration target may vary (i.e is assumed to be unknown),
  -- set width to 0.0, otherwise width can be set to the width of our calibration pattern.
  -- The "wsg50" needs the width of the part to be grasped -> set width to 0.0115.
  t = self.gripper:grasp{width=0.0115, speed=0.2, force=gripper_force} -- grasp target
  if t:hasCompletedSuccessfully() ~= true then
    print("Grasp has not been successful.")
    print("Want to try again? Type 1 (Yes) or 2 (No) and press \'Enter\'.")
    local again = io.read("*n")
    if again == 1 then
      print('grasp calibration target')
      t = self.gripper:grasp{width=0.0115, speed=0.2, force=gripper_force}
      if t:hasCompletedSuccessfully() ~= true then
        print("Again, grasping the pattern has not been successful.")
        return false
      end
    else
      return false
    end
  end
  --t = self.gripper:move{width=0.0, speed=0.2, force=30, stop_on_block=false} -- move grasp
  --assert(t:hasCompleted() == true, 'Cannot grasp pattern.')
  print('move to post pick pose')
  moveJ(self, base_poses['post_pick_marker'], self.configuration.velocity_scaling * 0.25)
  print('move to start')
  moveJ(self, base_poses['start'])
end


function AutoCalibration:returnCalibrationTarget()
  local base_poses = self.configuration.base_poses
  assert(base_poses ~= nil, 'Base poses are not defined.')
  print('move to start')
  moveJ(self, base_poses['start'])
  print('move to post pick (= pre return) pose')
  moveJ(self, base_poses['post_pick_marker'])
  print('move to pick (= return) pose')
  sys.sleep(3)
  moveJ(self, base_poses['pick_marker'], self.configuration.velocity_scaling * 0.25)
  print('release calibration target')
  local t = self.gripper:release{width=0.05, speed=0.2, force=gripper_force} -- release target
  if t:hasCompletedSuccessfully() ~= true then
    print("Failed to release the pattern.")
    print("Want to try again? Type 1 (Yes) or 2 (No) and press \'Enter\'.")
    local again = io.read("*n")
    if again == 1 then
      print('release calibration target')
      t = self.gripper:release{width=0.05, speed=0.2, force=gripper_force}
      if t:hasCompletedSuccessfully() ~= true then
        print("Again, failed to release the pattern.")
        return false
      end
    else
      return false
    end
  end
  print('move to pre pick (= post return) pose')
  moveJ(self, base_poses['pre_pick_marker'])
end


function AutoCalibration:simulateCapture()

  -- assume the captured images are already at the capture folder

  --load jsposes.t7 file from the offline folder
  local offline_jsposes
  if path.exists(self.offline_jsposes_fn) then
    print('Reading offline poses file ' .. self.offline_jsposes_fn)
    offline_jsposes = torch.load(self.offline_jsposes_fn)
  else
    print(self.offline_jsposes_fn.. ': file does not exist')
    return false
  end

  -- we restore the values so they can be saved afterwards
  self.recorded_joint_values = {}
  self.recorded_poses = {}
  for i = 1, #offline_jsposes.recorded_joint_values do
    print('restoring pose #'..i)
    self.recorded_joint_values[i] = offline_jsposes.recorded_joint_values[i]
    self.recorded_poses[i] = offline_jsposes.recorded_poses[i]
  end
  self:savePoses()
end


function AutoCalibration:runCaptureSequenceWithoutCapture()
  local pos_list = self.configuration.capture_poses
  assert(pos_list ~= nil and #pos_list > 0)
  local recorded_joint_values = {}   -- joint values
  local recorded_poses = {}          -- end effector poses
  for i,p in ipairs(pos_list) do
    printf('Moving to position #%d...', i)
    local ee_names = self.move_group:getEndEffectorNames()
    local ee = self.move_group:getEndEffector(ee_names[1])
    moveJointsCollisionFree(ee, p, self.configuration.velocity_scaling)
    sys.sleep(1)
    -- get joint values and pose
    local joint_values = self.move_group:getCurrentJointValues()
    local pose = self.move_group:getCurrentPose()
    recorded_joint_values[#recorded_joint_values+1] = joint_values
    recorded_poses[#recorded_poses+1] = pose:toTensor()
  end
  self.recorded_joint_values = recorded_joint_values
  self.recorded_poses = recorded_poses
  self:savePoses()
end


function AutoCalibration:closeGripper()
  if self.gripper ~= nil then
    print('close the gripper')
    self.gripper:move{width=0.0, speed=0.2, force=gripper_force, stop_on_block=false} -- move closed
  else
    print('Need to initialize the gripper')
  end
end


function AutoCalibration:openGripper()
  if self.gripper ~= nil then
    print('open the gripper')
    self.gripper:move{width=0.06, speed=0.2, force=gripper_force} -- move open
  else
    print('Need to initialize the gripper')
  end
end


function AutoCalibration:homeGripper()
  if self.gripper ~= nil then
    print('Homeing gripper')
    self.gripper:home()
  else
    print('Need to initialize the gripper')
  end
end


local function captureImage(self, i, camera_configuration, output_directory)

  local camera_serial = camera_configuration.serial
  local exposure = camera_configuration.exposure
  local sleep_before_capture = camera_configuration.sleep_before_capture

  -- wait configured time before capture
  if sleep_before_capture > 0 then
    printf('wait before capture %f s... ', sleep_before_capture)
    sys.sleep(sleep_before_capture)
  end

  -- capture image
  self.camera_client:setExposure(exposure, {camera_serial})
  local image = self.camera_client:getImages({camera_serial})

  -- get joint values and pose of image
  local joint_values = self.move_group:getCurrentJointValues()
  local pose = self.move_group:getCurrentPose()

  -- create output filename
  --output_directory = self.config_class:getCameraDataOutputPath(camera_serial)
  local fn = path.join(output_directory, string.format('cam_%s_%03d.png', camera_serial, i))
  if image:nDimension() > 2 then
    image = cv.cvtColor{image, nil, cv.COLOR_RGB2BGR}
  end

  -- write image to disk
  printf("Writing image: %s", fn)
  local ok = cv.imwrite{fn, image}
  assert(ok, 'Could not write image.')

  return fn, joint_values, pose
end


local function tryLoadCurrentCameraCalibration(self, camera_serial)
  --local current_output_directory = path.join(self.configuration.output_directory, 'current')
  local calibration_fn = string.format('cam_%s.t7', camera_serial)
  --local calibration_file_path = path.join(current_output_directory, calibration_fn)
  local calibration_file_path = path.join(self.output_directory, calibration_fn)

  printf("Probing for calibration file '%s'.", calibration_file_path)
  if path.exists(calibration_file_path) then
    return torch.load(calibration_file_path)
  else
    return nil
  end
end


local function tryLoadCurrentCameraCalibrationFromStereo(self, camera_left_serial, camera_right_serial)
  --local current_output_directory = path.join(self.configuration.output_directory, 'current')
  local calibration_fn = string.format('stereo_cams_%s_%s.t7', camera_left_serial, camera_right_serial)
  --local calibration_file_path = path.join(current_output_directory, calibration_fn)
  local calibration_file_path = path.join(self.output_directory, calibration_fn)

  printf("Probing for calibration file '%s'.", calibration_file_path)
  if path.exists(calibration_file_path) then
    return torch.load(calibration_file_path)
  else
    return nil
  end
end


function getHomogeneousFromRosStampedPose(msg)
  --convert to tensor
  transf = tf.Transform.new()
  q = msg.pose.orientation
  t = msg.pose.position
  transf:setRotation(tf.Quaternion.new(q.x,q.y,q.z,q.w))
  transf:setOrigin({t.x,t.y,t.z})
  return transf:toTensor()
end


function AutoCalibration:runCaptureSequence()
  local configuration = self.configuration
  local file_names = {}
  local generic_file_names = {}
  local recorded_joint_values = {}   -- joint values after getImage calls
  local recorded_poses = {}          -- end effector poses after getImage calls
  
  local output_directory = path.join(configuration.output_directory, 'capture')

  print('Deleting output directory')
  os.execute('rm -rf '.. output_directory)

  print('Creating output directory')
  os.execute('mkdir -p ' .. output_directory)

  local pos_list = configuration.capture_poses
  assert(pos_list ~= nil and #pos_list > 0)
  local left_camera = configuration.cameras[configuration.left_camera_id]
  local right_camera = configuration.cameras[configuration.right_camera_id]

  for i,p in ipairs(pos_list) do

    printf('Moving to position #%d...', i)
    -- move to capture pose
    local ee_names = self.move_group:getEndEffectorNames()
    local ee = self.move_group:getEndEffector(ee_names[1])
    moveJointsCollisionFree(ee, p, self.configuration.velocity_scaling)

    if left_camera ~= nil then
      local fn, joint_values, pose = captureImage(self, i, left_camera, output_directory)
      file_names[#file_names+1] = fn
      recorded_joint_values[#recorded_joint_values+1] = joint_values
      recorded_poses[#recorded_poses+1] = pose:toTensor()
    end

    if right_camera ~= nil then
      local fn, joint_values, pose = captureImage(self, i, right_camera, output_directory)
      file_names[#file_names+1] = fn
    end

    collectgarbage()
  end

  self.file_names = file_names
  self.recorded_joint_values = recorded_joint_values
  self.recorded_poses = recorded_poses

  self:savePoses()

  return file_names, recorded_joint_values, recorded_poses
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


-- Calculates the TCP pose required to point the TCP z axis to point at with TCP at position eye
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


local function RotVectorToRotMatrix(vec)
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


-- Sphere sampling with onboard camera setup
function AutoCalibration:captureSphereSampling_endOfArmCams()
  attr = xutils.saveTerminalAttributes()
  prompt:enableRawTerminal()
  print('How many? Enter the number of capture poses:')
  local count = prompt:readNumber()
  local min_radius = 0.35
  local max_radius = 0.6
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
  local output_directory = path.join(self.configuration.output_directory, 'capture_sphere_sampling')
  print('Deleting output directory')
  os.execute('rm -rf '.. output_directory)
  print('Creating output directory')
  os.execute('mkdir -p ' .. output_directory)
  local left_camera = self.configuration.cameras[self.configuration.left_camera_id]
  local right_camera = self.configuration.cameras[self.configuration.right_camera_id]
  local ee_names = self.move_group:getEndEffectorNames()
  local ee = self.move_group:getEndEffector(ee_names[1])

  local overviewPose = self.move_group:getCurrentPose()
  local check = false

  while ros.ok() and not ok do

    print("Please move robot to overview pose and press 'Enter' when ready.")
    print("Cameras have to look straight down to the table and capture the whole pattern.")
    prompt:waitEnterOrEsc()
    overviewPoseTensor = self.move_group:getCurrentPose():toTensor()
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
      camera_client:setExposure(left_camera.exposure, {left_camera.serial})
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
      camera_client:setExposure(right_camera.exposure, {right_camera.serial})
      image_right = camera_client:getImages({right_camera.serial})
      if image_right:nDimension() > 2 then
        image_right = cv.cvtColor{image_right, nil, cv.COLOR_RGB2BGR}
      end
    end
    -- write images to disk
    fn_startImg_left = string.format('cam_%s_start.png', left_camera.serial)
    printf("Writing image: %s", fn_startImg_left)
    ok_write_start_left = cv.imwrite{ fn_startImg_left, image_left }
    assert(ok_write_start_left, 'Could not write left start image.')
    fn_startImg_right = string.format('cam_%s_start.png', right_camera.serial)
    printf("Writing image: %s", fn_startImg_right)
    ok_write_start_right = cv.imwrite{ fn_startImg_right, image_right }
    assert(ok_write_start_right, 'Could not write right start image.')

    -- load images for simulation
    --image_left = cv.imread { string.format('cam_%s_start.png', left_camera.serial) }
    --image_right = cv.imread { string.format('cam_%s_start.png', right_camera.serial) }

    ok_left, centers_left = cv.findCirclesGrid{ image = image_left,
                                                patternSize = { height = self.configuration.circle_pattern_geometry[1], width = self.configuration.circle_pattern_geometry[2] },
                                                flags=cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING }
    local ok_right = true -- if we do not have a right camera, ok_right should be true per default
    if right_camera ~= nil then
      ok_right, centers_right = cv.findCirclesGrid{ image = image_right,
                                                    patternSize = { height = self.configuration.circle_pattern_geometry[1], width = self.configuration.circle_pattern_geometry[2] },
                                                    flags=cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING }
    end
    ok = ok_left and ok_right
    print("ok:")
    print(ok)
    if not ok then
      print('WARNING! Calibration pattern not found. Please move the target into camera view!')
    end
  end


  local circlePositions = CalcPointPositions{ pointsX = self.configuration.circle_pattern_geometry[2],
                                              pointsY = self.configuration.circle_pattern_geometry[1],
                                              pointDistance = self.configuration.circle_pattern_geometry[3] }
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
  local mg_names = self.move_group.motion_service:queryAvailableMoveGroups()
  local mg_all = self.move_group.motion_service:getMoveGroup(mg_names[1])
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

    -- with 50% random choice rotate 180° around z-axis
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
    print(self.move_group:getCurrentPose())
    printf('Moveing to movePose #%d ...', cnt)
    -- with this seed being set to random values, we have an arm rebuilding
    local seed = ee.move_group:getCurrentJointValues()
    seed.values = torch.randn(seed.values:size(1)) -- 1D Tensor of same size as seed filled with random numbers in ]-1,+1[
    check = movePoseCollisionFree(ee, movePose, seed, self.configuration.velocity_scaling)
    print("check:")
    print(check)

    if check then
      sys.sleep(0.5)

      --prompt:anyKey()

      -- Capture images and save joint values and poses:
      if left_camera ~= nil then
        camera_client:setExposure(left_camera.exposure, {left_camera.serial})
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
        camera_client:setExposure(right_camera.exposure, {right_camera.serial})
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
      local q = self.move_group:getCurrentJointValues()
      pos_list[cnt] = q
      printf("Joint configuration #%d:", cnt)
      print(q)

      local p = self.move_group:getCurrentPose()

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

  xutils.restoreTerminalAttributes(attr)
  print("Finished sphere sampling.")
end


-- Sphere sampling with on torso camera setup
function AutoCalibration:captureSphereSampling_torsoCams()
  attr = xutils.saveTerminalAttributes()
  prompt:enableRawTerminal()
  print('How many? Enter the number of capture poses:')
  local count = prompt:readNumber()

  local world_view_client = xamlamoveit.rosvita.WorldViewClient.new(self.move_group.motion_service.node_handle)

  print("Enter filename of initial guess for hand-pattern matrix (without quotation marks!)")
  local hand_pattern_fn = prompt:readLine()
  local hand_pattern = nil
  if hand_pattern_fn ~= "" then
    hand_pattern = torch.load(hand_pattern_fn)
  else
    print("Hand-Pattern-Matrix not found!")
    return nil
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
    print("Stereocalibration not found!")
    return nil
  end
  print("Left to right camera transformation:")
  print(trafoLeftToRightCam)
  print("Enter filename of initial guess for transformation from torso to left camera")
  local leftCam_torso_fn = prompt:readLine()
  local leftCam_torso = nil
  if leftCam_torso_fn ~= "" then
    leftCam_torso = torch.load(leftCam_torso_fn)
  else
    print("left camera - torso transformation matrix not found!")
    return nil
  end
  print("Torso to left camera transformation:")
  print(leftCam_torso)
  -- Publish "hand_pattern" in WorldView
  local hand_pattern_pose = datatypes.Pose()
  hand_pattern_pose.stampedTransform:fromTensor(hand_pattern)
  print("Which arm holds the pattern?")
  print("1: left arm")
  print("2: right arm")
  local which_arm = prompt:readNumber()
  local frame = 'arm_left_link_tool0'
  if which_arm == 2 then
    frame = 'arm_right_link_tool0'
  end
  hand_pattern_pose:setFrame(frame)
  world_view_client:addPose("hand_pattern_guess", 'Calibration', hand_pattern_pose)

  local ok, centers = false, nil

  local pos_list = {}
  local file_names = {}
  local recorded_joint_values = {}   -- joint values after getImage calls
  local recorded_joint_tensors = {}  -- joint values as tensor after getImage calls
  local recorded_poses = {}          -- end effector poses after getImage calls
  local output_directory = path.join(self.configuration.output_directory, 'capture_sphere_sampling')
  print('Deleting output directory')
  os.execute('rm -rf '.. output_directory)
  print('Creating output directory')
  os.execute('mkdir -p ' .. output_directory)
  local left_camera = self.configuration.cameras[self.configuration.left_camera_id]
  local right_camera = self.configuration.cameras[self.configuration.right_camera_id]
  local ee_names = self.move_group:getEndEffectorNames()
  local ee = self.move_group:getEndEffector(ee_names[1])

  -- Get current pose of left and right torso camera in world coordinates, as well as the pose between them:
  local err_code, torso_joint_b1_base, err_msg = self.move_group.motion_service:queryPose(self.move_group:getName(), self.move_group:getCurrentJointValues(), "torso_link_b1")
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

  local overviewPose = self.move_group:getCurrentPose()
  local check = false

  while ros.ok() and not ok do

    print("Please move robot to overview pose and press 'Enter' when ready.")
    prompt:waitEnterOrEsc()
    overviewPoseTensor = self.move_group:getCurrentPose():toTensor()
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
      camera_client:setExposure(left_camera.exposure, {left_camera.serial})
      image_left = camera_client:getImages({left_camera.serial})
      if image_left:nDimension() > 2 then
        image_left = cv.cvtColor{image_left, nil, cv.COLOR_RGB2BGR}
      end
      -- write image to disk
      local fn_startImg = string.format('cam_%s_start.png', left_camera.serial)
      printf("Writing image: %s", fn_startImg)
      local ok_write_start = cv.imwrite{fn_startImg, image_left}
      assert(ok_write_start, 'Could not write startimage.')
    end
    if right_camera ~= nil then
      if right_camera.sleep_before_capture > 0 then
        printf('wait before capture %f s... ', right_camera.sleep_before_capture)
        sys.sleep(right_camera.sleep_before_capture)
      end
      camera_client:setExposure(right_camera.exposure, {right_camera.serial})
      image_right = camera_client:getImages({right_camera.serial})
      if image_right:nDimension() > 2 then
        image_right = cv.cvtColor{image_right, nil, cv.COLOR_RGB2BGR}
      end
      -- write image to disk
      fn_startImg = string.format('cam_%s_start.png', right_camera.serial)
      printf("Writing image: %s", fn_startImg)
      ok_write_start = cv.imwrite{fn_startImg, image_right}
      assert(ok_write_start, 'Could not write startimage.')
    end

    -- load images for simulation
    --image_left = cv.imread { string.format('cam_%s_start.png', left_camera.serial) }
    --image_right = cv.imread { string.format('cam_%s_start.png', right_camera.serial) }

    ok_left, centers_left = cv.findCirclesGrid{ image = image_left,
                                                patternSize = { height = self.configuration.circle_pattern_geometry[1], width = self.configuration.circle_pattern_geometry[2] },
                                                flags=cv.CALIB_CB_ASYMMETRIC_GRID + cv.CALIB_CB_CLUSTERING }
    local ok_right = true -- if we do not have a right camera, ok_right should be true per default
    if right_camera ~= nil then
      ok_right, centers_right = cv.findCirclesGrid{ image = image_right,
                                                    patternSize = { height = self.configuration.circle_pattern_geometry[1], width = self.configuration.circle_pattern_geometry[2] },
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
  local mg_names = self.move_group.motion_service:queryAvailableMoveGroups()
  local mg_all = self.move_group.motion_service:getMoveGroup(mg_names[1])
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
      local world_pattern = self.move_group:getCurrentPose():toTensor() * hand_pattern
      print("world_pattern:")
      print(world_pattern)
      origin[1] = world_pattern[1][4]
      origin[2] = world_pattern[2][4]
      origin[3] = world_pattern[3][4]
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
    current_tcp_pose = self.move_group:getCurrentPose():toTensor()
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
      seed.values = torch.randn(seed.values:size(1)) -- 1D Tensor of same size as seed filled with random numbers in ]-1,+1[
    end
    check = movePoseCollisionFree(ee, movePose, seed, self.configuration.velocity_scaling)
    print("check:")
    print(check)

    if check then
      sys.sleep(0.5)

      --prompt:anyKey()

      -- Save joint values for image capturing:
      local q = self.move_group:getCurrentJointValues()
      pos_list[cnt] = q
      printf("Joint configuration #%d:", cnt)
      print(q)

      local p = self.move_group:getCurrentPose()

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
        camera_client:setExposure(left_camera.exposure, {left_camera.serial})
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
        camera_client:setExposure(right_camera.exposure, {right_camera.serial})
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

  xutils.restoreTerminalAttributes(attr)
  print("Finished sphere sampling.")
end


local function findImages(directory, pattern)
  pattern = pattern or "%.png$"
  local l = {}
  if path.exists(directory) then
    for file_name in path.dir(directory) do
      local full_path = path.join(directory, file_name)
      if path.isfile(full_path) and file_name:match(pattern) then
        l[#l+1] = full_path
      end
    end
  end
  return l
end


-- Generate ground truth circle center points of the calibration pattern.
-- Z is set to 0 for all points.
local function generatePatternPoints(rows, cols, pointSize)
  -- calculates the groundtruth x, y, z positions of the points of the asymmetric circle pattern
  local corners = torch.FloatTensor(rows * cols, 1, 3):zero()
  local i=1
  for y=1,rows do
    for x=1,cols do
      corners[i][1][1] = (2*(x-1) + (y-1)%2) * pointSize
      corners[i][1][2] = (y-1)*pointSize
      corners[i][1][3] = 0
      i = i+1
    end
  end
  return corners
end


local function findMarker(markerList, markerId)
  for i,m in ipairs(markerList) do
    if m.id == markerId then
      return m
    end
  end
  return nil
end


local function findPattern(img, rows, cols)
  return cv.findCirclesGrid{
    image = img,
    patternSize = { height=rows, width=cols },
    flags = cv.CALIB_CB_ASYMMETRIC_GRID
  }
end


local function extractPoints(image_paths, pattern_localizer, pattern_id)
  local found = {}
  local not_found_indices = {}
  local h,w
  for i,fn in ipairs(image_paths) do

    local img = cv.imread{fn}

    if img:size(3) > 1 then
      -- extract green channel (e.g. of color cams with RGB Bayer Matrix)
      local greenImg = img[{{},{},2}]:clone()
      img = greenImg
    end

    h,w = img:size(1),img:size(2)

    local foundMarkers = pattern_localizer:processImg(img)
    local m = findMarker(foundMarkers, pattern_id)
    if m then
      found[#found+1] = m.points
      printf('[OK] %s (%dx%d)', fn, w, h)
    else
      print('[Warning] Trying fallback with standard findCirclesGrid() function...')
      local cols,rows = pattern_localizer.pattern.width, pattern_localizer.pattern.height
      local ok, points = findPattern(img, rows, cols)
      if ok then
        found[#found+1] = points
        printf('[OK] %s (%dx%d)', fn, w, h)
      else
        printf("[Warning] Pattern not found in image file '%s'", fn)
        printf("Index of not usable image: %d", i)
        not_found_indices[#not_found_indices+1] = i
      end
    end
  end
  return found, w, h, not_found_indices
end


function AutoCalibration:stereoCalibration(calibrationFlags)

  local image_paths = self.file_names
  local pattern_id = self.configuration.circle_pattern_id
  local left_camera = self.configuration.cameras[self.configuration.left_camera_id]
  local right_camera = self.configuration.cameras[self.configuration.right_camera_id]
  local camera_serials = {left_camera.serial, right_camera.serial}
  calibrationFlags = calibrationFlags or CalibrationFlags.Default

  if image_paths == nil or #image_paths == 0 then
    local current_directory_path = path.join(self.configuration.output_directory, 'capture')
    printf("Looking for images in '%s'.", current_directory_path)
    image_paths = findImages(current_directory_path)
    if #image_paths == 0 then
      print('No images found.')
      return false
    end
  end

  createPatternLocalizer(self)

  local flags = 0
  for k,v in pairs(calibrationFlags) do
    if v == true then
      flags = bit.bor(flags, cv[k])
    end
  end
  printf('Using flags (%d):', flags)
  print(calibrationFlags)

  -- extract point centers
  local objectPoints = {}
  local objectPointsLeft = {}
  local objectPointsRight = {}
  local imagePointsLeft
  local imagePointsRight
  local not_found_left
  local not_found_right
  local width, height
  table.sort(image_paths)
  for _, serial in ipairs(camera_serials) do
    local image_path_single_cam = {}
    for _, image_path in pairs(image_paths) do
      if string.match(image_path, serial) then
        table.insert(image_path_single_cam, image_path)
      end
    end
    printf("Searching calibration target pattern on %d input images for serial %s.", #image_path_single_cam, serial)
    local imagePoints, w, h, not_found = extractPoints(image_path_single_cam, self.pattern_localizer, pattern_id)
    width = w
    height = h
    printf("Found pattern on %d images.", #imagePoints)
    if next(not_found) ~= nil then
      print("Indices of unused images:")
      print(not_found)
    end

    print(imagePoints)
    local pointImg = torch.ByteTensor(h, w, 1):zero()
    for i=1, #imagePoints do
      for j=1, imagePoints[i]:size(1) do
        pointImg[math.floor(imagePoints[i][j][1][2]+0.5)][math.floor(imagePoints[i][j][1][1]+0.5)]=255
      end
    end

    -- generate object points and mono calibrate both cameras
    if serial == left_camera.serial then
      local groundTruthPointsLeft = generatePatternPoints(self.pattern_localizer.pattern.height, self.pattern_localizer.pattern.width, self.pattern_localizer.pattern.pointDist)
      for i=1,#imagePoints do
        objectPointsLeft[#objectPointsLeft+1] = groundTruthPointsLeft
      end
      objectPoints = objectPointsLeft
      imagePointsLeft = imagePoints

      -- run calibration of left camera
      print('Running openCV camera calibration for left camera (serial: %s)...', serial)
      local reprojError, camLeftMatrix, camLeftDistCoeffs, rvecs, tvecs =
        cv.calibrateCamera{
          objectPoints=objectPointsLeft,
          imagePoints=imagePointsLeft,
          imageSize={width, height},
          flag=flags
        }
      print('Left camera calibration results:')
      printf('Reprojection error: %f', reprojError)
      print('Left camera matrix:')
      print(camLeftMatrix)
      print('Left camera distortion coefficients:')
      print(camLeftDistCoeffs)
      self.leftCameraCalibration = {
        date = os.date('%Y-%m-%d %H:%M:%S'),
        patternGeometry = self.pattern_localizer.pattern,
        reprojError = reprojError,
        camMatrix = camLeftMatrix,
        distCoeffs = camLeftDistCoeffs,
        imWidth = width,
        imHeight = height,
        calibrationFlags = calibrationFlags
      }
      not_found_left = not_found
    else
      local groundTruthPointsRight = generatePatternPoints(self.pattern_localizer.pattern.height, self.pattern_localizer.pattern.width, self.pattern_localizer.pattern.pointDist)
      for i=1,#imagePoints do
        objectPointsRight[#objectPointsRight+1] = groundTruthPointsRight
      end
      imagePointsRight = imagePoints

      -- run calibration of right camera
      print('Running openCV camera calibration for right camera (serial: %s)...', serial)
      local reprojError, camRightMatrix, camRightDistCoeffs, rvecs, tvecs =
        cv.calibrateCamera{
          objectPoints=objectPointsRight,
          imagePoints=imagePointsRight,
          imageSize={width, height},
          flag=flags
        }
      print('Right camera calibration results:')
      printf('Reprojection error: %f', reprojError)
      print('Right camera matrix:')
      print(camRightMatrix)
      print('Right camera distortion coefficients:')
      print(camRightDistCoeffs)
      self.rightCameraCalibration = {
        date = os.date('%Y-%m-%d %H:%M:%S'),
        patternGeometry = self.pattern_localizer.pattern,
        reprojError = reprojError,
        camMatrix = camRightMatrix,
        distCoeffs = camRightDistCoeffs,
        imWidth = width,
        imHeight = height,
        calibrationFlags = calibrationFlags
      }
      not_found_right = not_found
    end
  end

  -- run stereo calibration
  print("Running openCV stereo camera calibration...")
  print('calibrationFlags = ')
  print(calibrationFlags)

  if next(not_found_left) == nil and next(not_found_right) == nil then
    print("Ok, pattern has been found in same (all) images for left and right camera.")
  elseif next(not_found_left) ~= nil and next(not_found_right) == nil then
    for i = #not_found_left, 1, -1 do
      table.remove(imagePointsRight, not_found_left[i])
      objectPoints = objectPointsLeft
    end
  elseif next(not_found_right) ~= nil and next(not_found_left) == nil then
    for i = #not_found_right, 1, -1 do
      table.remove(imagePointsLeft, not_found_right[i])
      objectPoints = objectPointsRight
    end
  elseif next(not_found_left) ~= nil and next(not_found_right) ~= nil then
    local equal = true
    if #not_found_left == #not_found_right then     
      for i = 1,#not_found_left do
        if not_found_left[i] ~= not_found_right[i] then
          equal = false
        end
      end
    else
      equal = false
    end
    if equal == true then
      print("Ok, pattern has been found in same images for left and right camera.")
    else
      -- extract point centers
      objectPoints = {}
      imagePointsLeft = {}
      imagePointsRight = {}
      local image_path_left_cam = {}
      local image_path_right_cam = {}
      for _, image_path in pairs(image_paths) do
        if string.match(image_path, left_camera.serial) then
          table.insert(image_path_left_cam, image_path)
        elseif string.match(image_path, right_camera.serial) then
          table.insert(image_path_right_cam, image_path)
        end
      end
      for i = #not_found_right, 1, -1 do
        table.remove(image_path_left_cam, not_found_right[i])
      end
      for i = #not_found_left, 1, -1 do
        table.remove(image_path_right_cam, not_found_left[i])
      end
      print("image_path_left_cam without not_found_right:")
      print(image_path_left_cam)
      print("image_path_right_cam without not_found_left:")
      print(image_path_right_cam)
      imagePointsLeft = extractPoints(image_path_left_cam, self.pattern_localizer, pattern_id)
      imagePointsRight = extractPoints(image_path_right_cam, self.pattern_localizer, pattern_id)
      -- generate object points
      local gt_Points = generatePatternPoints(self.pattern_localizer.pattern.height, self.pattern_localizer.pattern.width, self.pattern_localizer.pattern.pointDist)
      for i=1,#imagePointsLeft do
        objectPoints[#objectPoints+1] = gt_Points
      end
    end
  end

  print("imagePointsLeft:")
  print(imagePointsLeft)
  print("imagePointsRight:")
  print(imagePointsRight)

  -- R and T are the pose of the left camera in the right camera coordinate system
  local reprojError, camLeftMatrix, camLeftDistCoeffs, camRightMatrix, camRightDistCoeffs, R, T, E, F =
    cv.stereoCalibrate {
      objectPoints=objectPoints,
      imagePoints1=imagePointsLeft,
      imagePoints2=imagePointsRight,
      cameraMatrix1 = self.leftCameraCalibration.camMatrix,
      distCoeffs1 = self.leftCameraCalibration.distCoeffs,
      cameraMatrix2 = self.rightCameraCalibration.camMatrix,
      distCoeffs2 = self.rightCameraCalibration.distCoeffs,
      imageSize = {width, height},
      calibrationFlags = CalibrationFlags.DefaultStereo -- force to set CV_CALIB_FIX_INTRINSIC
    }
  print('R = ')
  print(R)
  print('T = ')
  print(T)

  local trafoMatrix = torch.FloatTensor():eye(4)
  trafoMatrix[{{1, 3}, {1, 3}}] = R[{{1, 3}}]
  trafoMatrix[{{1, 3}, {4}}] = T[{{1, 3}}]

  print('Stereo calibration results:')
  printf('Reprojection error: %f', reprojError)
  print('Left camera matrix:')
  print(camLeftMatrix)
  print('Right camera matrix:')
  print(camRightMatrix)
  print('Left distortion coefficients:')
  print(camLeftDistCoeffs)
  print('Right distortion coefficients:')
  print(camRightDistCoeffs)
  print('Transformation of left camera to right camera:')
  print(trafoMatrix)

  self.stereo_calibration = {
    date = os.date('%Y-%m-%d %H:%M:%S'),
    patternGeometry = self.pattern_localizer.pattern,
    reprojError = reprojError,
    camLeftMatrix = camLeftMatrix,
    camLeftDistCoeffs = camLeftDistCoeffs,
    camRightMatrix = camRightMatrix,
    camRightDistCoeffs = camRightDistCoeffs,
    trafoLeftToRightCam = trafoMatrix,
    R = R,
    T = T,
    E = E,
    F = F,
    imWidth = width,
    imHeight = height,
    calibrationFlags = calibrationFlags
  }

  return true
end


function AutoCalibration:monoCalibration(calibrationFlags, folder, serial)
  local image_paths = self.file_names
  local pattern_id = self.configuration.circle_pattern_id
  calibrationFlags = calibrationFlags or CalibrationFlags.Default
  -- store the selected serial so that we know later on when we save the file
  self.mono_selected_cam_serial = serial

  print('AutoCalibration:monoCalibration')
  --new functionality to distinguish which camera we want to calibrate
  if folder ~= nil then
    printf("Looking for images in '%s'.", folder)
    local pattern = "cam_"..serial.."_%d+%.png$"
    image_paths = alphanumsort(findImages(folder, pattern))
    if #image_paths == 0 then
      print('No images found.')
      return false
    end

  elseif image_paths == nil or #image_paths == 0 then
    local current_directory_path = path.join(self.configuration.output_directory, 'capture')
    local pattern = "cam_"..serial.."_%d+%.png$"
    printf("Looking for images in '%s' with serial '%s'", current_directory_path, pattern)
    image_paths = alphanumsort(findImages(current_directory_path, pattern))
    if #image_paths == 0 then
      print('No images found.')
      return false
    end
  end

  createPatternLocalizer(self)

  -- extract point centers
  printf('Searching calibration target pattern on %d input images.', #image_paths)
  local imagePoints, w, h = extractPoints(image_paths, self.pattern_localizer, pattern_id)
  printf('Found pattern on %d images.', #imagePoints)

  if #imagePoints < 3 then
    print('Cannot calibratio with less than three valid input images.')
    return false
  end

  print(imagePoints)
  local pointImg = torch.ByteTensor(h, w, 1):zero()
  for i=1, #imagePoints do
    for j=1, imagePoints[i]:size(1) do
      pointImg[math.floor(imagePoints[i][j][1][2]+0.5)][math.floor(imagePoints[i][j][1][1]+0.5)]=255
    end
  end

  -- generate object points
  local groundTruthPoints = generatePatternPoints(self.pattern_localizer.pattern.height, self.pattern_localizer.pattern.width, self.pattern_localizer.pattern.pointDist)
  local objectPoints = {}
  for i=1,#imagePoints do
    objectPoints[#objectPoints+1] = groundTruthPoints
  end

  local flags = 0
  for k,v in pairs(calibrationFlags) do
    if v == true then
      flags = bit.bor(flags, cv[k])
    end
  end

  printf('Using flags (%d):', flags)
  print(calibrationFlags)

  -- run calibration
  print('Running openCV camera calibration...')
  local reprojError, camMatrix, distCoeffs, rvecs, tvecs =
    cv.calibrateCamera{
      objectPoints=objectPoints,
      imagePoints=imagePoints,
      imageSize={w, h},
      flag=flags
    }

  print('Calibration results:')
  printf('Reprojection error: %f', reprojError)
  print('Camera matrix:')
  print(camMatrix)
  print('Distortion coefficients:')
  print(distCoeffs)

  self.calibration = {
    date = os.date('%Y-%m-%d %H:%M:%S'),
    patternGeometry = self.pattern_localizer.pattern,
    reprojError = reprojError,
    camMatrix = camMatrix,
    distCoeffs = distCoeffs,
    imWidth = w,
    imHeight = h,
    calibrationFlags = calibrationFlags
  }

  return true
end


-- Saves the robot joint states and end effector poses of the capture run in a file: jsposes.t7
-- Accesses the self.recorded_poses values
function AutoCalibration:savePoses()
    local configuration = self.configuration
    --local current_output_directory = path.join(configuration.output_directory, 'current')
    local jsposes = {}
    jsposes.recorded_joint_values = {}
    jsposes.recorded_poses = {}
    --create the structure to be saved
    for i = 1, #self.recorded_joint_values do
      print('recording pose #'..i)
      jsposes.recorded_joint_values[i] = self.recorded_joint_values[i]
      jsposes.recorded_poses[i] = self.recorded_poses[i]
    end

    local error_code, pose_of_reference, error_msg
    if self.configuration.camera_reference_frame ~= 'BASE' then
      local move_group_names, move_group_details = self.move_group.motion_service:queryAvailableMoveGroups()
      local mg_all = self.move_group.motion_service:getMoveGroup(move_group_names[1])
      local joint_values_all = mg_all:getCurrentJointValues()
      local joint_name = configuration.camera_reference_frame
      local link_name = string.gsub(joint_name, "joint", "link")
      print("reference link:")
      print(link_name)
      error_code, pose_of_reference, error_msg = mg_all.motion_service:queryPose(move_group_names[1], joint_values_all, link_name)
      if pose_of_reference == nil then
        print("Pose of reference link could not be determined!")
        print("error_code:")
        print(error_code)
        print("error_msg:")
        print(error_msg)
      else
        jsposes.recorded_pose_of_reference = getHomogeneousFromRosStampedPose(pose_of_reference)
        print("pose of reference link in base coordinates:")
        print(jsposes.recorded_pose_of_reference)
      end
    end

    print('AutoCalibration:savePoses()')
    print(jsposes)
    local poses_fn = 'jsposes.t7'
    os.execute('mkdir -p ' .. self.output_directory)
    local poses_file_path = path.join(self.output_directory, poses_fn)
    os.execute('rm -f ' .. poses_file_path)
    print('Saving poses file to: ' .. poses_file_path)
    torch.save(poses_file_path, jsposes)

    -- also linking pose file in current directory
    --local current_output_path = path.join(current_output_directory, poses_fn)
    --os.execute('rm -f ' .. current_output_path)
    --local link_target = path.join('..', self.calibration_folder_name, poses_fn)
    --os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
    --printf("Created link in '%s' -> '%s'", current_output_path, link_target)
end


function AutoCalibration:saveCalibration()
  local configuration = self.configuration
  local mode = configuration.calibration_mode
  local camera_serial = self.mono_selected_cam_serial

  -- generate output directory path
  local calibration_name = self.calibration_folder_name
  local output_directory = self.output_directory
  --local current_output_directory = path.join(configuration.output_directory, 'current')
  os.execute('mkdir -p ' .. output_directory)
  --os.execute('mkdir -p ' .. current_output_directory)

  if mode == CalibrationMode.SingleCamera then

    if self.calibration == nil then
      print('No calibration to save available.')
      return false
    end

    local calibration_fn = string.format('cam_%s.t7', camera_serial)
    local calibration_file_path = path.join(output_directory, calibration_fn)
    torch.save(calibration_file_path, self.calibration)
    print('Single camera calibration saved to: ' .. calibration_file_path)

    -- also linking calibration in current directory
    --local current_output_path = path.join(current_output_directory, calibration_fn)
    --os.execute('rm -f ' .. current_output_path)
    --local link_target = path.join('..', calibration_name, calibration_fn)
    --os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
    --printf("Created link in '%s' -> '%s'", current_output_path, link_target)

    return true

  elseif mode == CalibrationMode.StereoRig then

    if self.stereo_calibration == nil then
      print('No stereo calibration to save available.')
      return false
    end

    local left_camera = self.configuration.cameras[configuration.left_camera_id]
    local right_camera = self.configuration.cameras[configuration.right_camera_id]
    local left_camera_serial = left_camera.serial
    local right_camera_serial = right_camera.serial

    local calibration_fn = string.format('stereo_cams_%s_%s.t7', left_camera_serial, right_camera_serial)
    local calibration_file_path = path.join(output_directory, calibration_fn)
    torch.save(calibration_file_path, self.stereo_calibration)
    print('Stereo camera calibration saved to: ' .. calibration_file_path)

    -- also linking stereo calibration in current directory
    --local current_output_path = path.join(current_output_directory, calibration_fn)
    --os.execute('rm -f ' .. current_output_path)
    --local link_target = path.join('..', calibration_name, calibration_fn)
    --os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
    --printf("Created link in '%s' -> '%s'", current_output_path, link_target)

    -- export in xml format
    if xml ~= nil then
      self:exportStereot7AsXmlFiles(calibration_file_path)
    end

    return true

  end

  return false
end


function AutoCalibration:concat(rows, cols, tensor)
  s = ""
  for r=1,rows do
    for c=1,cols do
      s = s .. tensor[r][c]
      if (r*c ~= rows*cols) then
        s = s .. " "
      end
    end
  end
  return s
end

function AutoCalibration:exportStereot7AsXmlFiles(path_stereo_calib)

  local output_directory = self.output_directory

  print('Reading calibration file ',path_stereo_calib)
  stereocalib = torch.load(path_stereo_calib)
  print(stereocalib)

  leftCam = {xml='opencv_storage',
    {xml = 'Kc', type_id = "opencv-matrix",
      {xml = 'rows', '3'},
      {xml = 'cols', '3'},
      {xml = 'dt', 'f'},
      {xml = 'data', self:concat(3, 3, stereocalib.camLeftMatrix)},
    },
    {xml = 'kc', type_id = "opencv-matrix",
      {xml = 'rows', '5'},
      {xml = 'cols', '1'},
      {xml = 'dt', 'f'},
      {xml = 'data', self:concat(1, 5, stereocalib.camLeftDistCoeffs)},
    },
  }

  rightCam = {xml='opencv_storage',
    {xml = 'Kc', type_id = "opencv-matrix",
      {xml = 'rows', '3'},
      {xml = 'cols', '3'},
      {xml = 'dt', 'f'},
      {xml = 'data', self:concat(3, 3, stereocalib.camRightMatrix)},
    },
    {xml = 'kc', type_id = "opencv-matrix",
      {xml = 'rows', '5'},
      {xml = 'cols', '1'},
      {xml = 'dt', 'f'},
      {xml = 'data', self:concat(1, 5, stereocalib.camRightDistCoeffs)},
    },
  }

  funMatrix = {xml='opencv_storage',
    {xml = 'F', type_id = "opencv-matrix",
      {xml = 'rows', '3'},
      {xml = 'cols', '3'},
      {xml = 'dt', 'f'},
      {xml = 'data', self:concat(3, 3, stereocalib.F)},
    },
    {xml = 'R', type_id = "opencv-matrix",
      {xml = 'rows', '3'},
      {xml = 'cols', '3'},
      {xml = 'dt', 'f'},
      {xml = 'data', self:concat(3, 3, stereocalib.R)},
    },
    {xml = 'T', type_id = "opencv-matrix",
      {xml = 'rows', '3'},
      {xml = 'cols', '1'},
      {xml = 'dt', 'f'},
      {xml = 'data', self:concat(3, 1, stereocalib.T)},
    },
    {xml = 'Left_Kc', type_id = "opencv-matrix",
      {xml = 'rows', '3'},
      {xml = 'cols', '3'},
      {xml = 'dt', 'f'},
      {xml = 'data', self:concat(3, 3, stereocalib.camLeftMatrix)},
    },
    {xml = 'Left_kc', type_id = "opencv-matrix",
      {xml = 'rows', '5'},
      {xml = 'cols', '1'},
      {xml = 'dt', 'f'},
      {xml = 'data', self:concat(1, 5, stereocalib.camLeftDistCoeffs)},
    },
    {xml = 'Right_Kc', type_id = "opencv-matrix",
      {xml = 'rows', '3'},
      {xml = 'cols', '3'},
      {xml = 'dt', 'f'},
      {xml = 'data', self:concat(3, 3, stereocalib.camRightMatrix)},
    },
    {xml = 'Right_kc', type_id = "opencv-matrix",
      {xml = 'rows', '5'},
      {xml = 'cols', '1'},
      {xml = 'dt', 'f'},
      {xml = 'data', self:concat(1, 5, stereocalib.camRightDistCoeffs)},
    },
    {xml = 'FrameWidth', stereocalib.imWidth},
    {xml = 'FrameHeight', stereocalib.imHeight},
    {xml = 'left_camera_id', self.configuration.left_camera_id},
    {xml = 'right_camera_id', self.configuration.right_camera_id},
  }

  print("")
  print("LEFT CAMERA:")
  s,_ = xml.dump(leftCam):gsub("'", '"')
  print(s)
  local fn = path.join(output_directory, 'LeftCameraIntrinsics.xml')
  local file = io.open(fn, "w")
  file:write('<?xml version="1.0"?>\n')
  file:write(s)
  file:close()
  -- create a symbolic link
  --local current_output_path = path.join(self.current_path, 'LeftCameraIntrinsics.xml')
  --os.execute('rm ' .. current_output_path)
  --local link_target = path.join('..', self.calibration_folder_name, 'LeftCameraIntrinsics.xml')
  --os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
  --printf("Created link in '%s' -> '%s'", current_output_path, link_target)

  print("")
  print("RIGHT CAMERA:")
  s,_ = xml.dump(rightCam):gsub("'", '"')
  print(s)
  fn = path.join(output_directory, 'RightCameraIntrinsics.xml')
  file = io.open(fn, "w")
  file:write('<?xml version="1.0"?>\n')
  file:write(s)
  file:close()
  -- create a symbolic link
  --current_output_path = path.join(self.current_path, 'RightCameraIntrinsics.xml')
  --os.execute('rm ' .. current_output_path)
  --local link_target = path.join('..', self.calibration_folder_name, 'RightCameraIntrinsics.xml')
  --os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
  --printf("Created link in '%s' -> '%s'", current_output_path, link_target)

  print("")
  print("FUN MATRIX:")
  s,_ = xml.dump(funMatrix):gsub("'", '"')
  print(s)
  fn = path.join(output_directory, 'StereoCalibration.xml')
  file = io.open(fn, "w")
  file:write('<?xml version="1.0"?>\n')
  file:write(s)
  file:close()
  -- create a symbolic link
  --current_output_path = path.join(self.current_path, 'StereoCalibration.xml')
  --os.execute('rm ' .. current_output_path)
  --local link_target = path.join('..', self.calibration_folder_name, 'StereoCalibration.xml')
  --os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
  --printf("Created link in '%s' -> '%s'", current_output_path, link_target)
end
