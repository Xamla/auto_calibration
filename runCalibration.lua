--[[
  runCalibration.lua

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
local datatypes = require 'xamlamoveit.datatypes'
local motionLibrary = require 'xamlamoveit.motionLibrary'
local xutils = require 'xamlamoveit.xutils'
local grippers = require 'xamlamoveit.grippers.env'

ac = require 'auto_calibration'
local CalibrationMode = ac.CalibrationMode
local BASE_POSE_NAMES = ac.BASE_POSE_NAMES

require 'ximea.ros.XimeaClient'
local GenICamClient = ac.GenICamCameraClient


local prompt
local configuration
local motion_service
local auto_calibration
local hand_eye
local offline


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
  print('Folder \'./calibration/capture\' and \'jsposes.t7\' will be deleted. Did you backup your old capture data?')
  local go = true
  if wait ~= false then
    print('Press \'Enter\' to continue, \'ESC\' to cancel')
    go = prompt:waitEnterOrEsc()
  end

  if go then

    -- check whether we are simulating or not
    if offline then
      auto_calibration:runCaptureSequenceWithoutCapture()
    else
      auto_calibration:runCaptureSequence()
    end
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
  for cam_index, serial in pairs(configuration.cameras) do
    print('camera=', configuration.cameras[cam_index].serial)
    ids[#ids + 1] = configuration.cameras[cam_index].serial
  end

  local choice = prompt:chooseFromList(ids, 'Available cameras:')
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
  local jsposes_fn = './calibration/current/jsposes.t7'
  local jsposes
  if path.exists(jsposes_fn) then
    jsposes = torch.load(jsposes_fn)
  else
    print('Joint poses file does not exist: '..jsposes_fn)
    return false
  end

  local left_cam_data = nil
  local right_cam_data = nil
  local mode = configuration.calibration_mode
  if mode == CalibrationMode.SingleCamera then
    -- still need to ask which camera, there need not only be one
    ids = {}
    for cam_index, serial in pairs(configuration.cameras) do
      print('camera=', configuration.cameras[cam_index].serial)
      ids[#ids + 1] = configuration.cameras[cam_index].serial
    end
    local serial = prompt:chooseFromList(ids, 'Available cameras:')
    while serial == nil do
      print('No camera selected!')
      serial = prompt:chooseFromList(ids, 'Available cameras:')
    end
    print('selected camera:', serial)

    if configuration.cameras[configuration.left_camera_id] ~= nil then
      if configuration.cameras[configuration.left_camera_id].serial == serial then
        print("This is the left camera!")
        prompt:anyKey()
        left_cam_data = generateCurrentCapturedImageLog(configuration.cameras[configuration.left_camera_id].serial)
      end
    end
    if configuration.cameras[configuration.right_camera_id] ~= nil then
      if configuration.cameras[configuration.right_camera_id].serial == serial then
        print("This is the right camera!")
        prompt:anyKey()
        right_cam_data = generateCurrentCapturedImageLog(configuration.cameras[configuration.right_camera_id].serial)
      end
    end
  elseif mode == CalibrationMode.StereoRig then
    left_cam_data = generateCurrentCapturedImageLog(configuration.cameras[configuration.left_camera_id].serial)
    right_cam_data = generateCurrentCapturedImageLog(configuration.cameras[configuration.right_camera_id].serial)
  end

  local img_data = {}
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
    print('Want to pick a calibration target?')
    print('Press \'Enter\' for \'Yes\', \'ESC\' for \'No\'.')
    local go_pick = prompt:waitEnterOrEsc()
    if go_pick then
      pickCalibrationTarget()
    end
    runCaptureSequence()
    print('Want to return calibration target?')
    print('Press \'Enter\' for \'Yes\', \'ESC\' for \'No\'.')
    local go_return = prompt:waitEnterOrEsc()
    if go_return then
      returnCalibrationTarget()
    end
    calibrateCamera()
    saveCalibration()
    handEye()
  end

end

local function publishHandEye()
  prompt:printTitle('Publish Hand Eye matrix to rosvita')
  hand_eye:publishHandEye()
end

local function evaluateCalibrationSimple()
  prompt:printTitle('Evaluate calibration by only one movement')
  hand_eye:evaluateCalibration()
end

local function evaluateCalibrationComplex()
  prompt:printTitle('Evaluate calibration by several movements')
  hand_eye:evaluateCalibrationComplex()
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
    { 's', 'Evaluate calibration simple', evaluateCalibrationSimple },
    { 'c', 'Evaluate calibration complex', evaluateCalibrationComplex },
    { 'p', 'Publish HandEye matrix to rosvita worldview', publishHandEye },
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
  if mode == CalibrationMode.SingleCamera then

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
  cmd:option('-offline', 'false', 'set for offline mode')

  local opt = cmd:parse(arg)
  local filename = opt.cfg
  offline = false
  -- in case we are reading images from files 
  -- and not really connecting to the driver 
  -- set offline to true
  if opt.offline == 'true' then
    offline = true
  end
  configuration = torch.load(filename)

  camera_client = {}
  if not offline then
    print("online mode")
    if configuration.camera_type == 'ximea' then
      camera_client = XimeaClient(nh, 'ximea_mono', false, false)
    elseif configuration.camera_type == 'genicam' then
      camera_client = GenICamClient(nh, 'genicam_mono', false, false)
    end
  else
    print("offline mode")
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

  auto_calibration = ac.AutoCalibration(configuration, move_group, camera_client)
  hand_eye = HandEye.new(configuration, auto_calibration.calibration_folder_name, move_group, motion_service, camera_client, auto_calibration.gripper, xamla_mg)

  local p = io.popen("pwd")
  local start_path = p:read("*l")
  p:close()
  if not string.match(start_path, "projects") then
    print('\n')
    print('===================================================================================')
    print('= WARNING:                                                                        =')
    print('= Calibration results will get lost after stopping Rosvita!                       =')
    print('= To permanently save calibration results, start script from your project folder: =')
    print('= cd /home/xamla/Rosvita.Control/projects/<your_project_folder>                   =')
    print('= th ../../lua/auto_calibration/runCalibration.lua -cfg <your_config_file>        =')
    print('===================================================================================')
  end

  showMainMenu()

  -- shutdown system
  if not offline then
    if camera_client then
      camera_client:shutdown()
    end
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
