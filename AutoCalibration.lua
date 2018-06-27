package.path = package.path .. ";../../lua/auto_calibration/?.lua"
package.path = package.path .. ";/home/xamla/Rosvita.Control/lua/auto_calibration/?.lua"
local ros = require 'ros'
local tf = ros.tf
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local xutils = require 'xamlamoveit.xutils'
local autoCalibration = require 'autoCalibration_env'
local CalibrationMode = autoCalibration.CalibrationMode
local CalibrationFlags = autoCalibration.CalibrationFlags
local xamlamoveit = require 'xamlamoveit'
local datatypes = xamlamoveit.datatypes

local cv = require 'cv'
require 'cv.imgproc'
require 'cv.imgcodecs'
require 'cv.highgui'
require 'cv.calib3d'

require 'ximea.ros.XimeaClient'
require 'GenICamClient'
require 'multiPattern.PatternLocalisation'

local grippers = require 'xamlamoveit.grippers.env'

local ConfigurationCalibration = require 'ConfigurationCalibration' --class to manage the calibration data

local function tryRequire(module_name)
  local ok, val = pcall(function() return require(module_name) end)
  if ok then
    return val
  else
    return nil
  end
end

local slstudio = tryRequire('slstudio')


local AutoCalibration = torch.class('autoCalibration.AutoCalibration', autoCalibration)


--creates a gripper client for the specified key
local function constructGripper(grippers, key, nh)
  local gripper_action_name = nil
    
  if string.find(key, 'robotiq') ~= nil then
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

  local ok, err = pcall(function() initializeGripperServices(self) end)
  if not ok then
    error('Gripper initialization failed: ' .. err)
  end

  self.tcp_frame_of_reference, self.tcp_end_effector_name = self:getEndEffectorName()
  --create the calibrate/current folder if it does not exist
  os.execute('mkdir -p ' .. configuration.output_directory)
  self.current_path = path.join(configuration.output_directory, 'current')
  os.execute('mkdir -p ' .. self.current_path)

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
  if slstudio ~= nil then
    slstudio:closeCalibration()
  end
end


local function createPatternLocalizer(self)
  local pattern_geometry = self.configuration.circle_pattern_geometry
  local pattern_localizer = PatternLocalisation()
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
  self.move_group:moveJoints(pos)
  --check that we arrived where we expected
  local curPose = self.move_group:getCurrentPose():toTensor()
  local curJoints = self.move_group:getCurrentJointValues()
end


function AutoCalibration:moveToStart()
  local base_poses = self.configuration.base_poses
  assert(base_poses ~= nil, 'Base poses are not defined.')
  moveJ(self, base_poses['start'])
end


function AutoCalibration:moveToCaptureBase()
  local base_poses = self.configuration.base_poses
  assert(base_poses ~= nil, 'Base poses are not defined.')
  moveJ(self, base_poses['camera1_base'])
end


function AutoCalibration:pickCalibrationTarget()
  self.gripper:home()
  print('close the gripper')
  self.gripper:move{width=0.0, speed=0.2, force=20, stop_on_block=false} -- move closed
  local base_poses = self.configuration.base_poses
  assert(base_poses ~= nil, 'Base poses are not defined.')
  print('move to start')
  moveJ(self, base_poses['start'])
  print('open the gripper')
  local t = self.gripper:move{width=0.05} -- move open
  assert(t:hasCompletedSuccessfully() == true, 'Cannot open gripper.')
  print('move to pre pick pose')
  moveJ(self, base_poses['pre_pick_marker'])
  print('move to pick pose')
  moveJ(self, base_poses['pick_marker'])
  print('grasp calibration target')
  -- If width of calibration target may vary (i.e is assumed to be unknown),
  -- set width to 0.0, otherwise width can be set to the width of our calibration pattern.
  -- The "wsg50" needs the width of the part to be grasped -> set width to 0.0115.
  t = self.gripper:grasp{width=0.0115, speed=0.2} -- grasp target
  assert(t:hasCompletedSuccessfully() == true, 'Cannot grasp pattern.')
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
  local t = self.gripper:release{width=0.05, speed=0.2} -- release target
  assert(t:hasCompletedSuccessfully() == true, 'Cannot release pattern.')
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
  for i = 1, #self.recorded_joint_values do
    print('restoring pose #'..i)
    self.recorded_joint_values[i] = offline_jsposes.recorded_joint_values[i]
    self.recorded_poses[i] = offline_jsposes.recorded_poses[i]
  end
  self:savePoses()
end


function AutoCalibration:runCaptureSequenceWithoutCapture()
  local pos_list = self.configuration.capture_poses
  assert(pos_list ~= nil and #pos_list > 0)
  for i,p in ipairs(pos_list) do
    moveJ(self, p)
  end
end


function AutoCalibration:closeGripper()
  if self.gripper ~= nil then
    print('close the gripper')
    self.gripper:move{width=0.0, speed=0.2, force=20, stop_on_block=false} -- move closed
  else
    print('Need to initialize the gripper')
  end
end


function AutoCalibration:openGripper()
  if self.gripper ~= nil then
    print('open the gripper')
    self.gripper:move{width=0.06, speed=0.2} -- move open
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
  local current_output_directory = path.join(self.configuration.output_directory, 'current')
  local calibration_fn = string.format('cam_%s.t7', camera_serial)
  local calibration_file_path = path.join(current_output_directory, calibration_fn)

  printf("Probing for calibration file '%s'.", calibration_file_path)
  if path.exists(calibration_file_path) then
    return torch.load(calibration_file_path)
  else
    return nil
  end
end


local function tryLoadCurrentCameraCalibrationFromStereo(self, camera_left_serial, camera_right_serial)
  local current_output_directory = path.join(self.configuration.output_directory, 'current')
  local calibration_fn = string.format('stereo_cams_%s_%s.t7', camera_left_serial, camera_right_serial)
  local calibration_file_path = path.join(current_output_directory, calibration_fn)

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
  local recorded_joint_values = {}   -- measured after getImage call
  local recorded_poses = {}          -- poses of all joints after getImage call

  local output_directory = path.join(configuration.output_directory, 'capture')

  print('Deleting output directory')
  os.execute('rm -rf '.. output_directory)

  print('Creating output directory')
  os.execute('mkdir -p ' .. output_directory)

   -- create the folder structure where to put the captured data
  --self.config_class:createOutputDirectories()

  local pos_list = configuration.capture_poses
  assert(pos_list ~= nil and #pos_list > 0)

  local left_camera = configuration.cameras[configuration.left_camera_id]
  local right_camera = configuration.cameras[configuration.right_camera_id]

  local mode = configuration.calibration_mode
  if mode == CalibrationMode.StructuredLightSingleCamera then
    assert(slstudio ~= nil, 'Structured light scanning module could not be loadad.')
    assert(left_camera ~= nil, "Left camera for SL scanning not correctly configured.")

    local intrinsics
    local distortion

    -- try to get camera parameters from current configuration
    local camera_calibration = tryLoadCurrentCameraCalibration(self, left_camera.serial)
    if camera_calibration ~= nil then
      print('Using existing camera calibration')
      intrinsics = camera_calibration.camMatrix:float()
      distortion = camera_calibration.distCoeffs:float()
      print('Camera matrix:')
      print(intrinsics)
      print('Distortion coefficients:')
      print(distortion)
    else
      printf("No existing camera calibration found for camera '%s'.", left_camera.serial)
    end

    print('using cam', left_camera)
    local result = slstudio:initCalibration(
      left_camera.exposure / 1000,
      intrinsics,
      distortion,
      configuration.checkerboard_pattern_geometry:float(),
      slstudio.CAMERA_ROS,
      left_camera.serial,
      path.join(output_directory, 'sls'),
      path.join(output_directory, 'newFrames')
    )

    printf('Structured light initializaton returned: %d', result)
  end

  for i,p in ipairs(pos_list) do

    printf('Moving to position #%d...', i)

    -- move to capture pose
    moveJ(self, p)

    if mode == CalibrationMode.SingleCamera or mode == CalibrationMode.StereoRig then

      if left_camera ~= nil then
        local fn, joint_values, pose = captureImage(self, i, left_camera, output_directory)
        file_names[#file_names+1] = fn
        recorded_joint_values[#recorded_joint_values+1] = joint_values
        recorded_poses[#recorded_poses+1] = pose:toTensor()
      end

      if right_camera ~= nil then
        local fn, joint_values, pose = captureImage(self, i, right_camera, output_directory)
        file_names[#file_names+1] = fn
        --recorded_joint_values[#recorded_joint_values+1] = joint_values
        --recorded_poses[#recorded_poses+1] = pose:toTensor()
      end

    elseif mode == CalibrationMode.StructuredLightSingleCamera then

      local result = slstudio:snapAndVerify()
      print("RESULT: " .. result)

    end

    collectgarbage()
  end

  self.file_names = file_names
  self.recorded_joint_values = recorded_joint_values
  self.recorded_poses = recorded_poses

  self:savePoses()

  return file_names, recorded_joint_values, recorded_poses
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
        print('[OK] (%dx%d)', fn, w, h)
      else
        printf("[Warning] Pattern not found in image file '%s'", fn)
        printf("Index of not usable image: %d", i)
        not_found_indices[#not_found_indices+1] = i
      end
    end
  end
  return found, w, h, not_found_indices
end


function AutoCalibration:monoStructuredLightCalibration()
  -- calibrate structured light system
  local calibration_errors = torch.FloatTensor({0, 0, 0})
  slstudio:calibrate(calibration_errors)
  print("Calibration erros:")
  print(calibration_errors)
  self.calibration_errors = calibration_errors
  return true
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
      local objectPointsLeft = {}
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
      local objectPointsRight = {}
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

  if next(not_found_left) ~= nil then
    for i = 1,#not_found_left do
      table.remove(imagePointsRight, not_found_left[i])
    end
  end
  if next(not_found_right) ~= nil then
    for i = 1,#not_found_right do
      table.remove(imagePointsLeft, not_found_right[i])
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
    printf("Looking for images in '%s' with serial '%s'", current_directory_path,pattern)
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
    local current_output_directory = path.join(configuration.output_directory, 'current')
    local jsposes = {}
    jsposes.recorded_joint_values = {}
    jsposes.recorded_poses = {}
    --create the structure to be saved
    for i = 1, #self.recorded_joint_values do
      print('recording pose #'..i)
      jsposes.recorded_joint_values[i] = self.recorded_joint_values[i]
      jsposes.recorded_poses[i] = self.recorded_poses[i]
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
    local current_output_path = path.join(current_output_directory, poses_fn)
    os.execute('rm -f ' .. current_output_path)
    local link_target = path.join('..', self.calibration_folder_name, poses_fn)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
    printf("Created link in '%s' -> '%s'", current_output_path, link_target)
end


function AutoCalibration:saveCalibration()
  local configuration = self.configuration
  local mode = configuration.calibration_mode
  local camera_serial = self.mono_selected_cam_serial

  -- generate output directory path
  local calibration_name = self.calibration_folder_name
  local output_directory = self.output_directory
  local current_output_directory = path.join(configuration.output_directory, 'current')
  os.execute('mkdir -p ' .. output_directory)
  os.execute('mkdir -p ' .. current_output_directory)

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
    local current_output_path = path.join(current_output_directory, calibration_fn)
    os.execute('rm -f ' .. current_output_path)
    local link_target = path.join('..', calibration_name, calibration_fn)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
    printf("Created link in '%s' -> '%s'", current_output_path, link_target)

    return true

  elseif mode == CalibrationMode.StructuredLightSingleCamera then

    if self.calibration_errors == nil then
      print('No calibration available.')
      return false
    end

    local calibration_fn = 'sls.xml'
    local calibration_file_path = path.join(output_directory, calibration_fn)
    slstudio:save(calibration_file_path)
    print('Structured light calibration saved to: ' .. calibration_file_path)

    local current_output_path = path.join(current_output_directory, calibration_fn)
    os.execute('rm -f ' .. current_output_path)
    local link_target = path.join('..', calibration_name, calibration_fn)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
    printf("Created link in '%s' -> '%s'", current_output_path, link_target)

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
    local current_output_path = path.join(current_output_directory, calibration_fn)
    os.execute('rm -f ' .. current_output_path)
    local link_target = path.join('..', calibration_name, calibration_fn)
    os.execute('ln -s -T ' .. link_target .. ' ' .. current_output_path)
    printf("Created link in '%s' -> '%s'", current_output_path, link_target)

    return true

  end

  return false
end
