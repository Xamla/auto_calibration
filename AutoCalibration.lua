local ros = require 'ros'
local actionlib = ros.actionlib
local SimpleClientGoalState = actionlib.SimpleClientGoalState
local xutils = require 'xamlamoveit.xutils'
local autoCalibration = require 'autoCalibration_env'
local CalibrationFlags = autoCalibration.CalibrationFlags
local CalibrationMode = autoCalibration.CalibrationMode

local cv = require 'cv'
require 'cv.imgproc'
require 'cv.imgcodecs'
require 'cv.highgui'
require 'cv.calib3d'

require 'ximea.ros.XimeaClient'
require 'multiPattern.PatternLocalisation'


local GraspingState = {
  Idle = 0,
  NoPartFound = 2,
  PartLost = 3,
  Hold = 4,
}


local AutoCalibration = torch.class('autoCalibration.AutoCalibration', autoCalibration)


local function initializeGripperServices(self)
  local node_handle = ros.NodeHandle()
  self.node_handle = node_handle
  self.gripper_status_client = node_handle:serviceClient('/wsg_50_driver/get_gripper_status', 'wsg_50_common/GetGripperStatus')
  self.ack_error_client = node_handle:serviceClient('/wsg_50_driver/acknowledge_error', 'std_srvs/Empty')
  self.set_force_client = node_handle:serviceClient('/wsg_50_driver/set_force', 'wsg_50_common/Conf')
  self.gripper_action_server = actionlib.SimpleActionClient('wsg_50_common/Command', '/wsg_50_driver/gripper/command', self.node_handle)
end


function AutoCalibration:__init(configuration, move_group, ximea_client, sl_studio)
  self.configuration = configuration
  self.move_group = move_group
  self.ximea_client = ximea_client
  self.sl_studio = sl_studio

  local ok, err = pcall(function() initializeGripperServices(self) end)
  if not ok then
    --error('Gripper initialization failed: ' .. err)
  end
end


function AutoCalibration:shutdown()
  self.gripper_status_client:shutdown()
  self.ack_error_client:shutdown()
  self.set_force_client:shutdown()
  self.gripper_action_server:shutdown()
  self.node_handle:shutdown()
end


local function createPatternLocalizer(self)
  local pattern_geometry = self.configuration.circle_pattern_geometry
  local pattern_localizer = PatternLocalisation()
  pattern_localizer.circleFinderParams.minArea = 300
  pattern_localizer.circleFinderParams.maxArea = 4000
  pattern_localizer:setPatternIDdictionary(torch.load("patternIDdictionary.t7"))
  pattern_localizer:setDBScanParams(100, 10)
  pattern_localizer.debugParams = { circleSearch = false, clustering = false, pose = false }
  pattern_localizer:setPatternData(pattern_geometry[2], pattern_geometry[1], pattern_geometry[3])
  self.pattern_localizer = pattern_localizer
end


local function moveJ(self, pos)
  assert(pos ~= nil, 'Target position is nil.')
  self.move_group:moveJ(pos)
  sys.sleep(1.0)
end


function AutoCalibration:moveToStart()
  local base_poses = self.configuration.base_poses
  assert(base_poses ~= nil)
  moveJ(self, base_poses['start'])
end


function AutoCalibration:moveToCaptureBase()
  local base_poses = self.configuration.base_poses
  assert(base_poses ~= nil)
  moveJ(self, base_poses['camera1_base'])
end


function AutoCalibration:pickCalibrationTarget()
  self:homeGripper()

  local base_poses = self.configuration.base_poses
  assert(base_poses ~= nil)
  moveJ(self, base_poses['start'])
  self:openGripper()
  moveJ(self, base_poses['pre_pick_marker'])
  moveJ(self, base_poses['pick_marker'])
  self:closeGripper()
  moveJ(self, base_poses['post_pick_marker'], self.configuration.velocity_scaling * 0.25)
  moveJ(self, base_poses['start'])
end


function AutoCalibration:returnCalibrationTarget()
  local base_poses = self.configuration.base_poses
  assert(base_poses ~= nil)
  moveJ(self, base_poses['start'])
  moveJ(self, base_poses['post_pick_marker'])
  moveJ(self, base_poses['pick_marker'], self.configuration.velocity_scaling * 0.25)
  self:openGripper()
  moveJ(self, base_poses['pre_pick_marker'])
end


function AutoCalibration:runCaptureSequenceWithoutCapture()
  local pos_list = self.configuration.capture_poses
  assert(pos_list ~= nil and #pos_list > 0)
  for i,p in ipairs(pos_list) do
    moveJ(self, p)
  end
end


function AutoCalibration:openGripper()
  -- this funtion should use move or release depending on gripper state -> open the gripper
  print('Opening gripper...')
  if self.gripper_status_client:exists() then
    if self.gripper_action_server:waitForServer(ros.Duration(5.0)) then
      local gripper_status = self.gripper_status_client:call()
      local g = self.gripper_action_server:createGoal()
      if (gripper_status ~= nil and gripper_status.status ~= nil) then
        if gripper_status.status.grasping_state_id == GraspingState.Idle then
          g.command.command_id = 101
        elseif gripper_status.status.grasping_state_id == GraspingState.Hold
          or gripper_status.status.grasping_state_id == GraspingState.NoPartFound
          or gripper_status.status.grasping_state_id == GraspingState.PartLost then

          g.command.command_id = 103
        else
          ros.ERROR("Gripper is in wrong state. Please acknowledge any error and use homing. Grasping State: %d", gripper_status.status.grasping_state_id)
          return
        end
      else
        ros.ERROR("Gripper is in wrong state. Please acknowledge any error and use homing. Grasping State: nil")
        return
      end

      g.command.speed = 200
      g.command.width = 50

      local state = self.gripper_action_server:sendGoalAndWait(g, 5, 5)
      local result = self.gripper_action_server:getResult()
      print(state, result)
      if state == 7 and result ~= nil and result.status ~= nil and result.status.return_code == 0 then
        ros.INFO("Opened gripper successfully")
        return
      else
        ros.ERROR("Could not open gripper")
      end
    else
      ros.ERROR("Could not contact gripper action server")
    end
  else
    ros.ERROR("Could not query gripper status")
  end

  --error('Release failed.')
end


function AutoCalibration:closeGripper()
  print('Closing gripper...')

  local set_force_response;
  if self.set_force_client:exists() then
    local req = self.set_force_client:createRequest()
    req.val = 50
    set_force_response = self.set_force_client:call(req)
  end

  if set_force_response ~= nil and set_force_response.error == 0 then
    if self.gripper_action_server:waitForServer(ros.Duration(5.0)) then
      local g = self.gripper_action_server:createGoal()
      g.command.command_id = 102
      g.command.speed = 100
      g.command.width = 10

      local state = self.gripper_action_server:sendGoalAndWait(g, 5, 5)
      local result = self.gripper_action_server:getResult()

      if state == 7 then
        if result ~= nil and result.status ~= nil then
          if result.status.return_code == 0 and result.status.grasping_state_id == GraspingState.Hold then
            ros.INFO("Picked part successfully")
            return
          else
            ros.ERROR("Could not pick part. gripper return code: %d, grasping state %d", result.status.return_code, result.status.grasping_state_id)
          end
        else
          ros.ERROR("Could not close gripper: action server returned nil as gripper state");
        end
      else
        ros.ERROR("Could not close gripper: action server returned state: %d: %s", state, SimpleClientGoalState[state])
      end
    else
      ros.ERROR("Could not contact gripper action server")
    end
  else
    ros.ERROR("Could not set gripper force")
  end
  --error('Grip failed.')
end


function AutoCalibration:ackGripper()
  print('ACK gripper')
  if self.ack_error_client:exists() then
    local req = self.ack_error_client:createRequest()
    local r = self.ack_error_client:call(req)
    print(r)
  else
    ros.ERROR("Could not contact error acknowledge service")
  end
end


function AutoCalibration:homeGripper()
  print('Homeing gripper')

  if self.gripper_action_server:waitForServer(ros.Duration(5.0)) then
    local g = self.gripper_action_server:createGoal()
    g.command.command_id = 104

    local state = self.gripper_action_server:sendGoalAndWait(g, 5, 5)
    local result = self.gripper_action_server:getResult()

    if state == 7 and result.return_code == 0 then
      ros.INFO("Homed gripper successfully")
    else
      ros.ERROR("Could not home gripper")
    end
  else
    ros.ERROR("Could not contact gripper action server")
  end
end


function AutoCalibration:runCaptureSequence()
  local file_names = {}
  local recorded_joint_values = {}   -- measured after getImage call
  local recorded_poses = {}          -- poses of all joints after getImage call

  local output_directory = path.join(self.configuration.output_directory, 'capture')

  print('Deleting output directory')
  os.execute('rm -r '.. output_directory)

  print('Creating output directory')
  os.execute('mkdir -p ' .. output_directory)

  local camera_serial = self.configuration.camera_serial
  local exposure = self.configuration.exposure
  local sleep_before_capture = self.configuration.sleep_before_capture

  local pos_list = self.configuration.capture_poses
  assert(pos_list ~= nil and #pos_list > 0)

  for i,p in ipairs(pos_list) do
    printf('Moving to position #%d...', i)

    -- move to capture pose
    moveJ(self, p)

    -- wait configured time before capture
    if sleep_before_capture > 0 then
      sys.sleep(sleep_before_capture)
    end

    -- capture image
    self.ximea_client:setExposure(exposure, {camera_serial})
    local image = self.ximea_client:getImages({camera_serial})

    -- get joint values and pose of image
    recorded_joint_values[#recorded_joint_values+1] = self.move_group:getCurrentJointValues()
    recorded_poses[#recorded_poses+1] = self.move_group:getCurrentPose():toTensor()

    -- create output filename
    local fn = path.join(output_directory, string.format('cam_%s_%03d.png', camera_serial, i))
    if image:nDimension() > 2 then
      image = cv.cvtColor{image, nil, cv.COLOR_RGB2BGR}
    end

    -- write image to disk
    printf("Writing image: %s", fn)
    local ok = cv.imwrite{fn, image}
    assert(ok, 'Could not write image.')

    file_names[#file_names+1] = fn

    collectgarbage()
  end

  self.file_names = file_names
  self.recorded_joint_values = recorded_joint_values
  self.recorded_poses = recorded_poses

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
      end
    end
  end
  return found, w, h
end


function AutoCalibration:monoStructuredLightCalibration()
  -- initialize structured light scanner
  local checkerboard_geometry = torch.FloatTensor({7, 11, 10})

  --code = slstudio:initCalibration(14, internal:float(), distort:float(), checker:float(), slstudio.CAMERA_ROS, 'CAMAU1639042', '', '/tmp/slstudio/')
  code = slstudio:initCalibration(10, nil, nil, checker:float(), slstudio.CAMERA_ROS, "CAMAU1710001", "/tmp/sls", "/tmp/newFrames/")




  slstudio:snapAndVerify()

  -- generate calibartion calibratio 
  local calib_errors = torch.FloatTensor({0, 0, 0})
  slstudio:calibrate(calib_errors)
  print("CALIB ERRORS:")
  print(calib_errors)

  -- save calibration and deinit
  slstudio:save("/tmp/calibration.xml")
  slstudio:close()
end


function AutoCalibration:stereoCalibration()

end


function AutoCalibration:monoCalibration(calibrationFlags)
  local image_paths = self.file_names
  local pattern_id = self.configuration.circle_pattern_id
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

  --cv.imshow{"Located point centers", pointImg}
  --cv.waitKey{-1}

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

--[[
 -- structured light scanning
    if self.sl_studio ~= nil then
      code, cloud, image_on, image_off = self.sl_studio:scan(20.0)
      print ("got code: " .. code)
      if code > -1 then
        local slsOutputPath = path.join(output_directory, "" .. i)
        path.mkdir(slsOutputPath)
        local on_fn = path.join(slsOutputPath, 'on.png')
        local off_fn = path.join(slsOutputPath, 'off.png')
        local cloud_fn = path.join(slsOutputPath, 'cloud.pcd')
        local pose_fn = path.join(slsOutputPath, 'pose.t7')

        local p = self.move_group:getCurrentPose():toTensor()
        local q = self.move_group:getCurrentJointValues()

        cv.imwrite{on_fn, image_on}
        cv.imwrite{off_fn, image_off}
        cloud:savePCDFile(cloud_fn)
        torch.save(pose_fn, { pose = p, joints = q })
      else
        printf("ERR: Error while scanning position #%d", i)
      end
    end
]]
