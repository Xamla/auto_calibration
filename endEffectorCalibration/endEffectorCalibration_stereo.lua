-- Lua application to measure the offset of the grippertip relative to the TCP 
-- (normally at the flange) with rectified and undistorted 2D stereo-images.

local torch = require 'torch'
local cv = require "cv"
require "cv.imgproc"
require "cv.imgcodecs"
require "cv.highgui"
require 'cv.calib3d'
local pcl = require "pcl"

-- parse command line
--[[
local cmd = torch.CmdLine()
cmd:text()
cmd:text()
cmd:text('Xamla Endeffector Calibration script.')
cmd:text()
cmd:option('-path', '180910T072439.494_inga/', 'path of point cloud and images')
cmd:option('-calibrationPath', '180910T072439.494_inga/calibration/current/', 'path of calibration')
cmd:option('-torsoToCam', 'LeftCam_torso_joint_b1.t7', 'left camera pose in torso coordinates calibration filename')
cmd:option('-poses_world', 'poses_world.t7', 'poses in world coordinates filename')

local opt = cmd:parse(arg)
local path = opt.path
local calibrationPath = opt.calibrationPath
local torso_to_camera = torch.load(calibrationPath .. opt.torsoToCam)
local poses_world = torch.load(path .. opt.poses_world)
local world_to_torso = poses_world.torso_link_b1
local world_to_camera = world_to_torso * torso_to_camera
local world_to_tcp = poses_world.tcp_left
]]

local rightImgXCoordinates = {}
local rightImgYCoordinates = {}
local leftImgXCoordinates = {}
local leftImgYCoordinates = {}
local mouseClickDataLeftImg = {}
mouseClickDataLeftImg.x = nil
mouseClickDataLeftImg.y = nil
mouseClickDataLeftImg.winID = nil
mouseClickDataLeftImg.clickType = nil
local mouseClickDataRightImg = {}
mouseClickDataRightImg.x = nil
mouseClickDataRightImg.y = nil
mouseClickDataRightImg.winID = nil
mouseClickDataRightImg.clickType = nil


local function recordXYRightImgDataMouseCallback(event, x, y, flags, userdata)
  if event == 1 then  -- left mouse button clicked
    mouseClickDataRightImg.x = x
    mouseClickDataRightImg.y = y
    mouseClickDataRightImg.winID = "scanCamRight image"
    mouseClickDataRightImg.clickType = event
    table.insert(rightImgXCoordinates, mouseClickDataRightImg.x)
    table.insert(rightImgYCoordinates, mouseClickDataRightImg.y)
  end
end


local function recordXYLeftImgDataMouseCallback(event, x, y, flags, userdata)
  if event == 1 then  -- left mouse button clicked
    mouseClickDataLeftImg.x = x
    mouseClickDataLeftImg.y = y
    mouseClickDataLeftImg.winID = "scanCamLeft image"
    mouseClickDataLeftImg.clickType = event
    table.insert(leftImgXCoordinates, mouseClickDataLeftImg.x)
    table.insert(leftImgYCoordinates, mouseClickDataLeftImg.y)
  end
end


local function estimateClickPosture(point)
  local click_pose_in_camera = torch.eye(4)
  click_pose_in_camera[{{1, 3}, {4}}] = point
  print("Click pose in camera coordinates (with same rotation as camera):")
  print(click_pose_in_camera)
  -- Let the user choose the camera location and determine the gripper tip pose in tcp coordinates
  print("Please choose the camera location and press 'Enter':")
  print('1: onboard')
  print('2: extern')
  print('3: on torso')
  local camera_location = io.read("*n")
  io.read()
  if camera_location == 1 then
    print("Camera location is 'onboard'.")
    print("Please enter the path to the 'tcp-to-camera' transformation (e.g. /home/username/calibration/HandEye.t7), then press 'Enter'.")
    local tcp_to_cam_path = io.read("*l")
    local tcp_to_camera = torch.load(tcp_to_cam_path)
    print("\n")
    click_pose_in_tcp = tcp_to_camera * click_pose_in_camera
    click_pose_in_tcp[{{1, 3}, {1, 3}}] = torch.eye(3)
    return click_pose_in_tcp
  elseif camera_location == 2 then
    print("Camera location is 'extern'.")
    print("Please enter the path to the 'world-to-camera' transformation (e.g. /home/username/calibration/LeftCam_base.t7), then press 'Enter'.")
    local world_to_cam_path = io.read("*l")
    local world_to_camera = torch.load(world_to_cam_path)
    print("Please enter the path to the tcp pose in world coordinates (e.g. /home/username/data/poses_world.t7), then press 'Enter'.")
    local poses_world_path = io.read()
    local poses_world = torch.load(poses_world_path)
    print("\n")
    local world_to_tcp = poses_world.tcp_left
    if poses_world.tcp_left == nil and poses_world.tcp_right ~= nil then
      world_to_tcp = poses_world.tcp_right
    elseif poses_world.tcp_left ~= nil and poses_world.tcp_right ~= nil then
      print("Choose the robot arm connected with the gripper we are calibrating and press 'Enter':")
      print('1: left')
      print('2: right')
      local which_arm = io.read("*n")
      io.read()
      if which_arm == 2 then
        world_to_tcp = poses_world.tcp_right
      end
    end
    local click_pose_in_world = world_to_camera * click_pose_in_camera
    click_pose_in_world[{{1, 3}, {1, 3}}] = world_to_tcp[{{1, 3}, {1, 3}}]
    click_pose_in_tcp = torch.inverse(world_to_tcp) * click_pose_in_world
    return click_pose_in_tcp
  elseif camera_location == 3 then
    print("Camera location is 'on torso'.")
    print("Please enter the path to the 'torso-to-camera' transformation (e.g. /home/username/calibration/LeftCam_torso_joint_b1.t7), then press 'Enter'.")
    local torso_to_cam_path = io.read("*l")
    local torso_to_camera = torch.load(torso_to_cam_path)
    print("Please enter the path to all given poses (torso, tcp, ...) in world coordinates (e.g. /home/username/data/poses_world.t7), then press 'Enter'.")
    local poses_world_path = io.read()
    local poses_world = torch.load(poses_world_path)
    local world_to_torso = poses_world.torso_link_b1
    local world_to_camera = world_to_torso * torso_to_camera
    print("\n")
    local world_to_tcp = poses_world.tcp_left
    if poses_world.tcp_left == nil and poses_world.tcp_right ~= nil then
      world_to_tcp = poses_world.tcp_right
    elseif poses_world.tcp_left ~= nil and poses_world.tcp_right ~= nil then
      print("Choose the robot arm connected with the gripper we are calibrating and press 'Enter':")
      print('1: left')
      print('2: right')
      local which_arm = io.read("*n")
      io.read()
      if which_arm == 2 then
        world_to_tcp = poses_world.tcp_right
      end
    end
    local click_pose_in_world = world_to_camera * click_pose_in_camera
    click_pose_in_world[{{1, 3}, {1, 3}}] = world_to_tcp[{{1, 3}, {1, 3}}]
    click_pose_in_tcp = torch.inverse(world_to_tcp) * click_pose_in_world
    return click_pose_in_tcp
  else
    print("Please enter only 1, 2, or 3.")
    return nil
  end
end


local function main()
  -- Load left and right 2d image
  print("Please enter the path to the left image, from which the position of the gripper tip will be read out, then press 'Enter'.")
  local path_to_imgLeft = io.read()
  print("Please enter the path to the right image, from which the position of the gripper tip will be read out, then press 'Enter'.")
  local path_to_imgRight = io.read()
  print("\n")
  local imgLeft = cv.imread {path_to_imgLeft}
  local imgRight = cv.imread {path_to_imgRight}

  print("Please enter the path to the stereo calibration.")
  local path_to_stereocalib = io.read()
  local stereoCalib = torch.load(path_to_stereocalib) --torch.load('180910T072439.494_inga/stereo_calibration.t7')
  local leftCamMat = stereoCalib.camLeftMatrix
  local rightCamMat = stereoCalib.camRightMatrix
  local leftDistCoeffs = stereoCalib.camLeftDistCoeffs
  local rightDistCoeffs = stereoCalib.camRightDistCoeffs
  local rightLeftCamTrafo = stereoCalib.trafoLeftToRightCam
  print("\n")

  -- Undistortion + rectification:
  local R = rightLeftCamTrafo[{{1, 3}, {1, 3}}]:double()
  local T = rightLeftCamTrafo[{{1, 3}, {4}}]:double()
  local leftR = torch.DoubleTensor(3, 3)
  local rightR = torch.DoubleTensor(3, 3)
  local leftP = torch.DoubleTensor(3, 4)
  local rightP = torch.DoubleTensor(3, 4)
  local Q = torch.DoubleTensor(4, 4)

  cv.stereoRectify {
    cameraMatrix1 = leftCamMat,
    distCoeffs1 = leftDistCoeffs,
    cameraMatrix2 = rightCamMat,
    distCoeffs2 = rightDistCoeffs,
    imageSize = {imgLeft:size(2), imgLeft:size(1)},
    R = R,
    T = T,
    R1 = leftR,
    R2 = rightR,
    P1 = leftP,
    P2 = rightP,
    Q = Q,
    flags = 0
  }

  local mapAImgLeft, mapBImgLeft =
    cv.initUndistortRectifyMap {
    cameraMatrix = leftCamMat,
    distCoeffs = leftDistCoeffs,
    R = leftR,
    newCameraMatrix = leftP,
    size = {imgLeft:size(2), imgLeft:size(1)},
    m1type = cv.CV_32FC1
  }
  local imgLeftRectUndist =
    cv.remap {src = imgLeft, map1 = mapAImgLeft, map2 = mapBImgLeft, interpolation = cv.INTER_NEAREST}

  local mapAImgRight, mapBImgRight =
    cv.initUndistortRectifyMap {
    cameraMatrix = rightCamMat,
    distCoeffs = rightDistCoeffs,
    R = rightR,
    newCameraMatrix = rightP,
    size = {imgRight:size(2), imgRight:size(1)},
    m1type = cv.CV_32FC1
  }
  local imgRightRectUndist =
    cv.remap {src = imgRight, map1 = mapAImgRight, map2 = mapBImgRight, interpolation = cv.INTER_NEAREST}

  -- Let user click the gripper tip and determine the tip position in camera coordinates
  print("Press left mouse button for picking a point at the gripper tip, then close the corresponding window.")
  cv.namedWindow {"ClickGripperTipLeftImg"}
  cv.namedWindow {"ClickGripperTipRightImg"}
  -- Set callback functions for left mouse click
  cv.setMouseCallback{winname="ClickGripperTipLeftImg", onMouse=recordXYLeftImgDataMouseCallback}
  cv.setMouseCallback{winname="ClickGripperTipRightImg", onMouse=recordXYRightImgDataMouseCallback}
  cv.imshow{winname="ClickGripperTipLeftImg", image=imgLeftRectUndist}
  cv.imshow{winname="ClickGripperTipRightImg", image=imgRightRectUndist} 
  cv.waitKey{-1}

  -- Save target points
  leftProjPoint = torch.DoubleTensor(2, 1) -- 2D coordinate (x,y) of gripper tip
  rightProjPoint = torch.DoubleTensor(2, 1)
  leftProjPoint[1][1] = leftImgXCoordinates[1]
  leftProjPoint[2][1] = leftImgYCoordinates[1]
  rightProjPoint[1][1] = rightImgXCoordinates[1]
  rightProjPoint[2][1] = rightImgYCoordinates[1]
  print("2D ProjPoint [in pixel] of leftCamImage:")
  print(leftProjPoint)
  print("2D ProjPoint [in pixel] of rightCamImage:")
  print(rightProjPoint)

  resulting4DPoint = torch.DoubleTensor(4, 1)

  -- TriangulatePoints:
  ---------------------
  cv.triangulatePoints{leftP, rightP, leftProjPoint, rightProjPoint, resulting4DPoint}

  resulting3DPoint = torch.DoubleTensor(1, 3)
  resulting3DPoint[1][1] = resulting4DPoint[1][1] / resulting4DPoint[4][1]
  resulting3DPoint[1][2] = resulting4DPoint[2][1] / resulting4DPoint[4][1]
  resulting3DPoint[1][3] = resulting4DPoint[3][1] / resulting4DPoint[4][1]
  print("Corresponding 3D point:")
  print(resulting3DPoint)

  pointInLeftCamCoords = torch.DoubleTensor(3)
  pointInLeftCamCoords = torch.inverse(leftR) * resulting3DPoint[1]
  print("Distance to left camera:")
  print(pointInLeftCamCoords)

  -- Let the user choose the camera location and determine the gripper tip pose in tcp coordinates
  local tcp_to_tip = estimateClickPosture(pointInLeftCamCoords)
  if tcp_to_tip ~= nil then
    print("Gripper tip pose in tcp coordinates (with same rotation as tcp):")
    print(click_pose_in_tcp)
    torch.save("tip_pose_in_tcp_coordinates.t7", tcp_to_tip)

    print("To relocate the TCP in the 3D View, add a \'tcp_link\' to the file \'robotModel/main.xacro\' of your project folder.")
    print("As \'origin xyz\' of your tcp_link choose the translation vector of your calculated tip pose in TCP coordinates.")
    print("More precisely, you have to add the following lines inside the <robot> </robot> environment: \n")
    print('<link name=\"tcp_link\" />')
    print('<joint name=\"tcp_joint_F\" type=\"fixed\">')
    print('  <child link=\"tcp_link\" />')
    print('  <parent link=\"e.g. arm_left_link_tool0\" />   <- adapt this to your endeffector link name')
    print(string.format('  <origin xyz=\"%.6f %.6f %.6f\" rpy=\"0 0 0\" />', tcp_to_tip[1][4], tcp_to_tip[2][4], tcp_to_tip[3][4]))
    print('</joint>')
    print("\n")
    print("Now, compile the new main.xacro.")
    print("Then, in the \'Configuration View\' once press compile to make the \'tcp_link\' selectable.")
    print("In the \'Configuration View\' under \'MotionPlanning\'->\'MoveIt!\'->\'groups\'->\'<group_name>\' select the new \'tcp_link\' as \'Tip link\'.")
    print("Moreover, under \'MotionPlanning\'->\'MoveIt!\'->\'endEffectors\'->\'<end effector name>\' select the new \'tcp_link\' as \'Parent link\'.")
    print("After compiling the new robot configuration, starting ROS, changing into the 'World View' and choosing the corresponding move group and end effector, the interactive marker appears at the new tcp link.")
  end
end


main()
