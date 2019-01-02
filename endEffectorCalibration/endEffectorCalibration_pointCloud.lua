-- Lua application to measure the offset of the grippertip relative to the TCP 
-- (normally at the flange) with structured light scan 
-- Note: The measurement showing the point-cloud runs outside the docker container
--       and is independent of the scanning process.

local torch = require 'torch'
local cv = require "cv"
require "cv.imgproc"
require "cv.imgcodecs"
require "cv.highgui"
require 'cv.calib3d'
local pcl = require "pcl"
local x = 0.0
local y = 0.0
local z = 0.0


-- Press shift + left mouse button for point picking
local function onPointPickingEvent(idx1, idx2, x1, y1, z1, x2, y2, z2)
  print('Point picking event!')
  print(string.format('idx1: %d, idx2: %d, (%f, %f, %f) (%f, %f, %f)', idx1, idx2, x1, y1, z1, x2, y2, z2))
  x = x1
  y = y1
  z = z1
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
  -- Load point cloud
  print("Please enter the path to the point cloud, from which the position of the gripper tip will be read out (e.g. /home/username/data/scan.pcd), then press 'Enter'.")
  local path_to_cloud = io.read()
  print("Path to the point cloud: ", path_to_cloud)
  print("\n")
  local cloud = pcl.PointCloud(pcl.PointXYZ)
  cloud:loadPCDFile(path_to_cloud)
  print("Size of point cloud is:")
  print(cloud:points():size())
  local all_points = cloud:points():size(1) * cloud:points():size(2) * cloud:points():size(3)
  print(string.format('Number of all cloud points is : %d', all_points))

  -- Crop point cloud
  local removed = pcl.Indices()
  local box_filtered = pcl.filter.cropBox(
  cloud,          -- input
  {0.01,0.01,0.08},  -- min
  {3.0,3.0,1.0},     -- max
  nil,               -- rotation
  nil,               -- translation
  nil,               -- transform
  nil,               -- indices
  nil,               -- output
  false,             -- negative
  removed            -- removed_indices
  )
  print(string.format('Number of removed cloud points: %d', removed:size()))
  print("\n")

  -- Show point cloud and let the user click the gripper tip
  local inspector = pcl.PCLVisualizer('Cloud Inspector', true)
  inspector:addCoordinateSystem(0.1)
  inspector:addPointCloud(box_filtered, 'box_filtered')
  inspector:setPointCloudRenderingProperties3(pcl.RenderingProperties.PCL_VISUALIZER_COLOR, 0.0, 1.0, 0.0, 'box_filtered')
  inspector:setPointCloudRenderingProperties1(pcl.RenderingProperties.PCL_VISUALIZER_POINT_SIZE, 2, 'box_filtered')
  print("Press shift + left mouse button for picking a point at the gripper tip, then close visualizer window.")
  h3 = inspector:registerPointPickingCallback(onPointPickingEvent)
  inspector:spin()
  h3:disconnect()
  print("\n")

  -- Determine the tip position in camera coordinates
  local translation = torch.FloatTensor(1,1,3)
  translation[1][1][1] = x
  translation[1][1][2] = y
  translation[1][1][3] = z
  print("Distance to camera: ", translation)

  -- Let the user choose the camera location and determine the gripper tip pose in tcp coordinates
  local tcp_to_tip = estimateClickPosture(translation)
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
