-----------------------------------------------
-- End effector (e.g. gripper tip) calibration
-----------------------------------------------
local ros = require 'ros'
local tf = ros.tf
local moveit = require 'moveit'
local xutils = require 'xamlamoveit.xutils'
local components = require 'xamlamoveit.components'
local datatypes = require 'xamlamoveit.datatypes'
local motionLibrary = require 'xamlamoveit.motionLibrary'

--Init ros node
ros.init('endEffectorCalibration')
local nh = ros.NodeHandle()
local sp = ros.AsyncSpinner() -- background job
sp:start()

--Create you dedicated motion service
local motion_service = motionLibrary.MotionService(nh)


-- Determine a sphere that fits best through all given points.
-- Least squares solution to: min ||AX-B||_F 
-- (see "torch.gels" at https://github.com/torch/torch7/blob/master/doc/maths.md)
function determineSphereFromPointTable(spherePoints)

  assert(#spherePoints > 3)
  -- TODO: Check for further errors! 
  -- At least 4 points have to be not on a line or plane (i.e. matrix A has to be of full rank) 

  local A = torch.DoubleTensor(#spherePoints, 4)
  for i=1,#spherePoints do
    A[i][1] = 1.0
    A[i][2] = spherePoints[i][1]
    A[i][3] = spherePoints[i][2]
    A[i][4] = spherePoints[i][3]
  end
  print("A:")
  print(A)

  local B = torch.DoubleTensor(#spherePoints, 1)
  for i=1,#spherePoints do
    B[i][1] = - (   spherePoints[i][1]*spherePoints[i][1] 
                  + spherePoints[i][2]*spherePoints[i][2] 
                  + spherePoints[i][3]*spherePoints[i][3] )
  end
  print("B:")
  print(B)

  -- Solve system of linear equations AX = B with "torch.gels". 
  local X = torch.gels(B, A)
  print("X:")
  print(X)
  print("Error of solution:")
  print(B:dist(A*X))
  print("\n")

  -- Alternatively solve AX = B by calculating the pseudo inverse of A.
  -- => X = A^-1 * B = (A^T A)^-1 * A^T * B
  local B_alternative = torch.DoubleTensor(#spherePoints)
  for i=1,#spherePoints do
    B_alternative[i] = - (   spherePoints[i][1]*spherePoints[i][1] 
                           + spherePoints[i][2]*spherePoints[i][2] 
                           + spherePoints[i][3]*spherePoints[i][3] )
  end
  At = A:transpose(1, 2)
  local X_alternative = torch.mv(torch.mm(torch.inverse(torch.mm(At, A)), At), B_alternative)
  print("X_alternative:")
  print(X_alternative)
  print("Error of alternative solution:")
  print(B_alternative:dist(A*X_alternative))
  print("\n")

  local center = torch.DoubleTensor(3)
  center[1] = -X[2][1]/2.0
  center[2] = -X[3][1]/2.0
  center[3] = -X[4][1]/2.0
  print("Center of sphere:")
  print(center)

  local r = math.sqrt(center[1]*center[1] + center[2]*center[2] + center[3]*center[3] - X[1][1])
  print(string.format("Radius of sphere: r = %f\n", r))

  return X, center, r

end


function main()

  local move_group_names, move_group_details = motion_service:queryAvailableMoveGroups()
  print("Available Move Groups:")
  print(move_group_names)
  print("Choose a move group (i.e. a number) and press return when ready.")
  local move_group_number = io.read("*n")
  local move_group_name = move_group_names[move_group_number]
  print(string.format("Chosen move group is: %s \n", move_group_name))
  local succ, current_joint_values = motion_service:queryJointState(move_group_details[move_group_name].joint_names)
  local plan_parameters = motion_service:getDefaultPlanParameters(move_group_name, move_group_details[move_group_name].joint_names)
  local move_group = motionLibrary.MoveGroup(motion_service, move_group_name)

  local saved_joint_values = {}
  --local saved_joints1 = torch.load("sphere_point_in_joint_values_1.t7")
  --local saved_joints2 = torch.load("sphere_point_in_joint_values_2.t7")
  --local saved_joints3 = torch.load("sphere_point_in_joint_values_3.t7")
  --local saved_joints4 = torch.load("sphere_point_in_joint_values_4.t7")
  --local saved_joints5 = torch.load("sphere_point_in_joint_values_5.t7")
  --table.insert(saved_joint_values, saved_joints1)
  --table.insert(saved_joint_values, saved_joints2)
  --table.insert(saved_joint_values, saved_joints3)
  --table.insert(saved_joint_values, saved_joints4)
  --table.insert(saved_joint_values, saved_joints5)
  --print(string.format("Number of saved points on sphere: %d \n", #saved_joint_values))

  local sphere_points = {}

  print("To calibrate the end effector (e.g. the gripper tip), it has to be moved to a fixed point from at least 4 different directions.")
  print("Please choose a number of different directions and press return when ready.")
  local nPoints = io.read("*n")
  assert(nPoints > 3)
  print(string.format("Chosen number of directions: %d \n", nPoints))
  io.read()

  for i=1,nPoints do
    print(string.format("Please move end effector (gripper tip) to pose %d. Press return when ready.", i))
  
    if i <= #saved_joint_values then
      local ok, joint_path = motion_service:planJointPath(current_joint_values, saved_joint_values[i], plan_parameters)
      local success, joint_trajectory = motion_service:planMoveJoint(joint_path, plan_parameters)  
      if success == 1 then
        print("Moving ...")
        motion_service:executeJointTrajectory(joint_trajectory, plan_parameters.collision_check)
        print("Movement successfully finished.")
      else
        ros.ERROR("Planning FAILD")
      end
    end

    io.read()

    succ, current_joint_values = motion_service:queryJointState(plan_parameters.joint_names)
    torch.save(string.format("sphere_point_in_joint_values_%d.t7", i), current_joint_values)

    local joint_set = move_group.joint_set
    local joint_values = datatypes.JointValues(joint_set, current_joint_values)
    local error_codes, pose, error_msgs = motion_service:queryPose(move_group_name, joint_values, plan_parameters.joint_names[#plan_parameters.joint_names])
    local sphere_point = torch.DoubleTensor(3)
    sphere_point[1] = pose.pose.position.x
    sphere_point[2] = pose.pose.position.y
    sphere_point[3] = pose.pose.position.z
    print(string.format("Sphere point p%d:", i))
    print(sphere_point)
    table.insert(sphere_points, sphere_point)
  end

  print("Endeffector calibration:")
  print("========================\n")
  local X, sphere_center, sphere_radius = determineSphereFromPointTable(sphere_points)
  
  torch.save("sphere_center.t7", sphere_center)
  torch.save("sphere_radius.t7", sphere_radius)
  
  succ, current_joint_values = motion_service:queryJointState(plan_parameters.joint_names)
  local joint_set = move_group.joint_set
  local joint_values = datatypes.JointValues(joint_set, current_joint_values)
  local error_codes, tcp_pose, error_msgs = motion_service:queryPose(move_group_name, joint_values, plan_parameters.joint_names[#plan_parameters.joint_names])

  local tcp_pose_tf = tf.Transform()
  local quaternion = tf.Quaternion(tcp_pose.pose.orientation.x, tcp_pose.pose.orientation.y, tcp_pose.pose.orientation.z, tcp_pose.pose.orientation.w)
  local origin = torch.DoubleTensor{tcp_pose.pose.position.x, tcp_pose.pose.position.y, tcp_pose.pose.position.z}
  tcp_pose_tf:setRotation(quaternion)  
  tcp_pose_tf:setOrigin(origin)
  tcp_pose_4x4 = tcp_pose_tf:toTensor()
  print("Current TCP pose:") -- as 4x4 matrix (in base coordinates)
  print(tcp_pose_4x4)

  local tip_pose_4x4 = torch.DoubleTensor(4,4)
  --tip_pose_4x4[{{1,3},{1,3}}] = torch.eye(3) -- case 1: tip has same orientation as base
  tip_pose_4x4[{{1,3},{1,3}}] = tcp_pose_4x4[{{1,3},{1,3}}] -- case 2: tip has same orientation as tcp
  tip_pose_4x4[1][4] = sphere_center[1]
  tip_pose_4x4[2][4] = sphere_center[2]
  tip_pose_4x4[3][4] = sphere_center[3]
  tip_pose_4x4[4][1] = 0.0
  tip_pose_4x4[4][2] = 0.0
  tip_pose_4x4[4][3] = 0.0
  tip_pose_4x4[4][4] = 1.0
  print("Current gripper tip pose:") -- as 4x4 matrix (in base coordinates)
  print(tip_pose_4x4)

  print("Current gripper tip pose in TCP coordinates:")
  local tip_pose_in_tcp_coordinates = torch.inverse(tcp_pose_4x4) * tip_pose_4x4
  print(tip_pose_in_tcp_coordinates)
  torch.save("tip_pose_in_tcp_coordinates.t7", tip_pose_in_tcp_coordinates)

  print("To relocate the TCP in the 3D View, add a 'tcp_link' to the file 'robotModel/main.xacro' of your project folder.")
  print("As 'origin xyz' of your tcp_link choose the translation vector of your calculated tip pose in TCP coordinates.")
  print("More precisely, you have to add the following lines: \n")
  print('<link name="tcp_link" />')
  print('<joint name="tcp_joint_F" type="fixed">')
  print('  <child link="tcp_link" />')
  print(string.format('  <parent link="%s" />', move_group_details[move_group_name].end_effector_link_names[1]))
  print(string.format('  <origin xyz="%.6f %.6f %.6f" rpy="0 0 0" />', tip_pose_in_tcp_coordinates[1][4], tip_pose_in_tcp_coordinates[2][4], tip_pose_in_tcp_coordinates[3][4]))
  print('</joint>')
  print("\n")
  print("Now, compile the new main.xacro.")
  print("Then, in the 'Configuration View' under 'MotionPlanning'->'MoveIt!'->'groups'->'<group_name>' select the new 'tcp_link' as 'Tip link'.")
  print("After compiling the new robot configuration, starting ROS and changing into the 'World View', the interactive marker appears at the new tcp link.")

  motion_service:shutdown()
  sp:stop()
  ros.shutdown()
end


main()
