--[[
  endEffectorCalibration_env.lua

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


-- End effector (e.g. gripper tip) calibration
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

--Create motion service
local motion_service = motionLibrary.MotionService(nh)

-- Determine a sphere that fits best through all given points.
-- Least squares solution to: min ||AX-B||_F
function determineSphereFromPointTable(spherePoints)

  assert(#spherePoints > 3)
  -- Note: At least 4 points have to be not on a line or plane
  --       (i.e. matrix A has to be of full rank)

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
  local end_effector = move_group:getEndEffector()

  -- If we have already saved joint sets:
  local joint_names = move_group:getJointNames()
  local saved_joint_values = {}
  --for i = 1,8 do
  --  local jv = torch.load(string.format("sphere_point_in_joint_values_%d.t7", i))
  --  saved_joint_values[#saved_joint_values + 1] = datatypes.JointValues(datatypes.JointSet(joint_names), jv)
  --end
  --print(string.format("Number of saved points on sphere: %d \n", #saved_joint_values))

  local sphere_points = {}

  print("To calibrate the end effector (e.g. the gripper tip), it has to be moved to a fixed point from at least 4 different directions.")
  print("Please choose a number of different directions and press return when ready.")
  local nPoints = io.read("*n")
  io.read()
  assert(nPoints > 3)
  print(string.format("Chosen number of directions: %d \n", nPoints))
  
  for i=1,nPoints do

    --[[
    print(string.format("Please move end effector (gripper tip) to pose %d. Press return when ready.", i))
    if i <= #saved_joint_values then
      print("Moving ...")
      move_group:moveJoints(saved_joint_values[i], 0.03, true)
      print("Movement successfully finished.")
      sys.sleep(1)
    end
    io.read()
    succ, current_joint_values = motion_service:queryJointState(plan_parameters.joint_names)
    --torch.save(string.format("sphere_point_in_joint_values_%d.t7", i), current_joint_values)
    local joint_set = move_group.joint_set
    local joint_values = datatypes.JointValues(joint_set, current_joint_values)
    local pose = end_effector:computePose(joint_values)
    ]]
    local pose
    if i <= #saved_joint_values then
      print(string.format("Pose %d is already saved.", i))
      pose = end_effector:computePose(saved_joint_values[i])
    else
      print(string.format("Please move end effector (gripper tip) to pose %d. Press return when ready.", i))
      io.read()
      succ, current_joint_values = motion_service:queryJointState(plan_parameters.joint_names)
      torch.save(string.format("sphere_point_in_joint_values_%d.t7", i), current_joint_values)
      local joint_set = move_group.joint_set
      local joint_values = datatypes.JointValues(joint_set, current_joint_values)
      pose = end_effector:computePose(joint_values)
    end

    local sphere_point = pose:getTranslation()
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
  local pose = end_effector:computePose(joint_values)
  tcp_pose_4x4 = pose.stampedTransform:toTensor()
  print("Current TCP pose:") -- as 4x4 matrix (in base coordinates)
  print(tcp_pose_4x4)

  local tip_pose_4x4 = torch.DoubleTensor(4,4)
  tip_pose_4x4[{{1,3},{1,3}}] = tcp_pose_4x4[{{1,3},{1,3}}] -- tip has same orientation as tcp
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

  print("To relocate the TCP in the 3D View, add a \'tcp_link\' to the file \'robotModel/main.xacro\' of your project folder.")
  print("As \'origin xyz\' of your tcp_link choose the translation vector of your calculated tip pose in TCP coordinates.")
  print("More precisely, you have to add the following lines inside the <robot> </robot> environment: \n")
  print('<link name=\"tcp_link\" />')
  print('<joint name=\"tcp_joint_F\" type=\"fixed\">')
  print('  <child link=\"tcp_link\" />')
  print(string.format('  <parent link=\"%s\" />', move_group_details[move_group_name].end_effector_link_names[1]))
  print(string.format('  <origin xyz=\"%.6f %.6f %.6f\" rpy=\"0 0 0\" />', tip_pose_in_tcp_coordinates[1][4], tip_pose_in_tcp_coordinates[2][4], tip_pose_in_tcp_coordinates[3][4]))
  print('</joint>')
  print("\n")
  print("Now, compile the new main.xacro.")
  print("Then, in the \'Configuration View\' once press compile to make the \'tcp_link\' selectable.")
  print("In the \'Configuration View\' under \'MotionPlanning\'->\'MoveIt!\'->\'groups\'->\'<group_name>\' select the new \'tcp_link\' as \'Tip link\'.")
  print("Moreover, under \'MotionPlanning\'->\'MoveIt!\'->\'endEffectors\'->\'<end effector name>\' select the new \'tcp_link\' as \'Parent link\'.")
  print("After compiling the new robot configuration, starting ROS, changing into the 'World View' and choosing the corresponding move group and end effector, the interactive marker appears at the new tcp link.")
  print("\n")
  print("Note: Precision of the new \'tcp_link\' depends on how precisely the gripper tip had been moved to a fixed point from different directions.")

  motion_service:shutdown()
  sp:stop()
  ros.shutdown()
end


main()
