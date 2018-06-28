--[[
  ConfigurationCalibration.lua

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

local torch = require 'torch'
local datatypes = require 'xamlamoveit.datatypes.env'
local ConfigurationCalibration = torch.class('ConfigurationCalibration', datatypes)


function ConfigurationCalibration:__init(
    --[[calibration_mode,
    move_group_name,
    cameras,
    left_camera_id,
    right_camera_id,
    output_directory,
    calibration_directory_template,
    calibration_name_template,
    calibration_flags_name,
    circle_pattern_geometry,
    circle_pattern_id,
    checkerboard_pattern_geometry,
    velocity_scaling,
    base_poses,
    capture_poses]]
    configuration)

  self.calibration_mode = configuration.calibration_mode or CalibrationMode.SingleCamera
  self.move_group_name = configuration.move_group_name
  self.cameras = configuration.cameras or {}
  self.left_camera_id = configuration.left_camera_id
  self.right_camera_id = configuration.right_camera_id
  self.output_directory = configuration.output_directory or './calibration/'
  self.calibration_directory_template = configuration.calibration_directory_template or '%Y-%m-%d_%H%M%S/'
  self.calibration_name_template = configuration.calibration_name_template or '%Y-%m-%d_%H%M%S'
  self.calibration_flags_name = configuration.calibration_flags_name or 'Default'
  self.circle_pattern_geometry = configuration.circle_pattern_geometry or torch.Tensor({21, 8, 5.0}) -- rows, cols, pointDist
  self.circle_pattern_id = configuration.circle_pattern_id or 21
  self.checkerboard_pattern_geometry = configuration.checkerboard_pattern_geometry or torch.Tensor({7, 11, 10})
  self.velocity_scaling = configuration.velocity_scaling or 0.2
  self.base_poses = configuration.base_poses or {}
  self.capture_poses = configuration.capture_poses or {}
  self.camera_outputs = {} --this will contain paths to store the data for each individual camera
end


function ConfigurationCalibration:fromTable(t)
  assert(type(t) == 'table', 'Source table argument must not be nil.')
  for k, v in pairs(t) do
    self[k] = v
  end
end


function ConfigurationCalibration:toTable()
  return {
    calibration_mode = self.calibration_mode,
    move_group_name = self.move_group_name,
    cameras = self.cameras,
    left_camera_id = self.left_camera_id,
    right_camera_id = self.right_camera_id,
    output_directory = self.output_directory,
    calibration_directory_template = self.calibration_directory_template,
    calibration_name_template = self.calibration_name_template,
    calibration_flags_name = self.calibration_flags_name,
    circle_pattern_geometry = self.circle_pattern_geometry,
    circle_pattern_id = self.circle_pattern_id,
    checkerboard_pattern_geometry = self.checkerboard_pattern_geometry,
    velocity_scaling = self.velocity_scaling,
    base_poses = self.base_poses,
    capture_poses = self.capture_poses
  }
end


function ConfigurationCalibration:clone()
  local result = ConfigurationCalibration.new()
  result:fromTable(self:toTable())
  return result
end


function ConfigurationCalibration:__tostring()
  local res = 'ConfigurationCalibration:\n'
  for k, v in pairs(self:toTable()) do
    if type(v) == 'table'then
      local str_table = ''
      for ii,vv in ipairs(v) do
        str_table = string.format('%s %s', str_table, tostring(vv))
      end
      res = string.format('%s\t %s:\t %s\n', res, k, str_table)
    elseif torch.isTypeOf(v, torch.DoubleTensor) then
      local str = ''
      for ii = 1, v:size(1) do
        str = string.format('%s %s', str, tostring(v[ii]))
      end
      res = string.format('%s\t %s:\t %s\n', res, k, str)
    else
      res = string.format('%s\t %s:\t %s\n', res, k, tostring(v))
    end
  end
  return res
end


function ConfigurationCalibration:createOutputDirectories()
  local alt_directory = os.date(self.calibration_directory_template)
  local alt_output_directory = path.join(self.output_directory, alt_directory)
  print('creating directory.. '..alt_output_directory)
  os.execute('mkdir -p ' .. alt_output_directory)
  --we need one directory inside alt_output_directory for each available camera
  local camera_serials = {}
  for key, value in pairs(self.cameras) do
    self.camera_outputs[value.serial] = {path, path_capture}
    camera_serials[#camera_serials + 1] = value.serial
    local camera_directory = path.join(alt_output_directory, value.serial)
    print('creating directory.. '..camera_directory)
    self.camera_outputs[value.serial].path = camera_directory
    os.execute('mkdir -p ' .. camera_directory)
    local images_output_directory = path.join(camera_directory, 'capture')    
    print('creating directory.. '..images_output_directory)        
    os.execute('mkdir -p ' .. images_output_directory)
    self.camera_outputs[value.serial].path_capture = images_output_directory
  end
  print('self.camera_outputs=')
  print(self.camera_outputs)
end


function ConfigurationCalibration:debugOutputDirs()
  for key, value in pairs(self.camera_outputs) do
    print(key,value)
  end
end


function ConfigurationCalibration:getCameraDataOutputPath(serial)
  return self.camera_outputs[serial].path_capture
end


function ConfigurationCalibration:getCameraCalibrationFileOutputPath(serial)
  return path.join(self.camera_outputs[serial].path,'calibration.t7')
end


function ConfigurationCalibration:getSerialFromId(id)
  return self.cameras[id].serial
end


return ConfigurationCalibration
