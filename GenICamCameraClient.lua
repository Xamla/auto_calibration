--[[
  GenICamCameraClient.lua

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
local ros = require 'ros'
local cv = require 'cv'
require 'cv.imgproc'

local autocal = require 'auto_calibration.env'
local GenICamCameraClient = torch.class('GenICamCameraClient', autocal)

function GenICamCameraClient:__init(nodeHandle, mode, permute_channels, rgb_conversion, persistent)

  nodeHandle = nodeHandle or ros.NodeHandle()

  self.mode = mode or "genicam_stereo"
  local captureServiceName = '/camera_aravis_node/capture'
  local sendCommandServiceName = '/camera_aravis_node/send_command'

  self.captureClient = nodeHandle:serviceClient(captureServiceName, ros.SrvSpec('camera_aravis/Capture'), persistent)
  self.sendCommandClient = nodeHandle:serviceClient(sendCommandServiceName, ros.SrvSpec('camera_aravis/SendCommand'), persistent)
  self.permute_channels = permute_channels or false
  self.rgb_conversion = rgb_conversion or true

  local timeout = ros.Duration(5)
  local ok = self.captureClient:waitForExistence(timeout) and self.sendCommandClient:waitForExistence(timeout)
  if not ok then
    error('camera_aravis ROS node not running.')
  end
  -- check if services are valid (e.g. persistent services might require reconnect when service initially was not available)
  if not self.captureClient:isValid() then
    self.captureClient:shutdown()
    self.captureClient = nodeHandle:serviceClient(captureServiceName, ros.SrvSpec('camera_aravis/Capture'), persistent)
  end
  if not self.sendCommandClient:isValid() then
    self.sendCommandClient:shutdown()
    self.sendCommandClient = nodeHandle:serviceClient(sendCommandServiceName, ros.SrvSpec('camera_aravis/SendCommand'), persistent)
  end

  assert(self.captureClient:isValid() and self.sendCommandClient:isValid())
end


function GenICamCameraClient:shutdown()
  self.captureClient:shutdown()
  self.sendCommandClient:shutdown()
end


local function msg2image(self, m)
  if m == nil then
     return nil
  end
  local img
  if m.encoding == "rgb8"  then
    img = torch.ByteTensor(m.width * m.height * 3)
    img:copy(m.data)
    img = img:reshape(m.height, m.width, 3)
    if self.permute_channels then
      img = img:permute(3,1,2)
    end
  elseif m.encoding == "bgr8" then
    local imgbgr = torch.ByteTensor(m.width * m.height * 3)
    imgbgr:copy(m.data)
    imgbgr = imgbgr:reshape(m.height, m.width, 3)
    if self.rgb_conversion then
      img = cv.cvtColor{imgbgr, nil, cv.COLOR_BGR2RGB}
    else
      img = imgbgr
    end
    if self.permute_channels then
      img = img:permute(3,1,2)
    end
  elseif m.encoding == "mono8" then
    img = m.data:view(m.height, m.width)
  elseif m.encoding == "mono16" then
    img = torch.ByteTensor(m.width * m.height, 2)
    img:copy(m.data)
    img = img:reshape(m.height, m.width, 2)
    if self.permute_channels then
      img = img:transpose(1,3)
    end
  end
  return img
end


local function sendCommand(self, command_name, value, serials)
  if type(serials) == 'string' then
    serials = { serials }
  end
  local req = self.sendCommandClient:createRequest()
  req.command_name = command_name
  req.serials = serials or {}
  req.value = value or 0
  self.sendCommandClient:call(req)
end


function GenICamCameraClient:setExposure(exposure_micro_sec, serials)
  if type(serials) == 'string' then
    serials = { serials }
  end
  sendCommand(self, "ExposureTime", exposure_micro_sec, serials or {})
end


function GenICamCameraClient:getImage(index)
  local response = self.capture:call()
  return msg2image(self, response.images[index]), response.serials[index]
end


function GenICamCameraClient:capture(serials)
  local req = self.captureClient:createRequest()
  if type(serials) == 'string' then
    serials = { serials }
  end
  req.serials = serials or {}
  return self.captureClient:call(req)
end


function GenICamCameraClient:getImages(serials)
  local response = nil
  while response == nil do
    response = self:capture(serials)
  end
  assert(#response.images>0, "get images failed")
  return msg2image(self, response.images[1]), msg2image(self, response.images[2]), msg2image(self, response.images[3]), response.serials
end

return GenICamCameraClient
