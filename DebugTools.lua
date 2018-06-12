local ros = require 'ros'
local tf = ros.tf
local cv = require "cv"

local pcl = require 'pcl'
local ros = require 'ros'
local tf = ros.tf

require 'ros.PointCloud2SerializationHandler'


local debugTools = {}


local DebugTools = torch.class('autoCalibration.DebugTools', debugTools)

--
-- Class for publishing tfs, images on ROS
-- image_topics = {"img_topic_1", "img_topic_2", ...}
--
function DebugTools:__init(point_cloud_topics, image_topics)

  self.default_img_topic = 'debug_img_1'
  self.default_point_cloud_topic = 'point_cloud'
  image_topics = image_topics or {self.default_img_topic}
  point_cloud_topics = point_cloud_topics or {self.default_point_cloud_topic}
  ros.init('debug_tools')
  self.spinner = ros.AsyncSpinner()
  self.spinner:start()
  self.nh = ros.NodeHandle()

  self.handler = ros.PointCloud2SerializationHandler.new()
  self.nh:addSerializationHandler(self.handler)

  self.listener = tf.TransformListener.new()
  self.broadcaster = tf.TransformBroadcaster.new()
  self.img_publisher_1 = self.nh:advertise(self.default_img_topic, 'sensor_msgs/Image', 10)
  self.publisher_cloud_1 = self.nh:advertise(self.default_point_cloud_topic, 'sensor_msgs/PointCloud2', 10)

  -- iterate over image topics
  self.img_publishers = {}
  for _, topic in ipairs(image_topics) do
    print('Creating img publisher on topic ', topic)
    self.img_publishers[topic] = self.nh:advertise(topic, 'sensor_msgs/Image', 10)
  end

  self.pcloud_publishers = {}
  for _, topic in ipairs(point_cloud_topics) do
    print('Creating point cloud publisher on topic ', topic)
    self.pcloud_publishers[topic] = self.nh:advertise(topic, 'sensor_msgs/PointCloud2', 10)
  end

end


function DebugTools:publishCloud(cloud, frame_id, topic)

  topic = topic or self.default_point_cloud_topic
  cloud:setHeaderFrameId(frame_id)

  if self.pcloud_publishers[topic] == nil then
    self.publisher_cloud_1:publish(cloud)
  else
    self.pcloud_publishers[topic]:publish(cloud)
  end

  ros.Duration(0.1):sleep()
end


function DebugTools:requestTf(source_frame_id, target_frame_id)

  local available = self.listener:waitForTransform(target_frame_id, source_frame_id, ros.Time(0), ros.Duration(3))
  if available then
    local H_tf
    H_tf = self.listener:lookupTransform(target_frame_id, source_frame_id, ros.Time(0), H_tf)
    return available, H_tf
  end
  return available, nil
end


function DebugTools:publishTf(H, frame_id, child_frame_id)
  transf = tf.Transform.new()
  transf:fromTensor(H)

  stampedTransf = tf.StampedTransform.new(
      transf,
      ros.Time.getNow(),
      frame_id,
      child_frame_id)

  self.broadcaster:sendTransform(stampedTransf)
  ros.Duration(0.1):sleep()
  ros.spinOnce()
  return stampedTransf
end


function DebugTools:sleep(s)
  ros.Duration(s):sleep()
  ros.spinOnce()
end


function DebugTools:publishShiftTf(point_4d, frame_id)
  transf = tf.Transform.new()
  H = torch.eye(4,4)
  H[1][4] = point_4d[1]
  H[2][4] = point_4d[2]
  H[3][4] = point_4d[3]
  transf:fromTensor(H)
  child_frame_id = frame_id..'_shift'

  stampedTransf = tf.StampedTransform.new(
      transf,
      ros.Time.getNow(),
      frame_id,
      child_frame_id)

  self.broadcaster:sendTransform(stampedTransf)
  ros.Duration(0.1):sleep()
  ros.spinOnce()

end


--publish an opencv img as a ROS msg
function DebugTools:publishImg(img, topic)

  topic = topic or self.default_img_topic
  local msg = ros.Message('sensor_msgs/Image')
  msg.height = img:size(1)
  msg.width = img:size(2)
  msg.encoding = "bgr8"
  msg.is_bigendian = false
  msg.step = img:stride(1)
  msg.data = img:reshape(msg.height * msg.width * 3)
  if self.img_publishers[topic] == nil then
    self.img_publisher_1:publish(msg)
  else
    self.img_publishers[topic]:publish(msg)
  end
  ros.Duration(0.1):sleep()
  ros.spinOnce()
end

return DebugTools
