local autoCalibration = {}

local BASE_POSE_NAMES = {
  'start',
  'pre_pick_marker',
  'pick_marker',
  'post_pick_marker',
  'camera1_base'
}
autoCalibration.BASE_POSE_NAMES = BASE_POSE_NAMES


local CalibrationMode = {
  SingleCamera = 'SingleCamera',
  StereoRig = 'StereoRig',
  StructuredLightSingleCamera = 'StructuredLightSingleCamera',
}
autoCalibration.CalibrationMode = CalibrationMode


local DEFAULT_CALIBRATION_FLAGS = {
  CALIB_USE_INTRINSIC_GUESS  = false,    -- cameraMatrix contains valid initial values of fx, fy, cx, cy that are optimized further. Otherwise, (cx, cy) is initially set to the image center (imageSize is used), and focal distances are computed in a least-squares fashion. Note, that if intrinsic parameters are known, there is no need to use this function just to estimate extrinsic parameters. Use solvePnP() instead.
  CALIB_FIX_PRINCIPAL_POINT  = false,    -- The principal point is not changed during the global optimization. It stays at the center or at a different location specified when CV_CALIB_USE_INTRINSIC_GUESS is set too.
  CALIB_FIX_ASPECT_RATIO     = false,    -- The functions considers only fy as a free parameter. The ratio fx/fy stays the same as in the input cameraMatrix . When CV_CALIB_USE_INTRINSIC_GUESS is not set, the actual input values of fx and fy are ignored, only their ratio is computed and used further.
  CALIB_ZERO_TANGENT_DIST    = true,     -- Tangential distortion coefficients (p_1, p_2) are set to zeros and stay zero.
  CALIB_FIX_K1               = false,
  CALIB_FIX_K2               = false,
  CALIB_FIX_K3               = true,
  CALIB_RATIONAL_MODEL       = false,
  CALIB_FIX_K4               = true,
  CALIB_FIX_K5               = true,
  CALIB_FIX_K6               = true
}


local NO_DISTORTION_CALIBRATION_FLAGS = {
  CALIB_USE_INTRINSIC_GUESS  = false,
  CALIB_FIX_PRINCIPAL_POINT  = false,
  CALIB_FIX_ASPECT_RATIO     = false,
  CALIB_ZERO_TANGENT_DIST    = true,
  CALIB_FIX_K1               = true,
  CALIB_FIX_K2               = true,
  CALIB_FIX_K3               = true,
  CALIB_RATIONAL_MODEL       = false,
  CALIB_FIX_K4               = true,
  CALIB_FIX_K5               = true,
  CALIB_FIX_K6               = true
}


local ONLY_K1_DISTORTION_CALIBRATION_FLAGS = {
  CALIB_USE_INTRINSIC_GUESS  = false,
  CALIB_FIX_PRINCIPAL_POINT  = false,
  CALIB_FIX_ASPECT_RATIO     = false,
  CALIB_ZERO_TANGENT_DIST    = true,
  CALIB_FIX_K1               = false,
  CALIB_FIX_K2               = true,
  CALIB_FIX_K3               = true,
  CALIB_RATIONAL_MODEL       = false,
  CALIB_FIX_K4               = true,
  CALIB_FIX_K5               = true,
  CALIB_FIX_K6               = true
}


local FULL_CALIBRATION_FLAGS = {
  CALIB_USE_INTRINSIC_GUESS  = false,
  CALIB_FIX_PRINCIPAL_POINT  = false,
  CALIB_FIX_ASPECT_RATIO     = false,
  CALIB_ZERO_TANGENT_DIST    = false,
  CALIB_FIX_K1               = false,
  CALIB_FIX_K2               = false,
  CALIB_FIX_K3               = false,
  CALIB_RATIONAL_MODEL       = false,
  CALIB_FIX_K4               = true,
  CALIB_FIX_K5               = true,
  CALIB_FIX_K6               = true
}


local CalibrationFlags = {
  Default = DEFAULT_CALIBRATION_FLAGS,
  NoDistortion = NO_DISTORTION_CALIBRATION_FLAGS,
  OnlyK1Distortion = ONLY_K1_DISTORTION_CALIBRATION_FLAGS,
  Full = FULL_CALIBRATION_FLAGS
}
autoCalibration.CalibrationFlags = CalibrationFlags


return autoCalibration
