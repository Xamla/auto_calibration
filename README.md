... explanation will be added soon ...

### Run the calibration with already captured images (UR5 setting with extern stereo camera system)

* In Rosvita load a project with UR5 (and gripper), compile it and start ROS.
* Compress and package current local auto_calibration folder: ``tar cvfz auto_calibration.tgz auto_calibration``
* Upload ``auto_calibration.tgz`` to Rosvita
* Remove old auto_calibration folder from Rosvita
* Unpack uploaded auto_calibration folder: ``tar -xzvf auto_calibration.tgz`` (im Rosvita-Terminal)
* Rename **results_german_22_3_2018** into **calibration** (in Rosvita)
* Run the calibration with already captured images: ``th runCalibration.lua -cfg calibration/configurationStereo.t7``
a (Calibrate camera)
s (Save calibration)!!!
b (Hand-eye calibration)
e (Evaluate calibration) Is not possible without robot movement!!!

### Some notes about the result folder structure:
* Captured images will be stored in ``./calibration/capture/``
* Robot poses will be stored in ``./calibration/offline/jsposes.t7``
* Stereo calibration will be stored in ``./calibration/<date>_<time>/stereo_cams_<serial1>_<serial2>.t7`` (e.g. ``./calibration/2018-06-11_102302/stereo_cams_28670151_35371951.t7``)
* For an extern stereo setup, hand-pattern calibration will be stored in ``./calibration/<date>_<time>/HandPattern.t7``, and moreover the pose of the left camera relative to the robot base will be stored in ``./calibration/<date>_<time>/LeftCamBase.t7``.
* For an on-board stereo setup, ... to be continued ...


### Some notes about the pattern detector (in ``multiPattern/PatternLocalisation.lua``) 
* The first detected pattern point (i.e. the **pattern origin**) always is the **top right** pattern point.
  * To see this, run e.g. **th [camPoseCalcForComparison.lua](https://github.com/Xamla/prototyping_altrogge/blob/master/pipette_tip_detection/camPoseCalcForComparison.lua)**, then open the image **[leftrack1_003.png](https://github.com/Xamla/prototyping_altrogge/blob/master/pipette_tip_detection/leftrack1_003.png)** with gimp and compare the first pixel point of e.g. **"point_list_left\[1\]"** (see terminal output) with the pixel coordinates of the top right pattern point of the corresponding pattern in the image.
* The **z-axis** of the calculated camera pose relative to the pattern, always **points in the direction away from the camera**.
  This is valid for all methods, i.e. for the camera pose calculation via "solvePnP", as well as for the camera pose calculation via plane fit.
  * The reason for this is that:
    * The pattern origin is the top right pattern point (see above),
    * The x-axis points from the top right pattern point to the bottom right pattern point (see my camera pose calculation via plane fit in [handPatternCalibration_withTriangulatePointsAndPlanefit.lua](https://github.com/Xamla/prototyping_altrogge/blob/master/calibration/handPatternCalibration_withTriangulatePointsAndPlanefit.lua), lines 415-421 or [rotation_around_pattern_origin.lua](https://github.com/Xamla/prototyping_altrogge/blob/master/calibration/rotation_around_pattern_origin.lua), lines 614-618),
    * The y-axis points from the top right pattern point to the top left pattern point (see my camera pose calculation via plane fit),
    * The z-axis points into the direction of the cross product of the x- and y-axis (see my camera pose calculation via plane fit),
    * All camera pose calculations (via solvePnP or plane fit) give the same result (run e.g. **python3 [test_CamPoseCalculationViaPlaneFit.py](https://github.com/Xamla/prototyping_altrogge/blob/master/pipette_tip_detection/test_CamPoseCalculationViaPlaneFit.py)** and **th [camPoseCalcForComparison.lua](https://github.com/Xamla/prototyping_altrogge/blob/master/pipette_tip_detection/camPoseCalcForComparison.lua)** and compare **"camPoseList_left['3']"**, **"camPoseList_left_viaPlaneFit['3']"** and **"point_list_left[1].pose"**).
