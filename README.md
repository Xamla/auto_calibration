## Autocalibration

Copyright (c) 2018, Xamla and/or its affiliates. All rights reserved.

This package containts scripts for camera and hand-eye calibration.

License information can be found in the LICENSE file.

This distribution may include materials developed by third parties.
For license and attribution notices for these materials, please refer to the LICENSE file.

For more information on Rosvita, visit
  http://xamla.com/en/

#### Major features are:

1. [Camera calibration (single and stereo setup)](#camera-calibration)
2. [Hand-Eye calibration (including evaluation)](#hand-eye-calibration)
3. [End effector calibration](#end-effector-calibration)


#### Calibration pattern requirements:

For all camera and hand-eye calibrations one of our [circle patterns with ids](https://github.com/Xamla/auto_calibration/blob/master/Patterns_with_ID.pdf) (**Patterns_with_ID.pdf**) has to be used.
For a high-quality print of one of these patterns contact http://xamla.com/en/.


### Camera calibration

Calibration of Ximea cameras or GenICam based cameras as single or stereo camera setup mounted onboard (on the robot) or extern can easily be performed by running the scripts **configureCalibration.lua** and **runCalibration.lua**.

In more detail, the **calibration pipeline** is as follows:
* In Rosvita load or create a project (e.g. with UR5 or SDA10D, cameras and gripper), compile it and start ROS.
* With the Rosvita terminal go into your project folder and start the configuration script:
  ```
  cd /home/xamla/Rosvita.Control/projects/<your_project_folder>
  th /home/xamla/Rosvita.Control/lua/auto_calibration/configureCalibration.lua
  ```
  **Note**: To permanently save a configuration, it is important to run the configuration script from your projects folder!
* -> The **configuration main menu** will appear in the terminal. 
  * Now you can select the robot move group, the circle pattern id, the gripper, the camera type, etc. ...  
  * Moreover, you can teach base poses, capture poses and evaluation poses for the calibration. 
  * **Hint**: Base Poses are mainly used for picking a calibration target. If you don't want to pick a calibration target, only teach a start pose and successively press return afterwards for the remaining base poses. 
  * Don't forget to **save** the configuration by **pressing the 's' button**.
* Next run the calibration script from your project folder and with the previously saved configuration: 
  ```
  th /home/xamla/Rosvita.Control/lua/auto_calibration/runCalibration.lua -cfg <name_of_your_saved_configuration_file>.t7
  ```
  **Note**: To permanently save calibration results, it is important to run the calibration script from your projects folder!
* -> The **calibration main menu** will appear in the terminal.
  * Now, simply press:
    * f (Full calibraton cycle)  
  * or press the following sequence:
    * c (Capture calibration images)
    * a (Calibrate camera)
    * s (Save calibration)
    * b (Hand-eye calibration)
    * e (Evaluate calibration)
  * **Note**: Hand-eye calibration will only be possible, if you saved the camera calibration before.
  * **Note**: At the moment, hande-eye calibration only works with stereo camera setups.
  * **Note**: For all calibration processes one of our [circle patterns with ids](https://github.com/Xamla/auto_calibration/blob/master/Patterns_with_ID.pdf) (**Patterns_with_ID.pdf**) is required.
  
### Hand eye calibration

In case of an **onboard camera setup**, the hand-eye calibration detects the transformation (rotation and translation) between the tool center point (tcp) of the robot and a previously calibrated stereo camera system mounted on the robot.

In case of an **extern camera setup**, the calibration pattern has to be mounted on the robot (e.g. grasped by the gripper) and the hand-eye calibration detects the transformation between the tcp and the pattern.

**Note**: At the moment, the hand-eye calibration **requires a stereo camera setup**. Addition of a version working with a single camera, will be added soon.

To be able to perform hand-eye calibration, the camera calibration has to be performed and saved first (see above). <br />
To run the hand-eye calibration, type the following commands into the Rosvita terminal:
```
cd /home/xamla/Rosvita.Control/projects/<your_project_folder>
th /home/xamla/Rosvita.Control/lua/auto_calibration/runCalibration.lua -cfg <name_of_your_saved_configuration_file>.t7
```
Then press
```
* b (Hand-eye calibration)
```
You may want to evaluate your hand-eye calibration by some error metrics to be able to compare it with alternative hand-eye calibrations. Thereto, first you have to teach some tcp poses for evaluation (such that the cameras can capture the pattern from different angles and positions):
```
th /home/xamla/Rosvita.Control/lua/auto_calibration/configureCalibration.lua
* e (Teach poses for evaluation)
```

### End effector calibration

Calibration of an end effector, e.g. a gripper tip can be performed by running the script **endEffectorCalibration.lua**:
```
th /home/xamla/Rosvita.Control/lua/auto_calibration/endEffectorCalibration.lua
```

To calibrate the end effector (e.g. the gripper tip), it has to be moved to a fixed point from at least 4 or better more different directions.

To relocate the tool center point (tcp) to the end effector in Rosvita, add a **tcp_link** to the file **robotModel/main.xacro** of your project folder. As **origin xyz** of your new tcp_link choose the **translation vector of** your calculated **tcp<->end effector transformation**. Then compile the **main.xacro** and adapt your robot configuration ("tip link" of move group and "parent link" of end effector). For more details see the terminal output when running the script.
  
  
### Some notes about the result folder structure of the camera and hand-eye calibration:
* Captured images will be stored in ``./calibration/capture/``
* Robot poses will be stored in ``./calibration/<date>_<time>/jsposes.t7``
* Stereo calibration will be stored in ``./calibration/<date>_<time>/stereo_cams_<serial1>_<serial2>.t7``
* For a single camera setup, camera calibration will be stored in ``./calibration/<date>_<time>/cam_<serial>.t7``
* For an extern stereo setup, hand-pattern calibration will be stored in ``./calibration/<date>_<time>/HandPattern.t7``, and moreover the pose of the left camera relative to the robot base will be stored in ``./calibration/<date>_<time>/LeftCamBase.t7``
* For an on-board stereo setup, hand-eye (with 'eye' = left cam) calibration will be stored in ``./calibration/<date>_<time>/HandEye.t7``, and moreover the pose of the pattern relative to the robot base will be stored in ``./calibration/<date>_<time>/PatternBase.t7``
* Links to robot poses and calibration results will be created in ``./calibration/current/``
