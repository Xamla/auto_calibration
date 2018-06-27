### Calibration Pipeline:

* In Rosvita load or create a project (e.g. with UR5 or SDA10D, cameras and gripper), compile it and start ROS.
* With the Rosvita terminal go into your project folder and start the configuration script:
  ```
  cd /home/xamla/Rosvita.Control/projects/<your_project_folder>
  th ../../lua/auto_calibration/configureCalibration.lua
  ```
  **Note**: To permanently save a configuration, it is important to run the configuration script from your projects folder!
* -> The **configuration main menu** will appear in the terminal. 
  * Now you can select the robot move group for calibration, the circle pattern id, the gripper, the camera type, etc. ...  
  * Moreover, you can teach base poses, capture poses and evaluation poses for the calibration. 
  * **Hint**: Base Poses are mainly used for picking a calibration target. If you don't want to pick a calibration target, only teach a start pose and successively press return afterwards for the remaining base poses. 
  * Don't forget to **save** the configuration by **pressing the 's' button**.
* Next run the calibration script from your project folder and with the previously saved configuration: 
  ```
  th ../../lua/auto_calibration/runCalibration.lua -cfg <name_of_your_saved_configuration_file>.t7
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

### Some notes about the result folder structure:
* Captured images will be stored in ``./calibration/capture/``
* Robot poses will be stored in ``./calibration/offline/jsposes.t7``
* Stereo calibration will be stored in ``./calibration/<date>_<time>/stereo_cams_<serial1>_<serial2>.t7`` (e.g. ``./calibration/2018-06-11_102302/stereo_cams_28670151_35371951.t7``)
* For an extern stereo setup, hand-pattern calibration will be stored in ``./calibration/<date>_<time>/HandPattern.t7``, and moreover the pose of the left camera relative to the robot base will be stored in ``./calibration/<date>_<time>/LeftCamBase.t7``.
* For an on-board stereo setup, ... to be continued ...
