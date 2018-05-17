Example images for running **Rack_Check.py** can be found e.g. [here](https://github.com/Xamla/prototyping_altrogge/tree/master/pipette_tip_detection): <br /> 
* Ground truth image: **rightrack1_002.png**
* Image to be analyzed: **leftrack1_003.png**

Example images for running **test_CamPoseCalculationViaPlaneFit.py** again can be found e.g. [here](https://github.com/Xamla/prototyping_altrogge/tree/master/pipette_tip_detection): <br />
* Left image: **leftrack1_003.png**
* Right image: **rightrack1_003.png**

**Note:** 
* The first detected pattern point (i.e. the **pattern origin**) always is the **top right** pattern point.
  This is valid for both pattern detector methods, i.e. for the old pattern detector, which uses clustering and "findCirclesGrid", as well as for the new, modified pattern detector, which uses "findCirclesGrid" and masking of already found patterns.
* The **z-axis** of the calculated camera pose relative to the pattern, always **points in the direction away from the camera**.
  This is valid for all methods, i.e. for the camera pose calculation via "solvePnP" (using the old or the modified pattern detector), as well as for the camera pose calculation via plane fit.
