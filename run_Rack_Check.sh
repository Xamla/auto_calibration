#/!bin/bash

python3 Rack_Check.py 'rightrack1_002.png' 'leftrack1_003.png' 'stereo_calibration_as_dict_python3.npy' 'GroundTruthImgWithROIs.png' 'ROIs.npy' 3 5 0.0025 '3,11' 'leftrack1_003_warped.png' 'empty_slots.npy' 'leftrack1_003_warped_with_ROIs.png' 'patDictData.npy'
#python3 test_CamPoseCalculationViaPlaneFit.py 'leftrack1_003.png' 'rightrack1_003.png' 'stereo_calibration_as_dict_python3.npy' 3 5 0.0025 '3,11' 'patDictData.npy'
