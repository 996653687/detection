# Detection
Detection, filter and tracking moving objects experiments (Tested on OS X)

Prerequisite:
G++
Jupyter (https://jupyter.org)

Main interface:
setup_visualization.ipynb

Code structure:
main.cpp - main pipeline
Detector* - object detection in frames of images (video) given template
KalmanFilter* - kalman filter (2D, constant 1st derivative so far) used to filter detection in each frame and estiamate states (position)

Test data:
Under ./data/test_data/NAME, must be named in ascending order according to time stamps (01.jpg, 02.jpg,...)

Template:
./data/test_data/NAME/template/template.jpg


Example Usage:
 
0 - Prepares test image frames and template.

1 - Runs PREPROCESS part of the setup_visualization script to converts images into csv (under ./data/test_data/NAME/csv/).

2 - compiles and runs detections on the test images:

     ./a.out [BASE_PATH] [OUTPUT_SUBDIR] [DOWNSAMPLE_SCALE] [TOP_K_PARTICLES]" << std::endl;
     
       BASE_PATH - where test images and templates are stored;
       OUTPUT_SUBDIR - subdirectory to store outputs;
       DOWNSAMPLE_SCALE - scale to downsample images and templates;
       TOP_K_PARTICLES - the number of top convolution response pixels used to improve detection and initialize KalmanFilter;
       
     e.g. g++ main.cpp Detector.cpp KalmanFilter.cpp Util.cpp  -std=c++2a && ./a.out ./data/test_data/ball/ output/ 8 20
     
3 - Runs VISUALIZE part of the setup_visualization script to debug the detection/filtering results

