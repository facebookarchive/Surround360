# Surround 360 Calibration

In order to produce a more accurate and comfortable result in VR, the Surround 360 rendering software uses several calibration config files to correct optical and mechanical issues. This document describes the process of generating the calibration config files.

WARNING: you should not attempt to render videos captured with Surround 360 without first reading this document. Uncalibrated results may be severely distorted in VR to the point of breaking stereo perception of 3D.

There are three specialized calibration processes: intrinsic calibration to correct barrel distortion, stereo rectification for the side cameras, and optical center for the fisheye lenses at the top and bottom. Each part of calibration requires different data and different steps to process with our software.

## Rectification

The purpose of the rectification step is to align everything so that both eyes see every point in the scene at the same y-coordinate. This is the most important step in calibration; skipping this step will likely result in stereo which causes eye-strain or which cannot be fused as 3D at all. Fortunately, this step can be done with any unstructured data (if you captured a single frame of video, you have the necessary data). Rectification must be done once per rig, but can be left alone for subsequent scenes captured with the same rig.

The config file for rectification has a .yml extension. It is passed to run_all.py via the --rectify_file command-line arugment, or "rectification file" in the GUI.

For fast preview rendering modes, rectification may be skipped, and it is allowed to pass "NONE" for --rectify_file.

The process for rectification is somewhat involved; we will simplify it and provide a GUI in the near future. For now, follow these steps:

* We will assume that we already have a dataset captured, and that it has gone through the 'unpack', 'arrange', and 'isp' steps (at least for 1 frame). For the sake of this example, we will assume everything has been unpacked to a folder located at ~/Desktop/palace1 ('palace1' is the name of the scene we are rendering, as in, "Palace of Fine Arts"). In the 'arrange' step of the pipeline, several subdirectories should be created, such as
  * ~/Desktop/palace1/vid
  * ~/Desktop/palace1/logs
  * etc ...

* In this example we will use frame 000000 for rectification (this needs to be done with 1 or more frames of sample data, but it can then be applied to the whole video, or other videos captured with the same rig).

* Navigate to ~/Desktop/palace1/vid/000000. It should contain the following subdirectories:
  * /flow
  * /flow_images
  * /isp_out
  * /projections
  * /raw

* Make sure the /isp_out directory contains images named cam0.png, cam1.png, ... up to cam16.png. If these files are missing, it means the 'isp' step hasn't been run. You must run the 'isp' step for at least one frame before proceeding with rectification, although color adjustment is not required.

* Under /000000, create several additional empty folders (we will fill these with data in the next steps):
  * /undistorted
  * /undistorted_features
  * /warped
  * /keypoint_vis
  * /4point

* The rectification calibrator expects as input, images which have no barrel distortion. We will generate those images by undistorting the contents of /isp_out, and write the results to /undistorted. cd to /surround360_render, then run the following command:
<pre>
./bin/TestIntrinsicCalibration \
--logbuflevel -1 --stderrthreshold 0 --v 0 \
--mode undistort \
--src_intrinsic_param_file ./res/config/sunex_intrinsic.xml \
--src_fisheye_img_dir ~/Desktop/palace1/vid/000000/isp_out \
--dest_undistorted_dir ~/Desktop/palace1/vid/000000/undistorted \
--resize_width 2048 \
--resize_height 2048
</pre>
After running this command, /undistorted should contain a new version of cam0.png, ..., cam16.png.

* Next, we will prepare a folder of data to be used for rectification. It is possible to use multiple frames of data for rectification, but in this example we will use only one frame. Under /undistorted_features, create a new folder named /00. The full path to the new folder should be ~/Desktop/palace1/vid/000000/undistorted_features/00.

* Copy all of the files cam0.png, ..., cam16.png from ~/Desktop/palace1/vid/000000/undistorted to ~/Desktop/palace1/vid/000000/undistorted_features/00.

* NOTE: here /00 contains the first frame of data for rectification. If we were using multiple frames, the data for these would go in /01, /02, ... etc.

* Delete cam0.png, cam15.png, and cam16.png from undistorted_features/00 (DO NOT delete them from /undistorted, only delete them from /undistorted_features/00). We are deleting these because cam0, cam15, and cam16 are fisheye lenses at the top and bottom, not side cameras. These are not part of rectificaiton.

* Now we are ready to run the rectification code. cd to /surround360_render, then run the following command:
<pre>
./bin/TestRingRectification \
--logbuflevel -1 --stderrthreshold 0 --v 0 \
--rig_json_file ./res/config/17cmosis_default.json \
--src_undistorted ~/Desktop/palace1/vid/000000/undistorted \
--src_undistorted_features ~/Desktop/palace1/vid/000000/undistorted_features \
--match_vis_dir ~/Desktop/palace1/vid/000000/keypoint_vis \
--four_pt_vis_dir ~/Desktop/palace1/vid/000000/4point \
--warped_output_dir ~/Desktop/palace1/vid/000000/warped \
--output_transforms_file ~/Desktop/rectify_my_rig.yml
</pre>

* Once the command above finishes, a new file is created at ~/Desktop/rectify_my_rig.yml. This is the file you need to pass in to run_all.py for "rectification file" (GUI) or --rectify_file (command-line). You may wish to copy this file to /surround360_render/res/config.

* That is all you need to do, but some interesting visualizations are generated in /keypoint_vis and /warped. For example, /warped/stacked.png shows all of the side cameras stacked horizontally into one big image, after rectification. If you draw a horizontal line across stacked.png, you should see everything lining up between adjacent images.

## Optical Centering

The idea of optical centering is applicable to all lenses, but in the current Surround 360 rendering pipeline, we only handle it as a special case for the top and bottom fisheye lenses. The Fujinon 185 degree lens creates a circle image on the CMOSIS sensor, and that circle is completely contained within the sensor (i.e. it is not "full-frame"). Unfortunately, even with high-end sensors and lenses, the center of the circle shifts around by ~40 pixels between different lenses. Results in VR are quite sensitive to these shifts, and if not corrected, it tends to make objects that should be straight look twisted.

* Before starting, you will need to capture an image with each of the fisheye cameras (one top, two bottom), where a piece of diffusing material such as lens-cleaning cloth is placed over the lens, then the lens is illuminated with a bright light. An example of such an image can be found in /surround360_render/res/example_data/test_fisheye_diffuse.png.

* For this step, we will assume you have data at ~/Desktop/optical_center_calib. Copy the following files to ~/Desktop/optical_center_calib:
  * /surround360_render/res/example_data/test_fisheye_diffuse.png
  * /surround360_render/res/config/17cmosis_default.json

* We will pretend that test_fisheye_diffuse.png is an image captured with cam0 in the rig (i.e. the top camera). In practice, you need to repeat these steps for cam0, cam15, and cam16.

* cd to /surround360_render and run the following command:
<pre>
./bin/TestIntrinsicCalibration \
--logbuflevel -1 --stderrthreshold 0 --v 0 \
--mode fisheye_optical_center \
--fisheye_optical_center_threshold 128 \
--fisheye_optical_center_src_image ~/Desktop/optical_center_calib/test_fisheye_diffuse.png \
--vis_output_dir ~/Desktop/optical_center_calib
</pre>

* This generates a new file named mask.png in ~/Desktop/optical_center_calib. This should look like a white circle on a black background, and should be very close to a perfect circle. If the output does not look like a circle, you may need to adjust the --fisheye_optical_center_threshold parameter, or capture better input data, making the illumination of the diffusing material as uniform as possible.

* When you run TestIntrinsicCalibration with the command above, it should output some text on the terminal like this:
<pre>
I0711 15:14:01.539598 2031423488 IntrinsicCalibration.cpp:386] radius = 810.065
I0711 15:14:01.540668 2031423488 IntrinsicCalibration.cpp:387] centerOfMassX = 1021.78
I0711 15:14:01.540685 2031423488 IntrinsicCalibration.cpp:388] centerOfMassY = 1010.87
</pre>
Note the values for centerOfMassX and centerOfMassY. This is the estimated coordinate of the optical center for this lens.

* Rename the file 17cmosis_default.json to 17cmosis_calibrated.json in ~/Desktop/optical_center_calib. Open 17cmosis_calibrated.json with a text editor. Find lines 6 and 7, which should look like:
<pre>
  "image_center_x": 1024,
  "image_center_y": 1024,
</pre>
These lines specify the optical center for camera 'cam0'. Replace the default values of 1024 with the output from running TestIntrinsicCalibration. For example, if the output was centerOfMassX = 1021.78 and centerOfMassY = 1010.87, then you should change lines 6 and 7 of 17cmosis_calibrated.json to:
<pre>
  "image_center_x": 1021.78,
  "image_center_y": 1010.87,
</pre>

* NOTE: do not change the radius parameters in the json file unless you know what you are doing. 781 pixels and 185 degrees are the correct values for the Fujinon lens on the CMOSIS sensor in our reference design.

* Next, we will generate a visualization to test whether these results are plausible. We will need a frame of images for testing purposes, i.e., find the /isp_out folder in one frame. For this example command, lets assume these images are located at ~/Desktop/palace1/vid/000000/isp_out.

* cd to /surround360_render and run the following command:
<pre>
./bin/TestOpticalCenterVisualization \
--logbuflevel -1 --stderrthreshold 0 --v 0 \
--rig_json_file ~/Desktop/optical_center_calib/17cmosis_calibrated.json \
--images_dir ~/Desktop/palace1/vid/000000/isp_out \
--vis_output_dir ~/Desktop/optical_center_calib
</pre>

* The above command generates 3 new images in ~/Desktop/optical_center_calib, named top_vis.png, bottom_vis.png, and bottom2_vis.png. These visualization images show a crosshair target on top of the original image, which indicates the optical center that is specified in the rig's .json file.

* So far, we have updated the optical center parameters for cam0, which is the top camera. You must repeat the steps above starting with running TestIntrinsicCalibration for the two bottom cameras as well, and update the corresponding lines of 17cmosis_calibrated.json.

* After finishing, ~/Desktop/optical_center_calib/17cmosis_calibrated.json is your new calibrated rig specification .json file. This should be passed to run_all.py for the "Rig Geometry File" field (GUI), or --rig_json_file (command-line).

## Intrinsic / Barrel Distortion

This step is only required if you are using different lenses or sensors than our reference design! If you are using the reference Sunex lenses and PointGrey camera/CMOSIS sensor, the necessary calibration data is already provided, and you don't need to do anything more for this step. The pre-made calibration config is located at:

/surround360_render/res/config/sunex_intrinsic.xml

This file should be passed to run_all.py in the "intrinsic parameters file" field via the GUI, or the --src_intrinsic_param_file argument via the command line.

* Assuming you need to compute intrinsic calibration for a different lens/sensor combination, you will need a collection of images of checkerboards in different poses. We use at least 40 images. The checkerboard images should completely contain the checkerboard (i.e. it is not cut off at the edges), and should have the checkerboards in many different orientations and distances from the camera. The goal is to get some data to correct every part of the image. An example image can found in /surround360_render/res/example_data/intrinsic_checkerboard.png.

* We will assume you are working in a folder located at ~/Desktop/intrinsic.

* Under ~/Desktop/intrinsic, create two subdirectories: /src, and /undistorted.

* Copy your checkerboard images to ~/Desktop/intrinsic/src.

* cd to /surround360_render and run the following command:

<pre>
./bin/TestIntrinsicCalibration \
--logbuflevel -1 --stderrthreshold 0 --v 0 \
--mode build_intrinsic_model \
--checkerboard_width 22 \
--checkerboard_height 28 \
--resize_width 2048 \
--resize_height 2048 \
--checker_size 29.5 \
--sensor_width 14.0 \
--sensor_height 14.0 \
--src_checkerboards_dir ~/Desktop/intrinsic/src \
--dest_param_file ~/Desktop/intrinsic/my_intrinsic.xml
</pre>


NOTE: the --checkerboard_width and --checkerboard_height parameters should match the number of checkers in your printed target, and the --resize_width and --resize_height parameters should match the image resolution. Other parameters such as --checker_size and --sensor_width won't have any effect.

* The output is a new file at ~/Desktop/intrinsic/my_intrinsic.xml. This is what you would pass to run_all.py for the --src_intrinsic_param_file argument.

* We can test the results by running the following:
<pre>
./bin/TestIntrinsicCalibration \
--logbuflevel -1 --stderrthreshold 0 --v 0 \
--mode undistort \
--src_intrinsic_param_file ~/Desktop/intrinsic/my_intrinsic.xml \
--src_fisheye_img_dir ~/Desktop/intrinsic/src \
--dest_undistorted_dir ~/Desktop/intrinsic/undistorted \
--resize_width 2048 \
--resize_height 2048
</pre>

This will output a modified version of each checkerboard image to ~/Desktop/intrinsic/undistorted. If the intrinsic calibration worked as intended, straight lines from the checkerboards should appear as straight lines in the undistorted images.
