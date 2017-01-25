# Surround 360 Calibration

In order to produce a more accurate and comfortable result in VR, the Surround 360 rendering software uses several calibration config files to correct optical and mechanical issues. This document describes the process of generating the calibration config files.

WARNING: you should not attempt to render videos captured with Surround 360 without first reading this document. Uncalibrated results may be severely distorted in VR to the point of breaking stereo perception of 3D.

There are four specialized calibration processes: intrinsic calibration to correct barrel distortion, color calibration, stereo rectification for the side cameras, and optical center for the fisheye lenses at the top and bottom. Each part of calibration requires different data and different steps to process with our software.

## Rectification

The purpose of the rectification step is to align everything so that both eyes see every point in the scene at the same y-coordinate. This is the most important step in calibration; skipping this step will likely result in stereo which causes eye-strain or which cannot be fused as 3D at all. Fortunately, this step can be done with any unstructured data (if you captured a single frame of video, you have the necessary data). Rectification must be done once per rig, but can be left alone for subsequent scenes captured with the same rig.

* The config file for rectification has a .yml extension. It is passed to run_all.py via the --rectify_file command-line arugment, or "rectification file" in the GUI.

* For fast preview rendering modes, rectification may be skipped, and it is allowed to pass "NONE" for --rectify_file.

* A rectify file can be generated from the run_all.py GUI, or by directly from the command line. To generate it using the GUI, execute run_all.py, then make sure that the "Steps" list is either 'all', or includes 'rectify'. Note that 'rectify' must happen in between the 'isp' step and the 'render' step (or the 'isp' step should already have been run). You must also specify a file path in the 'Rectification File' parameter; this should be a path to a file with a .yml extension. A good default choice is to take the 'Destination Directory" path and add 'rectify.yml' to that, e.g., if 'Destination Directory' is ~Desktop/my360video, then 'Rectification File' could be ~Desktop/my360video/rectify.yml.

* WARNING: running the 'rectify' step is not necessary if you already have a rectification file, and pointing the 'Rectification file' argument to an existing file may cause it to be overwritten.

* WARNING: in rare cases, rectification may fail to produce valid results (if keypoint matching fails, e.g. in extremely low light conditions). If this happens, there will be a warning message in the logs, and it can be seen in the visualizations that are generated (see below). If the scene you are trying to render is not usable for rectification, use the same camera on a different scene, then apply that rectification file to the other scene.

* Here is a sample command for computing rectification from the command line:
<pre>
./bin/TestRingRectification \
--logbuflevel -1 --stderrthreshold 0 --v 0 \
--rig_json_file ./res/config/17cmosis_default.json \
--src_intrinsic_param_file ./res/config/sunex_intrinsic.xml \
--output_transforms_file ~/Desktop/new_rectify_test/rectify.yml \
--root_dir ~/Desktop/new_rectify_test \
--frames_list 000000 \
--visualization_dir ~/Desktop/new_rectify_test/rectify_vis
</pre>

* Once the command above finishes, a new file is created at ~/Desktop/new_rectify_test/rectify.yml. This is the file you need to pass in to run_all.py for "rectification file" (GUI) or --rectify_file (command-line). You may wish to copy this file to /surround360_render/res/config.

* That is all you need to do, but some interesting visualizations are generated in /keypoint_vis and /warped (relative to the --visualization_dir flag if using the command line, or in the /rectify_vis directory under a video's root folder if using the GUI). For example, /stacked.png shows all of the side cameras stacked horizontally into one big image, after rectification. If you draw a horizontal line across stacked.png, you should see everything lining up between adjacent images.

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


NOTE: the --checkerboard_width and --checkerboard_height parameters should match the number of checkers in your printed target minus 1, and the --resize_width and --resize_height parameters should match the image resolution. Other parameters such as --checker_size and --sensor_width won't have any effect.

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


## Color Calibration

When converting RAW images to RGB we use the files cmosis_sunex.json and cmosis_fujinon.json in /surround360_render/res/config/isp. These contain fields to configure the soft ISP for each side and top/bottom cameras, respectively. A detailed description of each field can be found on the README.txt file under the same directory.

Even though all the sensors from the same camera model should behave the same way, in practice there are subtle differences that can cause two images from two sensors to look different under the same illuminant (light source).

This calibration process lets us create ISP config json files for a set of cameras, which can be then used by referencing them in cam_to_isp_config.json.

To calibrate against a known illuminant, we need something on the scene for which we know its ground truth RGB values. We use a MacBeth ColorChecker, which contains 24 square color patches. We also use a SpyderCUBE device, which allows us to find the darkest point on the image for black level adjustment.

The steps below describe the color calibration process for a set of cameras from the same rig.

* Under the known illuminant, place the MacBeth chart and the SpyderCUBE in front of a camera and take a picture using our camera control software. An example image can found in /surround360_render/res/example_data/color_calibration_input.png. Repeat for each camera.

* Save the images inside a directory called "charts". For this example, we are assuming they are under ~/Desktop/color_calibration/charts/cam[0-16].tiff. Go to /surround360_render and run the following command:
<pre>
python scripts/color_calibrate_all.py \
--data_dir ~/Desktop/color_calibration \
--black_level_hole
</pre>

* This generates a directory called "isp", with all the ISP json config files. It also generates an output directory with several debug images of each step of the detection process, for all the cameras. /surround360_render/res/example_data/color_calibration_output.png is an example of the output of the last step (gamma correction) on the input image mentioned above.

* Check the file scripts/color_calibrate_all.py for more options. Use the attributes min_area_chart_perc and max_area_chart_perc to set a range for the expected size of the color chart; this is useful when pictures are taken at different distances. Use the attribute black_level_adjust to set the black level of each camera to the median of all the cameras; this is useful when we expect the black level to be the same in all the cameras (note that this is not true for all the sensors.)

To run the pipeline with these new ISP config files we just modify /surround360_render/res/config/isp/cam_to_isp_config.json so that it looks like:

<pre>
{
    "cam0": "~/Desktop/color_calibration/isp/isp0.json",
    "cam1": "~/Desktop/color_calibration/isp/isp1.json",
    "cam10": "~/Desktop/color_calibration/isp/isp10.json",
    "cam11": "~/Desktop/color_calibration/isp/isp11.json",
    "cam12": "~/Desktop/color_calibration/isp/isp12.json",
    "cam13": "~/Desktop/color_calibration/isp/isp13.json",
    "cam14": "~/Desktop/color_calibration/isp/isp14.json",
    "cam15": "~/Desktop/color_calibration/isp/isp15.json",
    "cam16": "~/Desktop/color_calibration/isp/isp16.json",
    "cam2": "~/Desktop/color_calibration/isp/isp2.json",
    "cam3": "~/Desktop/color_calibration/isp/isp3.json",
    "cam4": "~/Desktop/color_calibration/isp/isp4.json",
    "cam5": "~/Desktop/color_calibration/isp/isp5.json",
    "cam6": "~/Desktop/color_calibration/isp/isp6.json",
    "cam7": "~/Desktop/color_calibration/isp/isp7.json",
    "cam8": "~/Desktop/color_calibration/isp/isp8.json",
    "cam9": "~/Desktop/color_calibration/isp/isp9.json"
}
</pre>

and then run run_all.py as usual.

## Optical Vignetting Calibration

All lenses create a type of vignetting called optical vignetting, which results in a fall-off in brightness as we move away from the center of the image and towards the edges. This effect is especially undesirable in overlapping camera scenarios, since we expect overlapping areas to have the same color and brightness.

This calibration process lets us model the vignetting fall-off and update the camera ISP json config file accordingly. This is done by taking a picture of a gray chart while rotating the camera along its exit pupil (or as close as possible) so that we get samples of the chart across the entire image region.

The steps below describe the calibration process for a set of cameras.

* Under a uniform and constant illuminant, place the grayscale chart in front of a camera and take as many pictures as desired (recommended more than 20) using our camera control software so as to cover the entire image region with samples of the chart in all positions. An example image can found in /surround360_render/res/example_data/vignetting_calibration_sample.tiff. Repeat for each camera.

* Save the set of images for each camera inside a directory called "charts". For this example, we acquired 100 images per camera, for 17 cameras, and we placed them under ~/Desktop/vignetting_calibration/cam[0-16]/charts/[0-99].tiff. Note the file structure, where each camera has its own directory. We also assume that color calibration has been run on these cameras, and we already have a directory ~/Desktop/vignetting_calibration/isp with each camera's ISP json config file. Go to /surround360_render and run the following command:
<pre>
python scripts/vignetting_calibrate.py \
--data_dir ~/Desktop/vignetting_calibration \
--num_cams 17 \
--save_debug_images
</pre>

* This generates a directory called isp_new with all the updates ISP config files. Also, each camera directory has two new directories. 1) acquisition: contains a mask image with all the detected charts, a data file called data.json, containing location and color intensity values for each patch, and other debugging data. 2) calibration: contains plots of the vignetting models for each channel, an updated ISP json config file, and other debugging data. /surround360_render/res/example_data/vignetting_calibration_fit.png is an example of a surface fit that models the vignetting of the red channel of the camera used in this example. It shows the center of the image and the point of minimum vignetting, as well as the Bezier control points on the top left.

* Check the file scripts/vignetting_calibrate.py for more options. Use the attributes image_width and image_height if using non-default image size. Use the attribute load_data to load the location and color intensity data, skip the acquisition step and go straight to the calibration step.

To run the pipeline with these new ISP config files, just replace the original ones with the ones created by the script.
