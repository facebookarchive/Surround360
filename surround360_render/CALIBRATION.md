# Surround 360 Calibration

In order to produce a more accurate and comfortable result in VR, the Surround 360 rendering software uses several calibration config files to correct optical and mechanical issues. This document describes the process of generating the calibration config files.

WARNING: you should not attempt to render videos captured with Surround 360 without first reading this document. Uncalibrated results may be severely distorted in VR to the point of breaking stereo perception of 3D.

There are three specialized calibration processes: color calibration, optical vignetting calibration, and geometric calibration. Each part of calibration requires different data and different steps to process with our software. For best results, execute each calibration process in the order below.

## Color Calibration

When converting RAW images to RGB we use the files cmosis_sunex.json and cmosis_fujinon.json in /surround360_render/res/config/isp. These contain fields to configure the soft ISP for each side and top/bottom cameras, respectively. A detailed description of each field can be found on the README.txt file under the same directory.

Even though all the sensors from the same camera model should behave the same way, in practice there are subtle differences that can cause two images from two sensors to look different under the same illuminant (light source).

This calibration process lets us create ISP config json files for a set of cameras, which can be then used by referencing them in cam_to_isp_config.json.

To calibrate against a known illuminant, we need something on the scene for which we know its ground truth RGB values. We use a MacBeth ColorChecker, which contains 24 square color patches. We also use a SpyderCUBE device, which allows us to find the darkest point on the image for black level adjustment.

The steps below describe the color calibration process for a set of cameras from the same rig.

* Under the known illuminant, place the MacBeth chart and the SpyderCUBE in front of a camera and take a picture using our camera control software. An example image can found in /surround360_render/res/example_data/color_calibration_input.png. Repeat for each camera.

* Save the images inside a directory called "charts". For this example, we are assuming they are under ~/Desktop/color_calibration/charts/<serial_number>.tiff. Go to /surround360_render and run the following command:
<pre>
python scripts/color_calibrate_all.py \
--data_dir ~/Desktop/color_calibration \
--black_level_hole
</pre>

* This generates a directory called "isp", with all the ISP json config files. It also generates an output directory with several debug images of each step of the detection process, for all the cameras. /surround360_render/res/example_data/color_calibration_output.png is an example of the output of the last step (gamma correction) on the input image mentioned above.

* Check the file scripts/color_calibrate_all.py for more options. Use the attributes min_area_chart_perc and max_area_chart_perc to set a range for the expected size of the color chart; this is useful when pictures are taken at different distances. Use the attribute black_level_adjust to set the black level of each camera to the median of all the cameras; this is useful when we expect the black level to be the same in all the cameras (note that this is not true for all the sensors.)

To run the pipeline with these new ISP config files we just need to copy the generated ISP config files to the output directory, e.g. ~/Desktop/render/config/isp/<serial_number>.json, and then run run_all.py as usual. Note that the ISP config files are named after each corresponding camera serial number.

## Optical Vignetting Calibration

All lenses create a type of vignetting called optical vignetting, which results in a fall-off in brightness as we move away from the center of the image and towards the edges. This effect is especially undesirable in overlapping camera scenarios, since we expect overlapping areas to have the same color and brightness.

This calibration process lets us model the vignetting fall-off and update the camera ISP json config file accordingly. This is done by taking a picture of a gray chart while rotating the camera along its exit pupil (or as close as possible) so that we get samples of the chart across the entire image region.

The steps below describe the calibration process for a set of cameras.

* Under a uniform and constant illuminant, place the grayscale chart in front of a camera and take as many pictures as desired (recommended more than 20) using our camera control software so as to cover the entire image region with samples of the chart in all positions. An example image can found in /surround360_render/res/example_data/vignetting_calibration_sample.tiff. Repeat for each camera.

* Save the set of RAW images for each camera inside a directory called "charts". For this example, we acquired 100 images per camera, for 17 cameras, and we placed them under ~/Desktop/vignetting_calibration/<serial_number>/charts/[000000-000099].tiff. Note the file structure, where each camera has its own directory. We also assume that color calibration has been run on these cameras, and we already have a directory ~/Desktop/vignetting_calibration/isp with each camera's ISP json config file. Go to /surround360_render and run the following command:
<pre>
python scripts/vignetting_calibrate.py \
--data_dir ~/Desktop/vignetting_calibration \
--num_cams 17 \
--save_debug_images
</pre>

* This generates a directory called isp_new with all the updates ISP config files. Also, each camera directory has two new directories. 1) acquisition: contains a mask image with all the detected charts, a data file called data.json, containing location and color intensity values for each patch, and other debugging data. 2) calibration: contains plots of the vignetting models for each channel, an updated ISP json config file, and other debugging data. /surround360_render/res/example_data/vignetting_calibration_fit.png is an example of a surface fit that models the vignetting of the red channel of the camera used in this example. It shows the center of the image and the point of minimum vignetting, as well as the Bezier control points on the top left.

* Check the file scripts/vignetting_calibrate.py for more options. Use the attributes image_width and image_height if using non-default image size. Use the attribute load_data to load the location and color intensity data, skip the acquisition step and go straight to the calibration step.

To run the pipeline with these new ISP config files, just replace the original ones with the ones created by the script.

# Geometric Calibration

No matter how well constructed the rig is, our software needs to know the geometric properties of the cameras (intrinsic and extrinsic) in order to accurately perform stereo reconstruction.

The steps below describe the geometric calibration process for a camera rig.

* Capture a single frame using the Surround360 capturing software in a scene with plenty of features, that is, containing objects with sharp edges and corners of different sizes. A good example is the interior of an office.

* Unpack the frames and run the ISP step to get RGB images. Put them in a separate directory. For this example we assume they are in ~/Desktop/geometric_calibration/rgb/cam[0-16]/000000.png

* Go to surround360_render and run the following command:
<pre>
python scripts/geometric_calibration.py \
--data_dir  ~/Desktop/geometric_calibration \
--rig_json $PWD/res/config/camera_rig.json \
--output_json ~/Desktop/geometric_calibration/camera_rig.json \
--save_debug_images
<pre>

* This generates a new JSON file, camera_rig.json, to be used when rendering by just copying it to the output directory, e.g. ~/Desktop/render/config/camera_rig.json. It also generates debug images under ~/Desktop/geometric_calibration showing the accuracy of the calibration process.
