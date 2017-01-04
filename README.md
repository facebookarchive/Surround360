# Surround360 System

Surround360 is a hardware and software system for capturing and rendering 3D (stereo) 360 videos and photos, suitable for viewing in VR. We divide the open source components of the system into three subdirectories:

* /surround360_design -- hardware designs and assembly instructions
* /surround360_camera_ctl -- software for controlling the camera to capture raw data
* /surround360_camera_ctl_ui -- Linux desktop application for controlling the camera to capture raw data
* /surround360_render -- software for rendering the raw data that is captured into a format suitable for viewing in VR.

## Instruction Manual ##

Please refer to our instruction manual (which covers use of both hardware and software) here:

https://github.com/facebook/Surround360/blob/master/surround360_design/assembly_guide/Surround360_Manual.pdf

## Sample Data ##

We provide a sample dataset for those who are interested in testing the rendering software without first building a camera.

* "Palace of Fine Arts Take 1" - 2 frames - (337.4MB)
  * Raw data: http://surround360.hacktv.xyz/sample/sample_dataset.zip
  * Sample result: https://s3-us-west-2.amazonaws.com/surround360/sample/sample_result.zip
  * NOTE: The Surround 360 hardware and camera control software records to a RAW binary format which needs to be unpacked to get to the individual images. In this smaller dataset, the 'unpack' and 'arrange' steps of the pipeline have already been run (see run_all.py), so you do not need to run them again.

* "Palace of Fine Arts Take 3" - 1 minute, RAW binary - (117.5GB)
  * Binary file 1: https://s3-us-west-2.amazonaws.com/surround360/palace3_0.bin
  * Binary file 2: https://s3-us-west-2.amazonaws.com/surround360/palace3_1.bin
  * Camera names file: https://s3-us-west-2.amazonaws.com/surround360/palace3_cameranames.txt
  * Image signal processor (ISP) configs: https://s3-us-west-2.amazonaws.com/surround360/isp_config.zip
  * NOTE: this is the RAW binary format. You will need to download both .bin files and palace3_cameranames.txt, put them in the same directory (e.g. *palace3*), and remove their "palace3_" prefix before running the 'unpack' and 'arrange' steps of the pipeline.
  * NOTE: Unzip the contents of isp_config.zip to /surround360/surround360_render/res/config/isp.

## Join the Surround360 community

* Website: https://facebook360.fb.com/facebook-surround-360/

See the CONTRIBUTING file in each subdirectory for how to help out.

## License

The hardware designs, camera control software, and rendering software are subject to different licenses. Please refer to the LICENSE file in each subdirectory. We also provide an additional patent grant.
