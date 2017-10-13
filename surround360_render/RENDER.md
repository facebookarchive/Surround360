# Surround 360 Render steps

In order to render captured footage, we use the Python script run_all.py located in surround360_render/scripts/run_all.py

The script spawns a user interface by just running

<pre>
python surround360_render/scripts/run_all.py
</pre>

There are a few required fields to populate:

- Data directory: directory containing .bin files from a previous capture.
- Destination directory: where we want to save the process data.

If older version of the capturing system was used, that created a file named cameranames.txt along with the .bin files, we need to specify the image width, height, and bit depth used during capture.

The rest of the fields are self explanatory.

The software expects some files to be under a directory called "config" inside of the specified destination directory. For example, if the output directory is ~/Desktop/render:

<pre>
render
└── config
    ├── camera_rig.json
    └── isp
      └── <i>serial_number</i>.json
</pre>

It is strongly recommended that the steps in CALIBRATION.md are followed in other to create customized files for each camera. These are:

- config/isp/*serial_number*.json: output of color calibration and optical vignetting calibration
- config/camera_rig.json: output of geometric calibration

If an older version of the software that did not have geometric calibration steps in CALIBRATION.md is used, the software will expect these files instead:

- config/camera_rig.json: default file under surround360_render/res/config/17cmosis_default.json. Just copy it over and rename it
- config/rectify.yml: output of the "Rectification" step in CALIBRATION.md
- config/instrinsics.xml: output of the "Intrinsic / Barrel Distortion" step in CALIBRATION.md
