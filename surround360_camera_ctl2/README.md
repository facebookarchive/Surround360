---[ OVERVIEW ]---

This is a desktop application for controlling surround360 camera and capturing footage.

---[ BUILD INSTRUCTIONS ]---

Please install the following packages:

$ apt-get install libgtkmm-3.0-1v5 libgtkmm-3.0-dev libglibmm-2.4-dev libglibmm-2.4-1v5 libgtk-3.0 libgtk-3-dev libglew-dev libglew1.13 zlib1g zlib1g-dev 

Afterwards, proceed to set up Nvidia proprietary drivers in "Additional drivers" section of "Software & Updates" application.
For details, see https://help.ubuntu.com/community/BinaryDriverHowto/Nvidia

Next, proceed with building the software as follows

$ cd surround360/surround360_camera_ctl2
$ cmake -DCMAKE_BUILD_TYPE=Release .
$ make

To use the control software, please run it as

$ ./bin/CameraControl2

