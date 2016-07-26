# Surround 360 Camera Control Software

This part of the Surround 360 software controls the hardware (cameras) and captures the data they collect to a local file storage system (e.g., a RAID of SSD drives).

## Requirements

* The Surround 360 camera control software requires Ubuntu Linux 14.04 64-bit (required by the PointGrey camera SDK)

* The essential dependencies of the Surround 360 camera control software are:
  * FlyCapture SDK (from PointGrey), and its prereqs:
    * libraw1394-11
    * libgtkmm-2.4-1c2a
    * libglademm-2.4-1c2a
    * libgtkglextmm-x11-1.2-dev
    * libgtkglextmm-x11-1.2
    * libusb-1.0-0
    * libglademm-2.4-dev
  * CMake
  * gflags
  * OpenCV 3.0+
  * ffmpeg

## Installing Dependencies of the Surround 360 Camera Control Software

* Download Point Grey FlyCapture SDK and Grasshopper 3 USB 3.0 (GS3-U3-41C6C-C) Firmware:
  * Go to https://www.ptgrey.com/support/downloads
  * You will need to register (free) to download software
  * Product Families: Grasshopper3
  * Camera Models: GS3-U3-41C6C-C
  * Operating Systems: Linux Ubuntu 14.04
  * Software -> download latest FlyCapture SDK - Linux Ubuntu (64-bit)
  * (optional, see below) Firmware -> download GS3-U3-41C6 Firmware 2.23.3.0

* Install FlyCapture SDK (method 1):
  Uncompress SDK file and follow instructions from README file

* Install FlyCapture SDK (method 2):
  * Install FlyCapture SDK dependencies:
<pre>
    sudo apt-get install libraw1394-11 libgtkmm-2.4-1c2a libglademm-2.4-1c2a libgtkglextmm-x11-1.2-dev libgtkglextmm-x11-1.2 libusb-1.0-0 libglademm-2.4-dev
</pre>

  * Open /etc/modules and append
<pre>
    raw1394
</pre>

  * Install FlyCapture2 deb files using the script provided:
<pre>
    chmod +x install_flycapture.sh
    sudo sh install_flycapture.sh
</pre>

  * By default, Linux limits image capture to 2 MB. To capture images over 2 MB,
    extend the USBFS limit on how many buffers can be locked into the driver.
    This is done by updating the boot params in grub

    Open /etc/default/grub, and find and replace:
<pre>
    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
</pre>
    with
<pre>
    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=2000"
</pre>
    Then update grub:
<pre>
    sudo update-grub
</pre>

* Install Grasshopper 3 USB 3.0 (GS3-U3-41C6C-C) Firmware (optional):
  Cameras should come with latest firmware installed, but you can follow these
  steps to upgrade it
  WARNING: This can only be done in Windows, so you would need to download and
  install the Windows version of FlyCapture SDK (see steps above)
  * Open UpdatorGUI:
    Located under Point Grey FlyCapture2 SDK -> Utilities -> UpdatorGUI3
  * Select the camera(s) to update, point to the uncompressed .ez2 file and select "Update"
  * A power cycle of the cameras should not be required, as they are automatically rebooted after firmware update

* Install CMake (method 2 - Linux only)
<pre>
  sudo apt-get install software-properties-common
  sudo add-apt-repository ppa:george-edison55/cmake-3.x
  sudo apt-get update
  sudo apt-get install cmake && sudo apt-get upgrade cmake
</pre>

* Install gflags:
<pre>
  sudo apt-get install libgflags2 libgflags-dev
</pre>

* Install ffmpeg:
  see https://trac.ffmpeg.org/wiki/CompilationGuide

## Compiling the Surround 360 Camera Control Software

After installing all of the dependencies as described above, run:
<pre>
  cd <install path>/surround360/surround360_camera_ctl
  cmake -DCMAKE_BUILD_TYPE=Release
  make
</pre>

To test that compilation is successful, run:

<pre>
  ./bin/CameraControl -numcams 17 -raw -nbits 8 -shutter 20 -gain 0 -debug
</pre>

We recommend configuring CMake to compile in Release mode because the code will execute faster. However, you can also set it up for debug mode with:
<pre>
  cmake -DCMAKE_BUILD_TYPE=Debug
</pre>

## Join the Surround 360 community

* Website: https://facebook360.fb.com/facebook-surround-360/

See the CONTRIBUTING file for how to help out.

## License

The Surround 360 camera control code is licensed as described in LICENSE_camera_ctl.md under /surround360_camera_ctl.
