# Surround 360 Camera Control Software (UI)

This part of the Surround 360 software controls the hardware (cameras) and captures the data they collect to a local file storage system (e.g., a RAID of SSD drives).

## Requirements

* The Surround 360 camera control software requires Ubuntu Linux (tested on 16.04 64-bit)

## Installing Dependencies of the Surround 360 Camera Control Software

* Compile the Surround 360 Rendering Software first. The Camera Control Software uses its ISP. Follow the steps in surround360_render/README.md (compile to use accelerated ISP)

* Install dependencies:
```
  sudo apt-get install libgtkmm-3.0-1v5 libgtkmm-3.0-dev libglibmm-2.4-dev libglibmm-2.4-1v5 libgtk-3.0 libgtk-3-dev libglew-dev libglew1.13 zlib1g zlib1g-dev
  sudo apt-get install libglademm-2.4-1v5 libgtkmm-2.4-1v5
  sudo apt-get install libgtkmm-2.4-dev libglademm-2.4-dev libgtkglextmm-x11-1.2-dev
  sudo apt-get install libtinfo-dev
```

* Download Point Grey FlyCapture SDK and Grasshopper 3 USB 3.0 (GS3-U3-41C6C-C) Firmware:
  * Go to https://www.ptgrey.com/support/downloads
  * You will need to register (free) to download software
  * Product Families: Grasshopper3
  * Camera Models: GS3-U3-41C6C-C
  * Operating Systems: Linux Ubuntu 16.04
  * Software -> download latest FlyCapture SDK - Linux Ubuntu (64-bit)
  * (optional, see below) Firmware -> download GS3-U3-41C6 Firmware 2.23.3.0

* Install FlyCapture SDK:
  * Install FlyCapture2 deb files using the script provided:
```
    chmod +x install_flycapture.sh
    sudo sh install_flycapture.sh
```

To install extra dependencies when prompted:
```
  sudo apt-get -f install
```

  * By default, Linux limits image capture to 2 MB. To capture images over 2 MB,
    extend the USBFS limit on how many buffers can be locked into the driver.
    This is done by updating the boot params in grub

    Open /etc/default/grub, and find and replace:
```
    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash"
```
with
```
    GRUB_CMDLINE_LINUX_DEFAULT="quiet splash usbcore.usbfs_memory_mb=8000 transparent_hugepage=never"
```
Then update grub:
```
    sudo update-grub
```

* Install Grasshopper 3 USB 3.0 (GS3-U3-41C6C-C) Firmware (optional):
  Cameras should come with latest firmware installed, but you can follow these
  steps to upgrade it
  WARNING: This can only be done in Windows, so you would need to download and
  install the Windows version of FlyCapture SDK (see steps above)
  * Open UpdatorGUI:
    Located under Point Grey FlyCapture2 SDK -> Utilities -> UpdatorGUI3
  * Select the camera(s) to update, point to the uncompressed .ez2 file and select "Update"
  * A power cycle of the cameras should not be required, as they are automatically rebooted after firmware update

* Set up Nvidia proprietary drivers:
  Go to Software & Updates -> Additional drivers and follow the steps
  For details, see https://help.ubuntu.com/community/BinaryDriverHowto/Nvidia

* Install FreeGLUT:
```
sudo apt-get install freeglut3 freeglut3-dev
```

## Compiling the Surround 360 Camera Control Software

After installing all of the dependencies as described above, run:
```
cd <install path>/surround360/surround360_camera_ctl_ui
cmake -DCMAKE_BUILD_TYPE=Release -DHALIDE_DIR=$HOME/Halide/cmake_build
make
```

To use the user interface, run:
```
./bin/CameraControlUI
```

We recommend configuring CMake to compile in Release mode because the code will execute faster. However, you can also set it up for debug mode with:
```
cmake -DCMAKE_BUILD_TYPE=Debug
```

## Join the Surround 360 community

* Website: https://facebook360.fb.com/facebook-surround-360/

See the CONTRIBUTING file for how to help out.

## License

The Surround 360 camera control code is licensed as described in LICENSE_camera_ctl_ui.md under /surround360_camera_ctl_ui.
