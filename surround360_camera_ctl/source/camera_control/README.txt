** NOTE **
Assuming we are located at ~/fbsource/fbcode/vr_camera_hw/bin

** NOTE **
CameraControl creates log files in /var/log/vrcam. If we need any of those for processing (e.g. cameranames.txt) we have to copy them afterwards to the desired location!
sudo cp /var/log/vrcam/captures/<TIMESTAMP>/cameranames.txt /media/snoraid/<TIMESTAMP>_cameranames.txt

** NOTE **
Before running CameraControl, we ALWAYS need to force completion of disk writes (via sync) and free pagecache, dentries and inodes (via drop_caches):
sync; echo 3 | sudo /usr/bin/tee /proc/sys/vm/drop_caches; sudo ./CameraControl [options]

== Print help ==
./CameraControl --help

== Print camera properties and value ranges ==
./CameraControl --props --debug

== Single image with default params (-d outputs debug statements) ==
./CameraControl --nframes 1  --numcams 17 --debug

== Single image with params ==
sudo ./CameraControl --nframes 1 --brightness 13.5 --exposure 0.18 --gamma 2.3 --shutter 10.0 --gain 4.500 --whitebalance "300 500" --numcams 17 --debug

== Single 8-bit RAW image ==
sudo ./CameraControl --nframes 1 --raw --nbits 8 --brightness 10.449 --exposure 2.400 --shutter 100.0 --gain 7.000 --numcams 17 --debug

== Video: 5 frames at 30 fps ==
sudo ./CameraControl --debug --raw --nbits 8 --fps 30 --shutter 10 --gain 0 --nframes 5 --numcams 17

== High Dynamic Range: shutter in [3ms 10ms 20ms 30ms], gain in [0dB 2dB 5dB 7dB] ==
sudo ./CameraControl --debug --raw --nbits 8 --fps 30 --brightness 1 --nframes 1 --hdr "3 10 20 30 0 2 5 7" --numcams 17

== Autoexposure (use as -fps the frame rate that will be used for video, so we don't go over 1/fps) ==
No need to add "--gain 0". If camera auto gain is used it's because its auto shutter is maxed out, and that's what we care about

sudo ./CameraControl --debug --brightness 1 --gain 0 --autoexposure --fps 30 --numcams 17

NOTE: If sh << 1/fps we may see aliasing. Try to keep the shutter close to 1/fps (may introduce motion blur, though).

== Restore cameras ==
sudo bin/CameraControl --debug --numcams 17 --restore

== List all serial numbers for cameras

sudo ./CameraControl --list

== Common command for Point Grey's ==
sync; echo 3 | sudo /usr/bin/tee /proc/sys/vm/drop_caches; sudo ./CameraControl -n 5 -raw -nbits 8 -br 1 -sh 20.0 -ga 0 -fps 30 -master 15405803 -numcams 17 -dir <TIMESTAMP> -d; sudo cp /var/log/vrcam/captures/<TIMESTAMP>/cameranames.txt /media/snoraid/<TIMESTAMP>_cameranames.txt
