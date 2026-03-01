# Surround 360 Rig Geometry Format

The Surround 360 software needs to know the parameters of each camera in the
rig. There parameters include camera position, orientation, lens type,
resolution, field-of-view, etc. For this, it uses a rig file in JSON format

For example, an approximate version - e.g. derived from a CAD model - is
provided to geometric calibration along with calibration images to produce a
final, accurate version that matches the physical rig. And the accurate version
is provided to the rendering software to produce the final output

## Here is a quick walk-through of the rig geometry file format

An example of such a rig geometry file is here:
  /surround360_render/res/config/camera_rig.json

It is a json file (http://www.json.org/) containing an array named "cameras"
that contains one element per camera in the rig

Here is an example of such a camera element:
  {
    "group" : "side camera",
    "id" : "cam1",
    "origin" : [21.799999237060547, 0, 0],
    "right" : [0, -1, 0],
    "up" : [0, 0, 1],
    "forward" : [1, 0, 0],
    "principal" : [1024, 1024],
    "focal" : [1269.580673376528, -1269.580673376528],
    "resolution" : [2048, 2048],
    "type" : "RECTILINEAR",
    "distortion" : [0,0],
    "version" : 1
  }

"group" is the group to which the camera belongs. All cameras in a group are
assumed to have the same distortion, so two cameras should only belong to the
same group if they have the same lens. Furthermore, cameras with the word "side"
in their group are treated as side cameras for rendering purposes

"id" is the name of the camera. The capture software sorts the cameras according
to serial number and then names their output "cam0", "cam1", "cam2". This means
that the order of the camera serial numbers much match the names in the json

"origin" is the position of the camera relative to the center of the rig, in cm

"right", "up", and "forward" specify the orientation of the camera. All three
must be unit vectors and each must be orthogonal to the other two

"principal" is the pixel coordinate of the optical axis of the camera. It is
around this point that radial distortion is applied. This point is usually near
the center of the sensor, i.e. half the resolution

"focal" is the number of pixels per radian at the principal point. Note that
focal is usually one value followed by the same value, but negative. That the
values are the same indicates that the pixels are square. As a curiousity, the
second value is negative because pixels are traditionally numbered from top to
bottom, whereas coordinate systems traditionally grow from bottom to top.

An f-theta (fisheye) lens aims to keep the pixels per radian constant across the
sensor, so this number is pretty straightforward to compute. if - for example -
the specification says "185 degree horizontal field of view" and the sensor is
2448 pixels wide, then the horizontal focal can be computed like so:
  185 degrees * pi/360 radians/degree = 3.22886 radians
  2448 pixels / 3.22886 radians = 758.162 pixels/radian, so
  "focal" : [758.162, -758.162]

For a rectilinear (non-fisheye) lens, the math is a little more complicated:
  focal = resolution/2 / tan(fov/2)
  
As an example, assume the specification says "135 degree horizontal fov" and the
sensor is 3264 pixels wide:
  135 degrees * pi/180 radians/degree = 2.35619 radians
  3264/2 pixels / tan(2.35619/2 radians) = 676.001 pixels/radian, so
  "focal" : [676.001, -676.001]

Note that this number can be approximate, geometric calibration will dial it in

"resolution" is the resolution of the camera, in pixels

"type" is either RECTILINEAR or FTHETA. These correspond to regular versus
fisheye lenses

"distortion" provides the coefficients for a function describing how the lens
deviates from a perfect RECTILINEAR or FTHETA lens. Zeros means the lens is
perfect. Again, it is ok to assume a perfect lens and let geometric calibration
figure out the correct value

"version" needs to be 1

## Here is another example, one of the bottom cameras

{
  "id" : "cam16",
  "origin" : [-10, 0, -13.1],
  "principal" : [1024, 1024],
  "right" : [-1, 0, 0],
  "up" : [0, -1, 0],
  "forward" : [0, 0, -1],
  "focal" : [483.76220324, -483.76220324],
  "resolution" : [2048, 2048],
  "type" : "FTHETA",
  "distortion" : [0, 0],
  "fov" : 1.61443,
  "version" : 1
}

"fov" is required if the fov does not cover the entire sensor. I.e. the field
of view is restricted not only by the size of the sensor, but by the field of
view of the lens. This is fairly common for fisheye cameras, and means that they
produce round pictures which include the inside of the lens and even a black
area beyond that. "fov" tells the software to ignore pixels corresponding to
angles outside the lens field of view
 
The angle is measured in radians from the edge to the camera's center axis.
Note that lenses usually specify twice that angle (the angle from one edge to
the other), and that they usually specify degress.

For example, if the lens fov is specified as being 185 degrees from edge to
edge, then it is half that - 92.5 degrees - from edge to center axis:
  92.5 degrees * pi/180 radians/degree = 1.61443 radians, so:
  "fov" : 1.61443
  
If "fov" is not provided, the software will assume that the lens fov covers the
entire sensor
