# Copyright (c) 2016-present, Facebook, Inc.
# All rights reserved.
#
# This source code is licensed under the BSD-style license found in the
# LICENSE_render file in the root directory of this subproject. An additional grant
# of patent rights can be found in the PATENTS file in the same directory.

import argparse
import datetime
import json
import os
import signal
import subprocess
import sys
import time
import types

from os import listdir
from os.path import isdir, isfile, join
from PIL import Image
from timeit import default_timer as timer

current_process = None
def signal_term_handler(signal, frame):
  if current_process:
    print "Terminating process: " + current_process.name + "..."
    current_process.terminate()
  sys.exit(1)

# Gooey imports
USE_GOOEY = (len(sys.argv) == 1 or "--ignore-gooey" in sys.argv)
if USE_GOOEY: from gooey import Gooey, GooeyParser # only load Gooey if needed
else: Gooey = lambda program_name, image_dir: (lambda fn: fn) # dummy for decorator
def conditional_decorator(pred, dec): return lambda fn: dec(fn) if pred else fn

script_dir = os.path.dirname(os.path.realpath(__file__))

# os.path.dirname(DIR) is the parent directory of DIR
surround360_render_dir = os.path.dirname(script_dir)

TITLE = "Surround 360 - Process Dataset"
NUM_CAMS = 17
FRAME_NUM_DIGITS = 6

UNPACK_COMMAND_TEMPLATE = """
{SURROUND360_RENDER_DIR}/bin/UnpackImageBundle
--logbuflevel -1
--log_dir {ROOT_DIR}/logs
--stderrthreshold 0
--v {VERBOSE_LEVEL}
--binary_prefix {BINARY_PREFIX}
--dest_path {ROOT_DIR}
--start_frame {START_FRAME}
--frame_count {FRAME_COUNT}
--file_count {DISK_COUNT}
{FLAGS_UNPACK_EXTRA}
"""

ISP_COMMAND_TEMPLATE = """
python {SURROUND360_RENDER_DIR}/scripts/batch_process_isp.py
--surround360_render_dir {SURROUND360_RENDER_DIR}
--root_dir {ROOT_DIR}
--start_frame {START_FRAME}
--end_frame {END_FRAME}
--isp_dir {ISP_DIR}
--nbits {NBITS}
{FLAGS_ISP_EXTRA}
"""

RENDER_COMMAND_TEMPLATE = """
python {SURROUND360_RENDER_DIR}/scripts/batch_process_video.py
--flow_alg {FLOW_ALG}
--root_dir {ROOT_DIR}
--surround360_render_dir {SURROUND360_RENDER_DIR}
--quality {QUALITY}
--start_frame {START_FRAME}
--end_frame {END_FRAME}
--cubemap_width {CUBEMAP_WIDTH}
--cubemap_height {CUBEMAP_HEIGHT}
--cubemap_format {CUBEMAP_FORMAT}
--rig_json_file {RIG_JSON_FILE}
{FLAGS_RENDER_EXTRA}
"""

FFMPEG_COMMAND_TEMPLATE = """
ffmpeg
-framerate 30
-start_number {START_NUMBER}
-i {ROOT_DIR}/eqr_frames/eqr_%06d.png
-pix_fmt yuv420p
-c:v libx264
-crf 10
-profile:v high
-tune fastdecode
-bf 0
-refs 3
-preset fast {MP4_PATH}
{FLAGS_FFMPEG_EXTRA}
"""

@conditional_decorator(USE_GOOEY, Gooey(program_name=TITLE, image_dir=os.path.dirname(script_dir) + "/res/img"))
def parse_args():
  dir_chooser = {"widget": "DirChooser"} if USE_GOOEY else {}
  file_chooser = {"widget": "FileChooser"} if USE_GOOEY else {}
  parse_type = GooeyParser if USE_GOOEY else argparse.ArgumentParser
  parser = parse_type(description=TITLE, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--data_dir',                   metavar='Data Directory', help='directory containing .bin files', required=True, **dir_chooser)
  parser.add_argument('--dest_dir',                   metavar='Destination Directory', help='destination directory', required=True, **({"widget": "DirChooser"} if USE_GOOEY else {}))
  parser.add_argument('--image_width',                metavar='Image Width', help='image width (ignore if no cameranames.txt)', required=False, default='2048')
  parser.add_argument('--image_height',               metavar='Image Height', help='image height (ignore if no cameranames.txt)', required=False, default='2048')
  parser.add_argument('--nbits',                      metavar='Bit Depth', help='bit depth (ignore if no cameranames.txt)', required=False, choices=['8', '12'], default='12')
  parser.add_argument('--quality',                    metavar='Quality', help='final output quality', required=False, choices=['3k', '4k', '6k', '8k'], default='6k')
  parser.add_argument('--start_frame',                metavar='Start Frame', help='start frame', required=False, default='0')
  parser.add_argument('--frame_count',                metavar='Frame Count', help='0 = all', required=False, default='0')
  parser.add_argument('--cubemap_format',             metavar='Cubemap Format', help='photo or video', required=False, choices=['photo', 'video'], default='video')
  parser.add_argument('--cubemap_width',              metavar='Cubemap Face Width', help='0 = no cubemaps', required=False, default='0')
  parser.add_argument('--cubemap_height',             metavar='Cubemap Face Height', help='0 = no cubemaps', required=False, default='0')
  parser.add_argument('--save_debug_images',          help='save debug images', action='store_true')
  parser.add_argument('--steps_unpack',               help='Step 1: convert data in .bin files to RAW .tiff files', action='store_true', required=False)
  parser.add_argument('--steps_isp',                  help='Step 3: convert RAW frames to RGB', action='store_true', required=False)
  parser.add_argument('--steps_rectify',              help='Step 4: use/create rectification file', action='store_true', required=False)
  parser.add_argument('--steps_render',               help='Step 5: render PNG stereo panoramas', action='store_true', required=False)
  parser.add_argument('--steps_ffmpeg',               help='Step 6: create MP4 output', action='store_true', required=False)
  parser.add_argument('--enable_top',                 help='enable top camera', action='store_true')
  parser.add_argument('--enable_bottom',              help='enable bottom camera', action='store_true')
  parser.add_argument('--enable_pole_removal',        help='false = use primary bottom camera', action='store_true')
  parser.add_argument('--dryrun',                     help='do not execute steps', action='store_true')
  parser.add_argument('--verbose',                    help='increase output verbosity', action='store_true')

  return vars(parser.parse_args())

def print_runall_command(args):
  cmd = "python " + os.path.realpath(__file__)
  for flag, value in args.iteritems():
    is_boolean = type(value) == types.BooleanType
    if is_boolean and value is False:
      continue
    cmd += " --%s %s" % (flag, value if not is_boolean else "")
  print cmd + "\n"

def start_subprocess(name, cmd):
  global current_process
  current_process = subprocess.Popen(cmd, shell=True)
  current_process.name = name
  current_process.communicate()

def save_step_runtime(file_out, step, runtime_sec):
  text_runtime = "\n" + step.upper() + " runtime: " + str(datetime.timedelta(seconds=runtime_sec)) + "\n"
  file_out.write(text_runtime)
  print text_runtime
  sys.stdout.flush()

# step_count will let us know how many times we've called this function
def run_step(step, cmd, verbose, dryrun, file_runtimes, num_steps, step_count=[0]):
  step_count[0] += 1

  print "** %s ** [Step %d of %d]\n" % (step.upper(), step_count[0], num_steps)

  if verbose:
    print cmd + "\n"

  sys.stdout.flush()

  if dryrun:
    return

  start_time = timer()
  start_subprocess(step, cmd)

  save_step_runtime(file_runtimes, step, timer() - start_time)

def list_only_files(src_dir): return filter(lambda f: f[0] != ".", [f for f in listdir(src_dir) if isfile(join(src_dir, f))])

if __name__ == "__main__":
  signal.signal(signal.SIGTERM, signal_term_handler)

  args = parse_args()
  data_dir                  = args["data_dir"]
  dest_dir                  = args["dest_dir"]
  image_width               = args["image_width"]
  image_height              = args["image_height"]
  nbits                     = args["nbits"]
  quality                   = args["quality"]
  start_frame               = int(args["start_frame"])
  frame_count               = int(args["frame_count"])
  cubemap_width             = int(args["cubemap_width"])
  cubemap_height            = int(args["cubemap_height"])
  cubemap_format            = args["cubemap_format"]
  enable_top                = args["enable_top"]
  enable_bottom             = args["enable_bottom"]
  enable_pole_removal       = args["enable_pole_removal"]
  save_debug_images         = args["save_debug_images"]
  dryrun                    = args["dryrun"];
  steps_unpack              = args["steps_unpack"]
  steps_isp                 = args["steps_isp"]
  steps_render              = args["steps_render"]
  steps_ffmpeg              = args["steps_ffmpeg"]
  verbose                   = args["verbose"]

  print "\n--------" + time.strftime(" %a %b %d %Y %H:%M:%S %Z ") + "-------\n"

  if dryrun:
    verbose = True

  if USE_GOOEY:
    print_runall_command(args)

  if quality not in ["3k", "4k", "6k", "8k"]:
    sys.stderr.write("Unrecognized quality setting: " + quality)
    exit(1)

  if enable_pole_removal and enable_bottom is False:
    sys.stderr.write("Cannot use enable_pole_removal if enable_bottom is not used")
    exit(1)

  os.system("mkdir -p " + dest_dir + "/logs")

  print "Checking required files..."

  dir_res_default = surround360_render_dir + "/res"
  dir_config = dest_dir + "/config"
  os.system("mkdir -p " + dir_config)

  dir_isp = dir_config + "/isp"
  if not os.path.isdir(dir_isp):
    print "ERROR: No color adjustment files not found in " + dir_isp + "\n"
    sys.stdout.flush()
    exit(1)

  new_rig_format = True
  file_camera_rig = "camera_rig.json"
  path_file_camera_rig = dir_config + "/" + file_camera_rig
  if not os.path.isfile(path_file_camera_rig):
    print "WARNING: Calibration file not found. Using default file.\n"
    sys.stdout.flush()
    path_file_camera_rig_default = dir_res_default + "/config/" + file_camera_rig
    os.system("cp " + path_file_camera_rig_default + " " + path_file_camera_rig)
  else:
    json_camera_rig = json.load(open(path_file_camera_rig))
    if "camera_ring_radius" in json_camera_rig:
      # If using old format, we also need rectification and intrinsics files
      new_rig_format = False
      path_file_rectify = dir_config + "/rectify.yml"
      if not os.path.isfile(path_file_rectify):
        print "ERROR: Rectification file (" + path_file_rectify + ") not found.\n"
        sys.stdout.flush()
        exit(1)
      file_intrinsics = "intrinsics.xml"
      path_file_instrinsics = dir_config + "/" + file_intrinsics
      if not os.path.isfile(path_file_instrinsics):
        print "WARNING: Instrinsics file not found. Using default file.\n"
        sys.stdout.flush()
        path_file_instrinsics_default = dir_res_default + "/config/" + file_intrinsics
        os.system("cp " + path_file_instrinsics_default + " " + path_file_instrinsics)

  dir_pole_masks = dest_dir + "/pole_masks"
  if not os.path.isdir(dir_pole_masks):
    print "WARNING: Pole masks not found. Using default files.\n"
    sys.stdout.flush()
    dir_pole_masks_default = dir_res_default + "/pole_masks"
    os.system("cp -R " + dir_pole_masks_default + " " + dir_pole_masks)

  # Open file (unbuffered)
  file_runtimes = open(dest_dir + "/runtimes.txt", 'w', 0)

  start_time = timer()

  os.chdir(surround360_render_dir)

  num_steps = sum([steps_unpack, steps_isp, steps_render, steps_ffmpeg])

  ### unpack step ###

  if steps_unpack:
    binary_files = [f for f in os.listdir(data_dir) if f.endswith('.bin')]
    binary_prefix = data_dir + "/" + os.path.commonprefix(binary_files)
    disk_count = len(binary_files)

    dir_raw = dest_dir + "/raw"
    if os.path.isdir(dir_raw) and len([f for f in os.listdir(dir_raw) if not f.startswith('.')]) > 0:
      print "ERROR: raw directory not empty!\n"
      sys.stdout.flush()
      exit(1)

    # If there is no cameranames we assume binaries are tagged with metadata
    unpack_extra_params = ""
    if not os.path.isfile(binary_prefix + "/cameranames.txt"):
      unpack_extra_params += " --tagged"
    else:
      unpack_extra_params += " --image_width " + image_width
      unpack_extra_params += " --image_height " + image_height
      unpack_extra_params += " --nbits " + nbits

    unpack_params = {
      "SURROUND360_RENDER_DIR": surround360_render_dir,
      "BINARY_PREFIX": binary_prefix,
      "ROOT_DIR": dir_raw,
      "VERBOSE_LEVEL": 1 if verbose else 0,
      "START_FRAME": start_frame,
      "FRAME_COUNT": frame_count,
      "DISK_COUNT": disk_count,
      "FLAGS_UNPACK_EXTRA": unpack_extra_params,
    }
    unpack_command = UNPACK_COMMAND_TEMPLATE.replace("\n", " ").format(**unpack_params)
    run_step("unpack", unpack_command, verbose, dryrun, file_runtimes, num_steps)

  # If unpack not in list of steps, we get frame count from raw directory
  cam0_image_dir = dest_dir + "/raw/cam0"
  if int(frame_count) == 0:
    if os.path.isdir(cam0_image_dir):
      frame_count = len(os.listdir(cam0_image_dir))
    else:
      print "No raw images in " + cam0_image_dir
      exit(1)

  file_runtimes.write("total frames: " + str(frame_count) + "\n")

  end_frame = int(start_frame) + int(frame_count) - 1

  ### ISP step ###

  if steps_isp:
    isp_extra_params = ""

    if verbose:
      isp_extra_params += " --verbose"

    # Force 16-bit output if input is 16-bit. Else use nbits flag
    image_path = list_only_files(cam0_image_dir)[0]
    image = Image.open(cam0_image_dir + "/" + image_path)
    nbits_isp = 16 if (image.mode == "I;16" or int(nbits) == 12) else 8;

    isp_params = {
      "SURROUND360_RENDER_DIR": surround360_render_dir,
      "ROOT_DIR": dest_dir,
      "START_FRAME": start_frame,
      "END_FRAME": end_frame,
      "ISP_DIR": dir_isp,
      "FLAGS_ISP_EXTRA": isp_extra_params,
      "NBITS": nbits_isp,
    }
    isp_command = ISP_COMMAND_TEMPLATE.replace("\n", " ").format(**isp_params)
    run_step("isp", isp_command, verbose, dryrun, file_runtimes, num_steps)

  ### render step ###

  if steps_render:
    render_extra_params = ""

    if enable_top:
      render_extra_params += " --enable_top"

    if enable_bottom:
      render_extra_params += " --enable_bottom"

      if enable_pole_removal:
        render_extra_params += " --enable_pole_removal"

    if new_rig_format:
      render_extra_params += " --new_rig_format"
    else:
      render_extra_params += " --rectify_file " + path_file_rectify
      render_extra_params += " --src_intrinsic_param_file " + path_file_instrinsics

    if save_debug_images:
      render_extra_params += " --save_debug_images"

    if verbose:
      render_extra_params += " --verbose"

    render_params = {
      "SURROUND360_RENDER_DIR": surround360_render_dir,
      "FLOW_ALG": "pixflow_low",
      "ROOT_DIR": dest_dir,
      "QUALITY": quality,
      "START_FRAME": start_frame,
      "END_FRAME": end_frame,
      "CUBEMAP_WIDTH": cubemap_width,
      "CUBEMAP_HEIGHT": cubemap_height,
      "CUBEMAP_FORMAT": cubemap_format,
      "RIG_JSON_FILE": path_file_camera_rig,
      "FLAGS_RENDER_EXTRA": render_extra_params,
    }
    render_command = RENDER_COMMAND_TEMPLATE.replace("\n", " ").format(**render_params)
    run_step("render", render_command, verbose, dryrun, file_runtimes, num_steps)

  ### ffmpeg step ###

  if steps_ffmpeg:
    ffmpeg_extra_params = ""

    if not verbose:
      ffmpeg_extra_params += " -loglevel error -stats"

    # Sequence name is the directory containing raw data
    sequence_name = binary_prefix.rsplit('/', 2)[-2]
    mp4_path = dest_dir + "/" + sequence_name + "_" + str(int(timer())) + "_" + quality + "_TB.mp4"

    ffmpeg_params = {
      "START_NUMBER": str(start_frame).zfill(FRAME_NUM_DIGITS),
      "ROOT_DIR": dest_dir,
      "MP4_PATH": mp4_path,
      "FLAGS_FFMPEG_EXTRA": ffmpeg_extra_params,
    }
    ffmpeg_command = FFMPEG_COMMAND_TEMPLATE.replace("\n", " ").format(**ffmpeg_params)
    run_step("ffmpeg", ffmpeg_command, verbose, dryrun, file_runtimes, num_steps)

  save_step_runtime(file_runtimes, "TOTAL", timer() - start_time)
  file_runtimes.close()
