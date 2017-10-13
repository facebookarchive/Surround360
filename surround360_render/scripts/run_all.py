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
{SURROUND360_RENDER_DIR}/bin/Unpacker
--logbuflevel -1
--log_dir "{ROOT_DIR}/logs"
--stderrthreshold 0
--isp_dir "{ISP_DIR}"
--output_dir "{ROOT_DIR}/rgb"
--bin_list "{BIN_LIST}"
--start_frame {START_FRAME}
--frame_count {FRAME_COUNT}
{FLAGS_UNPACK_EXTRA}
"""

RENDER_COMMAND_TEMPLATE = """
python {SURROUND360_RENDER_DIR}/scripts/batch_process_video.py
--flow_alg {FLOW_ALG}
--root_dir "{ROOT_DIR}"
--surround360_render_dir "{SURROUND360_RENDER_DIR}"
--quality {QUALITY}
--start_frame {START_FRAME}
--end_frame {END_FRAME}
--cubemap_width {CUBEMAP_WIDTH}
--cubemap_height {CUBEMAP_HEIGHT}
--cubemap_format {CUBEMAP_FORMAT}
--rig_json_file "{RIG_JSON_FILE}"
{FLAGS_RENDER_EXTRA}
"""

FFMPEG_COMMAND_TEMPLATE = """
ffmpeg
-framerate 30
-start_number {START_NUMBER}
-i "{ROOT_DIR}/eqr_frames/eqr_%06d.png"
-pix_fmt yuv420p
-c:v libx264
-crf 10
-profile:v high
-tune fastdecode
-bf 0
-refs 3
-preset fast "{MP4_PATH}"
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
  parser.add_argument('--start_frame',                metavar='Start Frame', help='start frame', required=False, default='0')
  parser.add_argument('--frame_count',                metavar='Frame Count', help='0 = all', required=False, default='0')
  parser.add_argument('--quality',                    metavar='Quality', help='final output quality', required=False, choices=['3k', '4k', '6k', '8k'], default='6k')
  parser.add_argument('--cubemap_format',             metavar='Cubemap Format', help='photo or video', required=False, choices=['photo', 'video'], default='video')
  parser.add_argument('--cubemap_width',              metavar='Cubemap Face Width', help='0 = no cubemaps', required=False, default='0')
  parser.add_argument('--cubemap_height',             metavar='Cubemap Face Height', help='0 = no cubemaps', required=False, default='0')
  parser.add_argument('--save_debug_images',          help='Save debug images', action='store_true')
  parser.add_argument('--save_raw',                   help='Save RAW images', action='store_true')
  parser.add_argument('--steps_unpack',               help='Step 1: convert data in .bin files to RGB files', action='store_true', required=False)
  parser.add_argument('--steps_render',               help='Step 2: render stereo panoramas', action='store_true', required=False)
  parser.add_argument('--steps_ffmpeg',               help='Step 3: create video output', action='store_true', required=False)
  parser.add_argument('--enable_top',                 help='Enable top camera', action='store_true')
  parser.add_argument('--enable_bottom',              help='Enable bottom camera', action='store_true')
  parser.add_argument('--enable_pole_removal',        help='false = use primary bottom camera', action='store_true')
  parser.add_argument('--dryrun',                     help='Do not execute steps', action='store_true')
  parser.add_argument('--verbose',                    help='Increase output verbosity', action='store_true')

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

def exit_with_error(message):
  print message
  sys.stdout.flush()
  exit(1)

if __name__ == "__main__":
  signal.signal(signal.SIGTERM, signal_term_handler)

  args = parse_args()
  data_dir                  = args["data_dir"]
  dest_dir                  = args["dest_dir"]
  start_frame               = int(args["start_frame"])
  frame_count               = int(args["frame_count"])
  quality                   = args["quality"]
  cubemap_width             = int(args["cubemap_width"])
  cubemap_height            = int(args["cubemap_height"])
  cubemap_format            = args["cubemap_format"]
  enable_top                = args["enable_top"]
  enable_bottom             = args["enable_bottom"]
  enable_pole_removal       = args["enable_pole_removal"]
  save_debug_images         = args["save_debug_images"]
  save_raw                  = args["save_raw"]
  dryrun                    = args["dryrun"]
  steps_unpack              = args["steps_unpack"]
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

  os.system("mkdir -p \"" + dest_dir + "/logs\"")

  print "Checking required files..."

  res_default_dir = surround360_render_dir + "/res"
  config_dir = dest_dir + "/config"
  os.system("mkdir -p \"" + config_dir + "\"")

  file_camera_rig = "camera_rig.json"
  path_file_camera_rig = config_dir + "/" + file_camera_rig
  if not os.path.isfile(path_file_camera_rig):
    print "WARNING: Calibration file not found. Using default file.\n"
    sys.stdout.flush()
    path_file_camera_rig_default = res_default_dir + "/config/" + file_camera_rig
    os.system("cp \"" + path_file_camera_rig_default + "\" \"" + path_file_camera_rig + "\"")

  pole_masks_dir = dest_dir + "/pole_masks"
  if not os.path.isdir(pole_masks_dir):
    print "WARNING: Pole masks not found. Using default files.\n"
    sys.stdout.flush()
    pole_masks_default_dir = res_default_dir + "/pole_masks"
    os.system("cp -R \"" + pole_masks_default_dir + "\" \"" + pole_masks_dir + "\"")

  # Open file (unbuffered)
  file_runtimes = open(dest_dir + "/runtimes.txt", 'w', 0)

  start_time = timer()

  os.chdir(surround360_render_dir)

  num_steps = sum([steps_unpack, steps_render, steps_ffmpeg])

  ### unpack step ###

  if steps_unpack:
    isp_dir = config_dir + "/isp"
    rgb_dir = dest_dir + "/rgb"
    os.system("mkdir -p \"" + rgb_dir + "\"")
    binary_files = [join(data_dir, f) for f in os.listdir(data_dir) if f.endswith('.bin')]
    unpack_extra_params = ""
    if save_raw:
      raw_dir = dest_dir + "/raw"
      os.system("mkdir -p \"" + raw_dir + "\"")
      unpack_extra_params += " --output_raw_dir \"" + raw_dir + "\""
      if os.path.isdir(raw_dir) and len([f for f in os.listdir(raw_dir) if not f.startswith('.')]) > 0:
        exit_with_error("ERROR: raw directory not empty!")
    else:
      # Only check for ISP files here. We could still unpack RAWs without them
      if not os.path.isdir(isp_dir) or os.listdir(isp_dir) == []:
        exit_with_error("ERROR: No color adjustment files not found in " + isp_dir)

    unpack_params = {
      "SURROUND360_RENDER_DIR": surround360_render_dir,
      "BIN_LIST": ",".join(binary_files),
      "ROOT_DIR": dest_dir,
      "ISP_DIR": isp_dir,
      "START_FRAME": start_frame,
      "FRAME_COUNT": frame_count,
      "FLAGS_UNPACK_EXTRA": unpack_extra_params,
    }
    unpack_command = UNPACK_COMMAND_TEMPLATE.replace("\n", " ").format(**unpack_params)
    run_step("unpack", unpack_command, verbose, dryrun, file_runtimes, num_steps)

  ### render step ###

  if steps_render:
    # Check if we have images in rgb directory
    cam0_image_dir = dest_dir + "/rgb/cam0"
    if not os.path.isdir(cam0_image_dir) or os.listdir(cam0_image_dir) == []:
      exit_with_error("ERROR: No RGB images in " + cam0_image_dir)

    frame_count = len(os.listdir(cam0_image_dir))
    file_runtimes.write("total frames: " + str(frame_count) + "\n")
    end_frame = int(start_frame) + int(frame_count) - 1

    render_extra_params = ""

    if enable_top:
      render_extra_params += " --enable_top"

    if enable_bottom:
      render_extra_params += " --enable_bottom"

      if enable_pole_removal:
        render_extra_params += " --enable_pole_removal"

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
    sequence_name = data_dir.rsplit('/', 2)[-1]
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
