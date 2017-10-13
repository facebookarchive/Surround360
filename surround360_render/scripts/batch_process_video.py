# Copyright (c) 2016-present, Facebook, Inc.
# All rights reserved.
#
# This source code is licensed under the BSD-style license found in the
# LICENSE_render file in the root directory of this subproject. An additional grant
# of patent rights can be found in the PATENTS file in the same directory.

import argparse
import os
import signal
import subprocess
import sys

from os import listdir, system
from os.path import isfile, join
from timeit import default_timer as timer

DELETE_OLD_FLOW_FILES = True # if true, we will trash flow files once we are done
DELETE_OLD_FLOW_IMAGES = True # if true, we will trash images used by flow once done

current_process = None
def signal_term_handler(signal, frame):
  if current_process:
    print "Terminating process: " + current_process.name + "..."
    current_process.terminate()
  sys.exit(0)

RENDER_COMMAND_TEMPLATE = """
{SURROUND360_RENDER_DIR}/bin/TestRenderStereoPanorama
--logbuflevel -1
--log_dir "{LOG_DIR}"
--stderrthreshold 0
--v {VERBOSE_LEVEL}
--rig_json_file "{RIG_JSON_FILE}"
--imgs_dir "{SRC_DIR}/rgb"
--frame_number {FRAME_ID}
--output_data_dir "{SRC_DIR}"
--prev_frame_data_dir "{PREV_FRAME_DIR}"
--output_cubemap_path "{OUT_CUBE_DIR}/cube_{FRAME_ID}.png"
--output_equirect_path "{OUT_EQR_DIR}/eqr_{FRAME_ID}.png"
--cubemap_format {CUBEMAP_FORMAT}
--side_flow_alg {SIDE_FLOW_ALGORITHM}
--polar_flow_alg {POLAR_FLOW_ALGORITHM}
--poleremoval_flow_alg {POLEREMOVAL_FLOW_ALGORITHM}
--cubemap_width {CUBEMAP_WIDTH}
--cubemap_height {CUBEMAP_HEIGHT}
--eqr_width {EQR_WIDTH}
--eqr_height {EQR_HEIGHT}
--final_eqr_width {FINAL_EQR_WIDTH}
--final_eqr_height {FINAL_EQR_HEIGHT}
--interpupilary_dist 6.4
--zero_parallax_dist 10000
--sharpening {SHARPENNING}
{EXTRA_FLAGS}
"""

def start_subprocess(name, cmd):
  global current_process
  current_process = subprocess.Popen(cmd, shell=True)
  current_process.name = name
  current_process.communicate()

def list_only_files(src_dir): return filter(lambda f: f[0] != ".", [f for f in listdir(src_dir) if isfile(join(src_dir, f))])

if __name__ == "__main__":
  signal.signal(signal.SIGTERM, signal_term_handler)

  parser = argparse.ArgumentParser(description='batch process video frames')
  parser.add_argument('--root_dir', help='path to frame container dir', required=True)
  parser.add_argument('--surround360_render_dir', help='project root path, containing bin and scripts dirs', required=False, default='.')
  parser.add_argument('--start_frame', help='first frame index', required=True)
  parser.add_argument('--end_frame', help='last frame index', required=True)
  parser.add_argument('--quality', help='3k,4k,6k,8k', required=True)
  parser.add_argument('--cubemap_width', help='default is to not generate cubemaps', required=False, default=0)
  parser.add_argument('--cubemap_height', help='default is to not generate cubemaps', required=False, default=0)
  parser.add_argument('--cubemap_format', help='photo,video', required=False, default='photo')
  parser.add_argument('--save_debug_images', dest='save_debug_images', action='store_true')
  parser.add_argument('--enable_top', dest='enable_top', action='store_true')
  parser.add_argument('--enable_bottom', dest='enable_bottom', action='store_true')
  parser.add_argument('--enable_pole_removal', dest='enable_pole_removal', action='store_true')
  parser.add_argument('--resume', dest='resume', action='store_true', help='looks for a previous frame optical flow instead of starting fresh')
  parser.add_argument('--rig_json_file', help='path to rig json file', required=True)
  parser.add_argument('--flow_alg', help='flow algorithm e.g., pixflow_low, pixflow_search_20', required=True)
  parser.add_argument('--verbose', dest='verbose', action='store_true')
  parser.set_defaults(save_debug_images=False)
  parser.set_defaults(enable_top=False)
  parser.set_defaults(enable_bottom=False)
  parser.set_defaults(enable_pole_removal=False)
  args = vars(parser.parse_args())

  root_dir                  = args["root_dir"]
  surround360_render_dir    = args["surround360_render_dir"]
  log_dir                   = root_dir + "/logs"
  out_eqr_frames_dir        = root_dir + "/eqr_frames"
  out_cube_frames_dir       = root_dir + "/cube_frames"
  flow_dir                  = root_dir + "/flow"
  debug_dir                 = root_dir + "/debug"
  min_frame                 = int(args["start_frame"])
  max_frame                 = int(args["end_frame"])
  cubemap_width             = int(args["cubemap_width"])
  cubemap_height            = int(args["cubemap_height"])
  cubemap_format            = args["cubemap_format"]
  quality                   = args["quality"]
  save_debug_images         = args["save_debug_images"]
  enable_top                = args["enable_top"]
  enable_bottom             = args["enable_bottom"]
  enable_pole_removal       = args["enable_pole_removal"]
  resume                    = args["resume"]
  rig_json_file             = args["rig_json_file"]
  flow_alg                  = args["flow_alg"]
  verbose                   = args["verbose"]

  start_time = timer()

  os.system("mkdir -p \"" + out_eqr_frames_dir + "\"")
  os.system("mkdir -p \"" + out_cube_frames_dir + "\"")
  os.system("mkdir -p \"" + flow_dir + "\"")

  brightness_adjust_path = root_dir + "/brightness_adjust.txt"
  frame_range = range(min_frame, max_frame + 1)

  for i in frame_range:
    frame_to_process = format(i, "06d")
    is_first_frame = (i == min_frame)

    print "----------- [Render] processing frame ", i, " of ", max_frame
    sys.stdout.flush()

    debug_frame_dir = debug_dir + "/" + frame_to_process
    flow_images_dir = debug_frame_dir + "/flow_images"
    projections_dir = debug_frame_dir + "/projections"
    os.system("mkdir -p \"" + flow_dir + "/" + frame_to_process + "\"")
    os.system("mkdir -p \"" + flow_images_dir + "\"")
    os.system("mkdir -p \"" + projections_dir + "\"")

    prev_frame_dir = format(i - 1, "06d")
    render_params = {
      "SURROUND360_RENDER_DIR": surround360_render_dir,
      "LOG_DIR": log_dir,
      "VERBOSE_LEVEL": 1 if verbose else 0,
      "FRAME_ID": frame_to_process,
      "SRC_DIR": root_dir,
      "PREV_FRAME_DIR": "NONE",
      "OUT_EQR_DIR": out_eqr_frames_dir,
      "OUT_CUBE_DIR": out_cube_frames_dir,
      "CUBEMAP_WIDTH": cubemap_width,
      "CUBEMAP_HEIGHT": cubemap_height,
      "CUBEMAP_FORMAT": cubemap_format,
      "RIG_JSON_FILE": rig_json_file,
      "SIDE_FLOW_ALGORITHM": flow_alg,
      "POLAR_FLOW_ALGORITHM": flow_alg,
      "POLEREMOVAL_FLOW_ALGORITHM": flow_alg,
      "EXTRA_FLAGS": "",
    }

    if resume or not is_first_frame:
      render_params["PREV_FRAME_DIR"] = "\"" + prev_frame_dir + "\""

    if save_debug_images:
      render_params["EXTRA_FLAGS"] += " --save_debug_images"

    if enable_top:
      render_params["EXTRA_FLAGS"] += " --enable_top"

    if enable_pole_removal and enable_bottom is False:
      sys.stderr.write("Cannot use enable_pole_removal if enable_bottom is not used")
      exit(1)

    if enable_bottom:
      render_params["EXTRA_FLAGS"] += " --enable_bottom"
      if enable_pole_removal:
        render_params["EXTRA_FLAGS"] += " --enable_pole_removal"
        render_params["EXTRA_FLAGS"] += " --bottom_pole_masks_dir \"" + root_dir + "/pole_masks\""

    if quality == "3k":
      render_params["SHARPENNING"]                  = 0.25
      render_params["EQR_WIDTH"]                    = 3080
      render_params["EQR_HEIGHT"]                   = 1540
      render_params["FINAL_EQR_WIDTH"]              = 3080
      render_params["FINAL_EQR_HEIGHT"]             = 3080
    elif quality == "4k":
      render_params["SHARPENNING"]                  = 0.25
      render_params["EQR_WIDTH"]                    = 4200
      render_params["EQR_HEIGHT"]                   = 1024
      render_params["FINAL_EQR_WIDTH"]              = 4096
      render_params["FINAL_EQR_HEIGHT"]             = 2048
    elif quality == "6k":
      render_params["SHARPENNING"]                  = 0.25
      render_params["EQR_WIDTH"]                    = 6300
      render_params["EQR_HEIGHT"]                   = 3072
      render_params["FINAL_EQR_WIDTH"]              = 6144
      render_params["FINAL_EQR_HEIGHT"]             = 6144
    elif quality == "8k":
      render_params["SHARPENNING"]                  = 0.25
      render_params["EQR_WIDTH"]                    = 8400
      render_params["EQR_HEIGHT"]                   = 4096
      render_params["FINAL_EQR_WIDTH"]              = 8192
      render_params["FINAL_EQR_HEIGHT"]             = 8192
    else:
      sys.stderr.write("Unrecognized quality setting: " + quality)
      exit(1)

    render_command = RENDER_COMMAND_TEMPLATE.replace("\n", " ").format(**render_params)

    if verbose:
      print render_command
      sys.stdout.flush()

    start_subprocess("render", render_command)

    if DELETE_OLD_FLOW_FILES and not is_first_frame:
      rm_old_flow_command = "rm \"" + flow_dir + "/" + prev_frame_dir + "/*\""

      if verbose:
        print rm_old_flow_command
        sys.stdout.flush()

      subprocess.call(rm_old_flow_command, shell=True)

    if DELETE_OLD_FLOW_IMAGES and not is_first_frame:
      rm_old_flow_images_command = "rm \"" + debug_dir + "/" + prev_frame_dir + "/flow_images/*\""

      if verbose:
        print rm_old_flow_images_command
        sys.stdout.flush()

      subprocess.call(rm_old_flow_images_command, shell=True)

  end_time = timer()

  if verbose:
    total_runtime = end_time - start_time
    avg_runtime = total_runtime / float(max_frame - min_frame + 1)
    print "Render total runtime:", total_runtime, "sec"
    print "Average runtime:", avg_runtime, "sec/frame"
    sys.stdout.flush()
