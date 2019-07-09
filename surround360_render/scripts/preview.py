#!/usr/bin/env python2
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
import string
import subprocess
import sys
import time
import types
from timeit import default_timer as timer

# Gooey imports
USE_GOOEY = (len(sys.argv) == 1 or "--ignore-gooey" in sys.argv)
if USE_GOOEY: from gooey import Gooey, GooeyParser # only load Gooey if needed
else: Gooey = lambda program_name, image_dir: (lambda fn: fn) # dummy for decorator
def conditional_decorator(pred, dec): return lambda fn: dec(fn) if pred else fn

script_dir = os.path.dirname(os.path.realpath(__file__))
surround360_render_dir = os.path.dirname(script_dir)

TITLE = "Surround 360 - Preview"
NUM_CAMS = 17
FRAME_NUM_DIGITS = 6

PREVIEW_COMMAND_TEMPLATE = """
time {SURROUND360_RENDER_DIR}/bin/TestHyperPreview
--logbuflevel -1 --stderrthreshold 0 --v 0
--log_dir "{DEST_DIR}/logs"
--rig_json_file "{RIG_JSON_FILE}"
--binary_prefix "{BINARY_PREFIX}"
--preview_dest "{DEST_DIR}/eqr_preview"
--start_frame {START_FRAME}
--frame_count {FRAME_COUNT}
--eqr_width {EQR_WIDTH}
--eqr_height {EQR_HEIGHT}
--gamma {GAMMA}
 --file_count 2
"""

FFMPEG_COMMAND_TEMPLATE = """
time ffmpeg
-y
-framerate 30
-i "{DEST_DIR}/eqr_preview/%06d.jpg"
-pix_fmt yuv420p
-c:v libx264
-crf 20
-preset ultrafast
"{DEST_DIR}/{VIDEO_NAME}preview.mp4"
"""

@conditional_decorator(USE_GOOEY, Gooey(program_name=TITLE, image_dir=os.path.dirname(script_dir) + "/res/img"))
def parse_args():
  dir_chooser = {"widget": "DirChooser"} if USE_GOOEY else {}
  file_chooser = {"widget": "FileChooser"} if USE_GOOEY else {}

  parse_type = GooeyParser if USE_GOOEY else argparse.ArgumentParser
  parser = parse_type(description=TITLE, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--data_dir',         metavar='Data Directory', help='directory containing .bin files', required=True, **dir_chooser)
  parser.add_argument('--dest_dir',         metavar='Destination Directory', help='destination directory', required=True, **({"widget": "DirChooser"} if USE_GOOEY else {}))
  parser.add_argument('--start_frame',      metavar='Start Frame', help='start frame', required=False, default='0')
  parser.add_argument('--frame_count',      metavar='Frame Count', help='0 = all', required=False, default='0')
  parser.add_argument('--rig_json_file',    metavar='Rig Geometry File', help='json file with rig geometry info', required=False, default="./res/config/17cmosis_default.json", **file_chooser)
  parser.add_argument('--gamma',            metavar='Exponent for gamma correction',required=False, default='1.0')
  parser.add_argument('--eqr_width',        metavar='Output equirect width', required=False, default='2048')
  parser.add_argument('--eqr_height',       metavar='Output equirect height',required=False, default='1024')

  return vars(parser.parse_args())

current_process = None
def signal_term_handler(signal, frame):
  if current_process:
    print "Terminating process: " + current_process.name + "..."
    current_process.terminate()
  sys.exit(0)

def start_subprocess(name, cmd):
  global current_process
  current_process = subprocess.Popen(cmd, shell=True)
  current_process.name = name
  current_process.communicate()

if __name__ == "__main__":
  signal.signal(signal.SIGTERM, signal_term_handler)

  args = parse_args()
  data_dir                  = args["data_dir"]
  dest_dir                  = args["dest_dir"]
  start_frame               = int(args["start_frame"])
  frame_count               = int(args["frame_count"])
  rig_json_file             = args["rig_json_file"]
  eqr_width                 = args["eqr_width"]
  eqr_height                = args["eqr_height"]
  gamma                     = args["gamma"]

  os.chdir(surround360_render_dir)

  binary_files = [f for f in os.listdir(data_dir) if f.endswith('.bin')]
  binary_prefix = data_dir + "/" + os.path.commonprefix(binary_files)
  file_count = len(binary_files)

  os.system("mkdir -p " + dest_dir + "/logs")
  os.system("mkdir -p " + dest_dir + "/eqr_preview")

  preview_params = {
    "SURROUND360_RENDER_DIR": surround360_render_dir,
    "BINARY_PREFIX": binary_prefix,
    "DEST_DIR": dest_dir,
    "START_FRAME": start_frame,
    "FRAME_COUNT": frame_count,
    "FILE_COUNT": file_count,
    "RIG_JSON_FILE": rig_json_file,
    "EQR_WIDTH": eqr_width,
    "EQR_HEIGHT": eqr_height,
    "GAMMA": gamma,
  }
  preview_command = PREVIEW_COMMAND_TEMPLATE.replace("\n", " ").format(**preview_params)
  start_subprocess("preview", preview_command)

  video_name = binary_prefix.split("/")[-1]
  print "video_name=", video_name
  ffmpeg_params = {
    "SURROUND360_RENDER_DIR": surround360_render_dir,
    "DEST_DIR": dest_dir,
    "VIDEO_NAME": video_name,
  }
  ffmpeg_command = FFMPEG_COMMAND_TEMPLATE.replace("\n", " ").format(**ffmpeg_params)
  start_subprocess("ffmpeg", ffmpeg_command)
