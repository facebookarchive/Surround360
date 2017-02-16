# Copyright (c) 2016-present, Facebook, Inc.
# All rights reserved.
#
# This source code is licensed under the BSD-style license found in the
# LICENSE_render file in the root directory of this subproject. An additional grant
# of patent rights can be found in the PATENTS file in the same directory.

import argparse
import json
import multiprocessing
import os
import subprocess
import sys

from os import listdir, system
from os.path import isdir, isfile, join
from timeit import default_timer as timer

RAW2RGB_COMMAND_TEMPLATE = """
{SURROUND360_RENDER_DIR}/bin/Raw2Rgb
--logbuflevel -1
--log_dir {LOG_DIR}
--stderrthreshold 0
--input_image_path {INPUT_IMAGE_PATH}
--isp_config_path {ISP_CONFIG_PATH}
--black_level_offset {BLACK_LEVEL_OFFSET}
--output_image_path {OUTPUT_IMAGE_PATH}
--output_bpp {NBITS}
--accelerate
{FLAGS_RAW2RGB_EXTRA}
"""

CAM_TO_BLACKLEVEL = {
  "cam0":  0,
  "cam1":  0,
  "cam2":  0,
  "cam3":  0,
  "cam4":  0,
  "cam5":  0,
  "cam6":  0,
  "cam7":  0,
  "cam8":  0,
  "cam9":  0,
  "cam10": 0,
  "cam11": 0,
  "cam12": 0,
  "cam13": 0,
  "cam14": 0,
  "cam15": 0,
  "cam16": 0
}

def list_only_files(src_dir): return filter(lambda f: f[0] != ".", [f for f in listdir(src_dir) if isfile(join(src_dir, f))])
def list_only_dirs(src_dir): return filter(lambda d: d[0] != ".", [d for d in listdir(src_dir) if isdir(join(src_dir, d))])
def run_shell(cmd): return subprocess.call(cmd, shell=True)

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='batch process ISP')
  parser.add_argument('--root_dir', help='path to frame container dir', required=True)
  parser.add_argument('--surround360_render_dir', help='project root path, containing bin and scripts dirs', required=False, default='.')
  parser.add_argument('--nbits', help='bits per pixel', required=True)
  parser.add_argument('--start_frame', help='first frame index', required=True)
  parser.add_argument('--end_frame', help='last frame index', required=True)
  parser.add_argument('--delete_raws', dest='delete_raws', action='store_true')
  parser.add_argument('--isp_dir', help='directory containing ISP config files', required=True)
  parser.add_argument('--verbose', dest='verbose', action='store_true')
  parser.set_defaults(delete_raws=False)
  args = vars(parser.parse_args())

  root_dir                = args["root_dir"]
  surround360_render_dir  = args["surround360_render_dir"]
  nbits                   = args["nbits"]
  min_frame               = int(args["start_frame"])
  max_frame               = int(args["end_frame"])
  delete_raws             = args["delete_raws"]
  isp_dir                 = args["isp_dir"]
  verbose                 = args["verbose"]
  log_dir                 = root_dir + "/logs"

  start_time = timer()
  num_cpus = multiprocessing.cpu_count()
  pool = multiprocessing.Pool(processes=num_cpus)

  raw_dir = root_dir + "/raw"
  rgb_dir = root_dir + "/rgb"
  system("mkdir -p " + rgb_dir)

  cameras_dirs = list_only_dirs(raw_dir)
  for camera_dir in cameras_dirs:
    system("mkdir -p " + rgb_dir + "/" + camera_dir)

  # Get file extension from any file
  raw_extension = os.walk(raw_dir + "/" + cameras_dirs[0]).next()[2][0].split(".")[1]

  for i in range(min_frame, max_frame + 1):
    frame_to_process = format(i, "06d")
    print "----------- [ISP] processing frame:", frame_to_process
    sys.stdout.flush()

    start_frame_time = timer()
    isp_commands = []
    isp_files = sorted(list_only_files(isp_dir))

    for camera_dir in cameras_dirs:
      frame_semi_path = camera_dir + "/" + frame_to_process
      raw_img_path = raw_dir + "/" + frame_semi_path + "." + raw_extension
      rgb_img_path = rgb_dir + "/" + frame_semi_path + ".png"
      isp_config_path = isp_dir + "/" + isp_files[int(camera_dir.split("cam", 1)[1])]

      raw2rgb_extra_params = ""

      if delete_raws:
        raw2rgb_extra_params += " && rm " + raw_img_path

      raw2rgb_params = {
        "SURROUND360_RENDER_DIR": surround360_render_dir,
        "LOG_DIR": log_dir,
        "INPUT_IMAGE_PATH": raw_img_path,
        "ISP_CONFIG_PATH": isp_config_path,
        "BLACK_LEVEL_OFFSET": str(CAM_TO_BLACKLEVEL[camera_dir]),
        "OUTPUT_IMAGE_PATH": rgb_img_path,
        "NBITS": nbits,
        "FLAGS_RAW2RGB_EXTRA": raw2rgb_extra_params,
      }
      raw2rgb_command = RAW2RGB_COMMAND_TEMPLATE.replace("\n", " ").format(**raw2rgb_params)

      if verbose:
        print raw2rgb_command
        sys.stdout.flush()

      isp_commands.append(raw2rgb_command)

    isp_rets = pool.map(run_shell, isp_commands)

    if any(n != 0 for n in isp_rets):
      sys.stderr.write("Process failed\n")
      exit(1)

  end_time = timer()

  if verbose:
    total_runtime = end_time - start_time
    avg_runtime = total_runtime / float(max_frame - min_frame + 1)
    print "ISP total runtime:", total_runtime, "sec"
    print "Average runtime:", avg_runtime, "sec/frame"
    sys.stdout.flush()
