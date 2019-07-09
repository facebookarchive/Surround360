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
import re
import subprocess
import sys
import threading
import time
from timeit import default_timer as timer

script_dir = os.path.dirname(os.path.realpath(__file__))

# os.path.dirname(DIR) is the parent directory of DIR
surround360_render_dir = os.path.dirname(script_dir)

TITLE = "Surround 360 - Color Calibration"

COLOR_CALIBRATION_COMMAND_TEMPLATE = """
{SURROUND360_RENDER_DIR}/bin/TestColorCalibration
--image_path "{IMAGE_PATH}"
--illuminant {ILLUMINANT}
--isp_passthrough_path "{ISP_JSON}"
--num_squares_w {NUM_SQUARES_W}
--num_squares_h {NUM_SQUARES_H}
--min_area_chart_perc {MIN_AREA_CHART_PERC}
--max_area_chart_perc {MAX_AREA_CHART_PERC}
--output_data_dir "{OUTPUT_DIR}"
--log_dir "{LOG_DIR}"
--logbuflevel -1
--stderrthreshold 0
{FLAGS_EXTRA}
"""

ILLUMINANTS = ["D50", "D65"]

def list_tiff(src_dir): return [os.path.join(src_dir, fn) for fn in next(os.walk(src_dir))[2] if fn.endswith('.tiff')]

def parse_args():
  parse_type = argparse.ArgumentParser
  parser = parse_type(description=TITLE, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--data_dir',                 help='directory containing raw calibration images', required=True)
  parser.add_argument('--output_dir',               help='output directory', required=False)
  parser.add_argument('--illuminant',               help='illuminant', required=False, choices=ILLUMINANTS, default=ILLUMINANTS[0])
  parser.add_argument('--black_level_hole',         help='if true, get black from black hole in image', action='store_true')
  parser.add_argument('--black_level_hole_pixels',  help='estimated size of black hole (pixels)', required=False, default=500)
  parser.add_argument('--black_level_y_intercept',  help='if true, get black level from Y-intercept of RGB response', action='store_true')
  parser.add_argument('--black_level_adjust',       help='if true, sets each channel black level to median of all cameras', action='store_true')
  parser.add_argument('--black_level',              help='manual black level', required=False, default='NONE')
  parser.add_argument('--num_squares_w',            help='number of squares horizontally', required=False, default='6')
  parser.add_argument('--num_squares_h',            help='number of squares vertically', required=False, default='4')
  parser.add_argument('--min_area_chart_perc',      help='expected min chart area (% of entire image)', required=False, default='0.5')
  parser.add_argument('--max_area_chart_perc',      help='expected max chart area (% of entire image)', required=False, default='40.0')

  return vars(parser.parse_args())

def start_subprocess(name, cmd):
  global current_process
  current_process = subprocess.Popen(cmd, shell=True)
  current_process.name = name
  current_process.communicate()

def print_and_save(file_out, str):
  print str
  file_out.write(str)
  sys.stdout.flush()

def save_step_runtime(file_out, step, runtime_sec):
  text_runtime = "\n" + step + " runtime: " + str(datetime.timedelta(seconds=runtime_sec)) + "\n"
  file_out.write(text_runtime)
  print text_runtime
  sys.stdout.flush()

def run_step(step, cmd, file_runtimes):
  file_runtimes.write("\n" + cmd + "\n")
  print cmd + "\n"
  sys.stdout.flush()
  start_time = timer()
  start_subprocess(step, cmd)
  save_step_runtime(file_runtimes, step, timer() - start_time)

def run_threads(thread_list):
  for thread in thread_list:
    thread.start()
  for thread in thread_list:
    thread.join()

def median(list):
  q, r = divmod(len(list), 2)
  return sorted(list)[q] if r else sum(sorted(list)[q - 1:q + 1]) / 2.0

if __name__ == "__main__":
  args = parse_args()
  data_dir                = args["data_dir"]
  illuminant              = args["illuminant"]
  black_level_hole        = args["black_level_hole"]
  black_level_hole_pixels = args["black_level_hole_pixels"]
  black_level_y_intercept = args["black_level_y_intercept"]
  black_level_adjust      = args["black_level_adjust"]
  black_level             = args["black_level"]
  num_squares_w           = int(args["num_squares_w"])
  num_squares_h           = int(args["num_squares_h"])
  min_area_chart_perc     = float(args["min_area_chart_perc"])
  max_area_chart_perc     = float(args["max_area_chart_perc"])

  print "\n--------" + time.strftime(" %a %b %d %Y %H:%M:%S %Z ") + "-------\n"

  os.chdir(surround360_render_dir)

  if illuminant not in ILLUMINANTS:
    sys.stderr.write("Unrecognized illuminant setting: " + illuminant + "\n")
    exit(1)

  if args["output_dir"] is not None:
    out_dir = args["output_dir"]
  else:
    out_dir = data_dir + "/output"
  os.system("mkdir -p \"" + out_dir + "\"")

  isp_dir = data_dir + "/isp"
  os.system("mkdir -p \"" + isp_dir + "\"")

  file_runtimes = open(out_dir + "/runtimes.txt", 'w', 0)
  start_time = timer()

  isp_passthrough_json = surround360_render_dir + "/res/config/isp/passthrough.json"
  raw_charts = list_tiff(data_dir + "/charts")

  flags_extra = ""
  if black_level_hole:
    flags_extra += " --black_level_hole --black_level_hole_pixels " + str(black_level_hole_pixels)
  elif black_level_y_intercept:
    flags_extra += " --black_level_y_intercept"
  elif black_level != 'NONE':
    flags_extra += " --black_level \"" + black_level + "\""

  flags_extra += " --save_debug_images"

  out_dirs = {}
  camera_names = {}
  thread_list = []
  for i in range(len(raw_charts)):
    camera_names[i] = os.path.basename(raw_charts[i]).split('.')[0]
    out_dirs[i] = out_dir + "/" + camera_names[i]
    color_calibrate_params = {
      "SURROUND360_RENDER_DIR": surround360_render_dir,
      "IMAGE_PATH": raw_charts[i],
      "ILLUMINANT": illuminant,
      "ISP_JSON": isp_passthrough_json,
      "NUM_SQUARES_W": num_squares_w,
      "NUM_SQUARES_H": num_squares_h,
      "MIN_AREA_CHART_PERC": min_area_chart_perc,
      "MAX_AREA_CHART_PERC": max_area_chart_perc,
      "OUTPUT_DIR": out_dirs[i],
      "LOG_DIR": out_dirs[i],
      "FLAGS_EXTRA": flags_extra,
    }
    color_calibrate_command = COLOR_CALIBRATION_COMMAND_TEMPLATE.replace("\n", " ").format(**color_calibrate_params)
    t = threading.Thread(target=run_step, args=(camera_names[i], color_calibrate_command, file_runtimes,))
    thread_list.append(t)

  run_threads(thread_list)

  if black_level_adjust:
    ### Adapt all cameras to same per-channel black level (median) ###

    print "Adjusting black levels...\n"

    step = "black_level_adjusted"

    NUM_CHANNELS = 3
    black_levels = [[] for j in range(NUM_CHANNELS)]

    for i in range(len(out_dirs)):
      black_level = json.loads(open("\"" + out_dirs[i] + "/black_level.txt\"").read())
      print_and_save(file_runtimes, camera_names[i] + ": " + str(black_level) + "\n")
      for j in range(NUM_CHANNELS):
        black_levels[j].append(black_level[j])
      out_dirs[i] += "_" + step

    black_level_median = [median(black_levels[j]) for j in range(NUM_CHANNELS)]

    print_and_save(file_runtimes, "Black level median: " + str(black_level_median) + "\n")
    flags_extra += " --black_level \"" + " ".join(map(str, black_level_median)) + "\""

    thread_list = []
    for i in range(len(raw_charts)):
      color_calibrate_params = {
        "SURROUND360_RENDER_DIR": surround360_render_dir,
        "IMAGE_PATH": raw_charts[i],
        "ILLUMINANT": illuminant,
        "ISP_JSON": isp_passthrough_json,
        "NUM_SQUARES_W": num_squares_w,
        "NUM_SQUARES_H": num_squares_h,
        "MIN_AREA_CHART_PERC": min_area_chart_perc,
        "MAX_AREA_CHART_PERC": max_area_chart_perc,
        "OUTPUT_DIR": out_dirs[i],
        "LOG_DIR": out_dirs[i],
        "FLAGS_EXTRA": flags_extra,
      }
      color_calibrate_command = COLOR_CALIBRATION_COMMAND_TEMPLATE.replace("\n", " ").format(**color_calibrate_params)
      t = threading.Thread(target=run_step, args=(camera_names[i] + " second pass", color_calibrate_command, file_runtimes,))
      thread_list.append(t)

    run_threads(thread_list)

  print "Finding worst-case X-intercepts...\n"

  intercept_x_min = 1.0
  intercept_x_max = 0.0
  print_and_save(file_runtimes, "\n")
  for i in range(len(out_dirs)):
    intercepts = json.loads(open(out_dirs[i] + "/intercept_x.txt").read())
    print_and_save(file_runtimes, camera_names[i] + ": " + str(intercepts) + "\n")

    intercept_x_max = max(intercept_x_max, max(intercepts[0]))
    intercept_x_min = min(intercept_x_min, min(intercepts[1]))

  text_intercepts = ("Intercept Xmin max: " + str(intercept_x_max) + ", " +
                     "Intercept Xmax min: " + str(intercept_x_min) + "\n")
  print_and_save(file_runtimes, text_intercepts)

  thread_list = []
  for i in range(len(out_dirs)):
    serial_number = re.findall("(\d+)", camera_names[i])[0]
    isp_src = out_dirs[i] + "/isp_out.json"
    isp_dst = isp_dir + "/" + serial_number + ".json"

    print "Copying " + isp_src + " to " + isp_dst + "..."
    os.system("cp \"" + isp_src + "\" \"" + isp_dst + "\"")

    flags_extra = " --update_clamps --clamp_min " + str(intercept_x_max) + " --clamp_max " + str(intercept_x_min)
    color_calibrate_params = {
      "SURROUND360_RENDER_DIR": surround360_render_dir,
      "IMAGE_PATH": raw_charts[i],
      "ILLUMINANT": illuminant,
      "ISP_JSON": isp_dst,
      "NUM_SQUARES_W": num_squares_w,
      "NUM_SQUARES_H": num_squares_h,
      "MIN_AREA_CHART_PERC": min_area_chart_perc,
      "MAX_AREA_CHART_PERC": max_area_chart_perc,
      "OUTPUT_DIR": out_dirs[i],
      "LOG_DIR": out_dirs[i],
      "FLAGS_EXTRA": flags_extra,
    }
    color_calibrate_command = COLOR_CALIBRATION_COMMAND_TEMPLATE.replace("\n", " ").format(**color_calibrate_params)
    t = threading.Thread(target=run_step, args=(camera_names[i] + " update clamps", color_calibrate_command, file_runtimes,))
    thread_list.append(t)

  print "Updating ISP clamps..."
  run_threads(thread_list)

  save_step_runtime(file_runtimes, "TOTAL", timer() - start_time)
  file_runtimes.close()
