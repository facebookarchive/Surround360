#!/usr/bin/env python2
# Copyright (c) 2016-present, Facebook, Inc.
# All rights reserved.
#
# This source code is licensed under the BSD-style license found in the
# LICENSE_render file in the root directory of this subproject. An additional grant
# of patent rights can be found in the PATENTS file in the same directory.

import argparse
import datetime
import os
import subprocess
import sys
import time

from os import listdir
from os.path import isdir, isfile, join
from timeit import default_timer as timer

script_dir = os.path.dirname(os.path.realpath(__file__))

# os.path.dirname(DIR) is the parent directory of DIR
surround360_render_dir = os.path.dirname(script_dir)

TITLE = "Surround 360 - Vignetting Calibration"

ACQUISITION_COMMAND_TEMPLATE = """
{SURROUND360_RENDER_DIR}/bin/TestVignettingDataAcquisition
--input_dir "{INPUT_DIR}"
--output_dir "{OUTPUT_DIR}"
--isp_json "{ISP_JSON}"
--log_dir "{LOG_DIR}"
--logbuflevel -1
--stderrthreshold 0
{FLAGS_EXTRA}
"""

CALIBRATION_COMMAND_TEMPLATE = """
{SURROUND360_RENDER_DIR}/bin/TestVignettingCalibration
--data_path "{DATA_PATH}"
--output_dir "{OUTPUT_DIR}"
--test_image_path "{TEST_IMAGE_PATH}"
--test_isp_path "{TEST_ISP_PATH}"
--image_width {IMAGE_WIDTH}
--image_height {IMAGE_HEIGHT}
--logbuflevel -1
--stderrthreshold 0
{FLAGS_EXTRA}
"""

def list_dirs_numbers(src_dir):
  return filter(
    lambda f: f.isdigit(),
    [f for f in listdir(src_dir) if isdir(join(src_dir, f))])

def list_tiff(src_dir): return [os.path.join(src_dir, fn) for fn in next(os.walk(src_dir))[2] if fn.endswith('.tiff')]

def parse_args():
  parse_type = argparse.ArgumentParser
  parser = parse_type(description=TITLE, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--data_dir',             help='directory containing calibration images and ISP config files', required=True)
  parser.add_argument('--image_width',          help='image width', required=False, default=2048)
  parser.add_argument('--image_height',         help='image height', required=False, default=2048)
  parser.add_argument('--load_data',            help='skip acquisition step and load data containing locations and RGB medians ', action='store_true')
  parser.add_argument('--save_debug_images',    help='save debug images', action='store_true')

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
  print_and_save(file_runtimes, "\n" + cmd + "\n")
  start_time = timer()
  start_subprocess(step, cmd)
  save_step_runtime(file_runtimes, step, timer() - start_time)

def file_exists(file_path, file_runtimes):
  if not os.path.isfile(file_path):
    msg = "\nFile not found: " + file_path + "\n"
    print_and_save(file_runtimes, msg)
    return False
  return True

if __name__ == "__main__":
  args = parse_args()
  data_dir          = args["data_dir"]
  image_width       = args["image_width"]
  image_height      = args["image_height"]
  load_data         = args["load_data"]
  save_debug_images = args["save_debug_images"]

  print "\n--------" + time.strftime(" %a %b %d %Y %H:%M:%S %Z ") + "-------\n"

  os.chdir(surround360_render_dir)

  file_runtimes = open(data_dir + "/runtimes.txt", 'w', 0)
  start_time = timer()

  isp_new_dir = data_dir + "/isp_new"
  os.system("mkdir -p \"" + isp_new_dir + "\"")

  flags_extra = ""
  if save_debug_images:
    flags_extra += " --save_debug_images"

  for serial_number in list_dirs_numbers(data_dir):
    input_dir = data_dir + "/" + serial_number
    dir_acquisition = input_dir + "/acquisition"
    isp_json = data_dir + "/isp/" + serial_number + ".json"

    if not file_exists(isp_json, file_runtimes):
      sys.exit()

    if not load_data:
      print "[" + serial_number + "] Generating data for vignetting correction..."

      acquisition_params = {
        "SURROUND360_RENDER_DIR": surround360_render_dir,
        "INPUT_DIR": input_dir,
        "OUTPUT_DIR": dir_acquisition,
        "ISP_JSON": isp_json,
        "LOG_DIR": input_dir,
        "FLAGS_EXTRA": flags_extra,
      }
      acquisition_command = ACQUISITION_COMMAND_TEMPLATE.replace("\n", " ").format(**acquisition_params)
      run_step("acquisition", acquisition_command, file_runtimes)

    print "[" + serial_number + "] Computing vignetting..."

    data_json_file = dir_acquisition + "/data.json"

    if not file_exists(data_json_file, file_runtimes):
      sys.exit()

    dir_calibration = input_dir + "/calibration"
    filename = list_tiff(input_dir + "/charts")[0]

    if not file_exists(filename, file_runtimes):
      sys.exit()

    calibration_params = {
      "SURROUND360_RENDER_DIR": surround360_render_dir,
      "DATA_PATH": data_json_file,
      "OUTPUT_DIR": dir_calibration,
      "TEST_IMAGE_PATH": filename,
      "TEST_ISP_PATH": isp_json,
      "IMAGE_WIDTH": image_width,
      "IMAGE_HEIGHT": image_height,
      "FLAGS_EXTRA": flags_extra,
    }
    calibration_command = CALIBRATION_COMMAND_TEMPLATE.replace("\n", " ").format(**calibration_params)
    run_step("calibration", calibration_command, file_runtimes)

    isp_out = dir_calibration + "/isp_out.json"
    isp_updated = isp_new_dir + "/" + os.path.basename(isp_json)
    os.rename(isp_out, isp_updated)

  save_step_runtime(file_runtimes, "TOTAL", timer() - start_time)
  file_runtimes.close()
