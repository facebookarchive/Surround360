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
--dest_path {ROOT_DIR}/raw
--start_frame {START_FRAME}
--frame_count {FRAME_COUNT}
--file_count {DISK_COUNT}
{FLAGS_UNPACK_EXTRA}
"""

ARRANGE_COMMAND_TEMPLATE = """
python {SURROUND360_RENDER_DIR}/scripts/arrange_dataset.py
--root_dir {ROOT_DIR}
{FLAGS_ARRANGE_EXTRA}
"""

ISP_COMMAND_TEMPLATE = """
python {SURROUND360_RENDER_DIR}/scripts/batch_process_isp.py
--surround360_render_dir {SURROUND360_RENDER_DIR}
--root_dir {ROOT_DIR}
--start_frame {START_FRAME}
--end_frame {END_FRAME}
--cam_to_isp_config_file {CAM_TO_ISP_CONFIG_FILE}
{FLAGS_ISP_EXTRA}
"""

RECTIFY_COMMAND_TEMPLATE = """
{SURROUND360_RENDER_DIR}/bin/TestRingRectification
--logbuflevel -1 --stderrthreshold 0 --v 0 \
--rig_json_file {RIG_JSON_FILE} \
--src_intrinsic_param_file {SRC_INSTRINSIC_PARAM_FILE} \
--output_transforms_file {RECTIFY_FILE} \
--root_dir {ROOT_DIR} \
--frames_list {FRAME_LIST} \
--visualization_dir {ROOT_DIR}/rectify_vis
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
--src_intrinsic_param_file {SRC_INSTRINSIC_PARAM_FILE}
--rectify_file {RECTIFY_FILE}
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

def create_default_path(path, default_str):
  return path if os.path.exists(path) else default_str

@conditional_decorator(USE_GOOEY, Gooey(program_name=TITLE, image_dir=os.path.dirname(script_dir) + "/res/img"))
def parse_args():
  res_path = surround360_render_dir + "/res"
  dir_chooser = {"widget": "DirChooser"} if USE_GOOEY else {}
  file_chooser = {"widget": "FileChooser"} if USE_GOOEY else {}

  # Create default paths (if they exist)
  cam_to_isp_config_file = res_path + "/config/isp/cam_to_isp_config.json"
  pole_masks_dir = res_path + "/pole_masks"
  src_intrinsic_param_file = res_path + "/config/sunex_intrinsic.xml"
  rectify_file = res_path + "/config/rectify.yml"
  rig_json_file = res_path + "/config/17cmosis_default.json"

  # Make sure we have per camera color adjustment files. If not, copy from template
  config_isp_path = res_path + "/config/isp"
  if not os.path.isfile(cam_to_isp_config_file):
    print "WARNING: Color adjustment files not found. Using default files.\n"
    sys.stdout.flush()
    duplicate_isp_files(config_isp_path)
    update_isp_mappings(cam_to_isp_config_file, config_isp_path)

  parse_type = GooeyParser if USE_GOOEY else argparse.ArgumentParser
  parser = parse_type(description=TITLE, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--data_dir',                   metavar='Data Directory', help='directory containing .bin files', required=True, **dir_chooser)
  parser.add_argument('--dest_dir',                   metavar='Destination Directory', help='destination directory', required=True, **({"widget": "DirChooser"} if USE_GOOEY else {}))
  parser.add_argument('--image_width',                metavar='Image Width', help='image width (ignore if no cameranames.txt)', required=False, default='2048')
  parser.add_argument('--image_height',               metavar='Image Height', help='image height (ignore if no cameranames.txt)', required=False, default='2048')
  parser.add_argument('--nbits',                      metavar='Bit Depth', help='bit depth (ignore if no cameranames.txt)', required=False, choices=['8', '12'], default='8')
  parser.add_argument('--quality',                    metavar='Quality', help='final output quality', required=False, choices=['3k', '4k', '6k', '8k'], default='6k')
  parser.add_argument('--start_frame',                metavar='Start Frame', help='start frame', required=False, default='0')
  parser.add_argument('--frame_count',                metavar='Frame Count', help='0 = all', required=False, default='0')
  parser.add_argument('--cubemap_format',             metavar='Cubemap Format', help='photo or video', required=False, choices=['photo', 'video'], default='video')
  parser.add_argument('--cubemap_width',              metavar='Cubemap Face Width', help='0 = no cubemaps', required=False, default='0')
  parser.add_argument('--cubemap_height',             metavar='Cubemap Face Height', help='0 = no cubemaps', required=False, default='0')
  parser.add_argument('--save_debug_images',          help='save debug images', action='store_true')
  parser.add_argument('--steps_unpack',               help='Step 1: convert data in .bin files to RAW .tiff files', action='store_true', required=False)
  parser.add_argument('--steps_arrange',              help='Step 2: arrange .tiff files for further processing', action='store_true', required=False)
  parser.add_argument('--steps_isp',                  help='Step 3: convert RAW frames to RGB', action='store_true', required=False)
  parser.add_argument('--steps_rectify',              help='Step 4: use/create rectification file', action='store_true', required=False)
  parser.add_argument('--steps_render',               help='Step 5: render PNG stereo panoramas', action='store_true', required=False)
  parser.add_argument('--steps_ffmpeg',               help='Step 6: create MP4 output', action='store_true', required=False)
  parser.add_argument('--enable_top',                 help='enable top camera', action='store_true')
  parser.add_argument('--enable_bottom',              help='enable bottom camera', action='store_true')
  parser.add_argument('--enable_pole_removal',        help='false = use primary bottom camera', action='store_true')
  parser.add_argument('--enable_render_coloradjust',  help='modify color/brightness in the renderer to improve blending (increases runtime)', action='store_true')
  parser.add_argument('--dryrun',                     help='do not execute steps', action='store_true')
  parser.add_argument('--flow_alg',                   metavar='Flow Algorithm', help='optical flow algorithm', required=False, choices=['pixflow_low', 'pixflow_search_20'], default='pixflow_low')
  parser.add_argument('--cam_to_isp_config_file',     metavar='Camera to ISP Mappings File', help='camera to ISP config file mappings', required=False, default=create_default_path(cam_to_isp_config_file, ""), **file_chooser)
  parser.add_argument('--pole_masks_dir',             metavar='Pole Masks Directory', help='directory containing pole masks', required=False, default=create_default_path(pole_masks_dir, ""), **dir_chooser)
  parser.add_argument('--src_intrinsic_param_file',   metavar='Intrinsic Parameters File', help='intrinsic parameters file', required=False, default=create_default_path(src_intrinsic_param_file, ""), **file_chooser)
  parser.add_argument('--rectify_file',               metavar='Rectification File', help='rectification file [or NONE]', required=False, default=create_default_path(rectify_file, "NONE"), **file_chooser)
  parser.add_argument('--rig_json_file',              metavar='Rig Geometry File', help='json file with rig geometry info', required=False, default=create_default_path(rig_json_file, ""), **file_chooser)
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

def duplicate_isp_files(config_isp_path):
  cmosis_fujinon_path = config_isp_path + "/cmosis_fujinon.json"
  cmosis_sunex_path = config_isp_path + "/cmosis_sunex.json"

  for i in range(0, 17):
    cmosis_path = cmosis_fujinon_path if i in [0, 15, 16] else cmosis_sunex_path
    os.system("cp " + cmosis_path + " " + config_isp_path + "/isp" + str(i) + ".json")

def update_isp_mappings(cam_to_isp_config_file, config_isp_path):
  cam_json_map = {}
  for i in range(0, NUM_CAMS):
    cam_json_map["cam" + str(i)] = config_isp_path + "/isp" + str(i) + ".json"

  with open(cam_to_isp_config_file, "w") as json_file:
    json.dump(cam_json_map, json_file, indent=4, sort_keys=True)

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
  enable_render_coloradjust = args["enable_render_coloradjust"]
  save_debug_images         = args["save_debug_images"]
  dryrun                    = args["dryrun"];
  steps_unpack              = args["steps_unpack"]
  steps_arrange             = args["steps_arrange"]
  steps_isp                 = args["steps_isp"]
  steps_rectify             = args["steps_rectify"]
  steps_render              = args["steps_render"]
  steps_ffmpeg              = args["steps_ffmpeg"]
  flow_alg                  = args["flow_alg"]
  cam_to_isp_config_file    = args["cam_to_isp_config_file"]
  pole_masks_dir            = args["pole_masks_dir"]
  src_intrinsic_param_file  = args["src_intrinsic_param_file"]
  rectify_file              = args["rectify_file"]
  rig_json_file             = args["rig_json_file"]
  verbose                   = args["verbose"]

  print "\n--------" + time.strftime(" %a %b %d %Y %H:%M:%S %Z ") + "-------\n"

  if dryrun:
    verbose = True

  if USE_GOOEY:
    print_runall_command(args)

  binary_files = [f for f in os.listdir(data_dir) if f.endswith('.bin')]
  binary_prefix = data_dir + "/" + os.path.commonprefix(binary_files)
  disk_count = len(binary_files)

  # We want to be able to map a file path to its flag/variable name, so we keep
  # a dictionary of mappings between them
  paths_map = dict((eval(name), name) for name in ['cam_to_isp_config_file', 'src_intrinsic_param_file', 'rectify_file', 'rig_json_file'])
  for path, name in paths_map.iteritems():
    if not os.path.isfile(path):
      # rectify_file can be set to NONE
      if (path != rectify_file or path != "NONE") and "rectify" not in step_list:
        sys.stderr.write("Given --" + name + " (" + path + ") does not exist\n")
        exit(1)

  if quality not in ["3k", "4k", "6k", "8k"]:
    sys.stderr.write("Unrecognized quality setting: " + quality)
    exit(1)

  if enable_pole_removal and enable_bottom is False:
    sys.stderr.write("Cannot use enable_pole_removal if enable_bottom is not used")
    exit(1)

  os.system("mkdir -p " + dest_dir + "/logs")

  # Open file (unbuffered)
  file_runtimes = open(dest_dir + "/runtimes.txt", 'w', 0)

  start_time = timer()

  os.chdir(surround360_render_dir)

  num_steps = sum([steps_unpack, steps_arrange, steps_isp, steps_rectify, steps_render, steps_ffmpeg])

  ### unpack step ###

  if steps_unpack:
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
      "ROOT_DIR": dest_dir,
      "VERBOSE_LEVEL": 1 if verbose else 0,
      "START_FRAME": start_frame,
      "FRAME_COUNT": frame_count,
      "DISK_COUNT": disk_count,
      "FLAGS_UNPACK_EXTRA": unpack_extra_params,
    }
    unpack_command = UNPACK_COMMAND_TEMPLATE.replace("\n", " ").format(**unpack_params)
    run_step("unpack", unpack_command, verbose, dryrun, file_runtimes, num_steps)

  if int(frame_count) == 0 and os.path.isdir(dest_dir + "/vid"):
    frame_count = len(os.listdir(dest_dir + "/vid"))

  ### arrange step ###

  if steps_arrange:
    arrange_extra_params = ""

    if enable_pole_removal:
      arrange_extra_params += " --pole_masks_dir " + pole_masks_dir

    if verbose:
      arrange_extra_params += " --verbose"

    arrange_params = {
      "SURROUND360_RENDER_DIR": surround360_render_dir,
      "ROOT_DIR": dest_dir,
      "FLAGS_ARRANGE_EXTRA": arrange_extra_params,
    }
    arrange_command = ARRANGE_COMMAND_TEMPLATE.replace("\n", " ").format(**arrange_params)
    run_step("arrange", arrange_command, verbose, dryrun, file_runtimes, num_steps)

  # If unpack not in list of steps, we get frame count from vid directory
  if int(frame_count) == 0:
    dir_vid = dest_dir + "/vid"
    frame_count = len(os.listdir(dest_dir + "/vid")) if os.path.isdir(dir_vid) else 0

  file_runtimes.write("total frames: " + str(frame_count) + "\n")

  end_frame = int(start_frame) + int(frame_count) - 1

  ### isp step ###

  if steps_isp:
    isp_extra_params = ""

    if verbose:
      isp_extra_params += " --verbose"

    isp_params = {
      "SURROUND360_RENDER_DIR": surround360_render_dir,
      "ROOT_DIR": dest_dir,
      "START_FRAME": start_frame,
      "END_FRAME": end_frame,
      "CAM_TO_ISP_CONFIG_FILE": cam_to_isp_config_file,
      "FLAGS_ISP_EXTRA": isp_extra_params,
    }
    isp_command = ISP_COMMAND_TEMPLATE.replace("\n", " ").format(**isp_params)
    run_step("isp", isp_command, verbose, dryrun, file_runtimes, num_steps)

  ### rectify step ###

  if steps_rectify and rectify_file == "NONE":
    print """
    The 'rectify' step is enabled. The 'rectification file' parameter must not
    be NONE. Please supply a destination path to write the rectification file.
    A good path to use is your destination directory /rectify.yml.
    """
    exit(1)

  if rectify_file != "NONE" and os.path.isdir(rectify_file):
    print "The 'rectification file' parameter should be a path to a file, not a directory"
    exit(1)

  if rectify_file != "NONE" and not rectify_file.endswith(".yml"):
    print "The 'rectification file' parameter must end with .yml"
    exit(1)

  if steps_rectify:
    rectify_params = {
      "SURROUND360_RENDER_DIR": surround360_render_dir,
      "ROOT_DIR": dest_dir,
      "RIG_JSON_FILE": rig_json_file,
      "SRC_INSTRINSIC_PARAM_FILE": src_intrinsic_param_file,
      "RECTIFY_FILE": rectify_file,
      "FRAME_LIST": str(start_frame).zfill(FRAME_NUM_DIGITS),
    }
    rectify_command = RECTIFY_COMMAND_TEMPLATE.replace("\n", " ").format(**rectify_params)
    run_step("rectify", rectify_command, verbose, dryrun, file_runtimes, num_steps)

  ### render step ###

  if steps_render:
    render_extra_params = ""

    if enable_top:
      render_extra_params += " --enable_top"

    if enable_bottom:
      render_extra_params += " --enable_bottom"

      if enable_pole_removal:
        render_extra_params += " --enable_pole_removal"

    if enable_render_coloradjust:
      render_extra_params += " --enable_render_coloradjust"

    if save_debug_images:
      render_extra_params += " --save_debug_images"

    if verbose:
      render_extra_params += " --verbose"

    render_params = {
      "SURROUND360_RENDER_DIR": surround360_render_dir,
      "FLOW_ALG": flow_alg,
      "ROOT_DIR": dest_dir,
      "QUALITY": quality,
      "START_FRAME": start_frame,
      "END_FRAME": end_frame,
      "CUBEMAP_WIDTH": cubemap_width,
      "CUBEMAP_HEIGHT": cubemap_height,
      "CUBEMAP_FORMAT": cubemap_format,
      "SRC_INSTRINSIC_PARAM_FILE": src_intrinsic_param_file,
      "RECTIFY_FILE": rectify_file,
      "RIG_JSON_FILE": rig_json_file,
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

