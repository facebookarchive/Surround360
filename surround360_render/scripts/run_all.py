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
--disk_count {DISK_COUNT}
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

RENDER_COMMAND_TEMPLATE = """
python {SURROUND360_RENDER_DIR}/scripts/batch_process_video.py
--flow_alg {FLOW_ALG}
--root_dir {ROOT_DIR}
--surround360_render_dir {SURROUND360_RENDER_DIR}
--quality {QUALITY}
--start_frame {START_FRAME}
--end_frame {END_FRAME}
--cubemap_face_resolution {CUBEMAP_FACE_RESOLUTION}
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

  parse_type = GooeyParser if USE_GOOEY else argparse.ArgumentParser
  parser = parse_type(description=TITLE, formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('--data_dir',                 metavar='Data Directory', help='directory containing .bin files', required=True, **dir_chooser)
  parser.add_argument('--dest_dir',                 metavar='Destination Directory', help='destination directory', required=True, **({"widget": "DirChooser"} if USE_GOOEY else {}))
  parser.add_argument('--quality',                  metavar='Quality', help='final output quality', required=False, choices=['preview', '3k', '4k', '6k', '8k'], default='6k')
  parser.add_argument('--start_frame',              metavar='Start Frame', help='start frame', required=False, default='0')
  parser.add_argument('--frame_count',              metavar='Frame Count', help='0 = all', required=False, default='0')
  parser.add_argument('--cubemap_face_resolution',  metavar='Cubemap Face Resolution', help='0 = no cubemaps', required=False, default='0')
  parser.add_argument('--cubemap_format',           metavar='Cubemap Format', help='photo or video', required=False, choices=['photo', 'video'], default='video')
  parser.add_argument('--enable_top',               help='enable top camera', action='store_true')
  parser.add_argument('--enable_bottom',            help='enable bottom camera', action='store_true')
  parser.add_argument('--enable_pole_removal',      help='false = use primary bottom camera', action='store_true')
  parser.add_argument('--save_debug_images',        help='save debug images', action='store_true')
  parser.add_argument('--dryrun',                   help='do not execute steps', action='store_true')
  parser.add_argument('--steps',                    metavar='Steps', help='[unpack,arrange,isp,render,ffmpeg,all]', required=False, default='all')
  parser.add_argument('--flow_alg',                 metavar='Flow Algorithm', help='optical flow algorithm', required=False, choices=['pixflow_low', 'pixflow_med',  'pixflow_ultra'], default='pixflow_low')
  parser.add_argument('--cam_to_isp_config_file',   metavar='Camera to ISP Mappings File', help='camera to ISP config file mappings', required=False, default=create_default_path(cam_to_isp_config_file, ""), **file_chooser)
  parser.add_argument('--pole_masks_dir',           metavar='Pole Masks Directory', help='directory containing pole masks', required=False, default=create_default_path(pole_masks_dir, ""), **dir_chooser)
  parser.add_argument('--src_intrinsic_param_file', metavar='Intrinsic Parameters File', help='intrinsic parameters file', required=False, default=create_default_path(src_intrinsic_param_file, ""), **file_chooser)
  parser.add_argument('--rectify_file',             metavar='Rectification File', help='rectification file [or NONE]', required=False, default=create_default_path(rectify_file, "NONE"), **file_chooser)
  parser.add_argument('--rig_json_file',            metavar='Rig Geometry File', help='json file with rig geometry info', required=False, default=create_default_path(rig_json_file, ""), **file_chooser)
  parser.add_argument('--verbose',                  help='increase output verbosity', action='store_true')

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
def run_step(step, step_list, cmd, verbose, dryrun, file_runtimes, step_count=[0]):
  if not step in step_list:
    return

  step_count[0] += 1

  print "** %s ** [Step %d of %d]\n" % (step.upper(), step_count[0], len(step_list))

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

def update_isp_mappings(cam_to_isp_config_file):
  with open(cam_to_isp_config_file, "r") as json_file:
    cam_json_map = json.load(json_file)

  for i in range(0, NUM_CAMS):
    cam_json_map["cam" + str(i)] = "isp" + str(i) + ".json"

  with open(cam_to_isp_config_file, "w") as json_file:
    json.dump(cam_json_map, json_file, indent=4, sort_keys=True)

if __name__ == "__main__":
  signal.signal(signal.SIGTERM, signal_term_handler)

  args = parse_args()
  data_dir                  = args["data_dir"]
  dest_dir                  = args["dest_dir"]
  quality                   = args["quality"]
  start_frame               = int(args["start_frame"])
  frame_count               = int(args["frame_count"])
  cubemap_face_resolution   = int(args["cubemap_face_resolution"])
  cubemap_format            = args["cubemap_format"]
  enable_top                = args["enable_top"]
  enable_bottom             = args["enable_bottom"]
  enable_pole_removal       = args["enable_pole_removal"]
  save_debug_images         = args["save_debug_images"]
  dryrun                    = args["dryrun"];
  steps                     = args["steps"]
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
      if path != rectify_file or path != "NONE":
        sys.stderr.write("Given --" + name + " (" + path + ") does not exist\n")
        exit(1)

  step_list_all = ["unpack", "arrange", "isp", "render", "ffmpeg"]
  step_list = steps.split(',')

  if "all" in step_list:
    step_list = step_list_all

  for step in step_list:
    if step not in step_list_all:
      sys.stderr.write("Unrecognized step: " + step)
      exit(1)

  if quality not in ["preview", "3k", "4k", "6k", "8k"]:
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

  step = "unpack"
  unpack_params = {
    "SURROUND360_RENDER_DIR": surround360_render_dir,
    "BINARY_PREFIX": binary_prefix,
    "ROOT_DIR": dest_dir,
    "VERBOSE_LEVEL": 1 if verbose else 0,
    "START_FRAME": start_frame,
    "FRAME_COUNT": frame_count,
    "DISK_COUNT": disk_count,
  }
  unpack_command = UNPACK_COMMAND_TEMPLATE.replace("\n", " ").format(**unpack_params)

  run_step(step, step_list, unpack_command, verbose, dryrun, file_runtimes)

  if int(frame_count) == 0 and os.path.isdir(dest_dir + "/vid"):
    frame_count = len(os.listdir(dest_dir + "/vid"))

  step = "arrange"
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

  run_step(step, step_list, arrange_command, verbose, dryrun, file_runtimes)

  # If unpack not in list of steps, we get frame count from vid directory
  if int(frame_count) == 0:
    dir_vid = dest_dir + "/vid"
    frame_count = len(os.listdir(dest_dir + "/vid")) if os.path.isdir(dir_vid) else 0

  file_runtimes.write("total frames: " + str(frame_count) + "\n")

  end_frame = int(start_frame) + int(frame_count) - 1

  # Make sure we have per camera color adjustment files. If not, copy from template
  config_isp_path = surround360_render_dir + "/res/config/isp"
  if not os.path.isfile(config_isp_path + "/isp0.json"):
    print "WARNING: Color adjustment files not found. Using default files.\n"
    sys.stdout.flush()
    duplicate_isp_files(config_isp_path)
    update_isp_mappings(cam_to_isp_config_file)

  step = "isp"
  isp_extra_params = ""

  if quality == "preview":
    isp_extra_params += " --preview_mode"

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

  run_step(step, step_list, isp_command, verbose, dryrun, file_runtimes)

  step = "render"
  render_extra_params = ""

  if enable_top and quality != "preview":
    render_extra_params += " --enable_top"

  if enable_bottom and quality != "preview":
    render_extra_params += " --enable_bottom"

    if enable_pole_removal:
      render_extra_params += " --enable_pole_removal"

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
    "CUBEMAP_FACE_RESOLUTION": cubemap_face_resolution,
    "CUBEMAP_FORMAT": cubemap_format,
    "SRC_INSTRINSIC_PARAM_FILE": src_intrinsic_param_file,
    "RECTIFY_FILE": rectify_file,
    "RIG_JSON_FILE": rig_json_file,
    "FLAGS_RENDER_EXTRA": render_extra_params,
  }
  render_command = RENDER_COMMAND_TEMPLATE.replace("\n", " ").format(**render_params)

  run_step(step, step_list, render_command, verbose, dryrun, file_runtimes)

  step = "ffmpeg"
  ffmpeg_extra_params = ""

  if not verbose:
    ffmpeg_extra_params += " -loglevel error -stats"

  mp4_path = dest_dir + "/" + str(int(timer())) + "_" + quality + "_TB.mp4"

  ffmpeg_params = {
    "START_NUMBER": str(start_frame).zfill(FRAME_NUM_DIGITS),
    "ROOT_DIR": dest_dir,
    "MP4_PATH": mp4_path,
    "FLAGS_FFMPEG_EXTRA": ffmpeg_extra_params,
  }
  ffmpeg_command = FFMPEG_COMMAND_TEMPLATE.replace("\n", " ").format(**ffmpeg_params)

  run_step(step, step_list, ffmpeg_command, verbose, dryrun, file_runtimes)

  save_step_runtime(file_runtimes, "TOTAL", timer() - start_time)
  file_runtimes.close()

