import argparse
import json
import multiprocessing
import os
import subprocess
import sys

from os import listdir, system
from os.path import isfile, join
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
def run_shell(cmd): return subprocess.call(cmd, shell=True)

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='batch process ISP')
  parser.add_argument('--root_dir', help='path to frame container dir', required=True)
  parser.add_argument('--surround360_render_dir', help='project root path, containing bin and scripts dirs', required=False, default='.')
  parser.add_argument('--start_frame', help='first frame index', required=True)
  parser.add_argument('--end_frame', help='last frame index', required=True)
  parser.add_argument('--preview_mode', dest='preview_mode', action='store_true')
  parser.add_argument('--delete_raws', dest='delete_raws', action='store_true')
  parser.add_argument('--cam_to_isp_config_file', help='file with mappings cam: isp file', required=True)
  parser.add_argument('--verbose', dest='verbose', action='store_true')
  parser.set_defaults(preview_mode=False)
  parser.set_defaults(delete_raws=False)
  args = vars(parser.parse_args())

  root_dir                = args["root_dir"]
  surround360_render_dir  = args["surround360_render_dir"]
  min_frame               = int(args["start_frame"])
  max_frame               = int(args["end_frame"])
  preview_mode            = args["preview_mode"]
  delete_raws             = args["delete_raws"]
  cam_to_isp_config_file  = args["cam_to_isp_config_file"]
  verbose                 = args["verbose"]
  log_dir                 = root_dir + "/logs"

  if preview_mode:
    print "*** FAST PREVIEW MODE ***"

  start_time = timer()
  num_cpus = multiprocessing.cpu_count()
  pool = multiprocessing.Pool(processes=num_cpus)

  cam_to_isp_config = json.load(open(cam_to_isp_config_file))

  for i in range(min_frame, max_frame + 1):
    frame_to_process = format(i, "06d")
    print "----------- [ISP] processing frame:", frame_to_process
    sys.stdout.flush()

    start_frame_time = timer()
    frame_dir = root_dir + "/vid/" + frame_to_process
    raw_filenames = list_only_files(frame_dir + "/raw")
    isp_commands = []
    for f in raw_filenames:
      filename_prefix = f.split(".")[0]
      raw_img_path = frame_dir + "/raw/" + f
      dest_rgb_path = frame_dir + "/isp_out/" + filename_prefix + ".png"

      raw2rgb_extra_params = ""

      if preview_mode: raw2rgb_extra_params += " --resize 8 --demosaic_filter 2"
      if delete_raws and not preview_mode: raw2rgb_extra_params += " && rm " + raw_img_path

      raw2rgb_params = {
        "SURROUND360_RENDER_DIR": surround360_render_dir,
        "LOG_DIR": log_dir,
        "INPUT_IMAGE_PATH": raw_img_path,
        "ISP_CONFIG_PATH": surround360_render_dir + "/res/config/isp/" + cam_to_isp_config[filename_prefix],
        "BLACK_LEVEL_OFFSET": str(CAM_TO_BLACKLEVEL[filename_prefix]),
        "OUTPUT_IMAGE_PATH": dest_rgb_path,
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
