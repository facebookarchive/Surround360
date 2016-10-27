# the purpose of this script is to extract just the frames from one camera so
# they can be rendered as a video.
import argparse
import multiprocessing
import os
import subprocess
import sys

from os import listdir, system
from os.path import isfile, join
from timeit import default_timer as timer

def list_only_files(src_dir): return filter(lambda f: f[0] != ".", [f for f in listdir(src_dir) if isfile(join(src_dir, f))])
def run_shell(cmd): return subprocess.call(cmd, shell=True)

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='batch process video frames')
  parser.add_argument('--root_dir', help='path to dataset dir', required=True)
  parser.add_argument('--start_frame', help='first frame index', required=True)
  parser.add_argument('--end_frame', help='last frame index', required=True)
  parser.add_argument('--extract_cam', help='name of camera to extract', required=True)
  args = vars(parser.parse_args())

  root_dir      = args["root_dir"]
  min_frame     = int(args["start_frame"])
  max_frame     = int(args["end_frame"])
  extract_cam   = args["extract_cam"]

  start_time = timer()
  for i in range(min_frame, max_frame + 1):
    frame_to_process = format(i, "06d")
    start_frame_time = timer()
    frame_dir = root_dir + "/vid/" + frame_to_process
    frame_path = frame_dir + "/isp_out/" + extract_cam + ".png"
    dest_path = root_dir + "/single_cam/" + frame_to_process + ".png"
    copy_cmd = "cp " + frame_path + " " + dest_path
    print copy_cmd
    run_shell(copy_cmd)

  end_time = timer()
  total_runtime = end_time - start_time
  avg_runtime = total_runtime / float(max_frame - min_frame + 1)
  print "total runtime:", total_runtime, "sec"
  print "avg runtime:", avg_runtime, "sec/frame"
