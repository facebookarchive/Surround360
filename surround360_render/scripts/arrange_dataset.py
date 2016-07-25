import argparse
import sys

from os import listdir, system
from os.path import isfile, join
from timeit import default_timer as timer

def list_only_files(src_dir): return filter(lambda f: f[0] != ".", [f for f in listdir(src_dir) if isfile(join(src_dir, f))])
def remove_txt_files(file_list): return filter(lambda f: f[-4:] != ".txt", file_list)

if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='batch process dataset')
  parser.add_argument('--root_dir',       help='root dir', required=True)
  parser.add_argument('--pole_masks_dir', help='directory containing pole masks', required=False)
  parser.add_argument('--verbose', dest='verbose', action='store_true')
  args = vars(parser.parse_args())

  root_dir        = args["root_dir"]
  pole_masks_dir  = args["pole_masks_dir"]
  verbose         = args["verbose"]

  src_dir = root_dir + "/raw"
  dest_dir = root_dir + "/vid"

  start_time = timer()

  system("mkdir " + root_dir + "/vid")
  system("mkdir -p " + root_dir + "/logs")
  system("mkdir " + root_dir + "/eqr_frames")
  system("mkdir " + root_dir + "/cube_frames")
  system("mkdir " + root_dir + "/pole_masks")
  system("mkdir " + root_dir + "/single_cam")

  if pole_masks_dir:
    # Copy pole masks over
    system("cp " + pole_masks_dir + "/cam15.png " + root_dir + "/pole_masks/")
    system("cp " + pole_masks_dir + "/cam16.png " + root_dir + "/pole_masks/")

  file_list = remove_txt_files(list_only_files(src_dir))
  unique_frames = set()

  frame_idx_str_prev = ""
  for f in file_list:
    frame_idx_str = f.replace("-", "_").split("_")[1]

    if verbose:
      print f
      sys.stdout.flush()

    if frame_idx_str != frame_idx_str_prev:
      print "----------- gathering frame:", frame_idx_str
      sys.stdout.flush()
      unique_frames.add(frame_idx_str)
      frame_idx_str_prev = frame_idx_str

  # Create directories for each frame
  for frame_idx_str in sorted(unique_frames):
    frame_idx = int(frame_idx_str)
    zero_pad_idx = format(frame_idx, '06d')
    system("mkdir " + dest_dir + "/" + zero_pad_idx)
    system("mkdir " + dest_dir + "/" + zero_pad_idx + "/raw")
    system("mkdir " + dest_dir + "/" + zero_pad_idx + "/projections")
    system("mkdir " + dest_dir + "/" + zero_pad_idx + "/isp_out")
    system("mkdir " + dest_dir + "/" + zero_pad_idx + "/flow")
    system("mkdir " + dest_dir + "/" + zero_pad_idx + "/flow_images")

  # Copy the raw files over to the /raw subdir for each frame
  for f in file_list:
    frame_idx_str = f.replace("-", "_").split("_")[1]
    frame_idx = int(frame_idx_str)
    zero_pad_idx = format(frame_idx, '06d')
    system("mv " + src_dir + "/" + f + " " + dest_dir + "/" + zero_pad_idx + "/raw/" + f)

  # Rename all the frames in /raw folders to sorted numerical names
  for frame_idx_str in sorted(unique_frames):
    frame_idx = int(frame_idx_str)
    print "----------- arranging frame:", frame_idx_str
    sys.stdout.flush()
    zero_pad_idx = format(frame_idx, "06d")
    raw_dir = dest_dir + "/" + zero_pad_idx + "/raw"
    sorted_images = sorted(list_only_files(raw_dir))
    for i in range(0, len(sorted_images)):
      prev_name = raw_dir + "/" + sorted_images[i]
      new_name = raw_dir + "/cam" + str(i) + ".bmp"
      system("mv " + prev_name + " " + new_name)

  end_time = timer()

  if verbose:
    print "Arrange total runtime:", (end_time - start_time), "sec"
    sys.stdout.flush()
