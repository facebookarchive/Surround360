from scannerpy import Database, DeviceType
from scannerpy.stdlib import NetDescriptor, parsers, bboxes
import numpy as np
import argparse
import os
import signal
import subprocess
import sys
import struct
import scipy.misc

from os import listdir, system
from os.path import isfile, join
from timeit import default_timer as timer

current_process = None
def signal_term_handler(signal, frame):
  if current_process:
    print "Terminating process: " + current_process.name + "..."
    current_process.terminate()
  sys.exit(0)

RENDER_COMMAND_TEMPLATE = """
{SURROUND360_RENDER_DIR}/bin/TestRenderStereoPanorama
--logbuflevel -1
--log_dir {LOG_DIR}
--stderrthreshold 0
--v {VERBOSE_LEVEL}
--rig_json_file {RIG_JSON_FILE}
--imgs_dir {SRC_DIR}/rgb
--frame_number {FRAME_ID}
--output_data_dir {SRC_DIR}
--prev_frame_data_dir {PREV_FRAME_DIR}
--output_cubemap_path {OUT_CUBE_DIR}/cube_{FRAME_ID}.png
--output_equirect_path {OUT_EQR_DIR}/eqr_{FRAME_ID}.png
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
--sharpenning {SHARPENNING}
{EXTRA_FLAGS}
"""

def start_subprocess(name, cmd):
  global current_process
  current_process = subprocess.Popen(cmd, shell=True)
  current_process.name = name
  current_process.communicate()


def project_images(db, videos, videos_idx, render_params):
  in_columns = ["index", "frame", "frame_info", "camera_index"]
  input_op = db.ops.Input(in_columns)

  args = db.protobufs.ProjectSphericalArgs()
  args.eqr_width = render_params["EQR_WIDTH"]
  args.eqr_height = render_params["EQR_HEIGHT"]
  args.camera_rig_path = render_params["RIG_JSON_FILE"]
  op = db.ops.ProjectSpherical(inputs=[(input_op, in_columns[1:])], args=args)

  tasks = []
  sampler_args = db.protobufs.AllSamplerArgs()
  sampler_args.warmup_size = 0
  sampler_args.sample_size = 32

  for i, (vid, vid_idx) in enumerate(zip(videos.tables(),
                                         videos_idx.tables())):
    task = db.protobufs.Task()
    task.output_table_name = vid.name() + str(i)

    sample = task.samples.add()
    sample.table_name = vid.name()
    sample.column_names.extend([c.name() for c in vid.columns()])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    sample = task.samples.add()
    sample.table_name = vid_idx.name()
    sample.column_names.extend(['camera_index'])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    tasks.append(task)

  return db.run(tasks, op, 'surround360_spherical', force=True)


def compute_temporal_flow(db, overlap, render_params):
  in_columns = ["index",
                "projected_left", "frame_info_left",
                "projected_right", "frame_info_right"]
  input_op = db.ops.Input(in_columns)

  args = db.protobufs.TemporalOpticalFlowArgs()
  args.flow_algo = render_params["SIDE_FLOW_ALGORITHM"]
  args.camera_rig_path = render_params["RIG_JSON_FILE"]
  op = db.ops.TemporalOpticalFlow(inputs=[(input_op, in_columns[1:])],
                                  args=args)

  tasks = []
  sampler_args = db.protobufs.AllSamplerArgs()
  sampler_args.warmup_size = 0
  sampler_args.sample_size = 32

  for left_cam_idx in range(0, 14):
    right_cam_idx = left_cam_idx + 1 if left_cam_idx != 13 else 1
    task = db.protobufs.Task()
    task.output_table_name = 'surround360_flow_{:d}'.format(left_cam_idx)

    sample = task.samples.add()
    sample.table_name = overlap.tables(left_cam_idx).name()
    sample.column_names.extend(['index', 'projected_frame', 'frame_info'])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    sample = task.samples.add()
    sample.table_name = overlap.tables(right_cam_idx).name()
    sample.column_names.extend(['projected_frame', 'frame_info'])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    tasks.append(task)

  return db.run(tasks, op, 'surround360_flow', force=True)


def render_stereo_panorama_chunks(db, overlap, flow, render_params):
  in_columns = ["index",
                "projected_left", "frame_info_left",
                "flow_left", "flow_info_left",
                "projected_right", "frame_info_right",
                "flow_right", "flow_info_right"]
  input_op = db.ops.Input(in_columns)

  args = db.protobufs.RenderStereoPanoramaChunkArgs()
  args.eqr_width = render_params["EQR_WIDTH"]
  args.eqr_height = render_params["EQR_HEIGHT"]
  args.camera_rig_path = render_params["RIG_JSON_FILE"]
  args.flow_algo = render_params["SIDE_FLOW_ALGORITHM"]
  args.zero_parallax_dist = 10000
  args.interpupilary_dist = 6.4
  op = db.ops.RenderStereoPanoramaChunk(inputs=[(input_op, in_columns[1:])],
                                        args=args)

  tasks = []
  sampler_args = db.protobufs.AllSamplerArgs()
  sampler_args.warmup_size = 0
  sampler_args.sample_size = 32

  for left_cam_idx in range(0, 14):
    right_cam_idx = left_cam_idx + 1 if left_cam_idx != 13 else 1
    task = db.protobufs.Task()
    task.output_table_name = 'surround360_chunk_{:d}'.format(left_cam_idx)

    sample = task.samples.add()
    sample.table_name = overlap.tables(left_cam_idx).name()
    sample.column_names.extend(['index', 'projected_frame', 'frame_info'])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    sample = task.samples.add()
    sample.table_name = flow.tables(left_cam_idx).name()
    sample.column_names.extend(['flow_left', 'frame_info_left'])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    sample = task.samples.add()
    sample.table_name = overlap.tables(right_cam_idx).name()
    sample.column_names.extend(['projected_frame', 'frame_info'])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    sample = task.samples.add()
    sample.table_name = flow.tables(left_cam_idx).name()
    sample.column_names.extend(['flow_right', 'frame_info_right'])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    tasks.append(task)

  return db.run(tasks, op, 'surround360_chunk', force=True)


def fused_flow_and_stereo_chunk(db, overlap, render_params):
  in_columns = ["index",
                "projected_left", "frame_info_left",
                "projected_right", "frame_info_right"]
  input_op = db.ops.Input(in_columns)

  args = db.protobufs.TemporalOpticalFlowArgs()
  args.flow_algo = render_params["SIDE_FLOW_ALGORITHM"]
  args.camera_rig_path = render_params["RIG_JSON_FILE"]
  temporal_op = db.ops.TemporalOpticalFlow(inputs=[(input_op, in_columns[1:])],
                                           args=args)

  args = db.protobufs.RenderStereoPanoramaChunkArgs()
  args.eqr_width = render_params["EQR_WIDTH"]
  args.eqr_height = render_params["EQR_HEIGHT"]
  args.camera_rig_path = render_params["RIG_JSON_FILE"]
  args.flow_algo = render_params["SIDE_FLOW_ALGORITHM"]
  args.zero_parallax_dist = 10000
  args.interpupilary_dist = 6.4
  op = db.ops.RenderStereoPanoramaChunk(
    inputs=[(input_op, ["projected_left", "frame_info_left"]),
            (temporal_op, ["flow_left", "frame_info_left"]),
            (input_op, ["projected_right", "frame_info_right"]),
            (temporal_op, ["flow_right", "frame_info_right"])],
    args=args)

  tasks = []
  sampler_args = db.protobufs.AllSamplerArgs()
  sampler_args.warmup_size = 0
  sampler_args.sample_size = 32

  for left_cam_idx in range(0, 14):
    right_cam_idx = left_cam_idx + 1 if left_cam_idx != 13 else 1
    task = db.protobufs.Task()
    task.output_table_name = 'surround360_flow_{:d}'.format(left_cam_idx)

    sample = task.samples.add()
    sample.table_name = overlap.tables(left_cam_idx).name()
    sample.column_names.extend(['index', 'projected_frame', 'frame_info'])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    sample = task.samples.add()
    sample.table_name = overlap.tables(right_cam_idx).name()
    sample.column_names.extend(['projected_frame', 'frame_info'])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    tasks.append(task)

  return db.run(tasks, op, 'surround360_chunk_fused', force=True)

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

  frame_range = range(min_frame, max_frame + 1)

  render_params = {
      "SURROUND360_RENDER_DIR": surround360_render_dir,
      "LOG_DIR": log_dir,
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
      render_params["EXTRA_FLAGS"] += " --bottom_pole_masks_dir " + root_dir + "/pole_masks"

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

  collection_name = 'surround360'
  idx_collection_name = 'surround360_index'
  with Database() as db:
      db.load_op('build/lib/libsurround360kernels.so',
                 'build/source/scanner_kernels/surround360_pb2.py')

      if False or not db.has_collection(collection_name):
          print "----------- [Render] loading surround360 collection"
          sys.stdout.flush()
          paths = [os.path.join(root_dir, 'rgb', 'cam{:d}'.format(i), 'vid.mp4')
                   for i in range(1, 15)]
          collection, _ = db.ingest_video_collection(collection_name, paths,
                                                  force=True)

          idx_table_names = []
          num_rows = collection.tables(0).num_rows()
          columns = ['camera_index']
          for c in range(0, len(paths)):
              rows = []
              for i in range(num_rows):
                  rows.append([struct.pack('i', c)])
              table_name = 'surround_cam_idx_{:d}'.format(c)
              db.new_table(table_name, columns, rows, force=True)
              idx_table_names.append(table_name)
          db.new_collection(idx_collection_name, idx_table_names,
                            force=True)


      videos = db.collection(collection_name)
      videos_idx = db.collection(idx_collection_name)

      print "----------- [Render] processing frames ", min_frame, max_frame
      sys.stdout.flush()
      if verbose:
          print(render_params)
          sys.stdout.flush()

      proj_col = project_images(db, videos, videos_idx, render_params)
      if save_debug_images:
        t1 = proj_col.tables(0)
        for fi, tup in t1.load(['projected_frame', 'frame_info']):
           frame_info = db.protobufs.FrameInfo()
           frame_info.ParseFromString(tup[1])

           frame = np.frombuffer(tup[0], dtype=np.uint8).reshape(
             frame_info.height,
             frame_info.width,
             4)
           scipy.misc.toimage(frame[:,:,0:3]).save('test.png')

      fused = False
      if fused:
        chunk_col = fused_flow_and_stereo_chunk(db, proj_col, render_params)
      else:
        flow_col = compute_temporal_flow(db, proj_col, render_params)
        if save_debug_images:
          t1 = flow_col.tables(0)
          for fi, tup in t1.load(['flow_left', 'frame_info_left']):
            frame_info = db.protobufs.FrameInfo()
            frame_info.ParseFromString(tup[1])

            frame = np.frombuffer(tup[0], dtype=np.float32).reshape(
              frame_info.height,
              frame_info.width,
              2)
            frame = np.append(frame, np.zeros((frame_info.height, frame_info.width, 1)), axis=2)
            scipy.misc.toimage(frame[:,:,:]).save('flow_test.png')

        chunk_col = render_stereo_panorama_chunks(db, proj_col, flow_col, render_params)
      render = True
      if render:
        panos = [None for _ in range(min_frame, max_frame + 1)]
        for table in chunk_col.tables():
          for fi, tup in table.load(['chunk_left', 'frame_info_left']):
            frame_info = db.protobufs.FrameInfo()
            frame_info.ParseFromString(tup[1])

            frame = np.frombuffer(tup[0], dtype=np.uint8).reshape(
              frame_info.height,
              frame_info.width,
              4)
            if panos[fi] is None:
              panos[fi] = frame
            else:
              panos[fi] = np.hstack((panos[fi], frame))
        scipy.misc.toimage(panos[1][:,:,0:3]).save('pano_test.png')

  end_time = timer()

  if verbose:
    total_runtime = end_time - start_time
    avg_runtime = total_runtime / float(max_frame - min_frame + 1)
    print "Render total runtime:", total_runtime, "sec"
    print "Average runtime:", avg_runtime, "sec/frame"
    sys.stdout.flush()
