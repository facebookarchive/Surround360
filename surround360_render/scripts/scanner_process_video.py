from scannerpy import Database, DeviceType, Job
from scannerpy.stdlib import NetDescriptor, parsers, bboxes
import numpy as np
import argparse
import os
import signal
import subprocess
import sys
import struct
import scipy.misc
import time

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


def project_tasks(db, videos, videos_idx):
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
  return tasks


def project_tasks_join(db, videos, videos_idx):
  tasks = []
  sampler_args = db.protobufs.AllSamplerArgs()
  sampler_args.warmup_size = 0
  sampler_args.sample_size = 32

  for i in range(len(videos.tables())):
    left_cam_idx = i
    if i == len(videos.tables()) - 1:
      right_cam_idx = 0
    else:
      right_cam_idx = left_cam_idx + 1

    vid_left = videos.tables(left_cam_idx)
    vid_idx_left = videos_idx.tables(left_cam_idx)
    vid_right = videos.tables(right_cam_idx)
    vid_idx_right = videos_idx.tables(right_cam_idx)

    task = db.protobufs.Task()
    task.output_table_name = vid_left.name() + str(i)

    sample = task.samples.add()
    sample.table_name = vid_left.name()
    sample.column_names.extend([c.name() for c in vid_left.columns()])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    sample = task.samples.add()
    sample.table_name = vid_idx_left.name()
    sample.column_names.extend(['camera_index'])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    sample = task.samples.add()
    sample.table_name = vid_right.name()
    sample.column_names.extend(['frame'])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    sample = task.samples.add()
    sample.table_name = vid_idx_right.name()
    sample.column_names.extend(['camera_index'])
    sample.sampling_function = "All"
    sample.sampling_args = sampler_args.SerializeToString()

    tasks.append(task)
  return tasks


def project_images(db, videos, videos_idx, render_params):
  jobs = []
  for i in range(len(video.tables())):
    left_frame = videos.tables(i).as_op().all()
    left_cam_idx = videos_idx.tables(i).as_op().all()

    args = db.protobufs.ProjectSphericalArgs()
    args.eqr_width = render_params["EQR_WIDTH"]
    args.eqr_height = render_params["EQR_HEIGHT"]
    args.camera_rig_path = render_params["RIG_JSON_FILE"]
    proj_frame = db.ops.ProjectSpherical(frame = frame, camra_id = cam_idx,
                                         args=args)

    job = Job(columns = [proj_frame],
              name = 'surround360_spherical_{:d}'.format(i))
    jobs.append(job)

  return db.run(jobs, 'surround360_spherical', force=True)


def compute_temporal_flow(db, overlap, render_params):
  jobs = []
  for i in range(len(overlap.tables())):
    left_cam_idx = i
    right_cam_idx = (left_cam_idx + 1) % len(overlap.tables())

    left_proj_frame = overlap.tables(left_cam_idx).as_op().all()
    right_proj_frame = overlap.tables(right_cam_idx).as_op().all()

    args = db.protobufs.TemporalOpticalFlowArgs()
    args.flow_algo = render_params["SIDE_FLOW_ALGORITHM"]
    args.camera_rig_path = render_params["RIG_JSON_FILE"]
    left_flow, right_flow = db.ops.TemporalOpticalFlow(
      left_projected_frame = left_proj_frame,
      right_projected_frame = right_proj_frame,
      args=args)

    job = Job(columns = [left_flow, right_flow],
              name = 'surround360_flow_{:d}'.format(i))
    jobs.append(job)

  return db.run(jobs, 'surround360_flow', force=True)


def render_stereo_panorama_chunks(db, overlap, flow, render_params):
  jobs = []
  for i in range(len(video.tables())):
    left_cam_idx = i
    right_cam_idx = (left_cam_idx + 1) % len(video.tables())

    left_proj_frame = overlap.tables(left_cam_idx).as_op().all()
    left_flow = flow.tables(left_cam_idx).as_op().all()
    right_proj_frame = videos.tables(right_cam_idx).as_op().all()
    right_flow = flow_idx.tables(right_cam_idx).as_op().all()

    left_chunk, right_chunk = db.ops.RenderStereoPanoramaChunk(
      left_projected_frame = left_proj_frame,
      left_flow = left_flow,
      right_projected_frame = right_proj_frame,
      right_flow = right_flow,
      eqr_width = render_params["EQR_WIDTH"],
      eqr_height = render_params["EQR_HEIGHT"],
      camera_rig_path = render_params["RIG_JSON_FILE"],
      flow_algo = render_params["SIDE_FLOW_ALGORITHM"],
      zero_parallax_dist = 10000,
      interpupilary_dist = 6.4)

    job = Job(columns = [left_chunk, right_chunk],
              name = 'surround360_chunk_{:d}'.format(i))
    jobs.append(job)

  return db.run(jobs, force=True)


def concat_stereo_panorama_chunks(db, chunks, render_params, is_left):
  num_cams = 14
  left_inputs = []
  right_inputs = []
  print(chunks)
  for c in range(num_cams):
    left_chunk, right_chunk = chunks[c].as_op().all(item_size=50)
    left_inputs.append(left_chunk)
    right_inputs.append(right_chunk)

  jobs = []
  args = db.protobufs.ConcatPanoramaChunksArgs()
  args.eqr_width = render_params["EQR_WIDTH"]
  args.eqr_height = render_params["EQR_HEIGHT"]
  args.camera_rig_path = render_params["RIG_JSON_FILE"]
  args.zero_parallax_dist = 10000
  args.interpupilary_dist = 6.4
  args.left = is_left
  panorama_left = db.ops.ConcatPanoramaChunks(*left_inputs, args=args)

  args = db.protobufs.ConcatPanoramaChunksArgs()
  args.eqr_width = render_params["EQR_WIDTH"]
  args.eqr_height = render_params["EQR_HEIGHT"]
  args.camera_rig_path = render_params["RIG_JSON_FILE"]
  args.zero_parallax_dist = 10000
  args.interpupilary_dist = 6.4
  args.left = False
  panorama_right = db.ops.ConcatPanoramaChunks(*right_inputs, args=args)
  job = Job(columns = [panorama_left, panorama_right],
            name = 'surround360_pano_both')

  return db.run(job, force=True)

def fused_project_flow_chunk_concat(db, videos, videos_idx, render_params,
                                    start, end):
  jobs = []
  item_size = 10

  proj_frames = []
  for i in range(len(videos.tables())):
    left_cam_idx = i
    frame = videos.tables(left_cam_idx).as_op().range(start, end, item_size = item_size)
    cam_idx = videos_idx.tables(left_cam_idx).as_op().range(start, end, item_size = item_size)

    args = db.protobufs.ProjectSphericalArgs()
    args.eqr_width = render_params["EQR_WIDTH"]
    args.eqr_height = render_params["EQR_HEIGHT"]
    args.camera_rig_path = render_params["RIG_JSON_FILE"]
    proj_frame = db.ops.ProjectSpherical(
      frame = frame,
      camera_id = cam_idx,
      args=args)
    proj_frames.append(proj_frame)

  left_chunks = []
  right_chunks = []
  for i in range(len(videos.tables())):
    left_cam_idx = i
    right_cam_idx = (left_cam_idx + 1) % len(videos.tables())

    left_proj_frame = proj_frames[left_cam_idx]
    right_proj_frame = proj_frames[right_cam_idx]

    left_flow, right_flow = db.ops.TemporalOpticalFlow(
      left_projected_frame = left_proj_frame,
      right_projected_frame = right_proj_frame,
      flow_algo = render_params["SIDE_FLOW_ALGORITHM"],
      camera_rig_path = render_params["RIG_JSON_FILE"])

    left_chunk, right_chunk = db.ops.RenderStereoPanoramaChunk(
      left_projected_frame = left_proj_frame,
      left_flow = left_flow,
      right_projected_frame = right_proj_frame,
      right_flow = right_flow,
      eqr_width = render_params["EQR_WIDTH"],
      eqr_height = render_params["EQR_HEIGHT"],
      camera_rig_path = render_params["RIG_JSON_FILE"],
      flow_algo = render_params["SIDE_FLOW_ALGORITHM"],
      zero_parallax_dist = 10000,
      interpupilary_dist = 6.4)
    left_chunks.append(left_chunk)
    right_chunks.append(right_chunk)

  jobs = []
  args = db.protobufs.ConcatPanoramaChunksArgs()
  args.eqr_width = render_params["EQR_WIDTH"]
  args.eqr_height = render_params["EQR_HEIGHT"]
  args.camera_rig_path = render_params["RIG_JSON_FILE"]
  args.zero_parallax_dist = 10000
  args.interpupilary_dist = 6.4
  args.left = True
  panorama_left = db.ops.ConcatPanoramaChunks(*left_chunks, args=args)

  args = db.protobufs.ConcatPanoramaChunksArgs()
  args.eqr_width = render_params["EQR_WIDTH"]
  args.eqr_height = render_params["EQR_HEIGHT"]
  args.camera_rig_path = render_params["RIG_JSON_FILE"]
  args.zero_parallax_dist = 10000
  args.interpupilary_dist = 6.4
  args.left = False
  panorama_right = db.ops.ConcatPanoramaChunks(*right_chunks, args=args)

  job = Job(columns = [panorama_left, panorama_right],
            name = 'surround360_pano_both')
  return db.run(job, force=True)


def fused_project_flow_and_stereo_chunk(db, videos, videos_idx, render_params, start, end):
  jobs = []
  item_size = 11
  for i in range(len(videos.tables())):
    left_cam_idx = i
    right_cam_idx = (left_cam_idx + 1) % len(videos.tables())

    left_frame = videos.tables(left_cam_idx).as_op().range(start, end, item_size = item_size)
    left_cam_idx = videos_idx.tables(left_cam_idx).as_op().range(start, end, item_size = item_size)
    right_frame = videos.tables(right_cam_idx).as_op().range(start, end, item_size = item_size)
    right_cam_idx = videos_idx.tables(right_cam_idx).as_op().range(start, end, item_size = item_size)

    args = db.protobufs.ProjectSphericalArgs()
    args.eqr_width = render_params["EQR_WIDTH"]
    args.eqr_height = render_params["EQR_HEIGHT"]
    args.camera_rig_path = render_params["RIG_JSON_FILE"]
    left_proj_frame = db.ops.ProjectSpherical(
      frame = left_frame,
      camera_id = left_cam_idx,
      args=args)
    right_proj_frame = db.ops.ProjectSpherical(
      frame = right_frame,
      camera_id = right_cam_idx,
      args=args)

    left_flow, right_flow = db.ops.TemporalOpticalFlow(
      left_projected_frame = left_proj_frame,
      right_projected_frame = right_proj_frame,
      flow_algo = render_params["SIDE_FLOW_ALGORITHM"],
      camera_rig_path = render_params["RIG_JSON_FILE"])

    left_chunk, right_chunk = db.ops.RenderStereoPanoramaChunk(
      left_projected_frame = left_proj_frame,
      left_flow = left_flow,
      right_projected_frame = right_proj_frame,
      right_flow = right_flow,
      eqr_width = render_params["EQR_WIDTH"],
      eqr_height = render_params["EQR_HEIGHT"],
      camera_rig_path = render_params["RIG_JSON_FILE"],
      flow_algo = render_params["SIDE_FLOW_ALGORITHM"],
      zero_parallax_dist = 10000,
      interpupilary_dist = 6.4)

    job = Job(columns = [left_chunk.lossless(), right_chunk.lossless()],
              name = 'surround360_chunk_{:d}'.format(i))
    jobs.append(job)

  return db.run(jobs, force=True)


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

  tasks = flow_tasks(db, overlap)

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
  db_start = timer()
  with Database() as db:
      print('DB', timer() - db_start)
      db.load_op('build/lib/libsurround360kernels.so',
                 'build/source/scanner_kernels/surround360_pb2.py')

      if not db.has_collection(collection_name):
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

      start_time = timer()
      print "----------- [Render] processing frames ", min_frame, max_frame
      sys.stdout.flush()
      if verbose:
          print(render_params)
          sys.stdout.flush()

      visualize = False
      fused_4 = False # Fused project, flow, stereo chunk, and pano
      fused_3 = True # Fused project, flow, and stereo chunk
      fused_2 = False # Fused flow and stereo chunk
      fused_1 = False # Nothing fused
      if fused_4:
        pano_col = fused_project_flow_chunk_concat(db, videos, videos_idx,
                                                   render_params,
                                                   min_frame, max_frame + 1)
        pano_col.profiler().write_trace('fused4.trace')
      elif fused_3:
        flow_stereo_start = timer()
        chunk_col = fused_project_flow_and_stereo_chunk(db, videos, videos_idx,
                                                        render_params,
                                                        min_frame, max_frame + 1)
        print(db.summarize())
        chunk_col = [db.table('surround360_chunk_{:d}'.format(i))
                              for i in range(14)]
        print('Proj flow stereo', timer() - flow_stereo_start)
        concat_start = timer()
        pano_col = concat_stereo_panorama_chunks(db, chunk_col, render_params,
                                                 True)
        print('Concat', timer() - concat_start)
        chunk_col[0].profiler().write_trace('fused3.trace')
      elif fused_2:
        proj_start = timer()
        proj_col = project_images(db, videos, videos_idx, render_params)
        print('Proj', timer() - proj_start)
        flow_stereo_start = timer()
        chunk_col = fused_flow_and_stereo_chunk(db, proj_col, render_params)
        print('Flow stereo', timer() - flow_stereo_start)
        concat_start = timer()
        pano_col = concat_stereo_panorama_chunks(db, chunk_col, render_params,
                                                 True)
        print('Concat', timer() - concat_start)
      elif fused_1:
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

      png_start = timer()
      pano_col = db.table('surround360_pano_both')
      # left_table = pano_col[0]
      # right_table = pano_col[1]
      if visualize:
        left_table.columns('panorama').save_mp4('pano_test.mp4')
      print('To png', timer() - png_start)

  end_time = timer()

  if verbose:
    total_runtime = end_time - start_time
    avg_runtime = total_runtime / float(max_frame - min_frame + 1)
    print "Render total runtime:", total_runtime, "sec"
    print "Average runtime:", avg_runtime, "sec/frame"
    sys.stdout.flush()
