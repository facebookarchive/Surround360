#include "render/RigDescription.h"
#include "optical_flow/NovelView.h"
#include "util/MathUtil.h"
#include "source/scanner_kernels/surround360.pb.h"

#include "scanner/api/kernel.h"
#include "scanner/api/op.h"
#include "scanner/util/memory.h"
#include "scanner/util/opencv.h"

using namespace scanner;

namespace surround360 {

using namespace optical_flow;
using namespace math_util;

class RenderStereoPanoramaChunkKernelCPU : public VideoKernel {
 public:
  RenderStereoPanoramaChunkKernelCPU(const Kernel::Config& config)
      : VideoKernel(config),
        device_(config.devices[0]),
        work_item_size_(config.work_item_size) {
    args_.ParseFromArray(config.args.data(), config.args.size());

    rig_.reset(new RigDescription(args_.camera_rig_path()));

    overlap_image_width_ = -1;
    novel_view_gen_.reset(
        new NovelViewGeneratorAsymmetricFlow(args_.flow_algo()));
  }

  void new_frame_info() override {
    const int numCams = 14;
    const float cameraRingRadius = rig_->getRingRadius();
    const float camFovHorizontalDegrees =
        2 * approximateFov(rig_->rigSideOnly, false) * (180 / M_PI);
    fov_horizontal_radians_ = toRadians(camFovHorizontalDegrees);
    const float overlapAngleDegrees =
        (camFovHorizontalDegrees * float(numCams) - 360.0) / float(numCams);
    const int camImageWidth = frame_info_.width();
    const int camImageHeight = frame_info_.height();
    overlap_image_width_ = float(camImageWidth) *
                           (overlapAngleDegrees / camFovHorizontalDegrees);
    num_novel_views_ =
        camImageWidth - overlap_image_width_; // per image pair

    const float v =
        atanf(args_.zero_parallax_dist() / (args_.interpupilary_dist() / 2.0f));
    const float psi =
        asinf(sinf(v) * (args_.interpupilary_dist() / 2.0f) / cameraRingRadius);
    const float vergeAtInfinitySlabDisplacement =
        psi * (float(camImageWidth) / fov_horizontal_radians_);
    const float theta = -M_PI / 2.0f + v + psi;
    const float zeroParallaxNovelViewShiftPixels =
        float(args_.eqr_width()) * (theta / (2.0f * M_PI));

    int currChunkX = 0;
    lazy_view_buffer_.reset(new LazyNovelViewBuffer(args_.eqr_width() / numCams,
                                                    camImageHeight));
    for (int nvIdx = 0; nvIdx < num_novel_views_; ++nvIdx) {
      const float shift = float(nvIdx) / float(num_novel_views_);
      const float slabShift =
          float(camImageWidth) * 0.5f - float(num_novel_views_ - nvIdx);

      for (int v = 0; v < camImageHeight; ++v) {
        lazy_view_buffer_->warpL[currChunkX][v] =
            Point3f(slabShift + vergeAtInfinitySlabDisplacement, v, shift);
        lazy_view_buffer_->warpR[currChunkX][v] =
            Point3f(slabShift - vergeAtInfinitySlabDisplacement, v, shift);
      }
      ++currChunkX;
    }
  }

  void execute(const BatchedColumns& input_columns,
               BatchedColumns& output_columns) override {
    auto& left_frame_col = input_columns[0];
    auto& left_flow_col = input_columns[1];
    auto& right_frame_col = input_columns[2];
    auto& right_flow_col = input_columns[3];
    check_frame(device_, left_frame_col[0]);
    assert(overlap_image_width_ != -1);

    i32 input_count = (i32)num_rows(left_frame_col);
    size_t output_image_width = num_novel_views_;
    size_t output_image_height = frame_info_.height();
    size_t output_image_size =
        output_image_width * output_image_height * 4;
    FrameInfo info(output_image_height, output_image_width, 4, FrameType::U8);
    std::vector<Frame*> frames = new_frames(device_, info, input_count * 2);

    for (i32 i = 0; i < input_count; ++i) {
      cv::Mat left_input = frame_to_mat(left_frame_col[i].as_const_frame());
      cv::Mat right_input = frame_to_mat(right_frame_col[i].as_const_frame());
      cv::Mat left_overlap_input =
          left_input(cv::Rect(left_input.cols - overlap_image_width_, 0,
                              overlap_image_width_, left_input.rows));
      cv::Mat right_overlap_input =
          right_input(cv::Rect(0, 0,
                               overlap_image_width_, right_input.rows));


      cv::Mat left_flow = frame_to_mat(left_flow_col[i].as_const_frame());
      cv::Mat right_flow = frame_to_mat(right_flow_col[i].as_const_frame());

      // Initialize NovelViewGenerator with images and flow since we are
      // bypassing the prepare method
      novel_view_gen_->setImageL(left_overlap_input);
      novel_view_gen_->setImageR(right_overlap_input);
      novel_view_gen_->setFlowLtoR(left_flow);
      novel_view_gen_->setFlowRtoL(right_flow);
      std::pair<Mat, Mat> lazyNovelChunksLR =
          novel_view_gen_->combineLazyNovelViews(*lazy_view_buffer_.get());
      const cv::Mat& chunkL = lazyNovelChunksLR.first;
      const cv::Mat& chunkR = lazyNovelChunksLR.second;

      u8* left_output = frames[2 * i]->data;
      u8* right_output = frames[2 * i + 1]->data;
      for (i32 r = 0; r < chunkL.rows; ++r) {
        memcpy(left_output + r * output_image_width * sizeof(char) * 4,
               chunkL.data + chunkL.step * r,
               chunkL.cols * sizeof(char) * 4);
        memcpy(right_output + r * output_image_width * sizeof(char) * 4,
               chunkR.data + chunkR.step * r,
               chunkR.cols * sizeof(char) * 4);
      }

      insert_frame(output_columns[0], frames[2 * i]);
      insert_frame(output_columns[1], frames[2 * i + 1]);
    }
  }

 private:
  surround360::proto::RenderStereoPanoramaChunkArgs args_;
  std::unique_ptr<RigDescription> rig_;
  DeviceHandle device_;
  int work_item_size_;
  float fov_horizontal_radians_;
  int overlap_image_width_;
  int num_novel_views_;

  std::unique_ptr<NovelViewGenerator> novel_view_gen_;
  std::unique_ptr<LazyNovelViewBuffer> lazy_view_buffer_;
};

REGISTER_OP(RenderStereoPanoramaChunk)
    .frame_input("left_projected_frame")
    .frame_input("left_flow")
    .frame_input("right_projected_frame")
    .frame_input("right_flow")
    .frame_output("left_chunk")
    .frame_output("right_chunk");

REGISTER_KERNEL(RenderStereoPanoramaChunk, RenderStereoPanoramaChunkKernelCPU)
    .device(DeviceType::CPU)
    .num_devices(1);
}
