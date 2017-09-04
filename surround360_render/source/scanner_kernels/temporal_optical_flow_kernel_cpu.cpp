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

class TemporalOpticalFlowKernelCPU : public BatchedKernel, public VideoKernel {
 public:
  TemporalOpticalFlowKernelCPU(const KernelConfig& config)
      : BatchedKernel(config),
        device_(config.devices[0]) {
    args_.ParseFromArray(config.args.data(), config.args.size());

    rig_.reset(new RigDescription(args_.camera_rig_path()));

    overlap_image_width_ = -1;
    novel_view_gen_.reset(
        new NovelViewGeneratorAsymmetricFlow(args_.flow_algo()));
  }

  void reset() override {
    prev_frame_flow_l_to_r_ = cv::Mat();
    prev_frame_flow_r_to_l_ = cv::Mat();
    prev_overlap_image_l_ = cv::Mat();
    prev_overlap_image_r_ = cv::Mat();
  }

  void new_frame_info() override {
    const int numCams = 14;
    const float cameraRingRadius = rig_->getRingRadius();
    const float camFovHorizontalDegrees =
        2 * approximateFov(rig_->rigSideOnly, false) * (180 / M_PI);
    const float fovHorizontalRadians = toRadians(camFovHorizontalDegrees);
    const float overlapAngleDegrees =
        (camFovHorizontalDegrees * float(numCams) - 360.0) / float(numCams);
    const int camImageWidth = frame_info_.width();
    const int camImageHeight = frame_info_.height();
    overlap_image_width_ =
        float(camImageWidth) * (overlapAngleDegrees / camFovHorizontalDegrees);
  }

  void execute(const BatchedColumns& input_columns,
               BatchedColumns& output_columns) override {
    auto& left_frame_col = input_columns[0];
    auto& right_frame_col = input_columns[1];
    check_frame(device_, left_frame_col[0]);
    assert(overlap_image_width_ != -1);

    i32 input_count = (i32)num_rows(left_frame_col);
    size_t output_image_width = overlap_image_width_;
    size_t output_image_height = frame_info_.height();
    size_t output_image_size =
        output_image_width * output_image_height * 2 * sizeof(float);
    FrameInfo info(output_image_height, output_image_width, 2, FrameType::F32);
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

      novel_view_gen_->prepare(left_overlap_input, right_overlap_input,
                               prev_frame_flow_l_to_r_, prev_frame_flow_r_to_l_,
                               prev_overlap_image_l_, prev_overlap_image_r_);

      left_overlap_input.copyTo(prev_overlap_image_l_);
      right_overlap_input.copyTo(prev_overlap_image_r_);
      prev_frame_flow_l_to_r_ = novel_view_gen_->getFlowLtoR();
      prev_frame_flow_r_to_l_ = novel_view_gen_->getFlowRtoL();

      u8* left_output = frames[2 * i]->data;
      u8* right_output = frames[2 * i + 1]->data;
      const auto& left_flow = novel_view_gen_->getFlowLtoR();
      const auto& right_flow = novel_view_gen_->getFlowRtoL();
      for (i32 r = 0; r < left_flow.rows; ++r) {
        memcpy(left_output + r * output_image_width * sizeof(float) * 2,
               left_flow.data + left_flow.step * r,
               left_flow.cols * sizeof(float) * 2);
        memcpy(right_output + r * output_image_width * sizeof(float) * 2,
               right_flow.data + right_flow.step * r,
               right_flow.cols * sizeof(float) * 2);
      }

      insert_frame(output_columns[0], frames[2 * i]);
      insert_frame(output_columns[1], frames[2 * i + 1]);
    }
  }

 private:
  surround360::proto::TemporalOpticalFlowArgs args_;
  std::unique_ptr<RigDescription> rig_;
  DeviceHandle device_;
  int overlap_image_width_;

  std::unique_ptr<NovelViewGenerator> novel_view_gen_;
  cv::Mat prev_frame_flow_l_to_r_;
  cv::Mat prev_frame_flow_r_to_l_;
  cv::Mat prev_overlap_image_l_;
  cv::Mat prev_overlap_image_r_;
};

REGISTER_OP(TemporalOpticalFlow)
    .frame_input("left_projected_frame")
    .frame_input("right_projected_frame")
    .frame_output("left_flow")
    .frame_output("right_flow");

REGISTER_KERNEL(TemporalOpticalFlow, TemporalOpticalFlowKernelCPU)
    .device(DeviceType::CPU)
    .num_devices(1);
}
