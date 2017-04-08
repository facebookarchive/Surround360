#include "render/NovelView.h"
#include "source/scanner_kernels/surround360.pb.h"

#include "scanner/api/kernel.h"
#include "scanner/api/op.h"
#include "scanner/util/memory.h"
#include "scanner/util/opencv.h"

using namespace scanner;

namespace surround360 {

class TemporalOpticalFlowKernelCPU : public VideoKernel {
 public:
  TemporalOpticalFlowKernelCPU(const Kernel::Config& config)
      : VideoKernel(config),
        device_(config.devices[0]),
        work_item_size_(config.work_item_size) {
    args_.ParseFromArray(config.args.data(), config.args.size());

    overlap_image_width_ = args_.overlap_image_width();
    novel_view_gen_.reset(
        new NovelViewGeneratorAsymmetricFlow(args_.flow_algo()));
  }

  void reset() override {
    prev_frame_flow_l_to_r_ = cv::Mat();
    prev_frame_flow_r_to_l_ = cv::Mat();
    prev_overlap_image_l_ = cv::Mat();
    prev_overlap_image_r_ = cv::Mat();
  }

  void execute(const BatchedColumns& input_columns,
               BatchedColumns& output_columns) override {
    auto& left_frame_col = input_columns[0];
    auto& left_frame_info_col = input_columns[1];
    auto& right_frame_col = input_columns[2];
    auto& right_frame_info_col = input_columns[3];
    check_frame_info(device_, left_frame_info_col);

    i32 input_count = (i32)frame_col.rows.size();
    size_t output_image_width = frame_info_.width() - overlap_image_width_;
    size_t output_image_height = frame_info_.height();
    size_t output_image_size =
        output_image_width * output_image_height * 2 * sizeof(float);
    u8 *output_block = new_block_buffer(
        device_, output_image_size * input_count * 2, input_count * 2);

    // Output frame info
    scanner::proto::FrameInfo output_frame_info;
    output_frame_info.set_width(output_image_width);
    output_frame_info.set_height(output_image_height);
    output_frame_info.set_channels(2);
    u8 *output_frame_info_buffer = new_block_buffer(
        device_, output_frame_info.ByteSize(), input_count * 2);
    output_frame_info.SerializeToArray(output_frame_info_buffer,
                                       output_frame_info.ByteSize());

    for (i32 i = 0; i < input_count; ++i) {
      cv::Mat left_input(frame_info_.height(), frame_info_.width(), CV_8UC3,
                         input_columns[0].rows[i].buffer);
      cv::Mat right_input(frame_info_.height(), frame_info_.width(), CV_8UC3,
                          input_columns[0].rows[i].buffer);

      cv::Mat left_overlap_input =
          left_input(cv::Rect(left_input.cols - overlap_image_width_, 0,
                              overlap_image_width_, left_input.rows));
      cv::Mat right_overlap_input =
          right_input(cv::Rect(0, 0,
                               overlap_image_width_, right_input.rows));

      novel_view_gen_->prepare(left_overlap_input, right_overlap_input,
                               prev_frame_flow_l_to_r_, prev_frame_flow_r_to_l_,
                               prev_overlap_image_l_, prev_overlap_image_r_);

      prev_overlap_image_l_ = left_overlap_input;
      prev_overlap_image_r_ = right_overlap_input;
      prev_frame_flow_l_to_r_ = novel_view_gen_->getFlowLtoR();
      prev_frame_flow_r_to_l_ = novel_view_gen_->getFlowRtoL();

      u8* left_output = output_block_buffer + output_image_size * (2 * i);
      u8* right_output = output_block_buffer + output_image_size * (2 * i + 1);
      auto& left_flow = novel_view_gen_->getFlowLtoR();
      auto& right_flow = novel_view_gen_->getFlowRtoL();
      for (i32 r = 0; r < left_flow.rows; ++r) {
        memcpy(left_output + r * output_image_width * sizeof(float) * 2,
               left_flow.data + left_flow.step * r,
               left_flow.cols * sizeof(float) * 2);
        memcpy(right_output + r * output_image_width * sizeof(float) * 2,
               right_flow.data + right_flow.step * r,
               right_flow.cols * sizeof(float) * 2);
      }

      output_columns[0].rows.push_back(
          Row{left_output, output_image_size});
      output_columns[1].rows.push_back(
          Row{output_frame_info_buffer, output_frame_info.ByteSize()});
      output_columns[2].rows.push_back(
          Row{right_output, output_image_size});
      output_columns[3].rows.push_back(
          Row{output_frame_info_buffer, output_frame_info.ByteSize()});
    }
  }

 private:
  surround360::proto::TemporalOpticalFlowArgs args_;
  std::unique_ptr<RigDescription> rig_;
  DeviceHandle device_;
  i32 work_item_size_;
  bool is_reset_ = true;

  std::unique_ptr<NovelViewGenerator> novel_view_gen_;
  int camIdx_;
  cv::Mat prev_frame_flow_l_to_r_;
  cv::Mat prev_frame_flow_r_to_l_;
  cv::Mat prev_overlap_image_l_;
  cv::Mat prev_overlap_image_r_;
};

REGISTER_OP(TemporalOpticalFlow)
    .inputs({"projected_frame_left", "frame_info_left", "projeted_frame_right",
             "frame_info_right"})
    .outputs({"flow_left", "frame_info_left", "flow_right",
              "frame_info_right"});

REGISTER_KERNEL(TemporalOpticalFlow, TemporalOpticalFlowKernelCPU)
    .device(DeviceType::CPU)
    .num_devices(1);
}
