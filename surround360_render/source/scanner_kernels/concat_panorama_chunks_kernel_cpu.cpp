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

class ConcatPanoramaChunksKernelCPU : public VideoKernel {
 public:
  ConcatPanoramaChunksKernelCPU(const Kernel::Config& config)
      : VideoKernel(config),
        device_(config.devices[0]),
        work_item_size_(config.work_item_size) {
    args_.ParseFromArray(config.args.data(), config.args.size());

    rig_.reset(new RigDescription(args_.camera_rig_path()));

    num_chunks_ = config.input_columns.size() / 2;
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
    const int overlapImageWidth =
        float(camImageWidth) * (overlapAngleDegrees / camFovHorizontalDegrees);

    const float v =
        atanf(args_.zero_parallax_dist() / (args_.interpupilary_dist() / 2.0f));
    const float psi =
        asinf(sinf(v) * (args_.interpupilary_dist() / 2.0f) / cameraRingRadius);
    const float vergeAtInfinitySlabDisplacement =
        psi * (float(camImageWidth) / fovHorizontalRadians);
    const float theta = -M_PI / 2.0f + v + psi;
    zeroParallaxNovelViewShiftPixels_ =
        float(args_.eqr_width()) * (theta / (2.0f * M_PI));
    if (!args_.left()) {
      zeroParallaxNovelViewShiftPixels_ *= -1;
    }
  }

  void execute(const BatchedColumns& input_columns,
               BatchedColumns& output_columns) override {
    check_frame_info(device_, input_columns[1]);

    i32 input_count = (i32)input_columns[0].rows.size();
    size_t output_image_width = frame_info_.width() * num_chunks_;
    size_t output_image_height = frame_info_.height();
    size_t output_image_size =
        output_image_width * output_image_height * 3;
    u8 *output_block_buffer = new_block_buffer(
        device_, output_image_size * input_count, input_count);

    // Output frame info
    scanner::proto::FrameInfo output_frame_info;
    output_frame_info.set_width(output_image_width);
    output_frame_info.set_height(output_image_height);
    output_frame_info.set_channels(3);
    u8 *output_frame_info_buffer = new_block_buffer(
        device_, output_frame_info.ByteSize(), input_count);
    output_frame_info.SerializeToArray(output_frame_info_buffer,
                                       output_frame_info.ByteSize());

    std::vector<cv::Mat> pano_chunks(num_chunks_, Mat());
    cv::Mat pano;
    for (i32 i = 0; i < input_count; ++i) {
      for (i32 c = 0; c < num_chunks_; ++c) {
        auto &chunk_col = input_columns[c * 2];
        pano_chunks[c] = cv::Mat(frame_info_.height(), frame_info_.width(),
                                 CV_8UC4, chunk_col.rows[i].buffer);
        cv::cvtColor(pano_chunks[c], pano_chunks[c], CV_BGRA2BGR);
      }
      pano = stackHorizontal(pano_chunks);
      pano = offsetHorizontalWrap(pano, zeroParallaxNovelViewShiftPixels_);

      u8 *output = output_block_buffer + output_image_size * i;
      for (i32 r = 0; r < pano.rows; ++r) {
        memcpy(output + r * output_image_width * sizeof(char) * 3,
               pano.data + pano.step * r, pano.cols * sizeof(char) * 3);
      }

      output_columns[0].rows.push_back(Row{output, output_image_size});
      output_columns[1].rows.push_back(
          Row{output_frame_info_buffer, output_frame_info.ByteSize()});
    }
  }

  private:
    surround360::proto::ConcatPanoramaChunksArgs args_;
    std::unique_ptr<RigDescription> rig_;
    DeviceHandle device_;
    int work_item_size_;
    int num_chunks_;
    float zeroParallaxNovelViewShiftPixels_;
};

REGISTER_OP(ConcatPanoramaChunks)
    .variadic_inputs()
    .outputs({"panorama", "frame_info"});

REGISTER_KERNEL(ConcatPanoramaChunks, ConcatPanoramaChunksKernelCPU)
    .device(DeviceType::CPU)
    .num_devices(1);
}
