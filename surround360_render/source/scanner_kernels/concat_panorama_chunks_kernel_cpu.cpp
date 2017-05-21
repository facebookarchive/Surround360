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
namespace {
void padToheight(Mat& unpaddedImage, const int targetHeight) {
  const int paddingAbove = (targetHeight - unpaddedImage.rows) / 2;
  const int paddingBelow = targetHeight - unpaddedImage.rows - paddingAbove;
  cv::copyMakeBorder(
    unpaddedImage,
    unpaddedImage,
    paddingAbove,
    paddingBelow,
    0,
    0,
    BORDER_CONSTANT,
    Scalar(0.0, 0.0, 0.0));
}
}

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
    check_frame(device_, input_columns[0][0]);

    i32 input_count = (i32)num_rows(input_columns[0]);
    size_t output_image_width = (size_t)args_.final_eqr_width();
    size_t output_image_height = (size_t)args_.final_eqr_height();
    output_image_width += (output_image_width % 2);
    output_image_height += (output_image_height % 2);
    size_t output_image_size =
        output_image_width * output_image_height * 3;
    FrameInfo info(output_image_height, output_image_width, 3, FrameType::U8);
    std::vector<Frame*> output_frames = new_frames(device_, info, input_count);

    std::vector<cv::Mat> left_pano_chunks(num_chunks_, Mat());
    std::vector<cv::Mat> right_pano_chunks(num_chunks_, Mat());
    cv::Mat left_pano;
    cv::Mat right_pano;
    for (i32 i = 0; i < input_count; ++i) {
      for (i32 c = 0; c < num_chunks_; ++c) {
        auto &left_chunk_col = input_columns[c];
        cv::cvtColor(frame_to_mat(left_chunk_col[i].as_const_frame()),
                     left_pano_chunks[c], CV_BGRA2BGR);

        auto &right_chunk_col = input_columns[num_chunks_ + c];
        cv::cvtColor(frame_to_mat(right_chunk_col[i].as_const_frame()),
                     right_pano_chunks[c], CV_BGRA2BGR);
      }
      left_pano = stackHorizontal(left_pano_chunks);
      left_pano =
          offsetHorizontalWrap(left_pano, zeroParallaxNovelViewShiftPixels_);

      right_pano = stackHorizontal(right_pano_chunks);
      right_pano =
          offsetHorizontalWrap(right_pano, -zeroParallaxNovelViewShiftPixels_);

      padToheight(left_pano, args_.eqr_height());
      padToheight(right_pano, args_.eqr_height());

      resize(left_pano, left_pano,
             Size(args_.final_eqr_width(), args_.final_eqr_height() / 2), 0, 0,
             INTER_CUBIC);
      resize(right_pano, right_pano,
             Size(args_.final_eqr_width(), args_.final_eqr_height() / 2), 0, 0,
             INTER_CUBIC);

      cv::Mat stereo_equirect =
          stackVertical(vector<cv::Mat>({left_pano, right_pano}));

      u8 *output = output_frames[i]->data;
      for (i32 r = 0; r < stereo_equirect.rows; ++r) {
        memcpy(output + r * output_image_width * sizeof(char) * 3,
               stereo_equirect.data + stereo_equirect.step * r,
               stereo_equirect.cols * sizeof(char) * 3);
      }

      insert_frame(output_columns[0], output_frames[i]);
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
    .frame_output("panorama");

REGISTER_KERNEL(ConcatPanoramaChunks, ConcatPanoramaChunksKernelCPU)
    .device(DeviceType::CPU)
    .num_devices(1);
}
