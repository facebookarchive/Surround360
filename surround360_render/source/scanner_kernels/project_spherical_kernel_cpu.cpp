#include "render/RigDescription.h"
#include "render/ImageWarper.h"
#include "source/scanner_kernels/surround360.pb.h"

#include "scanner/api/kernel.h"
#include "scanner/api/op.h"
#include "scanner/util/memory.h"
#include "scanner/util/opencv.h"

#include <opencv2/video.hpp>

using namespace scanner;

namespace surround360 {

class ProjectSphericalKernelCPU : public VideoKernel {
 public:
  ProjectSphericalKernelCPU(const Kernel::Config& config)
      : VideoKernel(config),
        device_(config.devices[0]),
        work_item_size_(config.work_item_size) {
    args_.ParseFromArray(config.args.data(), config.args.size());

    // Initialize camera rig
    rig_.reset(new RigDescription(args_.camera_rig_path()));

    hRadians_ = 2 * approximateFov(rig_->rigSideOnly, false);
    vRadians_ = 2 * approximateFov(rig_->rigSideOnly, true);
  }

  void reset() override {
    is_reset_ = true;
  }

  void execute(const BatchedColumns& input_columns,
               BatchedColumns& output_columns) override {
    auto& frame_col = input_columns[0];
    auto& camera_id_col = input_columns[1];
    check_frame(device_, frame_col[0]);

    if (is_reset_) {
      // Use the new camera id to update the spherical projection parameters
      is_reset_ = false;

      camIdx_ = *((int*)camera_id_col[0].buffer);
      const Camera& camera = rig_->rigSideOnly[camIdx_];

      // the negative sign here is so the camera array goes clockwise
      const int numCameras = 14;
      float direction = -float(camIdx_) / float(numCameras) * 2.0f * M_PI;
      leftAngle_ = direction + hRadians_ / 2;
      rightAngle_ = direction - hRadians_ / 2;
      topAngle_ = vRadians_ / 2;
      bottomAngle_ = -vRadians_ / 2;
    }

    i32 input_count = (i32)num_rows(frame_col);
    size_t output_image_width = args_.eqr_width() * hRadians_ / (2 * M_PI);
    size_t output_image_height = args_.eqr_height() * vRadians_ / M_PI;
    size_t output_image_size = output_image_width * output_image_height * 4;
    FrameInfo info(output_image_height, output_image_width, 4, FrameType::U8);
    std::vector<Frame*> output_frames = new_frames(device_, info, input_count);

    for (i32 i = 0; i < input_count; ++i) {
      cv::Mat input = frame_to_mat(frame_col[i].as_const_frame());
      cv::Mat tmp;
      cv::cvtColor(input, tmp, CV_BGR2BGRA);

      cv::Mat projection_image = frame_to_mat(output_frames[i]);

      surround360::warper::bicubicRemapToSpherical(
          projection_image, tmp, rig_->rigSideOnly[camIdx_], leftAngle_,
          rightAngle_, topAngle_, bottomAngle_);

      insert_frame(output_columns[0], output_frames[i]);
    }
  }

 private:
  surround360::proto::ProjectSphericalArgs args_;
  std::unique_ptr<RigDescription> rig_;
  DeviceHandle device_;
  i32 work_item_size_;
  bool is_reset_ = true;

  float hRadians_;
  float vRadians_;

  int camIdx_;
  float leftAngle_;
  float rightAngle_;
  float topAngle_;
  float bottomAngle_;
};

REGISTER_OP(ProjectSpherical)
    .frame_input("frame")
    .input("camera_id")
    .frame_output("projected_frame");

REGISTER_KERNEL(ProjectSpherical, ProjectSphericalKernelCPU)
    .device(DeviceType::CPU)
    .num_devices(1);
}
